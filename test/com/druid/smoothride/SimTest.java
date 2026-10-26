package com.druid.smoothride;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;

/**
 * Offline check of the remote-vehicle buffer: a scripted drive (acceleration, curve, hard braking,
 * slalom, a crash) is sampled the way the game does it, sent through a jittery lossy link and
 * replayed by (a) a model of vanilla VehicleInterpolation and (b) SmoothRide's Track.
 */
public final class SimTest {

    // ---------------- ground truth: 10 ms fixed-step car ----------------
    static final double STEP = 0.01;
    static final int DUR_STEPS = 3200; // 32 s
    static double[] tx = new double[DUR_STEPS + 1], ty = new double[DUR_STEPS + 1], th = new double[DUR_STEPS + 1],
            tv = new double[DUR_STEPS + 1];

    static void buildTruth() {
        double x = 0, y = 0, h = 0, v = 0;
        for (int i = 0; i <= DUR_STEPS; i++) {
            double t = i * STEP;
            tx[i] = x; ty[i] = y; th[i] = h; tv[i] = v;
            double acc, yaw;
            if (t < 4) { acc = 6; yaw = 0; }                       // launch to ~24 m/s
            else if (t < 9) { acc = 0; yaw = 0.35; }                // long curve
            else if (t < 11) { acc = -8; yaw = 0; }                 // hard braking
            else if (t < 14) { acc = 5; yaw = 0; }
            else if (t < 22) { acc = 0; yaw = 0.9 * Math.sin((t - 14) * 2.4); } // slalom
            else if (t < 25) { acc = 0; yaw = 0; }
            else { acc = 0; yaw = 0; }
            if (i == (int) (25.0 / STEP)) { v *= 0.25; h += Math.toRadians(25); } // crash: instant change
            if (t > 26 && t < 27) acc = -3;
            v = Math.max(0, v + acc * STEP);
            h += yaw * STEP;
            x += v * Math.cos(h) * STEP;
            y += v * Math.sin(h) * STEP;
        }
    }

    static double truthAt(double tMs, int comp) {
        double f = tMs / 10.0;
        if (f <= 0) return comp == 0 ? tx[0] : ty[0];
        if (f >= DUR_STEPS) return comp == 0 ? tx[DUR_STEPS] : ty[DUR_STEPS];
        int i = (int) Math.floor(f);
        double u = f - i;
        double[] a = comp == 0 ? tx : ty;
        return a[i] + (a[i + 1] - a[i]) * u;
    }

    static Sample snap(int step, long timeMs) {
        Sample s = new Sample();
        s.t = timeMs;
        s.x = (float) tx[step];
        s.y = (float) ty[step];
        s.z = 0f;
        double h = th[step];
        // rotation about the up axis (Bullet y)
        s.qx = 0f; s.qy = (float) Math.sin(h / 2); s.qz = 0f; s.qw = (float) Math.cos(h / 2);
        double v = tv[step];
        s.vx = (float) (v * Math.cos(h));
        s.vy = 0f;
        s.vz = (float) (v * Math.sin(h));
        s.wc = 4;
        return s;
    }

    // ---------------- network ----------------
    static final class Packet {
        Sample s;
        double arrive;
    }

    static List<Packet> transmit(Random rnd, double sendIntervalMs, double baseLat, double jitter,
                                 double spikeChance, double loss, double senderFps) {
        List<Packet> out = new ArrayList<>();
        double frame = 1000.0 / senderFps;
        double tReal = 0;
        double nextSend = 0;
        double last = 0;
        double delay = sendIntervalMs;
        double accum = 0;
        int step = 0;
        long simTime = 0;
        while (tReal < DUR_STEPS * 10.0 - 50) {
            double dt = frame * (0.85 + 0.3 * rnd.nextDouble());
            tReal += dt;
            accum += dt;
            while (accum >= 10.0 && step < DUR_STEPS) { accum -= 10.0; step++; simTime += 10; }
            // UpdateLimit semantics
            if (tReal - last > delay) {
                last = (tReal - last > 3 * delay) ? tReal : last + delay;
                if (rnd.nextDouble() >= loss) {
                    Packet p = new Packet();
                    p.s = snap(step, simTime);
                    double lat = baseLat + Math.abs(rnd.nextGaussian()) * jitter;
                    if (rnd.nextDouble() < spikeChance) lat += 60 + rnd.nextDouble() * 120;
                    p.arrive = tReal + lat + 5; // +5: client main-loop queue delay
                    out.add(p);
                }
            }
        }
        out.sort((a, b) -> Double.compare(a.arrive, b.arrive));
        return out;
    }

    // ---------------- receivers ----------------
    interface Receiver {
        void add(Sample s, double nowSmooth);
        /** returns false if no pose; writes x,y to out */
        boolean get(double nowSmooth, double nowQuantized, float[] out);
        String name();
    }

    /** Close model of vanilla VehicleInterpolation (500 ms delay, WorldSimulation 10 ms clock). */
    static final class Vanilla implements Receiver {
        final TreeMap<Long, Sample> buf = new TreeMap<>();
        boolean buffering = true;
        public String name() { return "vanilla"; }
        public void add(Sample s, double now) {
            buf.put(s.t, s);
            long cut = (long) now - 800;
            while (!buf.isEmpty() && buf.firstKey() < cut && buf.size() > 1) buf.pollFirstEntry();
        }
        public boolean get(double nowSmooth, double nowQ, float[] out) {
            long time = (long) nowQ - 500;
            Map.Entry<Long, Sample> a = buf.floorEntry(time);
            Map.Entry<Long, Sample> b = buf.higherEntry(time);
            if (buffering) {
                if (buf.size() < 2 || a == null || b == null) return false;
                buffering = false;
            }
            if (b == null) {
                if (a == null) { buffering = true; return false; }
                out[0] = a.getValue().x; out[1] = a.getValue().y;
                buffering = true;
                return true;
            }
            if (a == null) return false;
            double u = (double) (time - a.getKey()) / (b.getKey() - a.getKey());
            out[0] = (float) (a.getValue().x + (b.getValue().x - a.getValue().x) * u);
            out[1] = (float) (a.getValue().y + (b.getValue().y - a.getValue().y) * u);
            return true;
        }
    }

    /** Reference: the true path itself, to show the measurement floor. */
    static final class Truth implements Receiver {
        public String name() { return "truth (reference)"; }
        public void add(Sample s, double now) { }
        public boolean get(double now, double nowQ, float[] out) {
            out[0] = (float) truthAt(now, 0);
            out[1] = (float) truthAt(now, 1);
            return true;
        }
    }

    static final class Smooth implements Receiver {
        final Track tr = new Track();
        final float[] b1 = new float[27];
        final float[] b2 = new float[2];
        public String name() { return "smoothride-" + Config.mode; }
        public void add(Sample s, double now) {
            Sample c = tr.obtain();
            c.copyFrom(s);
            if (tr.beginAdd(c)) {
                // seed: nothing drawn yet in the sim, so seed at the sample itself slightly earlier
                Sample seed = tr.obtain();
                seed.copyFrom(c);
                seed.synthetic = true;
                seed.t = (long) Math.floor(tr.peekRenderTime(now));
                if (seed.t < c.t) tr.add(seed, now);
            }
            tr.add(c, now);
        }
        public boolean get(double now, double nowQ, float[] out) {
            if (tr.get(now, Double.NaN, b1, b2) != Track.OK) return false;
            out[0] = b1[0]; out[1] = b1[1];
            return true;
        }
    }

    // ---------------- run ----------------
    static final class Result {
        String name;
        double fps;
        int frames, noPose;
        double p99Acc, maxAcc, p95ErrRender, maxErrRender, meanLagMs, p95ErrNow;
        double meanSpeedErr;
    }

    static Result run(Receiver rx, List<Packet> packets, double fps, Random rnd) {
        double frame = 1000.0 / fps;
        double t = 0;
        int pi = 0;
        double wsTime = 0, accum = 0;
        float[] out = new float[2];
        List<Double> accs = new ArrayList<>();
        List<Double> errNow = new ArrayList<>();
        double prevX = Double.NaN, prevY = 0, prevVx = Double.NaN, prevVy = 0, prevT = 0;
        Result res = new Result();
        res.name = rx.name();
        res.fps = fps;
        double lagSum = 0;
        int lagN = 0;
        while (t < DUR_STEPS * 10.0 - 400) {
            double dt = frame * (0.9 + 0.2 * rnd.nextDouble());
            t += dt;
            accum += dt;
            while (accum >= 10.0) { accum -= 10.0; wsTime += 10; }
            while (pi < packets.size() && packets.get(pi).arrive <= t) {
                rx.add(packets.get(pi).s, t);
                pi++;
            }
            if (t < 1500) continue; // warm-up
            res.frames++;
            if (!rx.get(t, wsTime, out)) { res.noPose++; continue; }
            // lag: find truth time whose position is closest to the output (search back 1.5 s)
            double best = 1e9, bestT = t;
            for (double q = t; q > t - 1500; q -= 2) {
                double ex = truthAt(q, 0) - out[0], ey = truthAt(q, 1) - out[1];
                double e = ex * ex + ey * ey;
                if (e < best) { best = e; bestT = q; }
            }
            if (truthSpeed(t) > 3) { lagSum += t - bestT; lagN++; }
            double enx = truthAt(t, 0) - out[0], eny = truthAt(t, 1) - out[1];
            errNow.add(Math.sqrt(enx * enx + eny * eny));
            if (!Double.isNaN(prevX)) {
                double fdt = (t - prevT) / 1000.0;
                double vx = (out[0] - prevX) / fdt, vy = (out[1] - prevY) / fdt;
                if (!Double.isNaN(prevVx)) {
                    double ax = (vx - prevVx) / fdt, ay = (vy - prevVy) / fdt;
                    // skip the crash window: a real velocity jump there
                    if (Math.abs(bestT - 25000) > 400) accs.add(Math.sqrt(ax * ax + ay * ay));
                }
                prevVx = vx; prevVy = vy;
            }
            prevX = out[0]; prevY = out[1]; prevT = t;
        }
        res.p99Acc = pct(accs, 0.99);
        res.maxAcc = pct(accs, 1.0);
        res.meanLagMs = lagN > 0 ? lagSum / lagN : 0;
        res.p95ErrNow = pct(errNow, 0.95);
        return res;
    }

    static double truthSpeed(double tMs) {
        int i = (int) Math.max(0, Math.min(DUR_STEPS, tMs / 10.0));
        return tv[i];
    }

    static double pct(List<Double> v, double p) {
        if (v.isEmpty()) return Double.NaN;
        double[] a = v.stream().mapToDouble(Double::doubleValue).toArray();
        Arrays.sort(a);
        return a[(int) Math.round(p * (a.length - 1))];
    }

    public static void main(String[] args) {
        buildTruth();
        String[][] links = {
            // name, baseLat, jitter, spike%, loss%
            {"LAN-like 5ms", "5", "1", "0", "0"},
            {"Steam relay 35ms", "35", "6", "0.02", "0.01"},
            {"bad 90ms +spikes", "90", "20", "0.05", "0.03"},
        };
        double[] fpsList = {60, 144};
        System.out.println(String.format("%-20s %-22s %5s %8s %9s %9s %10s %9s",
                "link", "receiver", "fps", "noPose%", "acc p99", "acc max", "lag ms", "err@now95"));
        for (String[] l : links) {
            for (double fps : fpsList) {
                long seed = 1234 + (long) fps;
                // vanilla: 150 ms sends
                Random r1 = new Random(seed);
                List<Packet> pv = transmit(r1, 150, Double.parseDouble(l[1]), Double.parseDouble(l[2]),
                        Double.parseDouble(l[3]), Double.parseDouble(l[4]), 60);
                if (l == links[0]) print(l[0], run(new Truth(), pv, fps, new Random(seed + 1)));
                print(l[0], run(new Vanilla(), pv, fps, new Random(seed + 1)));
                for (String mode : new String[]{"smooth", "balanced", "realtime"}) {
                    Config.mode = mode;
                    Random r2 = new Random(seed);
                    List<Packet> ps = transmit(r2, Config.sendIntervalMs, Double.parseDouble(l[1]), Double.parseDouble(l[2]),
                            Double.parseDouble(l[3]), Double.parseDouble(l[4]), 60);
                    print(l[0], run(new Smooth(), ps, fps, new Random(seed + 1)));
                }
                Config.mode = "balanced";
            }
        }
    }

    static void print(String link, Result r) {
        System.out.println(String.format("%-20s %-22s %5.0f %7.1f%% %9.1f %9.1f %10.0f %9.2f",
                link, r.name, r.fps, 100.0 * r.noPose / Math.max(1, r.frames), r.p99Acc, r.maxAcc, r.meanLagMs, r.p95ErrNow));
    }
}
