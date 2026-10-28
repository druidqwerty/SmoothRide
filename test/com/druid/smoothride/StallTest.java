package com.druid.smoothride;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * Offline check of server stalls (the co-op freeze + teleport bug): a 25 m/s highway drive, the
 * driver sends every 33 ms, the server relays nothing for S ms (world save, chunk loading), then
 * either drops the backlog (vanilla does when its loop is past 70 ms) or relays it in one burst.
 * Each run has one stall; the 1.0.0 Track (TrackV10) and the current Track replay the same packets.
 *
 * Per stall: freeze = longest stretch the drawn car moves at under 30 % of its true speed; jump =
 * largest one-frame displacement beyond the true one (a teleport shows here); err = distance from
 * where the car should be drawn (truth at t - normal lag); recovery = time after the stall until
 * err stays under 0.5 m.
 */
public final class StallTest {
    static final double STEP = 0.01;
    static final int DUR_STEPS = 6000; // 60 s
    static final double[] tx = new double[DUR_STEPS + 1], ty = new double[DUR_STEPS + 1], th = new double[DUR_STEPS + 1],
            tv = new double[DUR_STEPS + 1];

    static void buildTruth() {
        double x = 0, y = 0, h = 0, v = 25;
        for (int i = 0; i <= DUR_STEPS; i++) {
            double t = i * STEP;
            tx[i] = x; ty[i] = y; th[i] = h; tv[i] = v;
            double acc = 0, yaw = 0;
            if (t < 12) { yaw = 0; }                                         // straight
            else if (t < 20) { yaw = v / 200.0; }                            // gentle curve R=200 m
            else if (t < 26) { yaw = 0; }
            else if (t < 29) { yaw = 0.15 * Math.sin(2 * Math.PI * (t - 26) / 3.0); } // lane change
            else if (t < 34) { yaw = 0; }
            else if (t < 36.2) { acc = -6; }                                 // braking to ~12 m/s
            else if (t < 38.2) { acc = 0; }
            else if (t < 42.5) { acc = 3; }                                  // back to ~25 m/s
            else if (t < 46.5) { yaw = v / 80.0; }                           // sharp curve R=80 m
            v = Math.max(0, v + acc * STEP);
            h += yaw * STEP;
            x += v * Math.cos(h) * STEP;
            y += v * Math.sin(h) * STEP;
        }
    }

    static double truth(double tMs, double[] a) {
        double f = tMs / 10.0;
        if (f <= 0) return a[0];
        if (f >= DUR_STEPS) return a[DUR_STEPS];
        int i = (int) Math.floor(f);
        double u = f - i;
        return a[i] + (a[i + 1] - a[i]) * u;
    }

    static Sample snap(int step, long timeMs) {
        Sample s = new Sample();
        s.t = timeMs;
        s.x = (float) tx[step];
        s.y = (float) ty[step];
        s.z = 0f;
        double h = th[step];
        // the game's convention: body forward = local +z, heading h -> rotation about Bullet y by (pi/2 - h)
        double r = Math.PI / 2 - h;
        s.qx = 0f; s.qy = (float) Math.sin(r / 2); s.qz = 0f; s.qw = (float) Math.cos(r / 2);
        double v = tv[step];
        s.vx = (float) (v * Math.cos(h));
        s.vy = 0f;
        s.vz = (float) (v * Math.sin(h));
        s.wc = 4;
        return s;
    }

    static final class Packet {
        Sample s;
        double arrive;
        boolean stale; // relayed late by the stalled server (burst)
    }

    /** driver at 60 FPS sending every 33 ms; server stall [ts, ts+dur): drop or burst the backlog */
    static List<Packet> transmit(Random rnd, double ts, double dur, boolean burst) {
        List<Packet> out = new ArrayList<>();
        double frame = 1000.0 / 60.0;
        double tReal = 0, last = 0, accum = 0;
        int step = 0;
        long simTime = 0;
        double delay = 33;
        while (tReal < DUR_STEPS * 10.0 - 50) {
            double dt = frame * (0.85 + 0.3 * rnd.nextDouble());
            tReal += dt;
            accum += dt;
            while (accum >= 10.0 && step < DUR_STEPS) { accum -= 10.0; step++; simTime += 10; }
            if (tReal - last > delay) {
                last = (tReal - last > 3 * delay) ? tReal : last + delay;
                double atServer = tReal + 20 + Math.abs(rnd.nextGaussian()) * 6;
                double relay = 20 + Math.abs(rnd.nextGaussian()) * 6 + 5;
                Packet p = new Packet();
                p.s = snap(step, simTime);
                if (atServer >= ts && atServer < ts + dur) {
                    if (!burst) continue;
                    p.arrive = ts + dur + 22 + 0.01 * out.size(); // in send order, as the server relays its queue
                    p.stale = true;
                } else {
                    p.arrive = atServer + relay;
                }
                out.add(p);
            }
        }
        out.sort((a, b) -> Double.compare(a.arrive, b.arrive));
        return out;
    }

    interface Rx {
        void add(Sample s, double now);
        boolean get(double now, float[] out);
        int snaps();
        String name();
    }

    /** The game's side of a buffer: with no pose the car stays where it was drawn; a new buffer is seeded there. */
    abstract static class GameRx implements Rx {
        final float[] b1 = new float[27], b2 = new float[2];
        float lx = Float.NaN, ly;
        abstract boolean begin(Sample c);
        abstract Sample obtain();
        abstract void recycle(Sample s);
        abstract double peek(double now);
        abstract void put(Sample s, double now);
        abstract boolean pose(double now);
        public void add(Sample s, double now) {
            Sample c = obtain();
            c.copyFrom(s);
            if (begin(c)) {
                Sample seed = obtain();
                seed.copyFrom(c);
                if (!Float.isNaN(lx)) { seed.x = lx; seed.y = ly; }
                seed.synthetic = true;
                seed.t = (long) Math.floor(peek(now));
                if (seed.t < c.t) put(seed, now); else recycle(seed);
            }
            put(c, now);
        }
        public boolean get(double now, float[] out) {
            if (!pose(now)) {
                if (Float.isNaN(lx)) return false;
                out[0] = lx; out[1] = ly;
                return true;
            }
            out[0] = b1[0]; out[1] = b1[1];
            lx = out[0]; ly = out[1];
            return true;
        }
    }

    static final class NewRx extends GameRx {
        final Track tr = new Track();
        public String name() { return "1.1"; }
        public int snaps() { return tr.stSnap; }
        boolean begin(Sample c) { return tr.beginAdd(c); }
        Sample obtain() { return tr.obtain(); }
        void recycle(Sample x) { tr.recycle(x); }
        double peek(double now) { return tr.peekRenderTime(now); }
        void put(Sample x, double now) { tr.add(x, now); }
        boolean pose(double now) { return tr.get(now, Double.NaN, b1, b2) == Track.OK; }
    }

    static final class OldRx extends GameRx {
        final TrackV10 tr = new TrackV10();
        public String name() { return "1.0"; }
        public int snaps() { return tr.stSnap; }
        boolean begin(Sample c) { return tr.beginAdd(c); }
        Sample obtain() { return tr.obtain(); }
        void recycle(Sample x) { tr.recycle(x); }
        double peek(double now) { return tr.peekRenderTime(now); }
        void put(Sample x, double now) { tr.add(x, now); }
        boolean pose(double now) { return tr.get(now, Double.NaN, b1, b2) == TrackV10.OK; }
    }

    static final class Ev {
        double freezeMs, maxJump, errResume, maxErr, recoverMs, maxSpeedDev;
        int snaps;
    }

    static Ev run(Rx rx, List<Packet> pk, double ts, double dur, double fps, Random rnd) {
        double frame = 1000.0 / fps;
        double t = 0;
        int pi = 0;
        float[] out = new float[2];
        List<Double> lags = new ArrayList<>();
        double d0 = Double.NaN;
        Ev ev = new Ev();
        int snaps0 = 0;
        double px = Double.NaN, py = 0, pt = 0;
        double freezeStart = Double.NaN;
        double firstAfter = Double.NaN; // first post-stall packet arrival
        boolean resumedMeasured = false;
        double lastBad = ts + dur;
        double end = Math.min(DUR_STEPS * 10.0 - 500, ts + dur + 3500);
        while (t < end) {
            double dt = frame * (0.9 + 0.2 * rnd.nextDouble());
            t += dt;
            while (pi < pk.size() && pk.get(pi).arrive <= t) {
                Packet p = pk.get(pi);
                if (Double.isNaN(firstAfter) && p.arrive > ts + dur && !p.stale) firstAfter = p.arrive;
                rx.add(p.s, t);
                pi++;
            }
            if (t < 1500) continue;
            if (!rx.get(t, out)) { px = Double.NaN; continue; }
            if (t < ts) {
                if (t > ts - 2000) {
                    double best = 1e18, bestT = t;
                    for (double q = t; q > t - 1500; q -= 1) {
                        double ex = truth(q, tx) - out[0], ey = truth(q, ty) - out[1];
                        double e = ex * ex + ey * ey;
                        if (e < best) { best = e; bestT = q; }
                    }
                    lags.add(t - bestT);
                }
                px = out[0]; py = out[1]; pt = t;
                snaps0 = rx.snaps();
                continue;
            }
            if (Double.isNaN(d0)) {
                double[] a = lags.stream().mapToDouble(Double::doubleValue).toArray();
                Arrays.sort(a);
                d0 = a.length > 0 ? a[a.length / 2] : 100;
            }
            double ref = t - d0;
            double ex = out[0] - truth(ref, tx), ey = out[1] - truth(ref, ty);
            double err = Math.sqrt(ex * ex + ey * ey);
            ev.maxErr = Math.max(ev.maxErr, err);
            if (err > 0.5) lastBad = t;
            if (!resumedMeasured && !Double.isNaN(firstAfter) && t >= firstAfter) {
                ev.errResume = err;
                resumedMeasured = true;
            }
            if (!Double.isNaN(px)) {
                double fdt = t - pt;
                double tvx = truth(ref, tx) - truth(ref - fdt, tx), tvy = truth(ref, ty) - truth(ref - fdt, ty);
                double dvx = out[0] - px, dvy = out[1] - py;
                double jump = Math.hypot(dvx - tvx, dvy - tvy);
                ev.maxJump = Math.max(ev.maxJump, jump);
                if (debug && t > ts - 100 && t < ts + dur + 400 && rx instanceof NewRx) {
                    Track k = ((NewRx) rx).tr;
                    System.out.println(String.format(Locale.ROOT, "t=%8.1f mode=%d n=%3d delay=%5.1f err=%6.2f jump=%5.2f c=(%6.2f,%6.2f) arrived=%d",
                            t, k.lastMode, k.n, k.delay, err, jump, k.cx, k.cy, pi));
                }
                double vTrue = Math.hypot(tvx, tvy) / (fdt / 1000.0);
                double vDraw = Math.hypot(dvx, dvy) / (fdt / 1000.0);
                ev.maxSpeedDev = Math.max(ev.maxSpeedDev, Math.abs(vDraw - vTrue));
                if (vTrue > 5 && vDraw < 0.3 * vTrue) {
                    if (Double.isNaN(freezeStart)) freezeStart = pt;
                    ev.freezeMs = Math.max(ev.freezeMs, t - freezeStart);
                } else {
                    freezeStart = Double.NaN;
                }
            }
            px = out[0]; py = out[1]; pt = t;
        }
        ev.recoverMs = Math.max(0, lastBad - (ts + dur));
        ev.snaps = rx.snaps() - snaps0;
        return ev;
    }

    static boolean debug;

    public static void main(String[] args) {
        buildTruth();
        if (args.length >= 4) { // debug one run: fps scenarioStartMs durMs burst(0/1)
            debug = true;
            double ts = Double.parseDouble(args[1]), dur = Double.parseDouble(args[2]);
            boolean burst = args[3].equals("1");
            Ev e = run(new NewRx(), transmit(new Random(77), ts, dur, burst), ts, dur, Double.parseDouble(args[0]), new Random(78));
            System.out.println("freeze " + e.freezeMs + " jump " + e.maxJump + " snaps " + e.snaps);
            return;
        }
        double[] durs = {300, 600, 1000, 1500, 2000, 3000, 4000};
        String[] scen = {"straight", "curve R200", "lane change", "braking", "curve R80"};
        double[] starts = {5000, 14000, 26500, 34200, 43000};
        double fps = args.length > 0 ? Double.parseDouble(args[0]) : 100;
        for (boolean burst : new boolean[]{false, true}) {
            System.out.println("\n== server stall, backlog " + (burst ? "relayed in one burst" : "dropped") + ", observer " + (int) fps + " FPS ==");
            System.out.println(String.format(Locale.ROOT, "%-12s %5s | %-44s | %-44s", "scenario", "stall",
                    "1.0: freeze ms, jump m, err@resume, max err, recov ms, snaps", "1.1: same"));
            double[] sum = new double[12];
            int n = 0;
            for (int si = 0; si < scen.length; si++) {
                for (double dur : durs) {
                    long seed = 77 + si * 31 + (long) dur;
                    Ev[] e = new Ev[2];
                    Rx[] rxs = {new OldRx(), new NewRx()};
                    for (int k = 0; k < 2; k++) {
                        List<Packet> pk = transmit(new Random(seed), starts[si], dur, burst);
                        e[k] = run(rxs[k], pk, starts[si], dur, fps, new Random(seed + 1));
                        sum[k * 6] += e[k].freezeMs; sum[k * 6 + 1] = Math.max(sum[k * 6 + 1], e[k].maxJump);
                        sum[k * 6 + 2] += e[k].errResume; sum[k * 6 + 3] = Math.max(sum[k * 6 + 3], e[k].maxErr);
                        sum[k * 6 + 4] += e[k].recoverMs; sum[k * 6 + 5] += e[k].snaps;
                    }
                    n++;
                    System.out.println(String.format(Locale.ROOT, "%-12s %5.0f | %6.0f %6.2f %6.2f %6.2f %6.0f %3d       | %6.0f %6.2f %6.2f %6.2f %6.0f %3d",
                            scen[si], dur, e[0].freezeMs, e[0].maxJump, e[0].errResume, e[0].maxErr, e[0].recoverMs, e[0].snaps,
                            e[1].freezeMs, e[1].maxJump, e[1].errResume, e[1].maxErr, e[1].recoverMs, e[1].snaps));
                }
            }
            System.out.println(String.format(Locale.ROOT, "%-18s | avg freeze %4.0f ms, max jump %5.2f m, avg err@resume %5.2f m, max err %5.2f m, avg recov %4.0f ms, snaps %3.0f",
                    "1.0 summary", sum[0] / n, sum[1], sum[2] / n, sum[3], sum[4] / n, sum[5]));
            System.out.println(String.format(Locale.ROOT, "%-18s | avg freeze %4.0f ms, max jump %5.2f m, avg err@resume %5.2f m, max err %5.2f m, avg recov %4.0f ms, snaps %3.0f",
                    "1.1 summary", sum[6] / n, sum[7], sum[8] / n, sum[9], sum[10] / n, sum[11]));
        }
    }
}
