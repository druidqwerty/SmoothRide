package com.druid.smoothride;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;
import org.joml.Quaternionf;
import zombie.GameTime;
import zombie.network.GameClient;
import zombie.vehicles.BaseVehicle;
import zombie.vehicles.VehicleInterpolation;
import zombie.vehicles.VehicleInterpolationData;

/**
 * In-process integration test: the real game classes, patched by the real ZombieBuddy agent
 * (patches_jar=SmoothRide.jar), fed with a jittery 30 Hz packet stream in real time.
 * Run from the game folder with -agentlib:zbNative=...,patches_jar=/srtest/SmoothRide.jar:com.druid.smoothride
 */
public final class AgentTest {
    private static int failures;

    public static void main(String[] args) throws Throwable {
        zombie.core.random.RandStandard.INSTANCE.init();
        zombie.core.random.RandLua.INSTANCE.init();
        GameClient.client = true;
        GameTime.setServerTimeShift(0L);

        Engine.frame();
        String st = Engine.status();
        System.out.println(st);
        expect(st.contains("interpolation override OK"), "interpolationDataGet override self-test");
        expect(!st.contains("FAILED"), "engine not failed after self-test");

        VehicleInterpolation vi = newInterpolation();
        BaseVehicle car = fakeVehicle((short) 7);

        // 1) stream test
        double[] r = stream(vi, car, 8000, 35, 6, 0.01, 144, 1234);
        expect(r[0] > 0.97, String.format("pose available %.1f%% of frames", r[0] * 100));
        expect(r[2] < 160, String.format("lag behind truth %.0f ms", r[2]));
        expect(r[3] < 3.0, String.format("error vs truth p95 %.2f m", r[3]));
        expect(r[4] < 4 * r[5] + 20, String.format("jerkiness p99 %.1f vs truth %.1f", r[4], r[5]));
        System.out.println(Engine.status());

        // 2) server echo while the car is driven remotely: must be ignored
        int before = getStaticInt(Engine.class, "filteredServerSamples");
        Engine.serverUpdateEnter();
        vi.interpolationDataAdd(car, data(GameTime.getServerTimeMills(), 0f, 0f, 0f, 0f), GameTime.getServerTimeMills());
        Engine.serverUpdateExit();
        expect(getStaticInt(Engine.class, "filteredServerSamples") == before + 1, "server position echo filtered");

        // 3) clear() empties the track: no pose until new packets arrive
        vi.clear();
        float[] b1 = new float[27];
        float[] b2 = new float[2];
        Engine.frame();
        expect(!vi.interpolationDataGet(b1, b2, null), "no pose after clear()");

        // 4) master switch off: Engine hands the call back to vanilla (vanilla returns false while buffering)
        Config.enabled = false;
        Engine.frame();
        boolean vanilla;
        try {
            vanilla = vi.interpolationDataGet(b1, b2, null);
        } catch (NullPointerException e) {
            vanilla = false; // vanilla reads GameClient.connection, which is null here: proves vanilla ran
            System.out.println("    (vanilla path reached: " + e + ")");
        }
        expect(!vanilla, "disabled -> vanilla interpolation");
        Config.enabled = true;

        // 5) the 2-argument overload must still work (it calls the patched 3-argument one)
        stream(vi, car, 1500, 20, 3, 0.0, 60, 99);
        Engine.frame();
        expect(vi.interpolationDataGet(b1, b2), "2-arg overload goes through SmoothRide");

        System.out.println(failures == 0 ? "AGENT TEST PASSED" : ("AGENT TEST FAILED: " + failures));
        System.exit(failures == 0 ? 0 : 1);
    }

    // ------------------------------------------------------------------ stream

    /** returns {okFraction, unused, lagMs, errP95, jerkP99, truthJerkP99} */
    static double[] stream(VehicleInterpolation vi, BaseVehicle car, int durMs, double latMs, double jitMs, double loss,
                           double fps, long seed) throws Exception {
        Random rnd = new Random(seed);
        double t0 = now();
        setPose(car, t0 - 100);
        PriorityQueue<double[]> net = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
        double nextSend = t0;
        List<double[]> out = new ArrayList<>();
        float[] b1 = new float[27];
        float[] b2 = new float[2];
        int frames = 0, ok = 0;
        long frameNs = (long) (1e9 / fps);
        long next = System.nanoTime();
        while (true) {
            double n = now();
            double el = n - t0;
            if (el > durMs) break;
            while (nextSend <= n) {
                long stamp = (long) (Math.floor(nextSend / 10.0) * 10.0); // WorldSimulation clock: 10 ms steps
                if (rnd.nextDouble() >= loss) {
                    double lat = latMs + Math.abs(rnd.nextGaussian()) * jitMs + (rnd.nextDouble() < 0.02 ? 40 + 80 * rnd.nextDouble() : 0);
                    net.add(new double[]{nextSend + lat, stamp});
                }
                nextSend += 33 + rnd.nextInt(17); // UpdateLimit at 60 FPS: 33..50 ms
            }
            while (!net.isEmpty() && net.peek()[0] <= n) {
                double[] p = net.poll();
                long stamp = (long) p[1];
                double[] tr = truth(stamp - t0);
                vi.interpolationDataAdd(car, data(stamp, (float) (10000 + tr[0]), (float) (10000 + tr[1]), (float) tr[2], (float) tr[3]),
                        GameTime.getServerTimeMills());
            }
            Engine.frame();
            boolean got = vi.interpolationDataGet(b1, b2, null);
            frames++;
            if (got) {
                ok++;
                out.add(new double[]{n - t0, b1[0] - 10000, b1[1] - 10000});
                car.jniTransform.origin.set(b1[0], b1[2], b1[1]); // what VehicleManager.vehiclePosUpdate does
            }
            next += frameNs;
            long sleep = next - System.nanoTime();
            if (sleep > 0) {
                Thread.sleep(sleep / 1_000_000L, (int) (sleep % 1_000_000L));
            } else {
                next = System.nanoTime(); // behind schedule: no burst of zero-length catch-up frames
            }
        }
        // metrics after a 1.5 s warm-up
        List<Double> err = new ArrayList<>(), acc = new ArrayList<>(), tacc = new ArrayList<>();
        double lagSum = 0;
        int lagN = 0;
        double[] prev = null, prevV = null, prevT = null, prevTV = null;
        for (double[] o : out) {
            if (o[0] < 1500) continue;
            if (prev != null && o[0] - prev[0] < 3.0) continue;
            double[] tr = truth(o[0]);
            // same float quantization as the game's pose buffer
            tr[0] = (float) (10000 + tr[0]) - 10000.0;
            tr[1] = (float) (10000 + tr[1]) - 10000.0;
            err.add(Math.hypot(tr[0] - o[1], tr[1] - o[2]));
            double best = 1e18, bestD = 0;
            for (double d = 0; d < 1200; d += 2) {
                double[] q = truth(o[0] - d);
                double e = Math.hypot(q[0] - o[1], q[1] - o[2]);
                if (e < best) { best = e; bestD = d; }
            }
            lagSum += bestD;
            lagN++;
            if (prev != null) {
                double dt = (o[0] - prev[0]) / 1000.0;
                if (dt > 0) {
                    double[] v = {(o[1] - prev[1]) / dt, (o[2] - prev[2]) / dt};
                    double[] tv = {(tr[0] - prevT[0]) / dt, (tr[1] - prevT[1]) / dt};
                    if (prevV != null) {
                        acc.add(Math.hypot(v[0] - prevV[0], v[1] - prevV[1]) / dt);
                        tacc.add(Math.hypot(tv[0] - prevTV[0], tv[1] - prevTV[1]) / dt);
                    }
                    prevV = v;
                    prevTV = tv;
                }
            }
            prev = o;
            prevT = tr;
        }
        double[] res = {ok / (double) Math.max(1, frames), 0, lagN > 0 ? lagSum / lagN : 1e9, pct(err, 0.95), pct(acc, 0.99), pct(tacc, 0.99)};
        System.out.println(String.format("stream %d ms @%.0f fps, latency %.0f+-%.0f ms, loss %.0f%%: pose %.1f%%, lag %.0f ms, err p95 %.2f m, jerk p99 %.1f (truth %.1f)",
                durMs, fps, latMs, jitMs, loss * 100, res[0] * 100, res[2], res[3], res[4], res[5]));
        return res;
    }

    private static final int TRUTH_MS = 15000;
    private static double[][] truthTable;

    /** x, y (m), heading (rad), speed (m/s) at t ms: launch, slalom, hard stop; 1 ms table, linear between */
    static double[] truth(double tMs) {
        if (truthTable == null) {
            truthTable = new double[TRUTH_MS + 1][];
            double x = 0, y = 0, h = 0, v = 0;
            for (int i = 0; i <= TRUTH_MS; i++) {
                truthTable[i] = new double[]{x, y, h, v};
                double s = i / 1000.0, dt = 0.001;
                double acc = s < 2.5 ? 7 : (s > 6.5 ? -9 : 0);
                double yaw = (s > 2.5 && s < 6.5) ? 0.8 * Math.sin((s - 2.5) * 2.6) : 0;
                v = Math.max(0, v + acc * dt);
                h += yaw * dt;
                x += v * Math.cos(h) * dt;
                y += v * Math.sin(h) * dt;
            }
        }
        double t = Math.max(0, Math.min(TRUTH_MS - 1, tMs));
        int i = (int) t;
        double u = t - i;
        double[] a = truthTable[i], b = truthTable[i + 1];
        return new double[]{a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u, a[3] + (b[3] - a[3]) * u};
    }

    // ------------------------------------------------------------------ game objects

    private static VarHandle dTime, dX, dY, dZ, dQx, dQy, dQz, dQw, dVx, dVy, dVz, dWc;

    static VehicleInterpolationData data(long time, float x, float y, float heading, float speed) throws Exception {
        if (dTime == null) {
            MethodHandles.Lookup l = MethodHandles.privateLookupIn(VehicleInterpolationData.class, MethodHandles.lookup());
            Class<?> c = VehicleInterpolationData.class;
            dTime = l.findVarHandle(c, "time", long.class);
            dX = l.findVarHandle(c, "x", float.class);
            dY = l.findVarHandle(c, "y", float.class);
            dZ = l.findVarHandle(c, "z", float.class);
            dQx = l.findVarHandle(c, "qx", float.class);
            dQy = l.findVarHandle(c, "qy", float.class);
            dQz = l.findVarHandle(c, "qz", float.class);
            dQw = l.findVarHandle(c, "qw", float.class);
            dVx = l.findVarHandle(c, "vx", float.class);
            dVy = l.findVarHandle(c, "vy", float.class);
            dVz = l.findVarHandle(c, "vz", float.class);
            dWc = l.findVarHandle(c, "wheelsCount", short.class);
        }
        VehicleInterpolationData d = new VehicleInterpolationData();
        dTime.set(d, time);
        dX.set(d, x);
        dY.set(d, y);
        dZ.set(d, 0f);
        // yaw about Bullet's up axis (y)
        dQx.set(d, 0f);
        dQy.set(d, (float) Math.sin(heading / 2));
        dQz.set(d, 0f);
        dQw.set(d, (float) Math.cos(heading / 2));
        dVx.set(d, (float) (speed * Math.cos(heading)));
        dVy.set(d, 0f);
        dVz.set(d, (float) (speed * Math.sin(heading)));
        dWc.set(d, (short) 4);
        return d;
    }

    static VehicleInterpolation newInterpolation() throws Exception {
        Constructor<VehicleInterpolation> c = VehicleInterpolation.class.getDeclaredConstructor();
        c.setAccessible(true);
        return c.newInstance();
    }

    static BaseVehicle fakeVehicle(short id) throws Exception {
        Field uf = Class.forName("sun.misc.Unsafe").getDeclaredField("theUnsafe");
        uf.setAccessible(true);
        Object unsafe = uf.get(null);
        BaseVehicle v = (BaseVehicle) unsafe.getClass().getMethod("allocateInstance", Class.class).invoke(unsafe, BaseVehicle.class);
        setField(v, "jniTransform", new zombie.core.physics.Transform());
        setField(v, "jniLinearVelocity", new org.joml.Vector3f());
        BaseVehicle.WheelInfo[] wi = new BaseVehicle.WheelInfo[4];
        Constructor<BaseVehicle.WheelInfo> wc = BaseVehicle.WheelInfo.class.getDeclaredConstructor();
        wc.setAccessible(true);
        for (int i = 0; i < 4; i++) wi[i] = wc.newInstance();
        setField(v, "wheelInfo", wi);
        setField(v, "parts", new zombie.vehicles.VehicleParts());
        v.vehicleId = id;
        v.netPlayerAuthorization = BaseVehicle.Authorization.Remote;
        return v;
    }

    static void setPose(BaseVehicle v, double tMs) {
        v.jniTransform.origin.set(10000f, 0f, 10000f);
        v.jniTransform.setRotation(new Quaternionf());
    }

    static void setField(Object o, String name, Object value) throws Exception {
        Field f = BaseVehicle.class.getDeclaredField(name);
        f.setAccessible(true);
        f.set(o, value);
    }

    static int getStaticInt(Class<?> c, String name) throws Exception {
        Field f = c.getDeclaredField(name);
        f.setAccessible(true);
        return f.getInt(null);
    }

    static double now() {
        return GameTime.getServerTime() / 1.0e6;
    }

    static double pct(List<Double> v, double p) {
        if (v.isEmpty()) return Double.NaN;
        double[] a = v.stream().mapToDouble(Double::doubleValue).toArray();
        Arrays.sort(a);
        return a[(int) Math.round(p * (a.length - 1))];
    }

    static void expect(boolean ok, String what) {
        System.out.println((ok ? "  ok   " : "  FAIL ") + what);
        if (!ok) failures++;
    }
}
