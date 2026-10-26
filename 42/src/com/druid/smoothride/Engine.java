package com.druid.smoothride;

import java.io.File;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Locale;
import java.util.Map;
import java.util.WeakHashMap;
import org.joml.Quaternionf;
import zombie.GameTime;
import zombie.core.physics.WorldSimulation;
import zombie.core.utils.UpdateLimit;
import zombie.debug.DebugLog;
import zombie.network.GameClient;
import zombie.vehicles.BaseVehicle;
import zombie.vehicles.VehicleInterpolation;
import zombie.vehicles.VehicleInterpolationData;
import zombie.vehicles.VehicleManager;

/** Glue between the game's vehicle networking and {@link Track}. Everything runs on the main thread. */
public final class Engine {
    static final String TAG = "[SmoothRide] ";

    private static final Map<Object, Track> tracks = new WeakHashMap<>();

    // reflection, resolved lazily on the first in-game frame
    private static boolean reflectTried;
    private static boolean reflectOk;
    private static VarHandle dTime, dX, dY, dZ, dQx, dQy, dQz, dQw, dVx, dVy, dVz, dEng, dThr, dWc, dWs, dWr, dWk, dWl;
    private static VarHandle vNetworkUpdated, vLimitPhysicSend;

    // self test of the interpolationDataGet override (enter skip + exit return value)
    private static boolean selfTestDone;
    private static boolean getOverrideOk;
    private static boolean selfTesting;

    // enter -> exit hand-off for interpolationDataGet
    private static boolean pendingHandled;
    private static boolean pendingResult;

    private static int serverUpdateDepth;
    private static boolean failed;

    private static double frameNow = Double.NaN;
    private static long frameNanos;
    private static long nextStatsMs;
    private static int sentPackets;
    private static int boostedVehicles;
    private static int filteredServerSamples;

    private static final Quaternionf tmpQ = new Quaternionf();

    private Engine() {
    }

    // ---------------------------------------------------------------- lifecycle

    static void init() {
        String status;
        try {
            status = Config.loadOrCreate(configFile());
        } catch (Throwable t) {
            status = "config error: " + t;
        }
        log("loaded. " + status);
        log("settings: " + Config.describe());
    }

    static File configFile() {
        String dir = null;
        try {
            dir = zombie.ZomboidFileSystem.instance.getCacheDir();
        } catch (Throwable ignored) {
        }
        if (dir == null || dir.isEmpty()) {
            dir = System.getProperty("user.home") + File.separator + "Zomboid";
        }
        return new File(dir, "SmoothRide.properties");
    }

    static void log(String msg) {
        try {
            DebugLog.log(TAG + msg);
        } catch (Throwable t) {
            System.out.println(TAG + msg);
        }
    }

    private static void fail(String where, Throwable t) {
        if (failed) return;
        failed = true;
        log("disabled after an error in " + where + "; vehicles use vanilla sync until restart: " + t);
        for (StackTraceElement e : t.getStackTrace()) {
            log("    at " + e);
        }
        tracks.clear();
    }

    private static boolean ensureReflection() {
        if (reflectTried) return reflectOk;
        reflectTried = true;
        try {
            MethodHandles.Lookup d = MethodHandles.privateLookupIn(VehicleInterpolationData.class, MethodHandles.lookup());
            Class<?> c = VehicleInterpolationData.class;
            dTime = d.findVarHandle(c, "time", long.class);
            dX = d.findVarHandle(c, "x", float.class);
            dY = d.findVarHandle(c, "y", float.class);
            dZ = d.findVarHandle(c, "z", float.class);
            dQx = d.findVarHandle(c, "qx", float.class);
            dQy = d.findVarHandle(c, "qy", float.class);
            dQz = d.findVarHandle(c, "qz", float.class);
            dQw = d.findVarHandle(c, "qw", float.class);
            dVx = d.findVarHandle(c, "vx", float.class);
            dVy = d.findVarHandle(c, "vy", float.class);
            dVz = d.findVarHandle(c, "vz", float.class);
            dEng = d.findVarHandle(c, "engineSpeed", float.class);
            dThr = d.findVarHandle(c, "throttle", float.class);
            dWc = d.findVarHandle(c, "wheelsCount", short.class);
            dWs = d.findVarHandle(c, "wheelSteering", float[].class);
            dWr = d.findVarHandle(c, "wheelRotation", float[].class);
            dWk = d.findVarHandle(c, "wheelSkidInfo", float[].class);
            dWl = d.findVarHandle(c, "wheelSuspensionLength", float[].class);
            MethodHandles.Lookup v = MethodHandles.privateLookupIn(BaseVehicle.class, MethodHandles.lookup());
            vNetworkUpdated = v.findVarHandle(BaseVehicle.class, "networkUpdated", boolean.class);
            vLimitPhysicSend = v.findVarHandle(BaseVehicle.class, "limitPhysicSend", UpdateLimit.class);
            reflectOk = true;
        } catch (Throwable t) {
            reflectOk = false;
            log("this game build does not match (" + t + "); SmoothRide stays off, vanilla sync is used.");
        }
        return reflectOk;
    }

    /** Runs the interpolationDataGet override once against a private VehicleInterpolation. */
    private static void runSelfTest() {
        selfTestDone = true;
        getOverrideOk = false;
        Object vi = null;
        try {
            Constructor<VehicleInterpolation> ctor = VehicleInterpolation.class.getDeclaredConstructor();
            ctor.setAccessible(true);
            vi = ctor.newInstance();
            Track t = new Track();
            double now = 1.0e6;
            for (int i = 0; i < 2; i++) {
                Sample smp = t.obtain();
                smp.t = (long) (now - 400 + i * 800);
                smp.x = 4321.5f;
                smp.y = 1234.25f;
                smp.qw = 1f;
                smp.wc = 4;
                t.add(smp, now);
            }
            t.delay = 0.0;
            t.target = 0.0;
            tracks.put(vi, t);
            float[] b1 = new float[27];
            float[] b2 = new float[2];
            Method get = VehicleInterpolation.class.getMethod("interpolationDataGet", float[].class, float[].class, VehicleInterpolation.class);
            selfTesting = true;
            frameNow = now;
            frameNanos = System.nanoTime();
            Object ret;
            try {
                ret = get.invoke(vi, b1, b2, null);
            } finally {
                selfTesting = false;
                frameNow = Double.NaN;
            }
            boolean okRet = Boolean.TRUE.equals(ret);
            boolean okPose = Math.abs(b1[0] - 4321.5f) < 0.01f && Math.abs(b1[1] - 1234.25f) < 0.01f;
            getOverrideOk = okRet && okPose;
            log("self-test: interpolation override " + (getOverrideOk ? "OK" : "FAILED")
                    + " (return=" + ret + ", x=" + b1[0] + ", y=" + b1[1] + ")"
                    + (getOverrideOk ? "" : "; remote vehicles stay on vanilla interpolation"));
        } catch (Throwable t) {
            getOverrideOk = false;
            log("self-test error, remote vehicles stay on vanilla interpolation: " + t);
        } finally {
            if (vi != null) tracks.remove(vi);
        }
    }

    static boolean receiverActive() {
        return Config.enabled && !failed && reflectOk && getOverrideOk && GameClient.client
                && GameTime.getServerTimeShiftIsSet();
    }

    static double serverNowMs() {
        return GameTime.getServerTime() / 1.0e6;
    }

    // ---------------------------------------------------------------- hooks (called from Patches)

    /** VehicleManager.clientUpdate entry: once per frame in a multiplayer client. */
    public static void frame() {
        if (failed) return;
        try {
            if (!ensureReflection()) return;
            if (!selfTestDone) runSelfTest();
            frameNanos = System.nanoTime();
            frameNow = GameTime.getServerTimeShiftIsSet() ? serverNowMs() : Double.NaN;
            if (Config.logStats) {
                long ms = System.currentTimeMillis();
                if (nextStatsMs == 0L) {
                    nextStatsMs = ms + Config.statsIntervalSec * 1000L;
                } else if (ms >= nextStatsMs) {
                    nextStatsMs = ms + Config.statsIntervalSec * 1000L;
                    reportStats();
                }
            }
        } catch (Throwable t) {
            fail("frame", t);
        }
    }

    private static double now() {
        if (Double.isNaN(frameNow) || System.nanoTime() - frameNanos > 50_000_000L) {
            return serverNowMs();
        }
        return frameNow;
    }

    /** VehicleInterpolation.interpolationDataAdd entry. Returns true to skip the vanilla add. */
    public static boolean onAdd(Object interp, BaseVehicle vehicle, Object data) {
        boolean active = receiverActive();
        if ((!active && !(Config.trace && reflectOk)) || vehicle == null || !(data instanceof VehicleInterpolationData)) return false;
        try {
            double now = serverNowMs();
            Track tr = active ? tracks.get(interp) : null;
            if (active && tr == null) {
                tr = new Track();
                tracks.put(interp, tr);
            }
            boolean echo = serverUpdateDepth > 0 && isRemoteDriven(vehicle) && tr != null && !Double.isNaN(tr.lastArrive)
                    && now - tr.lastArrive < 1000.0;
            if (Trace.on()) {
                VehicleInterpolationData d = (VehicleInterpolationData) data;
                Trace.end(Trace.begin('P', now).append(',').append(vehicle.vehicleId).append(',').append((long) dTime.get(d))
                        .append(',').append((float) dX.get(d)).append(',').append((float) dY.get(d))
                        .append(',').append(echo ? 'e' : (serverUpdateDepth > 0 ? 'u' : 'p')));
            }
            if (!active) return false;
            tr.vehicleId = vehicle.vehicleId;
            if (echo) {
                // a server-side position echo stamped with the server's clock: it would sit after the
                // driver's own newest snapshot with an older position and make the car stutter
                filteredServerSamples++;
                return true;
            }
            Sample smp = tr.obtain();
            read((VehicleInterpolationData) data, smp);
            if (tr.beginAdd(smp)) {
                Sample seed = tr.obtain();
                readCurrent(vehicle, seed);
                seed.synthetic = true;
                seed.t = (long) Math.floor(tr.peekRenderTime(now));
                double dx = seed.x - smp.x, dy = seed.y - smp.y;
                if (seed.t < smp.t && seed.isFinite() && dx * dx + dy * dy < Config.teleportDistance * Config.teleportDistance) {
                    tr.add(seed, now);
                } else {
                    tr.recycle(seed);
                }
            }
            tr.add(smp, now);
        } catch (Throwable t) {
            fail("interpolationDataAdd", t);
        }
        return false;
    }

    /** VehicleInterpolation.interpolationDataGet(float[], float[], VehicleInterpolation) entry. */
    public static boolean onGetEnter(Object interp, Object b1o, Object b2o, Object towedBy) {
        pendingHandled = false;
        if (!(selfTesting || receiverActive())) return false;
        try {
            if (!(b1o instanceof float[]) || !(b2o instanceof float[])) return false;
            float[] b1 = (float[]) b1o;
            float[] b2 = (float[]) b2o;
            if (b1.length < 11) return false;
            double now = now();
            if (Double.isNaN(now)) return false;
            Track tr = tracks.get(interp);
            boolean res = false;
            if (tr != null) {
                double towRender = Double.NaN;
                if (towedBy != null) {
                    Track tow = tracks.get(towedBy);
                    if (tow != null && tow.lastNow == now && !Double.isNaN(tow.renderTime)) {
                        towRender = tow.renderTime;
                    }
                }
                res = tr.get(now, towRender, b1, b2) == Track.OK;
                tr.lastGetNow = now;
                tr.lastGetOk = res;
            }
            pendingHandled = true;
            pendingResult = res;
            return true;
        } catch (Throwable t) {
            fail("interpolationDataGet", t);
            pendingHandled = false;
            return false;
        }
    }

    /** interpolationDataGet exit: true when SmoothRide answered and the return value must be replaced. */
    public static boolean consumeHandled() {
        boolean h = pendingHandled;
        pendingHandled = false;
        return h;
    }

    public static boolean pendingResult() {
        return pendingResult;
    }

    /** VehicleInterpolation.clear entry (reset / clearAll / vehicle removal). */
    public static void onClear(Object interp) {
        if (tracks.isEmpty()) return;
        Track tr = tracks.get(interp);
        if (tr != null) tr.clear();
    }

    /** BaseVehicle.updatePhysicsNetwork exit, on the client that drives the vehicle. */
    public static void afterPhysicsSend(BaseVehicle v) {
        if (failed || !reflectOk || v == null) return;
        boolean boost = Config.enabled && Config.sendBoost;
        if (!boost && !Config.trace) return;
        try {
            if (!(boolean) vNetworkUpdated.get(v)) return;
            sentPackets++;
            UpdateLimit limit = (UpdateLimit) vLimitPhysicSend.get(v);
            // vanilla sets 150 ms after a send while the car moves and eases toward 300 ms when parked;
            // only the moving case is shortened
            if (boost && limit != null && limit.getDelay() == 150L) {
                limit.setUpdatePeriod(Config.sendIntervalMs);
                boostedVehicles++;
            }
            if (Trace.on()) {
                Trace.end(Trace.begin('S', serverNowMs()).append(',').append(v.vehicleId).append(',')
                        .append(WorldSimulation.instance.time).append(',').append(v.getX()).append(',').append(v.getY())
                        .append(',').append(limit != null ? limit.getDelay() : -1L));
            }
        } catch (Throwable t) {
            fail("updatePhysicsNetwork", t);
        }
    }

    /** VehicleManager.clientUpdate exit: every vehicle has its pose for this frame. Trace only. */
    public static void frameEnd() {
        if (!Trace.on() || failed) return;
        try {
            double now = Double.isNaN(frameNow) ? serverNowMs() : frameNow;
            ArrayList<BaseVehicle> list = VehicleManager.instance.getVehicles();
            for (int i = 0; i < list.size(); i++) {
                BaseVehicle v = list.get(i);
                if (v == null) continue;
                BaseVehicle.Authorization a = v.netPlayerAuthorization;
                char auth;
                if (a == BaseVehicle.Authorization.Local || a == BaseVehicle.Authorization.LocalCollide) {
                    auth = 'L';
                } else if (a == BaseVehicle.Authorization.Remote || a == BaseVehicle.Authorization.RemoteCollide) {
                    auth = 'R';
                } else {
                    continue; // parked / server-simulated
                }
                String mode = "-";
                if (auth == 'R') {
                    Track tr = tracks.get(v.interpolation);
                    mode = tr != null && tr.lastGetNow == now && tr.lastGetOk ? "s" + modeName(tr.lastMode).charAt(0) : "v";
                }
                Trace.end(Trace.begin('F', now).append(',').append(v.vehicleId).append(',').append(auth).append(',')
                        .append(v.getX()).append(',').append(v.getY()).append(',').append(v.jniTransform.origin.y)
                        .append(',').append(mode));
            }
        } catch (Throwable t) {
            log("trace error: " + t);
            Config.trace = false;
        }
    }

    public static void serverUpdateEnter() {
        serverUpdateDepth++;
    }

    public static void serverUpdateExit() {
        if (serverUpdateDepth > 0) serverUpdateDepth--;
    }

    // ---------------------------------------------------------------- data access

    private static boolean isRemoteDriven(BaseVehicle v) {
        BaseVehicle.Authorization a = v.netPlayerAuthorization;
        return a == BaseVehicle.Authorization.Remote || a == BaseVehicle.Authorization.RemoteCollide;
    }

    private static void read(VehicleInterpolationData d, Sample s) {
        s.synthetic = false;
        s.t = (long) dTime.get(d);
        s.x = (float) dX.get(d);
        s.y = (float) dY.get(d);
        s.z = (float) dZ.get(d);
        s.qx = (float) dQx.get(d);
        s.qy = (float) dQy.get(d);
        s.qz = (float) dQz.get(d);
        s.qw = (float) dQw.get(d);
        s.vx = (float) dVx.get(d);
        s.vy = (float) dVy.get(d);
        s.vz = (float) dVz.get(d);
        s.eng = (float) dEng.get(d);
        s.thr = (float) dThr.get(d);
        int wc = (short) dWc.get(d);
        float[] ws = (float[]) dWs.get(d);
        float[] wr = (float[]) dWr.get(d);
        float[] wk = (float[]) dWk.get(d);
        float[] wl = (float[]) dWl.get(d);
        wc = Math.max(0, Math.min(wc, Sample.MAX_WHEELS));
        if (ws != null) wc = Math.min(wc, ws.length);
        if (wr != null) wc = Math.min(wc, wr.length);
        if (wk != null) wc = Math.min(wc, wk.length);
        if (wl != null) wc = Math.min(wc, wl.length);
        s.wc = wc;
        for (int i = 0; i < wc; i++) {
            s.ws[i] = ws != null ? ws[i] : 0f;
            s.wr[i] = wr != null ? wr[i] : 0f;
            s.wk[i] = wk != null ? wk[i] : 0f;
            s.wl[i] = wl != null ? wl[i] : 0.3f;
        }
    }

    /** The pose the game is drawing right now, used to start a new buffer without a pop. */
    private static void readCurrent(BaseVehicle v, Sample s) {
        WorldSimulation ws = WorldSimulation.instance;
        s.x = v.jniTransform.origin.x + ws.offsetX;
        s.y = v.jniTransform.origin.z + ws.offsetY;
        s.z = v.jniTransform.origin.y;
        Quaternionf q = v.jniTransform.getRotation(tmpQ);
        q.normalize();
        s.qx = q.x;
        s.qy = q.y;
        s.qz = q.z;
        s.qw = q.w;
        s.vx = v.jniLinearVelocity.x;
        s.vy = v.jniLinearVelocity.y;
        s.vz = v.jniLinearVelocity.z;
        s.eng = (float) v.getEngineSpeed();
        s.thr = v.throttle;
        int wc = v.wheelInfo == null ? 0 : Math.min(v.wheelInfo.length, Sample.MAX_WHEELS);
        s.wc = wc;
        for (int i = 0; i < wc; i++) {
            BaseVehicle.WheelInfo w = v.wheelInfo[i];
            if (w == null) {
                s.wc = i;
                break;
            }
            s.ws[i] = w.steering;
            s.wr[i] = w.rotation;
            s.wk[i] = w.skidInfo;
            s.wl[i] = w.suspensionLength > 0f ? w.suspensionLength : 0.3f;
        }
    }

    // ---------------------------------------------------------------- reporting

    static String status() {
        StringBuilder sb = new StringBuilder();
        sb.append("SmoothRide: ").append(failed ? "FAILED (vanilla)" : (receiverActive() ? "active" : "idle"))
                .append(", interpolation override ").append(selfTestDone ? (getOverrideOk ? "OK" : "FAILED") : "not tested yet")
                .append('\n');
        sb.append(Config.describe()).append('\n');
        ArrayList<Track> list = new ArrayList<>(tracks.values());
        for (Track t : list) {
            if (t.n == 0) continue;
            sb.append(String.format(Locale.ROOT, "vid %d: delay %.0f ms (target %.0f), latency p50/p90 %.0f/%.0f ms, send gap %.0f ms, %s%n",
                    (int) t.vehicleId, t.delay, t.target, t.latencyP(0.5), t.latencyP(0.9), t.sendGap(),
                    t.idle ? "idle" : modeName(t.lastMode)));
        }
        return sb.toString();
    }

    private static String modeName(int m) {
        switch (m) {
            case Track.M_INTERP: return "interpolating";
            case Track.M_EXTRAP: return "predicting";
            case Track.M_HOLD: return "holding";
            case Track.M_FIRST: return "starting";
            default: return "-";
        }
    }

    private static void reportStats() {
        long ms = System.currentTimeMillis();
        if (sentPackets > 0) {
            log(String.format(Locale.ROOT, "driver: %d physics packets in %ds (%.1f/s), %d boosted", sentPackets,
                    Config.statsIntervalSec, sentPackets / (double) Config.statsIntervalSec, boostedVehicles));
        }
        sentPackets = 0;
        boostedVehicles = 0;
        if (filteredServerSamples > 0) {
            log("ignored " + filteredServerSamples + " server position echoes");
            filteredServerSamples = 0;
        }
        for (Track t : new ArrayList<>(tracks.values())) {
            if (t.stFrames == 0 && t.stSamples == 0) continue;
            int f = Math.max(1, t.stFrames);
            log(String.format(Locale.ROOT,
                    "vid %d: %d samples, gap %.0f ms, latency p50/p90 %.0f/%.0f ms, delay avg %.0f max %.0f ms, "
                            + "frames %d (interp %.1f%%, predict %.1f%%, hold %.1f%%), corrections %d (max %.2f m), snaps %d, teleports %d, idle %d",
                    (int) t.vehicleId, t.stSamples, t.sendGap(), t.latencyP(0.5), t.latencyP(0.9),
                    t.stSumDelay / f, t.stMaxDelay, t.stFrames,
                    100.0 * t.stInterp / f, 100.0 * t.stExtrap / f, 100.0 * t.stHold / f,
                    t.stCorr, t.stMaxCorr, t.stSnap, t.stTeleport, t.stIdle));
            t.resetStats(ms);
        }
    }
}
