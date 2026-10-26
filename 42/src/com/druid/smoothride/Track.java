package com.druid.smoothride;

import java.util.ArrayDeque;
import java.util.Arrays;

/**
 * Snapshot buffer for one remote vehicle.
 *
 * The vehicle is drawn at renderTime = now - delay, where the delay follows the measured packet
 * lateness instead of the vanilla fixed 500 ms. Between snapshots the position follows a cubic
 * Hermite curve built from the snapshot velocities (C1-continuous across snapshots); past the
 * newest snapshot it is predicted from the last velocity and turn rate. Whenever new data changes
 * the curve under the car (a prediction miss, a late packet), the position and velocity jump is
 * folded into a visual offset that a critically damped spring pulls back to zero, so the car never
 * pops and its speed never steps.
 *
 * No game classes are referenced here: the offline simulator drives this class directly.
 */
final class Track {
    static final int CAP = 128;
    static final int NONE = 0;
    static final int OK = 1;

    static final int M_INTERP = 0;
    static final int M_EXTRAP = 1;
    static final int M_HOLD = 2;
    static final int M_FIRST = 3;

    final Sample[] s = new Sample[CAP];
    int n;
    private final ArrayDeque<Sample> pool = new ArrayDeque<>();

    private final double[] lat = new double[64];
    private int latN, latPos;
    private final double[] gap = new double[32];
    private int gapN, gapPos;
    private final double[] tmp = new double[64];
    private long newestT = Long.MIN_VALUE;

    double delay = -1.0;
    private double delayRate;
    double target = Config.initialDelayMs;
    double lastNow = Double.NaN;
    double renderTime = Double.NaN;
    double lastArrive = Double.NaN;
    boolean idle;
    int lastMode = -1;
    double lastGetNow = Double.NaN; // frame time of the last get() and whether it produced a pose
    boolean lastGetOk;

    // which part of the buffer produced last frame's pose
    private int mMode = -1;
    private long mA, mB;
    private boolean mLinear;

    // previous frame's raw (uncorrected) pose
    private boolean hasRaw;
    private double rawTime;
    private float rpx, rpy, rpz;
    private float rdx, rdy, rdz;
    private final float[] rq = {0f, 0f, 0f, 1f};

    // visual correction on top of the raw pose: offset + its velocity (spring state), and a rotation
    float cx, cy, cz;
    private float cvx, cvy, cvz;
    private final float[] cq = {0f, 0f, 0f, 1f};

    // stats since the last report
    int stFrames, stInterp, stExtrap, stHold, stCorr, stSnap, stSamples, stTeleport, stDropped, stIdle;
    double stMaxCorr, stSumDelay, stMaxDelay;
    long stStart = -1;

    short vehicleId = -1;

    // scratch
    private final float[] q1 = new float[4];
    private final float[] q2 = new float[4];
    private final float[] q3 = new float[4];

    Sample obtain() {
        Sample x = pool.poll();
        if (x == null) return new Sample();
        x.synthetic = false;
        return x;
    }

    void recycle(Sample x) {
        if (x != null) pool.add(x);
    }

    void clear() {
        for (int i = 0; i < n; i++) {
            pool.add(s[i]);
            s[i] = null;
        }
        n = 0;
        newestT = Long.MIN_VALUE;
        hasRaw = false;
        mMode = -1;
        zeroCorrection();
        renderTime = Double.NaN;
        idle = false;
    }

    private void zeroCorrection() {
        cx = cy = cz = 0f;
        cvx = cvy = cvz = 0f;
        cq[0] = cq[1] = cq[2] = 0f;
        cq[3] = 1f;
    }

    /**
     * Call before {@link #add} for every network sample. Drops history that no longer applies and
     * tells the caller whether to seed the buffer with the vehicle's current pose first (so the car
     * starts moving from where it is drawn now instead of popping).
     */
    boolean beginAdd(Sample in) {
        if (in.synthetic || !in.isFinite()) return false;
        if (n == 0) return true;
        Sample last = s[n - 1];
        long dt = in.t - last.t;
        double d = dist(last, in);
        if (d > Config.teleportDistance && (dt <= 0L || d * 1000.0 / dt > 70.0)) {
            clear(); // teleport: show the new place at once
            stTeleport++;
            return false;
        }
        if (dt > 1000L) {
            clear(); // the stream stopped and resumed: old snapshots describe a different ride
            return true;
        }
        return false;
    }

    /** Takes ownership of {@code in} (it must come from {@link #obtain()}). */
    boolean add(Sample in, double now) {
        if (!in.isFinite()) {
            stDropped++;
            pool.add(in);
            return false;
        }
        in.arrive = now;
        if (!in.synthetic) {
            pushLat(now - in.t);
            if (newestT != Long.MIN_VALUE && in.t > newestT && in.t - newestT < 1000L) {
                pushGap(in.t - newestT);
            }
            if (in.t > newestT) newestT = in.t;
            lastArrive = now;
            idle = false;
            stSamples++;
        }
        insertSorted(in);
        prune(now);
        updateTarget();
        return true;
    }

    private void insertSorted(Sample in) {
        int i = n - 1;
        while (i >= 0 && s[i].t > in.t) i--;
        if (i >= 0 && s[i].t == in.t) {
            Sample old = s[i];
            if (old.synthetic || !in.synthetic) {
                s[i] = in;
                pool.add(old);
            } else {
                pool.add(in);
            }
            return;
        }
        if (n == CAP) {
            if (i < 0) {
                pool.add(in); // older than everything we keep
                return;
            }
            pool.add(s[0]);
            System.arraycopy(s, 1, s, 0, n - 1);
            n--;
            i--;
        }
        int pos = i + 1;
        System.arraycopy(s, pos, s, pos + 1, n - pos);
        s[pos] = in;
        n++;
    }

    private void prune(double now) {
        double ref = Double.isNaN(renderTime) ? now - Config.maxDelayMs : renderTime;
        double cutoff = ref - 1000.0;
        int drop = 0;
        while (n - drop > 3 && s[drop + 1].t < cutoff) drop++;
        if (drop > 0) {
            for (int i = 0; i < drop; i++) pool.add(s[i]);
            System.arraycopy(s, drop, s, 0, n - drop);
            for (int i = n - drop; i < n; i++) s[i] = null;
            n -= drop;
        }
    }

    private void pushLat(double v) {
        lat[latPos] = v;
        latPos = (latPos + 1) % lat.length;
        if (latN < lat.length) latN++;
    }

    private void pushGap(double v) {
        gap[gapPos] = v;
        gapPos = (gapPos + 1) % gap.length;
        if (gapN < gap.length) gapN++;
    }

    private double percentile(double[] src, int count, double p) {
        System.arraycopy(src, 0, tmp, 0, count);
        Arrays.sort(tmp, 0, count);
        int idx = (int) Math.round(p * (count - 1));
        return tmp[Math.max(0, Math.min(count - 1, idx))];
    }

    double latencyP(double p) {
        return latN > 0 ? percentile(lat, latN, p) : Double.NaN;
    }

    double sendGap() {
        return gapN > 0 ? percentile(gap, gapN, 0.5) : Double.NaN;
    }

    void updateTarget() {
        double t;
        if (latN < 4) {
            t = Config.initialDelayMs;
        } else {
            double p = percentile(lat, latN, Config.pct());
            double i = gapN >= 4 ? percentile(gap, gapN, 0.5) : 50.0;
            t = p + Config.kInterval() * i + Config.marginMs();
        }
        target = Math.max(Config.minDelayMs, Math.min(Config.maxDelayMs, t));
    }

    /** Render time this track would use at {@code now} if nothing towed it. */
    double peekRenderTime(double now) {
        return now - (delay < 0 ? target : delay);
    }

    /**
     * Writes the pose for this frame into the game's buffers.
     *
     * @param towRender render time of the vehicle towing this one, or NaN
     * @param b1 [x, y, z, qx, qy, qz, qw, vx, vy, vz, wheelCount, (steer, rot, skid, susp) * wheels]
     * @param b2 [engineSpeed, throttle]
     */
    int get(double now, double towRender, float[] b1, float[] b2) {
        if (n == 0) return NONE;
        if (stStart < 0) stStart = (long) now;

        double dtFrame = Double.isNaN(lastNow) ? 0.0 : now - lastNow;
        if (dtFrame < 0.0) dtFrame = 0.0;
        if (dtFrame > 250.0) dtFrame = 250.0;
        lastNow = now;

        // the delay moves toward its target with a limited, smoothly changing rate:
        // the car's apparent speed changes by at most a few percent and never in a step
        if (delay < 0.0) {
            delay = target;
            delayRate = 0.0;
        } else {
            double want = (target - delay) / 700.0;
            want = Math.max(-Config.delayShrinkRate, Math.min(Config.delayGrowRate, want));
            double maxChange = Config.delayRateAccel * dtFrame / 1000.0;
            delayRate += Math.max(-maxChange, Math.min(maxChange, want - delayRate));
            delay += delayRate * dtFrame;
            if (delay < Config.minDelayMs) { delay = Config.minDelayMs; delayRate = 0.0; }
            if (delay > Config.maxDelayMs) { delay = Config.maxDelayMs; delayRate = 0.0; }
        }

        double r = Double.isNaN(towRender) ? now - delay : towRender;
        if (!Double.isNaN(renderTime) && r < renderTime && Double.isNaN(towRender) && renderTime - r < 1000.0) {
            r = renderTime; // the server clock shift can step back a little; never run time backwards
        }
        renderTime = r;

        int ia = n - 1;
        while (ia >= 0 && s[ia].t > r) ia--;

        float px, py, pz, dx, dy, dz, vx, vy, vz, eng, thr;
        int mode;
        long nA, nB = Long.MIN_VALUE;
        boolean linear = false;
        Sample a;
        Sample b = null;
        double u = 0.0;
        if (ia < 0) {
            a = s[0];
            px = a.x; py = a.y; pz = a.z;
            dx = dy = dz = 0f;
            vx = a.vx; vy = a.vy; vz = a.vz;
            q1[0] = a.qx; q1[1] = a.qy; q1[2] = a.qz; q1[3] = a.qw;
            normalize(q1);
            eng = a.eng; thr = a.thr;
            mode = M_FIRST;
            nA = a.t;
        } else if (ia < n - 1) {
            a = s[ia];
            b = s[ia + 1];
            double span = (double) (b.t - a.t);
            double h = span / 1000.0;
            u = (r - a.t) / span;
            if (u < 0.0) u = 0.0;
            if (u > 1.0) u = 1.0;
            double u2 = u * u, u3 = u2 * u;
            float avx = a.vx, avy = a.vz, avz = a.vy; // world-space velocities
            float bvx = b.vx, bvy = b.vz, bvz = b.vy;
            double lx = a.x + (b.x - a.x) * u;
            double ly = a.y + (b.y - a.y) * u;
            double lz = a.z + (b.z - a.z) * u;
            boolean sane = h > 0.0 && h < 1.0 && speed(avx, avy, avz) < 120f && speed(bvx, bvy, bvz) < 120f;
            double hx = lx, hy = ly, hz = lz, hdx = 0, hdy = 0, hdz = 0;
            if (sane) {
                double h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
                hx = h00 * a.x + h10 * h * avx + h01 * b.x + h11 * h * bvx;
                hy = h00 * a.y + h10 * h * avy + h01 * b.y + h11 * h * bvy;
                hz = h00 * a.z + h10 * h * avz + h01 * b.z + h11 * h * bvz;
                double ddx = hx - lx, ddy = hy - ly, ddz = hz - lz;
                if (Math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz) > Config.hermiteMaxDeviation) {
                    sane = false;
                } else {
                    double d00 = 6 * u2 - 6 * u, d10 = 3 * u2 - 4 * u + 1, d01 = -6 * u2 + 6 * u, d11 = 3 * u2 - 2 * u;
                    hdx = (d00 * a.x + d01 * b.x) / h + d10 * avx + d11 * bvx;
                    hdy = (d00 * a.y + d01 * b.y) / h + d10 * avy + d11 * bvy;
                    hdz = (d00 * a.z + d01 * b.z) / h + d10 * avz + d11 * bvz;
                }
            }
            if (sane) {
                px = (float) hx; py = (float) hy; pz = (float) hz;
                dx = (float) hdx; dy = (float) hdy; dz = (float) hdz;
            } else {
                linear = true;
                px = (float) lx; py = (float) ly; pz = (float) lz;
                if (h > 0.0) {
                    dx = (float) ((b.x - a.x) / h); dy = (float) ((b.y - a.y) / h); dz = (float) ((b.z - a.z) / h);
                } else {
                    dx = dy = dz = 0f;
                }
            }
            float fu = (float) u;
            vx = a.vx + (b.vx - a.vx) * fu;
            vy = a.vy + (b.vy - a.vy) * fu;
            vz = a.vz + (b.vz - a.vz) * fu;
            q1[0] = a.qx; q1[1] = a.qy; q1[2] = a.qz; q1[3] = a.qw;
            q2[0] = b.qx; q2[1] = b.qy; q2[2] = b.qz; q2[3] = b.qw;
            normalize(q1);
            normalize(q2);
            slerp(q1, q2, fu, q1);
            eng = a.eng + (b.eng - a.eng) * fu;
            thr = a.thr + (b.thr - a.thr) * fu;
            mode = M_INTERP;
            nA = a.t;
            nB = b.t;
        } else {
            a = s[ia];
            double ex = r - a.t;
            boolean hold = ex > Config.maxExtrapolationMs;
            if (hold) {
                ex = Config.maxExtrapolationMs;
                if (!Double.isNaN(lastArrive) && now - lastArrive > Config.idleTimeoutMs) {
                    // nothing new for a long time: parked, or ownership moved; vanilla takes over
                    if (!idle) stIdle++;
                    idle = true;
                    hasRaw = false;
                    mMode = -1;
                    zeroCorrection();
                    lastMode = M_HOLD;
                    return NONE;
                }
            }
            float e = (float) (ex / 1000.0);
            float wvx = a.vx, wvy = a.vz, wvz = a.vy;
            px = a.x + wvx * e;
            py = a.y + wvy * e;
            float rise = wvz * e;
            if (rise > 0.3f) rise = 0.3f;
            if (rise < -0.3f) rise = -0.3f;
            pz = a.z + rise;
            if (hold) {
                dx = dy = dz = 0f;
            } else {
                dx = wvx; dy = wvy; dz = wvz;
            }
            vx = a.vx; vy = a.vy; vz = a.vz;
            q1[0] = a.qx; q1[1] = a.qy; q1[2] = a.qz; q1[3] = a.qw;
            normalize(q1);
            if (ia >= 1) {
                Sample p = s[ia - 1];
                long pdt = a.t - p.t;
                if (pdt >= 5L && pdt <= 500L) {
                    q2[0] = p.qx; q2[1] = p.qy; q2[2] = p.qz; q2[3] = p.qw;
                    normalize(q2);
                    conj(q2, q3);
                    mul(q1, q3, q3); // rotation that took the car from p to a (world frame)
                    if (q3[3] < 0f) { q3[0] = -q3[0]; q3[1] = -q3[1]; q3[2] = -q3[2]; q3[3] = -q3[3]; }
                    float w = Math.min(1f, q3[3]);
                    float ang = 2f * (float) Math.acos(w);
                    float sinHalf = (float) Math.sqrt(Math.max(0f, 1f - w * w));
                    if (ang > 1.0e-4f && sinHalf > 1.0e-6f) {
                        float rate = ang / (pdt / 1000f);
                        float extAng = Math.min(rate * e, 0.5f);
                        float ax = q3[0] / sinHalf, ay = q3[1] / sinHalf, az = q3[2] / sinHalf;
                        float sh = (float) Math.sin(extAng * 0.5f), ch = (float) Math.cos(extAng * 0.5f);
                        q2[0] = ax * sh; q2[1] = ay * sh; q2[2] = az * sh; q2[3] = ch;
                        mul(q2, q1, q1);
                        normalize(q1);
                    }
                }
            }
            eng = a.eng;
            thr = a.thr;
            mode = hold ? M_HOLD : M_EXTRAP;
            nA = a.t;
        }

        // 1) advance the correction spring (critically damped, so it never overshoots back)
        double w0 = 2000.0 / Config.correctionTimeMs; // 1/s
        double dts = dtFrame / 1000.0;
        if (dts > 0.0) {
            double ex = Math.exp(-w0 * dts);
            float[] st = springStep(cx, cvx, w0, dts, ex); cx = st[0]; cvx = st[1];
            st = springStep(cy, cvy, w0, dts, ex); cy = st[0]; cvy = st[1];
            st = springStep(cz, cvz, w0, dts, ex); cz = st[0]; cvz = st[1];
            q2[0] = 0f; q2[1] = 0f; q2[2] = 0f; q2[3] = 1f;
            slerp(cq, q2, (float) (1.0 - Math.exp(-dts * 1000.0 / Config.correctionTimeMs)), cq);
        }

        // 2) if the curve under the car changed (not just the next snapshot), fold the jump in
        if (hasRaw) {
            double dtr = (r - rawTime) / 1000.0;
            boolean natural = (mode == mMode && nA == mA && nB == mB)
                    || (mMode == M_INTERP && (mode == M_INTERP || mode == M_EXTRAP) && nA == mB);
            boolean foldPos = !natural;
            boolean foldVel = !natural || ((linear || mLinear) && nA != mA);
            if (dtr < 0.0 || dtr > 0.5) {
                zeroCorrection();
            } else if (foldPos || foldVel) {
                if (foldPos) {
                    float ex = (float) (rpx + rdx * dtr) - px;
                    float ey = (float) (rpy + rdy * dtr) - py;
                    float ez = (float) (rpz + rdz * dtr) - pz;
                    float j = (float) Math.sqrt(ex * ex + ey * ey + ez * ez);
                    if (j > Config.snapDistance) {
                        zeroCorrection();
                        stSnap++;
                        foldVel = false;
                    } else {
                        cx += ex; cy += ey; cz += ez;
                        if (j > Config.jumpThreshold) stCorr++;
                        // rotation: cq' = cq * prevRaw * conj(raw)
                        float ang = angleBetween(rq, q1);
                        if (ang > (float) Math.toRadians(60.0)) {
                            cq[0] = cq[1] = cq[2] = 0f;
                            cq[3] = 1f;
                        } else if (ang > 1.0e-4f) {
                            conj(q1, q3);
                            mul(rq, q3, q3);
                            mul(cq, q3, cq);
                            normalize(cq);
                        }
                    }
                }
                if (foldVel) {
                    cvx += rdx - dx; cvy += rdy - dy; cvz += rdz - dz;
                }
            }
        }
        float cl = (float) Math.sqrt(cx * cx + cy * cy + cz * cz);
        if (cl > 3.0f) {
            zeroCorrection();
            stSnap++;
            cl = 0f;
        }
        float cvl = (float) Math.sqrt(cvx * cvx + cvy * cvy + cvz * cvz);
        if (cvl > 30f) {
            cvx *= 30f / cvl; cvy *= 30f / cvl; cvz *= 30f / cvl;
        }

        hasRaw = true;
        rawTime = r;
        rpx = px; rpy = py; rpz = pz;
        rdx = dx; rdy = dy; rdz = dz;
        rq[0] = q1[0]; rq[1] = q1[1]; rq[2] = q1[2]; rq[3] = q1[3];
        mMode = mode; mA = nA; mB = nB; mLinear = linear;

        // 3) output: raw + correction
        mul(cq, q1, q3);
        normalize(q3);
        int o = 0;
        b1[o++] = px + cx;
        b1[o++] = py + cy;
        b1[o++] = pz + cz;
        b1[o++] = q3[0];
        b1[o++] = q3[1];
        b1[o++] = q3[2];
        b1[o++] = q3[3];
        b1[o++] = vx;
        b1[o++] = vy;
        b1[o++] = vz;
        int maxWheels = Math.max(0, (b1.length - 11) / 4);
        int wc = a.wc;
        if (b != null) wc = Math.min(wc, b.wc);
        wc = Math.max(0, Math.min(wc, Math.min(maxWheels, Sample.MAX_WHEELS)));
        b1[o++] = wc;
        float fu = (float) u;
        for (int i = 0; i < wc; i++) {
            if (b != null) {
                b1[o++] = a.ws[i] + (b.ws[i] - a.ws[i]) * fu;
                b1[o++] = a.wr[i] + (b.wr[i] - a.wr[i]) * fu;
                b1[o++] = a.wk[i] + (b.wk[i] - a.wk[i]) * fu;
                b1[o++] = a.wl[i] + (b.wl[i] - a.wl[i]) * fu;
            } else {
                b1[o++] = a.ws[i];
                b1[o++] = a.wr[i];
                b1[o++] = a.wk[i];
                b1[o++] = a.wl[i];
            }
        }
        if (b2 != null && b2.length >= 2) {
            b2[0] = eng;
            b2[1] = thr;
        }

        stFrames++;
        if (mode == M_INTERP) stInterp++;
        else if (mode == M_EXTRAP) stExtrap++;
        else stHold++;
        stSumDelay += delay;
        if (delay > stMaxDelay) stMaxDelay = delay;
        if (cl > stMaxCorr) stMaxCorr = cl;
        lastMode = mode;
        return OK;
    }

    private final float[] springOut = new float[2];

    /** Exact step of x'' = -2w x' - w^2 x. */
    private float[] springStep(float x, float v, double w, double dt, double e) {
        double k = v + w * x;
        springOut[0] = (float) ((x + k * dt) * e);
        springOut[1] = (float) ((v - w * k * dt) * e);
        return springOut;
    }

    void resetStats(long now) {
        stFrames = stInterp = stExtrap = stHold = stCorr = stSnap = stSamples = stTeleport = stDropped = stIdle = 0;
        stMaxCorr = stSumDelay = stMaxDelay = 0.0;
        stStart = now;
    }

    // ---- small math helpers (quaternions as float[4] x, y, z, w) ----

    private static float speed(float x, float y, float z) {
        return (float) Math.sqrt(x * x + y * y + z * z);
    }

    private static double dist(Sample a, Sample b) {
        double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    static void normalize(float[] q) {
        float l = (float) Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (l < 1.0e-8f) {
            q[0] = q[1] = q[2] = 0f;
            q[3] = 1f;
            return;
        }
        q[0] /= l; q[1] /= l; q[2] /= l; q[3] /= l;
    }

    static void conj(float[] q, float[] out) {
        out[0] = -q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = q[3];
    }

    /** out = a * b (out may alias a or b). */
    static void mul(float[] a, float[] b, float[] out) {
        float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
        float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
        float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
        float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
        out[0] = x; out[1] = y; out[2] = z; out[3] = w;
    }

    static float angleBetween(float[] a, float[] b) {
        float d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
        if (d >= 1f) return 0f;
        return 2f * (float) Math.acos(d);
    }

    /** Shortest-path slerp; out may alias a or b. */
    static void slerp(float[] a, float[] b, float t, float[] out) {
        float bx = b[0], by = b[1], bz = b[2], bw = b[3];
        float d = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
        if (d < 0f) {
            d = -d; bx = -bx; by = -by; bz = -bz; bw = -bw;
        }
        float s0, s1;
        if (d > 0.9995f) {
            s0 = 1f - t;
            s1 = t;
        } else {
            float th = (float) Math.acos(d);
            float sn = (float) Math.sin(th);
            s0 = (float) Math.sin((1f - t) * th) / sn;
            s1 = (float) Math.sin(t * th) / sn;
        }
        float x = s0 * a[0] + s1 * bx;
        float y = s0 * a[1] + s1 * by;
        float z = s0 * a[2] + s1 * bz;
        float w = s0 * a[3] + s1 * bw;
        out[0] = x; out[1] = y; out[2] = z; out[3] = w;
        normalize(out);
    }
}
