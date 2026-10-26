package com.druid.smoothride;

/**
 * One network snapshot of a vehicle, in the same layout the game uses.
 * Position is world space (x, y, height); velocity is Bullet space (x, up, z),
 * so world-space velocity is (vx, vz, vy).
 */
final class Sample {
    static final int MAX_WHEELS = 8;

    long t;          // sender's simulation time, server clock, ms
    double arrive;   // local server-clock ms when the sample arrived
    float x, y, z;
    float qx, qy, qz, qw;
    float vx, vy, vz;
    float eng, thr;
    int wc;
    final float[] ws = new float[MAX_WHEELS];
    final float[] wr = new float[MAX_WHEELS];
    final float[] wk = new float[MAX_WHEELS];
    final float[] wl = new float[MAX_WHEELS];
    boolean synthetic; // seeded from the vehicle's current pose, not from the network

    void copyFrom(Sample o) {
        t = o.t; arrive = o.arrive;
        x = o.x; y = o.y; z = o.z;
        qx = o.qx; qy = o.qy; qz = o.qz; qw = o.qw;
        vx = o.vx; vy = o.vy; vz = o.vz;
        eng = o.eng; thr = o.thr;
        wc = o.wc;
        for (int i = 0; i < MAX_WHEELS; i++) {
            ws[i] = o.ws[i]; wr[i] = o.wr[i]; wk[i] = o.wk[i]; wl[i] = o.wl[i];
        }
        synthetic = o.synthetic;
    }

    boolean isFinite() {
        return f(x) && f(y) && f(z) && f(qx) && f(qy) && f(qz) && f(qw) && f(vx) && f(vy) && f(vz)
                && (qx * qx + qy * qy + qz * qz + qw * qw) > 1.0e-6f;
    }

    private static boolean f(float v) {
        return !Float.isNaN(v) && !Float.isInfinite(v);
    }
}
