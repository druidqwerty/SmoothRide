package com.druid.smoothride;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Properties;

/** Tunables. Pure Java so the offline simulator can use it too. */
final class Config {
    // master switches
    static volatile boolean enabled = true;        // remote vehicles use the SmoothRide buffer
    static volatile boolean sendBoost = true;      // driver sends physics more often

    // driver side
    static volatile int sendIntervalMs = 33;       // vanilla: 150 ms while moving

    // receiver side: buffer delay = P(latency) + k * sendInterval + margin
    static volatile String mode = "balanced";      // smooth | balanced | realtime
    static volatile double minDelayMs = 20.0;
    static volatile double maxDelayMs = 700.0;
    static volatile double initialDelayMs = 120.0;
    static volatile double delayGrowRate = 0.15;   // max ms of delay gained per ms (remote car runs 15% slow)
    static volatile double delayShrinkRate = 0.06; // max ms of delay lost per ms (remote car runs 6% fast)
    static volatile double delayRateAccel = 0.3;   // how fast that rate itself may change, per second

    static volatile double maxExtrapolationMs = 250.0;
    static volatile double idleTimeoutMs = 1500.0; // no packets for this long -> hand back to vanilla
    static volatile double correctionTimeMs = 120.0;
    static volatile double jumpThreshold = 0.06;   // m; smaller differences are treated as motion
    static volatile double rotJumpDeg = 1.5;
    static volatile double snapDistance = 8.0;     // m; larger errors snap instead of blending
    static volatile double teleportDistance = 30.0;
    static volatile double hermiteMaxDeviation = 0.6;

    static volatile boolean logStats = true;
    static volatile int statsIntervalSec = 30;
    static volatile boolean trace = false;         // per-frame CSV in <Zomboid>/Logs, for diagnosing a session

    private Config() {
    }

    static double pct() {
        switch (mode) {
            case "smooth": return 0.95;
            case "realtime": return 0.50;
            default: return 0.85;
        }
    }

    static double kInterval() {
        switch (mode) {
            case "smooth": return 1.25;
            case "realtime": return 0.0;
            default: return 0.6;
        }
    }

    static double marginMs() {
        switch (mode) {
            case "smooth": return 15.0;
            case "realtime": return 0.0;
            default: return 10.0;
        }
    }

    static String describe() {
        return "enabled=" + enabled + " sendBoost=" + sendBoost + " sendIntervalMs=" + sendIntervalMs
                + " mode=" + mode + " minDelayMs=" + minDelayMs + " maxDelayMs=" + maxDelayMs
                + " maxExtrapolationMs=" + maxExtrapolationMs + " correctionTimeMs=" + correctionTimeMs + " trace=" + trace;
    }

    static boolean set(String key, String value) {
        try {
            switch (key) {
                case "enabled": enabled = Boolean.parseBoolean(value); return true;
                case "sendBoost": sendBoost = Boolean.parseBoolean(value); return true;
                case "sendIntervalMs": sendIntervalMs = clampI(Integer.parseInt(value.trim()), 16, 150); return true;
                case "mode":
                    String m = value.trim().toLowerCase();
                    if (!m.equals("smooth") && !m.equals("balanced") && !m.equals("realtime")) return false;
                    mode = m;
                    return true;
                case "minDelayMs": minDelayMs = clampD(Double.parseDouble(value), 0, 500); return true;
                case "maxDelayMs": maxDelayMs = clampD(Double.parseDouble(value), 100, 2000); return true;
                case "initialDelayMs": initialDelayMs = clampD(Double.parseDouble(value), 0, 1000); return true;
                case "maxExtrapolationMs": maxExtrapolationMs = clampD(Double.parseDouble(value), 0, 600); return true;
                case "idleTimeoutMs": idleTimeoutMs = clampD(Double.parseDouble(value), 300, 10000); return true;
                case "correctionTimeMs": correctionTimeMs = clampD(Double.parseDouble(value), 10, 1000); return true;
                case "snapDistance": snapDistance = clampD(Double.parseDouble(value), 1, 50); return true;
                case "logStats": logStats = Boolean.parseBoolean(value); return true;
                case "statsIntervalSec": statsIntervalSec = clampI(Integer.parseInt(value.trim()), 5, 600); return true;
                case "trace": trace = Boolean.parseBoolean(value.trim()); return true;
                default: return false;
            }
        } catch (RuntimeException e) {
            return false;
        }
    }

    private static final String[] KEYS = {
        "enabled", "sendBoost", "sendIntervalMs", "mode", "minDelayMs", "maxDelayMs", "initialDelayMs",
        "maxExtrapolationMs", "idleTimeoutMs", "correctionTimeMs", "snapDistance", "logStats", "statsIntervalSec", "trace"
    };

    private static String get(String key) {
        switch (key) {
            case "enabled": return String.valueOf(enabled);
            case "sendBoost": return String.valueOf(sendBoost);
            case "sendIntervalMs": return String.valueOf(sendIntervalMs);
            case "mode": return mode;
            case "minDelayMs": return String.valueOf(minDelayMs);
            case "maxDelayMs": return String.valueOf(maxDelayMs);
            case "initialDelayMs": return String.valueOf(initialDelayMs);
            case "maxExtrapolationMs": return String.valueOf(maxExtrapolationMs);
            case "idleTimeoutMs": return String.valueOf(idleTimeoutMs);
            case "correctionTimeMs": return String.valueOf(correctionTimeMs);
            case "snapDistance": return String.valueOf(snapDistance);
            case "logStats": return String.valueOf(logStats);
            case "statsIntervalSec": return String.valueOf(statsIntervalSec);
            case "trace": return String.valueOf(trace);
            default: return null;
        }
    }

    /** Loads the file if it exists, otherwise writes one with the defaults. Returns a status line. */
    static String loadOrCreate(File file) {
        if (file.isFile()) {
            Properties p = new Properties();
            try (InputStream in = new FileInputStream(file)) {
                p.load(new java.io.InputStreamReader(in, StandardCharsets.UTF_8));
            } catch (Exception e) {
                return "config read failed (" + e + "), using defaults";
            }
            int applied = 0;
            StringBuilder bad = new StringBuilder();
            for (String key : p.stringPropertyNames()) {
                if (set(key, p.getProperty(key))) {
                    applied++;
                } else {
                    bad.append(' ').append(key);
                }
            }
            return "config " + file + ": " + applied + " value(s)" + (bad.length() > 0 ? ", ignored:" + bad : "");
        }
        StringBuilder sb = new StringBuilder();
        sb.append("# SmoothRide MP settings. Delete this file to restore defaults.\n");
        sb.append("# mode: smooth (max smoothness, ~+40 ms), balanced (default), realtime (min delay, more prediction)\n");
        for (String key : KEYS) {
            sb.append(key).append('=').append(get(key)).append('\n');
        }
        try {
            File dir = file.getParentFile();
            if (dir != null && !dir.isDirectory()) dir.mkdirs();
            try (OutputStream out = new FileOutputStream(file)) {
                out.write(sb.toString().getBytes(StandardCharsets.UTF_8));
            }
            return "config created with defaults: " + file;
        } catch (Exception e) {
            return "config not written (" + e + "), using defaults";
        }
    }

    private static int clampI(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double clampD(double v, double lo, double hi) {
        if (Double.isNaN(v)) return lo;
        return Math.max(lo, Math.min(hi, v));
    }
}
