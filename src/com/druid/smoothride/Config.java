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

    // no new packets (a server hitch: world save, chunk loading): a moving car keeps going on
    // prediction this long, then coasts to a stop (1.0: maxExtrapolationMs=250, then it froze)
    static volatile double maxPredictMs = 2500.0;
    static volatile double idleTimeoutMs = 1500.0; // no packets for this long (and prediction over) -> vanilla
    static volatile double correctionTimeMs = 120.0;
    static volatile double jumpThreshold = 0.06;   // m; smaller differences are treated as motion
    static volatile double rotJumpDeg = 1.5;
    static volatile double snapDistance = 50.0;    // m; larger errors snap instead of blending (1.0: 8)
    static volatile double blendSpeedFrac = 0.35;  // a big correction moves at most this share of the car's speed...
    static volatile double blendSpeedMin = 2.0;    // ...plus this many m/s
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
                + " maxPredictMs=" + maxPredictMs + " snapDistance=" + snapDistance + " correctionTimeMs=" + correctionTimeMs
                + " trace=" + trace;
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
                case "maxPredictMs":
                case "maxExtrapolationMs": // 1.0 name
                    maxPredictMs = clampD(Double.parseDouble(value), 250, 5000); return true;
                case "idleTimeoutMs": idleTimeoutMs = clampD(Double.parseDouble(value), 300, 10000); return true;
                case "correctionTimeMs": correctionTimeMs = clampD(Double.parseDouble(value), 10, 1000); return true;
                case "snapDistance": snapDistance = clampD(Double.parseDouble(value), 2, 100); return true;
                case "blendSpeedFrac": blendSpeedFrac = clampD(Double.parseDouble(value), 0.05, 2); return true;
                case "blendSpeedMin": blendSpeedMin = clampD(Double.parseDouble(value), 0.2, 20); return true;
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
        "maxPredictMs", "idleTimeoutMs", "correctionTimeMs", "snapDistance", "blendSpeedFrac", "blendSpeedMin",
        "logStats", "statsIntervalSec", "trace"
    };

    private static final java.util.Map<String, String> startup = new java.util.HashMap<>();

    /** Remembers the values from the file / built-in defaults (Lua reads them as "startup.<key>"). */
    static void snapshotStartup() {
        for (String key : KEYS) startup.put(key, get(key));
    }

    static String get(String key) {
        if (key.startsWith("startup.")) return startup.get(key.substring(8));
        switch (key) {
            case "enabled": return String.valueOf(enabled);
            case "sendBoost": return String.valueOf(sendBoost);
            case "sendIntervalMs": return String.valueOf(sendIntervalMs);
            case "mode": return mode;
            case "minDelayMs": return String.valueOf(minDelayMs);
            case "maxDelayMs": return String.valueOf(maxDelayMs);
            case "initialDelayMs": return String.valueOf(initialDelayMs);
            case "maxPredictMs":
            case "maxExtrapolationMs": return String.valueOf(maxPredictMs);
            case "idleTimeoutMs": return String.valueOf(idleTimeoutMs);
            case "correctionTimeMs": return String.valueOf(correctionTimeMs);
            case "snapDistance": return String.valueOf(snapDistance);
            case "blendSpeedFrac": return String.valueOf(blendSpeedFrac);
            case "blendSpeedMin": return String.valueOf(blendSpeedMin);
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
            // 1.0 wrote every key with its default; values still at a 1.0 default whose default changed
            // in 1.1 are dropped, so the file does not pin the old behaviour (customised values stay)
            boolean v1 = !p.containsKey("configVersion");
            int upgraded = 0;
            for (String key : p.stringPropertyNames()) {
                if (key.equals("configVersion")) continue;
                if (v1 && isOldDefault(key, p.getProperty(key))) {
                    upgraded++;
                    continue;
                }
                if (set(key, p.getProperty(key))) {
                    applied++;
                } else {
                    bad.append(' ').append(key);
                }
            }
            String status = "config " + file + ": " + applied + " value(s)" + (bad.length() > 0 ? ", ignored:" + bad : "");
            if (v1) status += "; " + write(file, "upgraded from 1.0 (" + upgraded + " old default(s) replaced)");
            return status;
        }
        return write(file, "created with defaults");
    }

    private static boolean isOldDefault(String key, String value) {
        try {
            double v = Double.parseDouble(value.trim());
            switch (key) {
                case "maxExtrapolationMs": return v == 250.0;
                case "snapDistance": return v == 8.0;
                default: return false;
            }
        } catch (RuntimeException e) {
            return false;
        }
    }

    private static String write(File file, String what) {
        StringBuilder sb = new StringBuilder();
        sb.append("# SmoothRide MP settings. Delete this file to restore defaults. Read once, when the game starts.\n");
        sb.append("# Normally nothing here needs editing: on a server, enabled, mode, sendIntervalMs and maxPredictMs come from the\n");
        sb.append("# host's sandbox options (page SmoothRide, an admin can change them in a running game) and win over this file;\n");
        sb.append("# logStats and statsIntervalSec are in the game's Options > Mods and win over this file. The rest is for tuning.\n");
        sb.append("# mode: balanced (default: remote cars ~80 ms behind), smooth (~40 ms more), realtime (lowest delay, more prediction)\n");
        sb.append("# maxPredictMs: no packets (server hitch) -> a moving car keeps going this long, then coasts to a stop\n");
        sb.append("configVersion=2\n");
        for (String key : KEYS) {
            sb.append(key).append('=').append(get(key)).append('\n');
        }
        try {
            File dir = file.getParentFile();
            if (dir != null && !dir.isDirectory()) dir.mkdirs();
            try (OutputStream out = new FileOutputStream(file)) {
                out.write(sb.toString().getBytes(StandardCharsets.UTF_8));
            }
            return "config " + what + ": " + file;
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
