package com.druid.smoothride;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Optional CSV trace (trace=true in SmoothRide.properties) for diagnosing a session:
 *   F,serverMs,vid,auth,x,y,z,mode     pose of every driven vehicle after positioning, once per frame
 *   P,serverMs,vid,stampMs,x,y,src     a snapshot entering interpolation (src p=driver packet, u=server update, e=ignored echo)
 *   S,serverMs,vid,stampMs,x,y,period  this client sent its car's physics
 *   M,serverMs,text                    settings / state changes
 */
final class Trace {
    private static BufferedWriter out;
    private static boolean broken;
    private static long lastFlushMs;
    private static final StringBuilder sb = new StringBuilder(128);

    private Trace() {
    }

    static boolean on() {
        return Config.trace && !broken;
    }

    private static boolean open() {
        if (out != null) return true;
        if (broken) return false;
        try {
            File dir = new File(Engine.configFile().getParentFile(), "Logs");
            if (!dir.isDirectory()) dir.mkdirs();
            String ts = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss").format(new Date());
            File f = new File(dir, "SmoothRide_trace_" + ts + ".csv");
            out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(f), StandardCharsets.UTF_8), 1 << 16);
            Engine.log("trace file: " + f);
            return true;
        } catch (Throwable t) {
            broken = true;
            Engine.log("trace disabled: " + t);
            return false;
        }
    }

    static StringBuilder begin(char kind, double serverMs) {
        sb.setLength(0);
        sb.append(kind).append(',').append(Math.round(serverMs * 10.0) / 10.0);
        return sb;
    }

    static void end(StringBuilder line) {
        if (!open()) return;
        try {
            out.write(line.toString());
            out.write('\n');
            long ms = System.currentTimeMillis();
            if (ms - lastFlushMs > 1000L) {
                lastFlushMs = ms;
                out.flush();
            }
        } catch (Throwable t) {
            broken = true;
            Engine.log("trace write failed, trace off: " + t);
        }
    }

    static void note(double serverMs, String text) {
        if (!on()) return;
        end(begin('M', serverMs).append(',').append(text.replace(',', ';')));
    }

    static void close() {
        try {
            if (out != null) out.flush();
        } catch (Throwable ignored) {
        }
    }
}
