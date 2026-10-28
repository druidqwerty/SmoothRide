package com.druid.smoothride;

import se.krka.kahlua.integration.annotations.LuaMethod;

/** Lua side: SmoothRide_status(), SmoothRide_get("mode"), SmoothRide_set("mode", "smooth"); the in-game options use get/set. */
public class LuaApi {
    @LuaMethod(name = "SmoothRide_status", global = true)
    public static String status() {
        return Engine.status();
    }

    @LuaMethod(name = "SmoothRide_get", global = true)
    public static String get(String key) {
        return key == null ? null : Config.get(key);
    }

    @LuaMethod(name = "SmoothRide_set", global = true)
    public static boolean set(String key, String value) {
        boolean ok = Config.set(key, value);
        Engine.log("set " + key + "=" + value + (ok ? "" : " (rejected)"));
        if (ok) Trace.note(Engine.serverNowMs(), "set " + key + "=" + value);
        return ok;
    }
}
