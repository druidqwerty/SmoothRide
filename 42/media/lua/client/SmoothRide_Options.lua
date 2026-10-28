-- SmoothRide MP settings.
--   server    the host's sandbox options, page SmoothRide (SandboxVars.SmoothRide): Enabled, Mode,
--             SendIntervalMs, MaxPredictMs. Same for everyone; the host (or an admin in a running game,
--             Admin > Sandbox options) changes them, the server sends them to every client, and this
--             file applies them within 2 s (B42 has no Lua event for a sandbox change, so it polls).
--   personal  Options > Mods > SmoothRide MP: only this player's diagnostics (stats lines in console.txt).
-- Outside a server (main menu, single player) the global values come from Zomboid/SmoothRide.properties
-- or the built-in defaults. Values go to the Java side (SmoothRide_set) and apply at once.
if not PZAPI or not PZAPI.ModOptions then return end

local MODES = { "balanced", "smooth", "realtime" }

local function javaLoaded()
    return type(SmoothRide_get) == "function" and type(SmoothRide_set) == "function"
end

local function javaGet(key)
    if not javaLoaded() then return nil end
    local ok, value = pcall(SmoothRide_get, key)
    if ok and value ~= nil then return tostring(value) end
    return nil
end

local function javaSet(key, value)
    if javaGet(key) == value then return true end
    local ok, accepted = pcall(SmoothRide_set, key, value)
    return ok and accepted == true
end

local function startupNumber(key, default)
    return tonumber(javaGet("startup." .. key)) or default
end

local function intString(v)
    return string.format("%d", math.floor((tonumber(v) or 0) + 0.5))
end

local function clamp(v, lo, hi, default)
    v = tonumber(v) or default
    return math.max(lo, math.min(hi, v))
end

local options = PZAPI.ModOptions:create("SmoothRide", "SmoothRide MP")
options:addDescription(javaLoaded() and "UI_SR_Info" or "UI_SR_NotLoaded")
local logStats = options:addTickBox("logStats", "UI_SR_LogStats", true, "UI_SR_LogStats_tt")
local statsInterval = options:addSlider("statsIntervalSec", "UI_SR_StatsInterval", 5, 600, 5, 30)

-- the host's values on a server; nil in single player / main menu or if the server has no SmoothRide page
local function serverValues()
    if not isClient() then return nil end
    local sv = SandboxVars and SandboxVars.SmoothRide
    if type(sv) ~= "table" then return nil end
    return {
        enabled = sv.Enabled ~= false,
        mode = MODES[tonumber(sv.Mode)] or "balanced",
        sendIntervalMs = clamp(sv.SendIntervalMs, 16, 150, 33),
        maxPredictMs = clamp(sv.MaxPredictMs, 250, 5000, 2500),
    }
end

local function globalValues()
    local srv = serverValues()
    if srv then return srv, "server" end
    return {
        enabled = javaGet("startup.enabled") ~= "false",
        mode = javaGet("startup.mode") or "balanced",
        sendIntervalMs = startupNumber("sendIntervalMs", 33),
        maxPredictMs = startupNumber("maxPredictMs", 2500),
    }, "local"
end

local lastLine
local function applySettings()
    if not javaLoaded() then return end
    local g, source = globalValues()
    javaSet("enabled", tostring(g.enabled))
    javaSet("mode", g.mode)
    -- 150 ms is vanilla's own rate: no boost needed
    javaSet("sendBoost", tostring(g.sendIntervalMs < 150))
    javaSet("sendIntervalMs", intString(g.sendIntervalMs))
    javaSet("maxPredictMs", intString(g.maxPredictMs))
    javaSet("logStats", tostring(logStats:getValue()))
    javaSet("statsIntervalSec", intString(statsInterval:getValue()))
    local line = "[SmoothRide] settings (" .. source .. "): enabled=" .. tostring(g.enabled) .. " mode=" .. g.mode
        .. " sendIntervalMs=" .. intString(g.sendIntervalMs) .. " maxPredictMs=" .. intString(g.maxPredictMs)
        .. " | personal: logStats=" .. tostring(logStats:getValue())
        .. " statsIntervalSec=" .. intString(statsInterval:getValue())
    if line ~= lastLine then
        lastLine = line
        print(line)
    end
end

options.apply = applySettings
Events.OnMainMenuEnter.Add(applySettings)
Events.OnGameStart.Add(applySettings)

-- no event fires when an admin changes sandbox options in a running game: look every 2 s
local nextCheck = 0
local lastServer
Events.OnTick.Add(function()
    local now = getTimestampMs()
    if now < nextCheck then return end
    nextCheck = now + 2000
    local srv = serverValues()
    local key = srv and (tostring(srv.enabled) .. srv.mode .. srv.sendIntervalMs .. ":" .. srv.maxPredictMs) or "-"
    if key ~= lastServer then
        lastServer = key
        applySettings()
    end
end)
