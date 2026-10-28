# SmoothRide MP

<p align="center"><img src="docs/preview.png" alt="SmoothRide MP" width="320"></p>

**Smooth multiplayer vehicles for Project Zomboid (Build 42.21+)**

Remote vehicles are drawn with ~60-100 ms delay instead of 500+ ms. They follow smooth curves based on the driver's speed instead of straight-line segments and never pop in. Prediction errors are blended smoothly.

Server hitches (world save, loading new map areas on a big modded host) no longer freeze cars for a second and then teleport them: the car keeps going on prediction (up to 2.5 s) and glides back onto its true path when data resumes.

The driver sends car state ~30 times per second instead of ~6. Client-side Java mod (requires ZombieBuddy). Every player must have it. If the game build does not match, it disables itself and falls back to vanilla sync.

<p align="center"><a href="https://boosty.to/lrgpp"><img src="docs/banner.png" alt="LrgPP mods on Boosty" width="640"></a></p>

Free and open source. Almost all of the code is written by an AI; the subscription, the testing and the time are mine. If the mod made your game better: [support on Boosty](https://boosty.to/lrgpp).

## Features
- Low-latency remote vehicle rendering (~60-100 ms instead of 500+ ms)
- Smooth interpolation using speed-based curves
- Blended error correction (no sudden jumps)
- Server hitches of 0.3-2.5 s bridged by prediction: no freeze, no teleport (new in 1.1.0)
- High-frequency state updates from driver (30 Hz)
- Automatic fallback for version mismatch
- Host settings in the sandbox options (page SmoothRide), changeable in a running game

## Installation
**Required on every client and in server mod list.**

1. Install **ZombieBuddy** 2.3.4+ (if you use BetterVehicleDynamics, RealVehicleInteriors etc. you probably already have it).
2. Download `SmoothRide_1.1.0.zip` from [Releases](https://github.com/druidqwerty/SmoothRide/releases) and unpack the `SmoothRide` folder into your mods directory:
   - `%UserProfile%\Zomboid\mods\SmoothRide\`
   - You should see `... \mods\SmoothRide\42\mod.info` and `... \42\media\java\SmoothRide.jar`
3. On first connection, ZombieBuddy will prompt for Java mod permission for SmoothRide — allow it.
4. Add `SmoothRide` to your server's `Mods=` list in `Server\YourServer.ini`.

## Configuration
**Server-wide (same for every player): the host's sandbox options, page SmoothRide.**
Main menu: Host > Manage settings > Edit > Sandbox > SmoothRide (dedicated server: `SandboxVars.SmoothRide` in `YourServer_SandboxVars.lua`).
An admin can change them in a running game (Admin > Sandbox options); every client applies them within 2 s, no restart.

| Option | Default | |
|---|---|---|
| Enable SmoothRide | on | off = everyone sees vanilla sync |
| Smoothness mode | Balanced | Balanced: other cars ~80 ms behind. Smooth: ~120 ms, even smoother (still 4x less than vanilla's 500). Realtime: lowest delay, more prediction, cars may get corrected on sharp manoeuvres |
| Driver send interval, ms | 33 | 33 = 30 times a second, 150 = vanilla |
| Bridge server hitches up to, ms | 2500 | how long a moving car keeps going on prediction when the server sends nothing |

**Personal: Options > Mods > SmoothRide MP** - only diagnostics: write stats to console.txt, stats interval.

`%UserProfile%\Zomboid\SmoothRide.properties` holds tuning values for testing; normally nothing there needs editing (server values win over it).

## Verification
Check `%UserProfile%\Zomboid\console.txt` for `[SmoothRide]` lines:

- `loaded 1.1.0.` — the version that is installed
- `self-test: interpolation override OK` — mod is active
- `settings (server): ...` — the host's settings arrived
- `driver: ... (30.0/s)` — you are driving, sending 30 updates/sec
- `vid N: ... delay avg 70 ...` — remote vehicle stats (logged every 30s), including server gaps bridged

In debug Lua console:
- `SmoothRide_status()`

## What's new in 1.1.0
- **No more freeze + teleport at speed on a big host.** The co-op server stalls 0.3-2 s while it saves the world (every 5 min) or loads new map areas, and then drops the queued car updates. 1.0 predicted only 250 ms, then froze the car and snapped it 10-45 m forward. 1.1 keeps the car going on an arc (speed and turn rate from the last samples) for up to 2.5 s, then coasts to a stop; when data resumes, the error is blended out at a speed-bounded rate instead of a snap.
- Settings moved to the host's sandbox options (page SmoothRide): the same for everyone, changeable in a running game. Options > Mods keeps only the personal stats switches.
- A car righted with `setAngles` (rotation-only jump) is treated as a teleport, not spun.
- The console shows the version (`[SmoothRide] loaded 1.1.0.`).

## Live test
Local server + two clients (driver and passenger-observer), 145 FPS, no Steam. Normal driving (figure eight, 50 km/h):

| | vanilla | SmoothRide 1.1.0 |
|---|---|---|
| remote car drawn behind real position | 492 ms | 72 ms |
| mean position error | 5.81 m | 0.86 m |
| jerkiness: p99 acceleration of the drawn car (real car: 28 m/s²) | 4006 m/s² | 77 m/s² |
| driver updates sent | 6.7/s | 30/s |
| pops | 0 | 0 |

Server hitches (highway loop at 90 km/h, 16 injected server stalls of 0.3-2 s each, updates dropped or delivered in a burst afterwards):

| | vanilla | SmoothRide 1.0.0 | SmoothRide 1.1.0 |
|---|---|---|---|
| car frozen during a stall, avg / max | 0.9 s / 1.6 s | 0.95 s / 1.8 s | 0 / 0 |
| teleports (one-frame jumps > 1 m) | 49 | 24 | 0 |
| largest one-frame jump | 7.3 m | 46.8 m | 0.10 m |
| largest error | 41 m | 45 m | 2.2 m |

## Please note
- This mod was made 100% with AI.
- It was made only for playing with a friend.
- Live-tested on a local dedicated server. The 1.1.0 hitch fix comes from the logs of a server hosted from the game menu, and the host's settings page was checked there; a full session with a friend on such a server with 1.1.0 is still to come.

## Building from Source
Requires Java 25+ and the game + ZombieBuddy jars.

```
JDK=/path/to/jdk-25/bin GAME=/path/to/ProjectZomboid tools/build.sh
```

The script builds `build/SmoothRide.jar`, assembles the mod folder in `dist/SmoothRide/` and runs the offline simulators in `test/`: `SimTest` (normal driving over different links) and `StallTest` (server hitches, 1.1 against the frozen 1.0 copy `TrackV10`).

## My mods
- **SmoothRide MP** — smooth multiplayer vehicles (this mod).
- [ZombieSync MP](https://steamcommunity.com/sharedfiles/filedetails/?id=3827946115) — smooth multiplayer zombies.
- [Tripping Zombies MP Fix](https://steamcommunity.com/sharedfiles/filedetails/?id=3829034899) — the same Tripping Zombies trips for every player.
- [Wandering Hordes Save Trim](https://steamcommunity.com/sharedfiles/filedetails/?id=3829140377) — no more save freezes from Wandering Hordes' wall map.
- [Host Unstuck](https://steamcommunity.com/sharedfiles/filedetails/?id=3829163351) — un-sticks and speeds up the Workshop download when hosting from the menu.

## License
MIT License — see LICENSE file.

## Author
[druidqwerty](https://github.com/druidqwerty)

For questions or issues, open a GitHub issue.
