# SmoothRide MP

<p align="center"><img src="docs/preview.png" alt="SmoothRide MP" width="320"></p>

**Smooth multiplayer vehicles for Project Zomboid (Build 42.21+)**

Remote vehicles are drawn with ~60-100 ms delay instead of 500+ ms. They follow smooth curves based on the driver's speed instead of straight-line segments and never pop in. Prediction errors are blended smoothly.

The driver sends car state ~30 times per second instead of ~6. Client-side Java mod (requires ZombieBuddy). Every player must have it. If the game build does not match, it disables itself and falls back to vanilla sync.

## Features
- Low-latency remote vehicle rendering (~60-100 ms instead of 500+ ms)
- Smooth interpolation using speed-based curves
- Blended error correction (no sudden jumps)
- High-frequency state updates from driver (30 Hz)
- Automatic fallback for version mismatch
- Configurable modes: balanced (default), smooth, realtime

## Installation
**Required on every client and in server mod list.**

1. Install **ZombieBuddy** 2.3.4+ (if you use BetterVehicleDynamics, RealVehicleInteriors etc. you probably already have it).
2. Download `SmoothRide_1.0.0.zip` from [Releases](https://github.com/druidqwerty/SmoothRide/releases) and unpack the `SmoothRide` folder into your mods directory:
   - `%UserProfile%\Zomboid\mods\SmoothRide\`
   - You should see `... \mods\SmoothRide\42\mod.info` and `... \42\media\java\SmoothRide.jar`
3. On first connection, ZombieBuddy will prompt for Java mod permission for SmoothRide — allow it.
4. Add `SmoothRide` to your server's `Mods=` list in `Server\YourServer.ini`.

## Configuration
On first run a file is created: `%UserProfile%\Zomboid\SmoothRide.properties`

```
mode=balanced
```
- `balanced` — default, good compromise
- `smooth` — max smoothness (adds ~40 ms latency)
- `realtime` — lowest latency, more prediction

Delete the file to reset to defaults.

## Verification
Check `%UserProfile%\Zomboid\console.txt` for `[SmoothRide]` lines:

- `self-test: interpolation override OK` — mod is active
- `driver: ... (30.0/s)` — you are driving, sending 30 updates/sec
- `vid N: ... delay avg 80 ...` — remote vehicle stats (logged every 30s)

In debug Lua console:
- `SmoothRide_status()`
- `SmoothRide_set("mode", "smooth")`

## Live test
Local server + two clients (driver and passenger-observer), 145 FPS:

| | vanilla | SmoothRide |
|---|---|---|
| remote car drawn behind real position | 504 ms | 80 ms |
| mean position error | 5.94 m | 0.95 m |
| jerkiness: p99 acceleration of the drawn car (real car: 28 m/s²) | 4155 m/s² | 81 m/s² |
| driver updates sent | 6.7/s | 30/s |
| pops | 2 | 0 |

## Please note
- This mod was made 100% with AI.
- It was made only for playing with a friend.
- Tested only on a dedicated server. Not tested on a server hosted from the game menu.

## Building from Source
Requires Java 25+ and the game + ZombieBuddy jars.

```
JDK=/path/to/jdk-25/bin GAME=/path/to/ProjectZomboid tools/build.sh
```

The script builds `build/SmoothRide.jar`, assembles the mod folder in `dist/SmoothRide/` and runs the offline simulator (`test/`).

## License
MIT License — see LICENSE file.

## Author
[druidqwerty](https://github.com/druidqwerty)

For questions or issues, open a GitHub issue.
