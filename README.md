# game-lidar

A live "LiDAR" scan of PC games. A ReShade addon captures depth and the game's camera
matrices, and a viewer builds a point cloud of the level while you play. See
[spec.md](spec.md) for the design and [plan.md](plan.md) for milestones.

Current state: **M2**. The ReShade addon captures depth and sniffs the camera from the game's
constant buffers using a per-game profile, verified bit-exact against the fake D3D11 game.
Discovery mode (finding a new game's camera without knowing its offsets) comes next (M3).

## Build
Requires Visual Studio 2022 or 2026 with the C++ workload (it bundles CMake). There are no
third-party dependencies. Run from PowerShell or a Developer prompt. Git Bash confuses
MSBuild with duplicate `HOME` variables.

```powershell
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release
```

Binaries land in `build/bin/Release/`, except `fake_game.exe`, which goes to
`sandbox/fake_game/` so ReShade can be installed next to it without hooking the viewer.

## Try it
```powershell
sandbox\fake_game\fake_game.exe
build\bin\Release\lidar_viewer.exe
```

**fake_game**: `--depth standard|reversed|reversed-infinite`, `--capture-width 480`,
`--no-npc`, `--no-color`, `--fov 70`, `--no-publish` (leave the ring to the addon). Keys: `M` manual camera (WASD/QE + right-drag),
`Space` pause capture.

**lidar_viewer**: right-drag to look, WASD/QE to move, Shift for speed, the wheel changes base speed.
`F` follow player, `H` height/color mode, `T` trail, `X` toggle carving, `+/-` point size,
`C` clear, `P` save `.ply` to `scans/`, `Space` pause ingest.
Options: `--voxel 0.05`, `--capacity-m 16`, `--near-cut 0.3`, `--max-range 500`,
`--height-range -1 20`, `--no-carve`, `--carve-margin 0.15`, `--carve-rel 0.02`.

**Carving:** when the camera sees *through* a spot where an old point sits (because the thing
moved away), that point is erased. Ghost trails clean themselves up when you look at
the spot again.

## Verify
```powershell
build\bin\Release\lidar_tests.exe
# with `fake_game --no-npc` running:
build\bin\Release\lidar_verify.exe ring --frames 60
build\bin\Release\lidar_verify.exe ply scans\some_scan.ply
```

**Addon:** the build copies `lidar_capture.addon64` and `profiles/fake_game.toml` (as
`lidar_profile.toml`, the name the addon loads by default) next to the fake game. A different
profile can be set in the overlay's LiDAR tab or as `Profile=` under `[LIDAR]` in ReShade.ini.
Only edit the ini while the game is closed: ReShade rewrites it from memory. Without a profile,
the addon publishes camera-relative frames (no pose) using the overlay's fallback projection.

With the addon providing the pose, the ring check works just like with fake_game publishing:
```powershell
sandbox\fake_game\fake_game.exe --no-npc --no-publish --depth reversed
build\bin\Release\lidar_verify.exe ring --frames 60
```

For a bit-exact comparison, run the injected game and an un-injected reference, both frozen at
the same camera. The reference must be a copy of `fake_game.exe` outside `sandbox/`, or
ReShade would hook it too. The addon's depth, projection and view must match exactly.
```powershell
sandbox\fake_game\fake_game.exe --no-npc --no-publish --freeze 3
<copy>\fake_game.exe --no-npc --freeze 3 --ring Local\game_lidar_ref
build\bin\Release\lidar_verify.exe addon --frames 60
```
