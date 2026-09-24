# game-lidar

A live "LiDAR" scan of PC games. A ReShade addon captures depth and the game's camera
matrices, and a viewer builds a point cloud of the level while you play. See
[spec.md](spec.md) for the design and [plan.md](plan.md) for milestones.

Current state: **M0**. There's a fake D3D11 game, the shared-memory protocol and the live
viewer, all verified exact. The ReShade addon comes next (M1).

## Build
Requires Visual Studio 2022 or 2026 with the C++ workload (it bundles CMake). There are no
third-party dependencies. Run from PowerShell or a Developer prompt. Git Bash confuses
MSBuild with duplicate `HOME` variables.

```powershell
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release
```

Binaries land in `build/bin/Release/`.

## Try it
```powershell
build\bin\Release\fake_game.exe
build\bin\Release\lidar_viewer.exe
```

**fake_game**: `--depth standard|reversed|reversed-infinite`, `--capture-width 480`,
`--no-npc`, `--no-color`, `--fov 70`. Keys: `M` manual camera (WASD/QE + right-drag),
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
