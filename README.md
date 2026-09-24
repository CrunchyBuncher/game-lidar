# game-lidar

A live "LiDAR" scan of PC games. A ReShade addon captures depth and the game's camera
matrices, and a viewer builds a point cloud of the level while you play. See
[spec.md](spec.md) for the design and [plan.md](plan.md) for milestones.

Current state: **M2 + D3D12**. The ReShade addon captures depth and sniffs the camera from the
game's constant buffers using a per-game profile, on D3D11 and D3D12, verified bit-exact against
the fake game in both APIs. Discovery mode (finding a new game's camera without knowing its
offsets) comes next (M3).

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
`--api d3d12` renders the same level with D3D12 (render only, so the addon must publish),
keeping the cbuffers in a persistently mapped upload heap bound as root CBVs, or through a
descriptor table with `--cbv-tables`. `--d3d12-debug` enables the D3D12 debug layer and exits
with code 3 on any error.

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
The same works for D3D12 with the same profile (b0, space 0), with root CBVs or `--cbv-tables`:
```powershell
sandbox\fake_game\fake_game.exe --api d3d12 --no-npc --no-publish --depth reversed
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
Add `--api d3d12` to the injected game to check D3D12 against the same D3D11 reference.

## Use it in a game
Works with D3D11 and D3D12 games. Stick to single-player games: anti-cheat may block ReShade or
ban you for it. Check which API the game actually uses: `ReShade.log` names the device it
hooked (`D3D11CreateDevice`, `D3D12CreateDevice`, `IDirect3D9::CreateDevice`, ...). Older games
can be D3D9 even on a D3D12 system (e.g. Unreal Engine 3 titles, which may run through Windows'
D3D9On12 layer). With an unsupported API the addon stays inactive: it registers nothing but a
device-created callback, writes one "Inactive: ..." line to `ReShade.log`, and the game runs as
it would without it (there's no LiDAR tab in that case). D3D9 won't be supported.

1. **Install ReShade 6.8.0 with add-on support** (`ReShade_Setup_6.8.0_Addon.exe`, the
   "with full add-on support" download). The version has to match: the addon is built against
   ReShade 6.8.0's API and ImGui, and ReShade refuses add-ons built for another version. To use a
   newer ReShade, bump the ReShade tag and the ImGui commit in `CMakeLists.txt` and rebuild.
   In the installer, pick the game's executable and **DirectX 10/11/12** as the API, which
   installs ReShade as `dxgi.dll` next to the exe. Effects are optional; you can skip them all.
2. **Copy the addon next to the game's executable** (the folder where `dxgi.dll` went):
   ```powershell
   copy build\bin\Release\lidar_capture.addon64 "<game folder>\"
   ```
   Some games keep the exe in a subfolder (e.g. `bin\x64\` or `Binaries\Win64\`). Use the one
   holding the exe.
3. **Start the game**, then press **Home** to open the ReShade overlay. The **Add-ons** tab should
   list "game-lidar capture", and a **LiDAR** tab shows its status. If the add-on is missing,
   check `ReShade.log` next to the exe.
4. **Start the viewer** (`build\bin\Release\lidar_viewer.exe`). It can start before or after the game.
5. **In the LiDAR tab:**
   - **Depth buffer:** make sure the one marked *captured* is the scene depth (the one with the
     most draws, at screen size). Pick it by hand if the automatic choice is wrong.
   - **Camera:** without a profile for the game, frames go out camera-relative, so the viewer
     shows a live snapshot rather than a world scan. Set **Fallback projection** (vertical FOV,
     near/far, depth mode) to match the game; the tab hints at reversed depth from how the
     game clears it.
   - **With a profile:** save it as `lidar_profile.toml` next to the exe, or point the tab's
     **Profile** field at it. The format is described in spec.md §3.6, and
     `profiles/fake_game.toml` is an example. Finding a game's profile is what discovery mode
     (M3) is for; until then it means reading the cbuffers in a RenderDoc capture.

Settings live under `[LIDAR]` in the game's `ReShade.ini` (`Enabled`, `CaptureWidth`,
`Profile`, `FovY`, `Near`, `Far`, `DepthMode`). Only edit the ini while the game is closed:
ReShade rewrites it from memory. To remove everything, run the ReShade installer again and
uninstall, then delete `lidar_capture.addon64`.
