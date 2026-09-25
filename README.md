# game-lidar

A live "LiDAR" scan of PC games. A ReShade addon captures depth and the game's camera
matrices, and a viewer builds a point cloud of the level while you play. See
[spec.md](spec.md) for the design and [plan.md](plan.md) for milestones.

Current state: **M3 (discovery) + D3D12 + D3D9**. The ReShade addon captures depth and sniffs the
camera from the game's constant buffers (D3D9: shader constant registers) using a per-game profile,
on D3D9, D3D11 and D3D12, 64- and 32-bit. Discovery mode finds a new game's camera and writes that
profile. Everything is verified against the fake game in all three APIs; the first real game is next.

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

32-bit games need a 32-bit addon (`lidar_capture.addon32`), from a separate build folder. Build only
the addon (and tests) there: building `fake_game` would overwrite the 64-bit `sandbox\fake_game\fake_game.exe`.
```powershell
cmake -S . -B build-win32 -G "Visual Studio 18 2026" -A Win32
cmake --build build-win32 --config Release --target lidar_capture lidar_tests
```

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
`--api d3d9` renders it with D3D9 (render only too), with the camera in vertex shader constant
registers c0-c12 (the same bytes as b0) and a per-object decoy in c13-c17. It uses the device's
auto depth-stencil, or a `CreateDepthStencilSurface` one with `--own-depth`. `--d24` gives the
D3D11 renderer a 24-bit depth buffer, as a reference for D3D9. Run the D3D9 build from
`sandbox\fake_game_d3d9\`: the build copies `fake_game.exe` there, and ReShade goes in as
`d3d9.dll` (the same DLL as `dxgi.dll`, renamed; both in one folder would load ReShade twice).
`--camera-layout separate|viewproj|wvp` changes what the shaders get, for testing discovery:
separate view and proj (the default), only a view-projection in b0/c0, or only a per-draw
world·view·proj in b1/c13 (the level is drawn in three parts after the NPC in every layout).

**lidar_viewer**: right-drag to look, WASD/QE to move, Shift for speed, the wheel changes base speed.
`F` follow player, `H` height/color mode, `T` trail, `X` toggle carving, `+/-` point size,
`C` clear, `P` save `.ply` to `scans/`, `Space` pause ingest.
Options: `--voxel 0.05`, `--capacity-m 50` (millions of points; ~1.5 GB of GPU memory at 50), `--near-cut 0.3`, `--max-range 500`,
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
`lidar_profile.toml`, the name the addon loads by default) next to both fake games
(`sandbox\fake_game\` and `sandbox\fake_game_d3d9\`). A different profile can be set in the
overlay's LiDAR tab or as `Profile=` under `[LIDAR]` in ReShade.ini.
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

D3D9 uses the same profile (slot 0, view at byte 0 = c0, proj at 64 = c4):
```powershell
sandbox\fake_game_d3d9\fake_game.exe --api d3d9 --no-npc --depth reversed
build\bin\Release\lidar_verify.exe ring --frames 60
```
Discovery on the fake game: with no profile, set these under `[LIDAR]` in the rig's ReShade.ini
(game closed), run the game with the NPC on, then check `ReShade.log` and the saved `lidar_profile.toml`:
```ini
[LIDAR]
Profile=no_profile.toml
DiscoveryAutoStart=1
DiscoveryAutoSave=20
```
```powershell
sandbox\fake_game\fake_game.exe --camera-layout wvp --no-publish --duration 30
```
Then set `Profile=lidar_profile.toml` and `DiscoveryAutoStart=0`, and run the ring check above with
the same `--camera-layout`. Remove the `[LIDAR]` section afterwards; the next addon build restores
the rig's `lidar_profile.toml`.

D3D9 depth is 24-bit, so compare it against a `--d24` reference. Standard depth matches exactly.
In reversed modes about 5% of pixels are one 24-bit step apart (the rasterizers round differently
when D3D9's half-pixel offset is corrected), so allow that step:
```powershell
sandbox\fake_game_d3d9\fake_game.exe --api d3d9 --no-npc --depth reversed --freeze 3
<copy>\fake_game.exe --no-npc --d24 --depth reversed --freeze 3 --ring Local\game_lidar_ref
build\bin\Release\lidar_verify.exe addon --frames 60 --depth-tol 6e-8
```

## Use it in a game
Works with D3D9, D3D11 and D3D12 games. **Turn anti-aliasing (MSAA) off in the game's video
settings:** multisampled depth can't be captured. With MSAA on, the addon falls back to some other
depth buffer (often a post-process pass with a couple of draws), so capture shows nothing useful and
discovery can't find the camera. The LiDAR tab warns when this happens. Post-process AA (FXAA,
TAA, SMAA) is fine. Stick to single-player games: anti-cheat may block ReShade
or ban you for it. Check which API the game actually uses: `ReShade.log` names the device it
hooked (`D3D11CreateDevice`, `D3D12CreateDevice`, `IDirect3D9::CreateDevice`, ...). Older games
can be D3D9 even on a D3D12 system (e.g. Unreal Engine 3 titles, which may run through Windows'
D3D9On12 layer). With an unsupported API (OpenGL, Vulkan, D3D10) the addon stays inactive: it
registers nothing but a device-created callback, writes one "Inactive: ..." line to `ReShade.log`,
and the game runs as it would without it (there's no LiDAR tab in that case).

**D3D9 notes:** D3D9 depth buffers can't be read, so the addon has ReShade create the game's
screen-sized depth buffers as INTZ textures (the trick ReShade's own depth access uses). This
needs MSAA off in the game (see above), and depth is 24-bit. There are no constant buffers either: games put
their matrices in shader constant registers, so a D3D9 profile uses `slot = 0` and byte offsets
of register × 16 (c4 = 64). `size` and `space` are ignored. Games that use the fixed-function
pipeline (`SetTransform`, mostly pre-2004) have no shader constants to sniff.

1. **Install ReShade 6.8.0 with add-on support** (`ReShade_Setup_6.8.0_Addon.exe`, the
   "with full add-on support" download). The version has to match: the addon is built against
   ReShade 6.8.0's API and ImGui, and ReShade refuses add-ons built for another version. To use a
   newer ReShade, bump the ReShade tag and the ImGui commit in `CMakeLists.txt` and rebuild.
   In the installer, pick the game's executable and **DirectX 10/11/12** as the API, which
   installs ReShade as `dxgi.dll` next to the exe (for a D3D9 game pick **Direct3D 9**, which
   installs `d3d9.dll`). Effects are optional; you can skip them all.
2. **Copy the addon next to the game's executable** (the folder where `dxgi.dll` / `d3d9.dll` went):
   ```powershell
   copy build\bin\Release\lidar_capture.addon64 "<game folder>\"
   ```
   For a 32-bit game, copy `build-win32\bin\Release\lidar_capture.addon32` instead (see Build).
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
     **Profile** field at it. The format is described in spec.md §3.7, and
     `profiles/fake_game.toml` is an example.
6. **Discovery** (to find a game's profile): with the depth buffer right, open the **Discovery**
   section, press **Start**, then move and turn the camera for about 20 seconds (in game, not in a
   menu or cutscene). Candidates are ranked by score: whether they stay constant while the view
   doesn't change and change when it does, and how well they reproject one depth frame into a
   later one. A good one has its score in green (confident), most reprojection rounds passing and a
   plausible FOV/near. **Use** previews a candidate without saving it (check the scan in the
   viewer, e.g. by turning 360° in place: the level should line up with itself). **Save** writes it
   as `lidar_profile.toml` next to the exe (the old one becomes `lidar_profile.toml.bak`) and loads
   it. **Write report** writes `lidar_discovery.txt` next to the exe with everything discovery
   found; it's also written on Save. A summary goes to `ReShade.log` every 10 seconds.

   Discovery also finds cameras stored as a single matrix: layouts `viewproj` / `invviewproj`
   (split into view and projection) next to the paired ones. When the matrix is per object
   (world·view·proj), it saves `latch = "common"`: the value most draws into the depth buffer had,
   rather than the first or last draw's.

Settings live under `[LIDAR]` in the game's `ReShade.ini` (`Enabled`, `CaptureWidth`,
`Profile`, `FovY`, `Near`, `Far`, `DepthMode`). For unattended discovery runs there are also
`DiscoveryAutoStart=1` (start with the game), `DiscoveryAutoSave=<seconds>` (save the top
candidate once after that long, if it's confident) and `DiscoverySamples` (sampled draws per
frame, default 48). Only edit the ini while the game is closed: ReShade rewrites it from memory. To remove everything, run the ReShade installer again and
uninstall, then delete `lidar_capture.addon64`.
