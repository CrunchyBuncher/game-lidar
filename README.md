# game-lidar

**Scan PC games like a LiDAR.** A ReShade addon reads the depth buffer and the game's camera
while you play, and a standalone viewer turns it into a live 3D point cloud of the level.

![A game running with the addon on the left, the viewer building its point cloud live on the right](docs/media/hero.gif)

<sub>The addon capturing a game (left) while the viewer builds the scan from nothing (right).
[MP4](docs/media/hero.mp4)</sub>

## How it works
```
game ── ReShade ── lidar_capture addon ──▶ shared memory ──▶ lidar_viewer
                   (depth + camera)                          (point cloud)
```
- **The addon** runs inside the game through [ReShade](https://reshade.me). Each frame it finds the
  scene's depth buffer, reads a downsampled copy back without stalling the game, and pairs it with
  the camera matrices it picks out of the game's shader constants.
- **The viewer** is a separate app. It unprojects every depth frame into world space on the GPU,
  merges the points into one cloud, and lets you fly around it while you keep playing.

## Features
- **Live world-space scans.** Every frame's depth is placed with the game's own camera, so the
  level builds up as you move around it.
- **Automatic camera discovery.** No profile for your game? Walk around for about 20 seconds and
  the addon finds the camera matrices in the game's shader constants and saves a profile.
- **Self-cleaning.** Points left behind by things that moved (NPCs, doors) are carved away when you
  look through where they were.
- **Export to `.ply`** for Blender, MeshLab, CloudCompare and friends.
- **D3D9, D3D11 and D3D12**, 32- and 64-bit games. Tested in DX9 and DX11 games.

<p align="center">
  <img src="docs/media/addon_overlay.gif" alt="The LiDAR tab in the ReShade overlay" width="80%">
</p>
<p align="center"><sub>The LiDAR tab in the ReShade overlay: capture stats, the camera, and the
depth buffer in use. <b>Clear viewer points</b> wipes the scan from inside the game.
<a href="docs/media/addon_overlay.mp4">MP4</a></sub></p>

## Use it in a game
**Before you start:**
- **Single-player games only.** Anti-cheat may block ReShade or ban you for using it.
- **Turn off MSAA** in the game's video settings: multisampled depth can't be captured. Post-process
  AA (FXAA, TAA, SMAA) is fine. The LiDAR tab warns when MSAA is getting in the way.
- OpenGL, Vulkan and D3D10 aren't supported. The addon stays inactive and the game runs normally.

**Setup:**
1. **Install ReShade 6.8.0 with add-on support** (the "with full add-on support" download). The
   version has to match: ReShade refuses add-ons built for another version. In the installer,
   pick the game's executable and **DirectX 10/11/12** (or **Direct3D 9** for a D3D9 game).
   Effects are optional; you can skip them all.
2. **Copy the addon next to the game's executable**, in the folder where ReShade put `dxgi.dll` /
   `d3d9.dll`: `lidar_capture.addon64` for 64-bit games, `lidar_capture.addon32` for 32-bit ones.
   Some games keep the exe in a subfolder (e.g. `bin\x64\` or `Binaries\Win64\`).
3. **Start the game** and press **Home** to open the ReShade overlay. The **Add-ons** tab should list
   "game-lidar capture", and there's a new **LiDAR** tab. If not, check `ReShade.log` next to the exe.
4. **Start the viewer** (`lidar_viewer.exe`), before or after the game.
5. **In the LiDAR tab**, check that the depth buffer marked *captured* is the scene depth (the one
   with the most draws, at screen size). Pick it by hand if the automatic choice is wrong.

Not sure which API a game uses? `ReShade.log` names the device it hooked (`D3D11CreateDevice`,
`IDirect3D9::CreateDevice`, ...). Older games can be D3D9 even on a modern system (e.g. Unreal
Engine 3 titles).

### Profiles and discovery
A profile tells the addon where the game keeps its camera. With one, you get a proper world-space
scan. Without one, frames go out relative to the camera, so the viewer shows a live snapshot of
what's on screen (set **Fallback projection** to the game's FOV and near/far for that).

To find a game's profile, open **Discovery** in the LiDAR tab, press **Start**, then move and turn
the camera for about 20 seconds (in game, not in a menu or cutscene). Candidates are ranked by how
well they behave like a camera and how well they reproject one depth frame onto a later one; a good
one shows its score in green. **Use** previews a candidate in the viewer (turn 360° in place: the
level should line up with itself). **Save** writes it as `lidar_profile.toml` next to the exe and
loads it. **Write report** saves everything discovery found to `lidar_discovery.txt`.

Discovery handles separate view and projection matrices, a combined view-projection, and
per-object world·view·proj matrices. The profile format is described in
[spec.md §3.7](spec.md), with examples in [profiles/](profiles/).

**D3D9 games:** depth is 24-bit, and there are no constant buffers, so a D3D9 profile uses
`slot = 0` and byte offsets of register × 16 (c4 = 64). Games that use the fixed-function pipeline
(mostly pre-2004) have no shader constants to find.

### Settings
Everything is in the LiDAR tab, and saved under `[LIDAR]` in the game's `ReShade.ini`
(`Enabled`, `Color`, `CaptureWidth`, `Profile`, `FovY`, `Near`, `Far`, `DepthMode`). For unattended
discovery there are `DiscoveryAutoStart=1`, `DiscoveryAutoSave=<seconds>` and `DiscoverySamples`.
Only edit the ini while the game is closed: ReShade rewrites it from memory.

To uninstall, run the ReShade installer again and uninstall, then delete the addon file.

## Viewer
| Input | Action |
|---|---|
| Right-drag | Look around |
| WASD / Q E | Move / down, up (Shift: faster, wheel: base speed) |
| `F` | Follow the player |
| `H` | Color by height / captured color (the game's scene colors, D3D9 and D3D11) |
| `T` | Show the player's trail |
| `X` | Toggle carving |
| `+` / `-` | Point size |
| `C` | Clear the scan |
| `P` | Save a `.ply` to `scans/` |
| `Space` | Pause |
| `F1` | Hide the UI |

Command-line options: `--voxel 0.05` (point spacing in meters), `--capacity-m 50` (millions of
points; about 1.5 GB of GPU memory at 50), `--near-cut 0.3`, `--max-range 500`,
`--height-range -1 20`, `--no-carve`, `--carve-margin 0.15`, `--carve-rel 0.02`.

## Build
Windows only. Requires Visual Studio 2022 or 2026 with the C++ workload (it bundles CMake). There
are no other dependencies. Run from PowerShell or a Developer prompt (Git Bash confuses MSBuild).

```powershell
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release
```

The viewer and the 64-bit addon land in `build\bin\Release\`. For 32-bit games, build the addon
separately:
```powershell
cmake -S . -B build-win32 -G "Visual Studio 18 2026" -A Win32
cmake --build build-win32 --config Release --target lidar_capture
```

The addon is built against ReShade 6.8.0. To use a newer ReShade, bump the ReShade tag and the
ImGui commit in `CMakeLists.txt` and rebuild.

## Development
| Path | What |
|---|---|
| [`addon/`](addon/) | The ReShade addon: depth capture, camera tracking, discovery, per-API backends |
| [`viewer/`](viewer/) | The point-cloud viewer |
| [`common/`](common/) | The shared-memory frame protocol between them |
| [`tests/`](tests/), [`tools/`](tools/) | Unit tests, a verification tool, and a test game for checking the addon |

[docs/development.md](docs/development.md) covers the architecture, debugging the addon in a game,
and testing.
