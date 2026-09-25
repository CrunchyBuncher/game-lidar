# Development

How game-lidar is put together, and how to work on it. See [spec.md](../spec.md) for the full
design and [plan.md](../plan.md) for milestones.

## Architecture
Two programs talk through shared memory:

```
game ── ReShade ── lidar_capture addon ──▶ shared-memory ring ──▶ lidar_viewer
                   (depth + camera)        (Local\game_lidar_frames)  (point cloud)
```

### The addon (`addon/`)
`lidar_capture.addon64` / `.addon32` runs inside the game and publishes one frame per capture:
a downsampled depth image plus the camera's view and projection matrices.

- **Depth path:** `depth_tracker` counts draws per bound depth-stencil each frame and picks the
  scene depth at present (heuristics after ReShade's generic_depth). A `DepthCapture` copies it,
  point-samples it down to the capture size with a compute shader and reads it back through a
  ring of staging buffers, without stalling the game.
- **Camera path:** `camera_tracker` latches the profile's constant-buffer window at the draws into
  each depth-stencil. The bytes come from a `CbufferSource`. At present, the latch for the captured
  depth-stencil becomes the frame's pose. Without a profile, frames go out camera-relative with a
  projection from the overlay settings.
- **Discovery** (`discovery.cpp`) runs on top of both paths. It samples every bound constant buffer
  at the draws, scans them for view / projection / combined matrices, and ranks candidates by how
  they behave and how well they reproject one depth frame onto a later one. `modelview.cpp`
  recovers the camera from per-object world·view matrices for games that never upload it alone
  (see [plan_modelview.md](../plan_modelview.md)).
- **Profiles** (`profile.cpp`) say where a game keeps its camera: which stage, slot and offset,
  the layout and the conventions. Format in spec.md §3.7.
- **Graphics APIs:** only `DepthCapture` and `CbufferSource` are API-specific, one each per API in
  `d3d9/`, `d3d11/` and `d3d12/`, registered in `backends.cpp`. D3D11 shadows every cbuffer on
  Unmap / UpdateSubresource. D3D12 records where the camera cbuffer lives at the draw and reads it
  through the game's Map pointer after submission. D3D9 shadows the shader constant registers and
  has ReShade create depth-stencils as INTZ so they can be sampled.
- **Watchdog** (`watchdog.cpp`): if the game stops presenting, it writes to `ReShade.log` where the
  render thread is, so a hang leaves evidence behind.

### The frame protocol (`common/`)
`protocol.h` defines the ring: a few slots, each guarded by a seqlock, holding a frame header
(sizes, flags, view and projection) and the depth (and optional color) image. Matrices are
row-major with row-vector math, D3D clip space. Depth is the raw NDC z, so any depth convention
works: consumers unproject with the inverse projection. The producer can also ask the viewer to
clear its points (the overlay's **Clear viewer points**, or when the pose's frame changes).
`ring.cpp` is the writer and reader; `unproject.h` is the CPU reference for turning a pixel into a
world point.

### The viewer (`viewer/`)
`lidar_viewer` reads the newest frame from the ring and does everything else on the GPU:
unprojects the depth into world space, dedupes it into a voxel-hashed point pool, carves out
points that a newer frame sees straight through, and renders the pool with a free-fly camera.
It can start before or after the producer, and reconnects when the game restarts.

## Source layout
| Path | What |
|---|---|
| `addon/` | The ReShade addon; `d3d9/`, `d3d11/`, `d3d12/` hold the per-API halves |
| `viewer/` | The point-cloud viewer and its shaders |
| `common/` | Frame protocol, shared-memory ring, window/device helpers |
| `profiles/` | Game profiles (`.toml`) |
| `tests/` | Unit tests (`lidar_tests`): camera math, profiles, discovery, root layouts, model-view solver |
| `tools/verify/` | `lidar_verify`: checks a live ring or a saved `.ply` |
| `tools/fake_game/` | A test game for exercising the addon without a real game (below) |

## Build
See the README. Binaries land in `build/bin/Release/`; the test game goes to
`sandbox/fake_game/` so ReShade can be installed next to it without hooking the viewer.

For the 32-bit addon, build only the addon (and tests) in `build-win32`: building `fake_game` there
would overwrite the 64-bit `sandbox\fake_game\fake_game.exe`.
```powershell
cmake -S . -B build-win32 -G "Visual Studio 18 2026" -A Win32
cmake --build build-win32 --config Release --target lidar_capture lidar_tests
```

## Debugging in a game
- Everything the addon has to say goes to `ReShade.log` next to the game's exe: which API it
  hooked, profile loading, discovery summaries every 10 seconds, and watchdog reports.
- With an unsupported API (OpenGL, Vulkan, D3D10) the addon registers nothing but a device-created
  callback, writes one "Inactive: ..." line to the log, and stays out of the way.
- **Write report** in the Discovery section writes `lidar_discovery.txt` with every candidate.
- Settings are under `[LIDAR]` in `ReShade.ini`. Edit it only while the game is closed.
- `lidar_verify ring` works with any producer, so it's the quickest check that frames are arriving
  with a sane pose.

## Testing
```powershell
build\bin\Release\lidar_tests.exe
# with a producer running (the addon in a game, or the test game):
build\bin\Release\lidar_verify.exe ring --frames 60
build\bin\Release\lidar_verify.exe ply scans\some_scan.ply
```

## The test game
`tools/fake_game` renders a small level along a scripted camera path, in D3D9, D3D11 or D3D12,
with known depth conventions and camera layouts. It exists to check the addon against a ground
truth. The build puts it in `sandbox\fake_game\` (and a copy in `sandbox\fake_game_d3d9\`), along
with `lidar_capture.addon64` and `profiles/fake_game.toml` as `lidar_profile.toml`. Install ReShade
there as with any game.
```powershell
sandbox\fake_game\fake_game.exe --no-publish
build\bin\Release\lidar_viewer.exe
```
With `--no-publish` the addon does the capturing. Without it, the test game publishes its own
depth and camera, as a reference producer.

**Options:** `--depth standard|reversed|reversed-infinite`, `--capture-width 480`,
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
`sandbox\fake_game_d3d9\`, where ReShade goes in as `d3d9.dll` (the same DLL as `dxgi.dll`,
renamed; both in one folder would load ReShade twice).

`--camera-layout separate|viewproj|wvp` changes what the shaders get, for testing discovery:
separate view and proj (the default), only a view-projection in b0/c0, or only a per-draw
world·view·proj in b1/c13 (the level is drawn in three parts after the NPC in every layout).

### Checking the addon against the test game
With the addon providing the pose:
```powershell
sandbox\fake_game\fake_game.exe --no-npc --no-publish --depth reversed
build\bin\Release\lidar_verify.exe ring --frames 60
```
The same works for D3D12 with the same profile (b0, space 0), with root CBVs or `--cbv-tables`:
```powershell
sandbox\fake_game\fake_game.exe --api d3d12 --no-npc --no-publish --depth reversed
build\bin\Release\lidar_verify.exe ring --frames 60
```
And D3D9 (slot 0, view at byte 0 = c0, proj at 64 = c4):
```powershell
sandbox\fake_game_d3d9\fake_game.exe --api d3d9 --no-npc --depth reversed
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

D3D9 depth is 24-bit, so compare it against a `--d24` reference. Standard depth matches exactly.
In reversed modes about 5% of pixels are one 24-bit step apart (the rasterizers round differently
when D3D9's half-pixel offset is corrected), so allow that step:
```powershell
sandbox\fake_game_d3d9\fake_game.exe --api d3d9 --no-npc --depth reversed --freeze 3
<copy>\fake_game.exe --no-npc --d24 --depth reversed --freeze 3 --ring Local\game_lidar_ref
build\bin\Release\lidar_verify.exe addon --frames 60 --depth-tol 6e-8
```

### Discovery on the test game
With no profile, set these under `[LIDAR]` in the rig's ReShade.ini (game closed), run the game with
the NPC on, then check `ReShade.log` and the saved `lidar_profile.toml`:
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
