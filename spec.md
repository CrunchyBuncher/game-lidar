# game-lidar — Spec

## 1. Goal

Build a **live** "LiDAR scanner" for PC games that runs entirely as a **ReShade addon**.
While the player moves through a game, the addon captures depth and the game's own
camera matrices every frame. An external viewer places the depth in world space and
builds up a point cloud of the level in real time. You can fly around that viewer
freely while you keep playing.

**Priorities:** speed and live feedback first. Visual cleanliness second: ghost
trails from NPCs and other moving objects are acceptable.

## 2. Key decisions

| Decision | Choice | Why |
|---|---|---|
| Depth source | ReShade addon, scene depth buffer (`generic_depth` selection logic) | Works in any game ReShade can hook |
| Camera pose source | **Constant-buffer sniffing inside the same addon** | No second mod needed. Matrices come from the exact frame being rendered, so sync is perfect. |
| Per-game work | A one-time **discovery** step that finds where the camera matrices live, saved as a game profile | Guided by an in-overlay tool, not manual reverse engineering |
| First graphics API | **D3D11** | Constant buffers are updated via `Map`/`UpdateSubresource`, and ReShade reports both as events. That's the easiest to sniff. |
| Later APIs | D3D9 (easy via `push_constants`), then D3D12/Vulkan (harder, see §3.3) | |
| Unprojection | On the GPU in the viewer | The addon only ships a small depth image and two matrices |
| Transport | Shared-memory ring buffer (addon → viewer) | Zero-copy, non-blocking, same machine |
| Viewer | Native C++, D3D11 (compute + indirect draw), no third-party deps | Same API and toolchain as the addon and fake game, and it handles tens of millions of points. Dear ImGui can be added later for UI. |
| Fallbacks | Engine mods (BepInEx/UE4SS) or memory readers | **Only** if a game's matrices can't be recovered from constant buffers |

## 3. Architecture

```
┌──────────────────────────── Game process ─────────────────────────────┐
│ ReShade ─► lidar_capture.addon64                                      │
│                                                                       │
│   ┌─ Depth path ────────────────┐   ┌─ Camera path (cbuffer sniffer) ┐│
│   │ select scene depth buffer   │   │ track cbuffer resources        ││
│   │ GPU downsample → staging    │   │ shadow-copy their contents on  ││
│   │ async readback ring         │   │   map/unmap, update, push      ││
│   └──────────────┬──────────────┘   │ at draws into the scene depth: ││
│                  │                  │   latch view + proj from the   ││
│                  │                  │   profile's slot/offset        ││
│                  │                  └──────────────┬─────────────────┘│
│                  └────────── pair per frame ───────┘                  │
│                                   │                                   │
│   Overlay tab: status · discovery tool · live matrix readout          │
└───────────────────────────────────┼───────────────────────────────────┘
                                    │ shared memory: {frame#, depth[], view, proj, flags}
                                    ▼
┌──────────────────────────── Viewer process ───────────────────────────┐
│ GPU: linearize (from proj) → unproject → world → voxel-dedup → pool   │
│ free-fly camera · live player frustum/trail · save .ply               │
└───────────────────────────────────────────────────────────────────────┘
```

### 3.1 Depth path
- Selects the depth buffer like `generic_depth` does (most draw calls / matching the
  back-buffer size, with a manual override in the overlay).
- A GPU pass downsamples depth to a configurable size (default 480×270, float32). Low-res
  color is optional, for tinting.
- Copies into staging textures in a ring 2–3 frames deep, and reads each one back when it's
  ready so the GPU never waits on the CPU.

### 3.2 Camera path: constant-buffer sniffer
**Tracking constant buffers (D3D11):**
- `init_resource` / `destroy_resource`: keep a table of buffers created with the
  constant-buffer usage flag (size and handle).
- `map_buffer_region` + `unmap_buffer_region`: remember the mapped pointer at map time,
  and copy the contents into a CPU shadow copy at unmap time (ReShade fires this event
  before the real unmap, so the data has been written).
- `update_buffer_region` (`UpdateSubresource`): copy the incoming data into the shadow.
- `bind_descriptor_tables` / `push_descriptors`: track which cbuffer is bound to which
  slot in which shader stage.
- (D3D9 later: `push_constants` gives the constant registers directly.)

**Choosing the right moment in the frame:**
A frame usually contains several "cameras": shadow cascades, reflections, UI. The one we
want is the camera **used while drawing into the selected scene depth buffer**. On draw
events where the bound depth-stencil is the scene depth buffer, the addon reads the
profile's cbuffer (identified by stage + slot + size) from the shadow copy and latches
the matrices for this frame. The first valid latch per frame wins. The last one is available as an option.

**What gets extracted** (the profile says which layout the game uses):
- `view` + `proj` separately (best case), or
- `viewProj` + known projection, where `view = proj⁻¹ · viewProj`, or
- `invView` / `invViewProj` (common in deferred renderers), which are just inverted.
- Row-major vs column-major, and handedness, are set in the profile.
- The projection matrix also yields **near/far, FOV and the depth convention**
  (standard / reversed / infinite) automatically. That removes most manual depth tuning.
- TAA jitter (tiny offsets in the projection) is ignored at first, or stripped by zeroing
  the jitter terms.

**Discovery mode** (overlay tool, used once per game):
1. For every cbuffer bound during scene-depth draws, scan the shadow copy at
   16-byte-aligned offsets for 4×4 float windows that look like:
   - **view / invView:** upper 3×3 is orthonormal (|det| ≈ 1), with a row/column of (0,0,0,1)
   - **projection:** the characteristic zero pattern with a ±1 in the w-row/column
   - **viewProj:** not orthonormal, but its w-row/column is plausible and it changes with the camera
2. Scores each candidate over several seconds: stable when the camera is still, changing
   when it moves or turns, and consistent in both directions.
3. **Auto-validation:** reproject the depth of frame N into frame N+k using the candidate
   and measure the error. The right matrices give near-zero error on static geometry.
4. The overlay lists ranked candidates with live values. The user picks one, does the
   "360° turn" test in the viewer, then clicks **Save to profile**.

### 3.3 D3D12 / Vulkan (later)
Constant buffers in these APIs usually live in persistently mapped upload memory that
the game writes to without map/unmap calls, so there's no event to hook. The plan:
- Track upload-heap / host-visible buffers at creation, and map them from the addon.
- Resolve root constant-buffer views and descriptor bindings at scene-depth draws to
  a buffer + offset, then read that memory directly at that moment. The game may
  overwrite it later, but at draw time the content is valid for the current frame.
- Root/push constants arrive via `push_constants` and are easy.

### 3.4 Shared-memory protocol
`Local\game_lidar_frames`: a versioned header plus N slots. Each slot holds: frame#,
timestamp, depth w/h/format, `view` (4×4), `proj` (4×4), flags (paused, pose-valid, …),
depth data, and optional color. Writes never block: if the viewer falls behind, older
slots are overwritten.

### 3.5 Viewer (C++)
- A compute shader linearizes depth using `proj`, unprojects with `proj⁻¹`, transforms
  by `view⁻¹`, and appends into a GPU point pool.
- **Dedup:** a GPU voxel hash (default 5 cm cells) inserts a point only if its cell is
  empty. That caps memory and limits ghosting to one layer.
- **Free-space carving:** before each frame is added, every stored point is projected
  into it. If the camera sees clearly *past* the point at every pixel of a 3×3
  neighborhood (margin max(15 cm, 2% of distance), with sky counting as infinitely far and
  near/weapon pixels never carving), the point is deleted. Its voxel entry becomes a
  tombstone and its slot goes onto a free list for reuse. This erases ghosts of things
  that moved away once you look at that spot again, and it's conservative at
  silhouettes and grazing angles, so static geometry is left alone.
- **Budget:** a fixed pool (e.g. 50M points ≈ 800 MB VRAM). v1 stops adding when full.
- **Rendering:** point sprites colored by height or captured color, with optional eye-dome
  lighting. The live player frustum and trail are drawn on top.
- **Controls:** free-fly, follow-player, clear, save `.ply`.

### 3.6 Game profile (`profiles/<game>.toml`)
```toml
[depth]
buffer_hint   = "auto"          # or resolution / index override
near_cutoff_m = 0.5             # drop first-person hands/weapon
max_range_m   = 500

[camera]
stage   = "vertex"              # shader stage the cbuffer is bound to
slot    = 0                     # cbuffer register (b0)
size    = 1024                  # buffer size, to disambiguate
layout  = "view+proj"           # view+proj | viewproj+proj | invview+proj | invviewproj
view_offset = 0                 # bytes
proj_offset = 64
major   = "row"                 # row | column
handed  = "left"                # left | right
latch   = "first"               # first | last scene-depth draw of the frame
```

## 4. Handling the known quirks
- **Weapon/hands:** near-depth cutoff. **Sky:** discard samples near the far plane.
- **Menus, cutscenes, map screens:** pause hotkey. Auto-pause if no valid camera latch
  happens this frame (because no draws went to the scene depth buffer).
- **Resolution changes:** re-create resources. The viewer uses per-frame dimensions.
- **Buffer identity across sessions:** buffer handles change every run, so the profile
  identifies the cbuffer by *stage + slot + size*, not by handle.
- **Anti-cheat:** single-player / offline games only (addon support requires the full
  ReShade build).

## 5. Performance targets
- Game FPS impact **< 3%** at 30 captured frames/s with 480×270 depth. The cbuffer
  shadowing adds only memcpy of small buffers. Discovery mode is allowed to be slower.
- End-to-end latency from a game frame to its points in the viewer: **< 100 ms**.
- Viewer holds **60 fps** with 20M+ points.

## 6. Success criteria
1. **360° turn test:** standing still and turning draws a closed, non-smeared ring.
2. Walking a 2-minute loop live produces a coherent level with no doubled walls.
3. A second D3D11 game can be profiled via discovery mode in under 15 minutes.
4. The performance targets in §5 are met.

## 7. Out of scope (for now)
- Online / anti-cheat-protected games.
- Mesh reconstruction and textured output.
- Removing dynamic objects that are never looked at again. Carving only clears what the
  camera re-observes.
- Image-based tracking.

## 8. Open questions
- **Test game:** needs a single-player **D3D11** game with working ReShade depth.
- Viewer on the same PC only? (v1 assumes yes.)
