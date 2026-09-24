# game-lidar — Plan

See `spec.md` for the design. Everything lives in the ReShade addon (plus the viewer).
The order gets a **live scan on screen as fast as possible**, then generalizes.

## Repo layout (target)
```
game-lidar/
  CMakeLists.txt
  common/        protocol.h (shared-memory layout), math helpers, profile loader
  addon/         lidar_capture.addon64
    depth/       depth buffer selection, downsample, readback
    camera/      cbuffer tracking, shadow copies, latching, discovery
    overlay/     ReShade overlay tab
  viewer/        D3D11 live viewer
  tools/
    fake_game/   D3D11 test app: renders a scene with known matrices in cbuffers
    verify/      checks ring frames / saved .ply against the fake game's true scene
  tests/         unit tests (ring semantics, unprojection per depth convention)
  profiles/      per-game TOML
```

---

## M0 — Viewer + protocol against a fake game ✅ (2026-09-24)
Goal: the viewer is proven correct before a real game is involved.
- [x] `common/protocol.h` + `ring.h/.cpp`: versioned seqlock ring (frame#, depth, color,
      view, proj, flags).
- [x] `tools/fake_game`: **D3D11** test level, scripted path (a 360° turn, then laps).
      View/proj live in cbuffer b0 and a per-object cbuffer in b1 acts as a decoy.
      Supports standard, reversed and reversed-infinite depth.
- [x] Viewer: compute-shader unproject (via `inverse(proj)`, so it's convention-agnostic),
      GPU voxel-hash dedup, indirect-draw point sprites, free-fly/follow camera,
      player frustum + trail, height/color modes, save `.ply`.
- [x] `lidar_verify` + unit tests.

**Result:** the ring path (CPU unprojection) is exact in all 3 depth modes: max error
≤ 0.3 mm. A 30 s viewer scan (GPU path) gave 4.27M points, all within 0.3 mm. The viewer
runs at 120 fps (vsync).

### M0.5 — Free-space carving ✅ (2026-09-24)
- [x] GPU carve pass before each ingest (3×3 conservative test), hash tombstones, and a free
      list for slot reuse. Toggle with `X` / `--no-carve`.
- [x] A/B on a 90 s run with the moving NPC: ghost points (> 2 cm from static scene)
      went from 128,523 to 313. The total drop (6.65M → 6.52M) matches the removed ghosts, so
      no static geometry was lost. Viewer still at 120 fps.

## M1 — Addon: depth out of a game
Test first on the **fake game with ReShade injected**, then on the real game.
- [x] Addon skeleton (CMake, ReShade addon headers) and overlay tab. SDK pinned to ReShade
      6.8.0 (API 20). Test rig: `sandbox/fake_game/` with ReShade installed, run with `--no-publish`.
- [x] Depth buffer selection (`generic_depth` heuristics: draws/vertices per depth-stencil,
      frame-size fit, deferred-context merging), with a manual override in the overlay.
      Our own tracker, not the built-in one: M2 needs the real depth-stencil at draw time.
- [x] Copy → GPU downsample → staging ring → non-blocking readback → shared memory.
      Until M2, frames go out without a pose, using a projection built from overlay settings
      (FOV / near / far / depth mode). The viewer shows them as a live camera-relative snapshot.
- [x] `lidar_verify addon`: the addon's frames vs. an un-injected reference fake_game frozen at
      the same camera. **Bit-identical depth and 0.0 mm error in all 3 depth modes.**
      This caught a real bug: the protocol's pixel mapping was computed in float, so compilers
      could disagree at exact boundaries. It now uses exact integer math in every producer and consumer.
- [ ] Measure FPS impact (on the real game; fake_game is vsync-capped).
- [ ] Run on the real test game.

**Exit:** the viewer shows live unprojected depth (camera-relative, no world pose yet).

## M2 — Addon: constant-buffer sniffer (manual profile)
- [x] API seams, so M5 only adds implementations: `DepthCapture` (depth readback) and
      `CbufferSource` (cbuffer bytes at a draw), created per device API in `addon/backends.cpp`.
      The depth tracker, camera tracker, profile decoding, pairing and ring are shared.
- [x] D3D11 source: shadow copies of every cbuffer (init data, Unmap, UpdateSubresource on
      immediate and deferred contexts), bindings per stage/slot from `push_descriptors`.
- [x] At draws, latch the profile's byte window **per depth-stencil** (first or last draw).
      At present, the latch for the depth-stencil actually captured becomes the frame's pose,
      so selection and manual overrides pair with the same frame.
- [x] Layouts: view+proj, viewproj+proj, invview+proj, invviewproj+proj; row/column-major.
      Decoding in double; view+proj passes the game's floats through bit-exact. Handedness
      doesn't affect unprojection, so it's parsed and checked against the projection only.
- [x] Profile TOML loader (`[LIDAR] Profile=`, default `lidar_profile.toml` next to the exe),
      overlay readout of the latched matrices and the near/far/FOV/depth convention they imply.
- [x] Validated on the **fake game** (`profiles/fake_game.toml`):
      `lidar_verify addon` is bit-identical in all 3 depth modes: depth, proj **and view** 0 diff.
      `lidar_verify ring` matches fake_game's own publishing in every mode (reversed modes
      max ≤ 0.3 mm; standard depth p99 ≈ 3 mm, max ≈ 12 mm for both: float32 standard-depth
      precision, not the pose). `viewproj+proj` gives the same results.
- Known limits: a deferred-context Map is shadowed at record time, not execute time. TAA
  jitter isn't stripped. Every Unmap copies the whole cbuffer; measure on a real game (M4).

**Exit:** the fake game scans perfectly through ReShade using only the addon. ✅ (2026-09-24)

## M3 — Discovery mode → first real game scan
- [ ] Candidate scanner: 4×4 windows in cbuffers bound during scene-depth draws,
      classified as view / proj / viewProj / inverse.
- [ ] Temporal scoring (still vs moving) and ranked candidate list in the overlay.
- [ ] Reprojection auto-validation (depth frame N → N+k error).
- [ ] "Use candidate" (live preview in the viewer) and "Save to profile".
- [ ] Run it on the real test game and write its profile.

**Exit:** the 360° turn test passes in a real game. **First live scan.** 🎉

## M4 — Make it pleasant
- [ ] Near/far cutoffs, pause hotkey, and auto-pause when no latch happens.
- [ ] Low-res color capture for tinting. Height-ramp default. Eye-dome lighting.
- [ ] Follow-player camera, trail, clear/reset. Handle resolution changes.

## M5 — More APIs
- [x] **D3D12** (2026-09-24), ahead of M3.
  - `fake_game --api d3d12`: same scene, camera and depth modes, render only. Camera and
    per-object constants in one persistently mapped upload buffer, a region per frame,
    3 frames in flight. Root CBVs (b1 at parameter 0, b0 at 1; root signature 1.1), or
    `--cbv-tables`: one table [b1, b0 appended] copied every frame from a CPU-only heap
    (root signature 1.0). Depth ends each frame in PIXEL_SHADER_RESOURCE. `--d3d12-debug`
    turns on the debug layer and exits 3 on any error.
  - `D3D12CbufferSource`: upload buffers + Map pointers, root signature layouts, CBV
    descriptor shadows (creates and copies), per-command-list root CBVs / tables. Records at
    the draw, reads in `resolve()` at submit. Profiles gained `space`.
  - `D3D12Capture`: ReShade API only (immediate command list, no events), depth state tracked
    from the game's barriers and restored, fenced readback ring.
  - **Result:** `lidar_verify ring --frames 60` passes in all 3 depth modes for both binding
    modes with the same accuracy as D3D11 (standard max 0.3 mm, reversed ≤ 0.1 mm).
    `lidar_verify addon` against the un-injected D3D11 reference is bit-identical (depth,
    proj and view 0 diff) in all 6 combinations. The debug layer reports no errors while
    capturing. Forcing the addon to assume DEPTH_WRITE makes it report state mismatches,
    so the state tracking is needed and is working.
  - Known limits: only graphics bindings are tracked (no compute or bundles). Custom heaps
    with CPU access aren't tracked, and neither are enhanced-barrier layouts beyond what
    ReShade maps to states. D24S8/D32S8 depth copies are untested on D3D12. Descriptor
    shadows grow to the highest index written. Measure the cost of the descriptor events on
    a real D3D12 game.
- [x] Unsupported APIs are harmless (2026-09-24): until a D3D9/D3D11/D3D12 device appears, the addon
      registers only `init_device` (registering events changes how ReShade hooks D3D9), logs
      "Inactive" and does nothing else. `d3dcompiler_47.dll` is delay-loaded. Checked once with
      a throwaway D3D9 test (renders identically with and without the addon, "Inactive" logged).
      Found via A Hat in Time, whose "DX12" mode is D3D9On12.
- [x] **D3D9** (2026-09-24). Reverses the earlier "not D3D9" decision: many older games are D3D9.
  - `fake_game --api d3d9`: same scene, camera and depth modes, render only. Camera in VS
    constants c0-c12 (same bytes as b0, so the same profile works), decoy c13-c17 per draw.
    Auto depth-stencil, or `--own-depth`. The half-pixel offset is corrected in the VS. Runs from
    `sandbox/fake_game_d3d9/` with ReShade as `d3d9.dll`.
  - `d3d9::` constants source: shadows the VS/PS float register files from `push_constants`.
  - `D3D9Capture`: INTZ replacement at `create_resource`, ps_3_0 downsample, event query +
    GetRenderTargetData readback, state block save/restore, targets dropped before Reset.
  - Backends now register per API, so a D3D9 game doesn't get D3D11's map hooks (and vice versa).
  - **Result:** `lidar_verify ring --frames 60` passes in all 3 depth modes, with the auto and own
    depth-stencil (standard max 0.3 mm, reversed 0.1 mm, same as D3D11); the viewproj profile
    passes too (max 1 mm). `lidar_verify addon` against an un-injected D3D11 `--d24` reference:
    view and proj match exactly, and depth is bit-identical in standard mode. In reversed modes ~5%
    of pixels are one 24-bit step apart (rasterizer rounding), within `--depth-tol 6e-8`. Three
    window resizes (Resets) in a row keep capturing, and the game's Reset never fails.
  - Known limits: MSAA depth can't be INTZ (error in the overlay). Fixed-function games
    (`SetTransform`) have no shader constants. Depth buffers created before the addon loaded
    stay unreadable. Not yet measured: the addon's CPU cost on a real D3D9 game
    (GetRenderTargetData may sync), and D3D9On12 / D3D9Ex.
- [ ] Vulkan: same shape as D3D12 (host-visible memory, descriptor sets, pipeline layouts).

## M6 — Scale & extras (as needed)
- [ ] Chunked pool with eviction/streaming for huge levels. Resume saved scans.
- [ ] Network transport for a viewer on another machine.
- [ ] Engine-mod / memory-reader fallback providers for games where sniffing fails.

---

## Immediate next steps
1. Pick a single-player **D3D11** test game with working ReShade depth.
2. Start M0: CMake skeleton, `protocol.h`, D3D11 fake game, viewer.
