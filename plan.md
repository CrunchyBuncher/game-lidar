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
- [ ] Track cbuffer resources, and shadow-copy on map/unmap + update_buffer_region.
- [ ] Track cbuffer bindings per stage/slot.
- [ ] At draws into the scene depth buffer, latch the matrices from the profile's
      stage/slot/size/offsets. Pair them with the depth frame.
- [ ] Layout handling: view+proj, viewproj+proj, inverse variants, row/column-major,
      and handedness.
- [ ] Profile TOML loader, plus an overlay readout of the live latched matrices.
- [ ] Validate end-to-end on the **fake game** (known offsets), where it must be exact.

**Exit:** the fake game scans perfectly through ReShade using only the addon.

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
- [ ] D3D9 via `push_constants`.
- [ ] D3D12/Vulkan: track persistently mapped upload buffers, resolve root CBVs and
      descriptors at scene-depth draws, and read memory at draw time.

## M6 — Scale & extras (as needed)
- [ ] Chunked pool with eviction/streaming for huge levels. Resume saved scans.
- [ ] Network transport for a viewer on another machine.
- [ ] Engine-mod / memory-reader fallback providers for games where sniffing fails.

---

## Immediate next steps
1. Pick a single-player **D3D11** test game with working ReShade depth.
2. Start M0: CMake skeleton, `protocol.h`, D3D11 fake game, viewer.
