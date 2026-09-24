# game-lidar — Model-view camera recovery (plan)

A camera source for games that never upload the camera alone: every draw gets its object's
world matrix already multiplied into the view (**model-view**, `WV`) or into view and projection
(**model-view-projection**, `WVP`). Found on Sonic Adventure 2 (see below); common in D3D9 games
and in engines ported from OpenGL-style fixed-function stacks (Ninja, most pre-2005 console ports).

It sits next to the M2/M3 cbuffer latch as its own component: the tracker keeps latching as
before, this adds a per-draw capture and a solver that turns it into a view per frame. The
depth path, ring, viewer and profile loader are reused. See `plan.md` for the milestones this
branches from.

## Why: what SA2 showed (2026-09-24)
The M3 discovery report (`lidar_discovery.txt`, per-draw snapshot) on SA2, a level, camera moving:
- VS c8-c11: a constant projection (FOV 55.4°, near 1, infinite far, standard depth).
- VS c0-c3: a column-major rigid matrix that is **different on every draw** (24+ distinct values
  over 48 sampled draws). Rotations share the camera's pitch and differ by 90° yaw steps, and
  translations are all different: level pieces placed around the world, each with its own world
  matrix. Only the first few draws (sky/HUD) have identity.
- VS c12-c15: the inverse of c0 (the camera's position in each object's space).
- No draw has world = identity, so no latch (first/last/common) can ever return the view. Every
  M3 candidate scored 0 on reprojection, correctly.

## The idea
Column-vector convention (as SA2 stores it): `M_i(t) = V(t) · W_i` for object *i* at frame *t*.
For a **static** object `W_i` is fixed, so

    M_i(t1) · M_i(t0)⁻¹ = V(t1) · V(t0)⁻¹

is the camera's motion, independent of the object, and even if `W_i` scales. Keeping a world
placement `W_i` for each static object gives an absolute view per object, `V = M_i · W_i⁻¹`, and
static objects all agree on it. Moving things (the player, enemies, particles, the sky, which
follows the camera) disagree and get rejected by consensus.

The world frame is ours, not the game's: the placement of the first anchor object. Choosing the
most-drawn static object (level geometry) as the anchor keeps "up" the game's up in practice,
since level pieces are usually yaw-only. A point cloud only needs a consistent frame across frames.

For `WVP`, the constant projection is split off first: `decompose_view_proj` already splits a
view-projection whose view part is rigid; with a scaling world, fall back to a projection found
from the unscaled draws of the same frame (or the overlay FOV).

## Solver (per captured frame)
(Column-vector notation as above; `addon/modelview` works in `matrix.h`'s row vectors, `M = W · V`.)
Input: for each draw into the captured depth-stencil, an **object key** and its matrix `M`.
1. Look up each key's placement `W`. For each known static object: `V_i = M · W⁻¹`.
2. Consensus: the largest cluster of agreeing `V_i` (rotation angle < ~0.05°, translation relative
   to scene scale < ~1e-4) is the frame's view. Median or least-squares inside the cluster.
   Needs ≥ 3 inliers or ≥ 30% of known objects, whichever is larger, else: no pose this frame.
3. Objects outside the cluster: strike counter; after N strikes they're dynamic and never used
   again in that segment (a moving platform that stops is only re-placed in a later segment; MV1
   keeps it that simple, revisit if a game needs it).
4. New keys: provisional placement `W = V⁻¹ · M`. They get promoted to static after they've agreed
   for K frames. Provisional placements are averaged over those frames, then frozen (see MV1).
5. No consensus for many frames and no known key seen (level change, a cutscene cut, a load):
   start a **new segment**: forget all placements and re-anchor. Frames carry a segment id so the
   viewer can clear, or keep separate scans.
6. Bootstrapping (first frame / new segment): the anchor is the heaviest draw (most vertices);
   its placement is its own scale only (the view is its model-view with the scale stripped).

**Matching without keys** (fallback, if the draws can't be identified, see open questions):
successive frames' matrix sets differ by one common motion `D`, so RANSAC over pairs
`D = M_b(t1) · M_a(t0)⁻¹` finds the `D` most matrices agree on. It accumulates drift, so keys
come first.

## Object keys
What stays the same for an object across frames, readable at the draw:
- the vertex buffer(s) and index buffer bound, plus the draw's `first_index / vertex_offset /
  index_count` (or `first_vertex / vertex_count`);
- the pipeline / vertex shader (optional, splits keys that share geometry).

A key drawn more than once per frame with different matrices (rings, repeated props, instanced
trees) is ambiguous and is ignored for that frame. Buffers the game recreates or streams
(dynamic VBs that get refilled each frame) make useless keys: detected by a key never repeating,
and those draws are dropped.

### What ReShade 6.8.0 reports for D3D9 (answered 2026-09-24, from `source/d3d9/d3d9_device.cpp`)
- `DrawPrimitive` → `draw(vertex_count, 1, StartVertex, 0)`; `DrawIndexedPrimitive` →
  `draw_indexed(index_count, 1, StartIndex, BaseVertexIndex, 0)` (MinVertexIndex/NumVertices are
  dropped). Buffers come from `SetStreamSource` / `SetIndices` → `bind_vertex_buffers` /
  `bind_index_buffer`, handle = the native pointer.
- `DrawPrimitiveUP` → `draw(vertex_count, 1, 0, 0)`; `DrawIndexedPrimitiveUP` →
  `draw_indexed(index_count, 1, 0, 0, 0)`. Buffer events only fire if some addon registers
  `bind_vertex_buffers` / `bind_index_buffer`: ReShade then copies the data into **one shared
  stand-in buffer** (same handle for every UP draw, offset 0), with a `map_buffer_region` event whose
  data pointer is the game's. After the draw it binds null (strides = nullptr). So an UP draw's
  handle is no key: only its count and, via the pointer, the game's data.
- **Bug:** `resize_primitive_up_buffers` creates the stand-in *index* buffer into
  `_primitive_up_vertex_buffer`, so with those events registered every `DrawIndexedPrimitiveUP`
  creates (and leaks) a D3D9 buffer, and the index map fails. A 32-bit game would run out of memory.
  **Never register `bind_vertex_buffers` / `bind_index_buffer` on D3D9.**
- What we do instead (the diagnostic, and the plan for MV2's D3D9 keys): read the bindings back from
  the driver device at the draw (`GetStreamSource(0)`, `GetIndices`, `GetVertexShader`), and hook the
  driver device's vtable slots 83/84 (`Draw[Indexed]PrimitiveUP`) to get the game's pointers and hash
  its vertex/index data. Whether a draw is UP is only known when the hook runs (stream 0 still holds
  the last buffer until the first UP call unbinds it), so it is completed at the next draw.
  D3D11/D3D12 have no such bug; MV2 can use the bind events there.

### SA2 key diagnostic (discovery report, 2026-09-24)
Every 30th frame while discovering is a **census**: every draw into the captured depth-stencil
with its call, geometry (VB+offset/stride, IB, VS, or UP pointer + data hash) and VS c0-c3.
The report lists the latest census and, per pair of censuses 30 frames apart, the objects drawn
once in both frames under two key variants (UP by pointer / by data hash), each object's
`D = M(A)⁻¹·M(B)` in both majors, and the largest set agreeing on one `D` (< 0.05°, < 0.01 + 1e-4·|t|).
Checked on fake_game D3D9, with and without the new `--up` (alternating `DrawPrimitiveUP` /
`DrawIndexedPrimitiveUP`): 100% consensus, the UP hook fills pointers and hashes.
- [x] **SA2 result** (city level, camera moving; censuses at frames 9041 / 9071 / 9101):
  - ~900 draws per frame: **~90% `DrawPrimitive`** from per-object static vertex buffers (stride
    24, first vertex 0: one VB per mesh piece), ~88 `DrawPrimitiveUP` (HUD, billboards, small
    quads), no indexed draws at all. c0-c3 is rigid column-major on 93% of draws.
  - Keys repeat across frames: ~420 objects drawn exactly once in both frames 30 apart, another
    ~155 keys per frame drawn several times with different matrices (repeated props: ambiguous).
  - **Consensus: 87-88% of matched objects agree on one camera motion** (content keys; column-
    major; 0.675° + 24 units, and 0.285° + 44.5 units over 30 frames). Keying UP draws by the
    game's pointer instead of a data hash matches more UP draws but only 79-80% agree: SA2
    reuses UP pointers for different data.
  - The outliers are the expected ones: 5 draws whose motion is the identity (a camera-following
    sky/backdrop), ~17 stride-32 draws with wildly different motions (Sonic's body parts), ~20 UP
    quads fixed to the screen (HUD). The level geometry is all inliers.
  - Row-major reading also "agrees" (64-73%) but only on rotation, with zero translation: the
    solver must compare translations too, and takes the major from the profile.

**Key decision:** the solver takes an opaque 64-bit key. The capture layer hashes the draw call
(type, count, first, base vertex, instances), stream 0 (buffer + offset + stride), index buffer,
vertex shader, and for UP draws the **hash of the data** (not the pointer). The draw's vertex/index
count also goes in as a weight (bootstrap prefers big objects: level pieces, not HUD quads).

## Pieces
- **Per-draw capture** (camera tracker): with a model-view profile, for every draw into each
  depth-stencil, record `{key, 64-byte window}` (the `M` window; the proj window once per frame).
  ~1000 draws × ~100 bytes per frame; the solver runs on the discovery-style worker, not at the draw.
- **Solver** (`addon/modelview.{h,cpp}`): pure, no ReShade: frames of `{key, M}` in, view per frame
  plus stats out. Unit-tested with synthetic scenes.
- **Profile**: new layouts `modelview` (per-draw `view_offset`, constant `proj_offset`) and
  `modelviewproj` (single per-draw matrix). `latch` doesn't apply. Tuning keys (inlier
  thresholds) with defaults, only if needed.
- **Pairing / ring**: the solved view is paired with the frame's depth like a latched one; frames
  without a pose go out without a pose, as today. Add the segment id (protocol version bump).
- **Overlay**: known / static / dynamic objects, inliers this frame, segment, solve time.
- **Discovery**: a per-draw window that's rigid (or affine) in most draws and different on most of
  them, next to a constant projection, becomes a `modelview` candidate; a per-draw decomposable
  view-projection with no common value becomes `modelviewproj`. Scored with the same
  reprojection test, using views from a solver run on the sampled draws (samples gain keys).
- **fake_game**: `--camera-layout modelview` (per-draw `V·W` + constant `P`) and
  `--camera-layout mvp-pieces` (per-draw `P·V·W` with no identity world): the level split into
  translated, yawed, some scaled pieces; the NPC moving; a repeated prop drawn several times; a
  sky that follows the camera. D3D9 and D3D11. (`--api d3d9 --up` already draws with
  `Draw[Indexed]PrimitiveUP`: the UP key path.)
- **lidar_verify**: `--align`: solve the rigid transform between our world and the true one from
  the first posed frame, then check every later frame with it (so drift shows as error).

## Milestones
### MV1 — Solver core ✅ (2026-09-24)
- [x] `modelview.{h,cpp}` with the steps above; unit tests on synthetic scenes: static pieces
      (with scale), moving objects, a camera-following sky, ambiguous repeated keys, a segment cut,
      float noise at SA2 scale (coordinates ~2000 units).
- Decisions made while building it:
  - `mv::Draw{key, M, weight}`: opaque 64-bit key (see Object keys), M in row-vector form
    (the profile's major decides the load), weight = the draw's vertex/index count.
  - Consensus: up to 48 seeded hypotheses (statics first), votes weighted (static 2, provisional 1),
    agreement = translation within 1e-4 × the frame's scene scale (median object distance) and
    rotation within 0.05° (Frobenius test, no acos). The view is the mean of the seed's inliers
    (statics only if any), re-orthonormalized, then inliers are recounted against it. A pose needs
    ≥ 3 inliers and ≥ 30% of the votes present.
  - Placements: provisional ones are averaged over the frames they agree; after 10 they're
    **frozen** (static). Freezing is what stops drift: a refined-forever map can random-walk as a
    whole. A static object that disagrees 3 frames in a row becomes dynamic for good (in its segment).
  - Anchor: the heaviest draw; its model-view with the rows of the 3x3 normalized (strips the
    object's scale, so views stay rigid) is the segment's first view. With yaw-only level pieces
    our "up" is the game's.
  - New segment: when no usable (non-dynamic) known object is drawn and ≥ 3 unknown keys are,
    or after 30 frames without a pose. Known-dynamic draws alone (HUD, player) don't start one, so
    a pause screen doesn't wipe the map.
  - Non-static objects unseen for 300 frames are dropped (SA2's UP data hashes include keys that
    never repeat).
- **Result:** 64 pieces (some scaled uniformly or not, 90° yaw steps), an NPC, a camera-following
  sky, a HUD quad, a prop drawn 3× per frame, 5 never-repeating particles per frame; float-rounded
  matrices with view distances up to ~3000 units. 10,000 frames (a still start, then 16 laps):
  one segment, every frame posed, max camera position error **2.7e-8** relative to 2000 units
  (last lap 2.3e-8: no drift), rotation 8e-7°. NPC, sky (static while the camera stood still,
  then struck out), HUD all dynamic; the prop never placed; all 64 pieces static; particles evicted.
  A level change (all new keys, scaled anchor) starts segment 2 on its first frame, with the same
  accuracy and a rigid view that keeps the game's up. ~25 µs per frame at ~40 draws.
**Exit:** synthetic view error < 1e-5 relative, no drift over 10k frames on a looping path. ✅

### MV2a — Testable slice (2026-09-24)
Fastest path to "Use a candidate in SA2 and see the scan", ahead of the rest of MV2/MV3:
- [x] `modelview` profile layout (`view_offset` = per-draw world * view, `proj_offset` = constant
      projection; latch ignored). The addon latches the projection, the tracker records every draw
      into each depth-stencil (key parts + the view window), and `mv::Solver` runs at present on
      the captured depth-stencil's draws. Overlay: segment, draws, known/agreeing objects, static /
      provisional / dynamic, solve time. ReShade.log gets a line whenever the camera status changes.
- [x] Discovery: while discovering, every draw is recorded (D3D9), window at vertex c0. A rigid
      window that differs per draw next to a constant projection becomes a `modelview` candidate
      with its own solver; its solved views go through the same temporal + reprojection scoring,
      so Use / Save / auto-save work as for any candidate. Censuses for the report are now taken
      from those records every 30 frames.
- [x] `fake_game --api d3d9 --camera-layout modelview`: 24 level parts with their own placements,
      the heaviest at the origin (so the solved world is the true one), `profiles/fake_game_modelview.toml`.
- **Result (sandbox):** discovery with no profile ranks `modelview at vertex c0 per draw / c4` first,
  reprojection 274/274 (error 0.0000), confident, auto-saved. With the profile,
  `lidar_verify ring --frames 60` passes: reversed depth p99 0.5 mm / max 1.5 mm, standard p99
  2.9 mm / max 11.8 mm (standard is float-limited, as with `separate`).
- Limits of the slice: D3D9 only (the only source that reads draw geometry); discovery only tries
  the vertex c0 window; no segment id in the ring yet (after a new segment, clear the viewer).
- [ ] SA2: discovery → Use → 360° turn.

### MV2 — Capture + profile + fake game
- [ ] Draw keys in the tracker (draw arguments; D3D9: device readback + the UP hook, never the
      bind events; D3D11/12: bind VB/IB/pipeline events), per-draw capture.
- [ ] `modelview` / `modelviewproj` profiles, pairing, segment id in the ring, overlay readout.
- [ ] fake_game modes, `lidar_verify --align`.
**Exit:** `lidar_verify ring --align --frames 600` passes on D3D9 and D3D11 for both layouts
at the same accuracy as `separate` (max ≤ 1 mm), with the NPC on.

### MV3 — Discovery
- [ ] Detect and score both layouts; Use / Save as with M3 candidates.
**Exit:** discovery auto-saves a working profile on fake_game in both modes, with no profile.

### MV4 — SA2
- [ ] Discovery → profile → 360° turn test in SA2 (the M3 exit, via this path).
- [ ] Measure the per-draw capture's cost on a real game.

## Open questions / risks
- ~~**D3D9 `Draw*PrimitiveUP`**: does ReShade report a vertex buffer for UP draws?~~ Only a shared
  stand-in, and registering for it leaks (see Object keys). Keys for UP draws come from our own
  hook: the game's pointer and/or a data hash. Which of the two is stable in SA2: the census answers.
- Hashing UP data costs CPU per draw (up to 256 KB each): measure in MV4, and fall back to
  pointer + count if the pointers turn out stable.
- Skinned meshes (bones in constants): not a single object matrix; they fail consensus as dynamic.
- Particles and billboards built in view space: dynamic, rejected.
- Reflection / shadow passes: only draws into the captured scene depth-stencil count.
- Hardware instancing (world matrix in an instance stream): the constant then is `V` or `VP`,
  which M3 already finds.
- Scaled world matrices with `WVP` and no unscaled draw in a frame: the projection must come from
  earlier frames or the overlay FOV.
- A camera cut to a place with no known objects looks like a level change: a new segment, and
  the scan continues in a disconnected frame. Acceptable; could be merged later by re-seeing
  known objects.
