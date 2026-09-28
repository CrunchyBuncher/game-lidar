# Discovery

Finds a game's camera in its constant buffers when there's no profile for it. This is its own
module: the addon feeds it and shows its results, but everything that decides which candidate wins
lives here and can be built, tested, recorded and replayed without a game.

```
addon (camera_tracker samples draws, capture publishes depth)
   │  cam::DepthSamples + depth frames
   ▼
Discovery (discovery.h) ── worker thread ──▶ Analyzer (analyzer.h) ──▶ Status: ranked candidates
                                  │
                                  └──▶ Recorder (recording.h) ──▶ lidar_discovery_<game>_<date>.disc
                                                                        │
                        lidar_discover / tests  ◀── replay (replay.h) ◀─┘
```

| File | What |
|---|---|
| `input.h` | Everything the analyzer consumes: `SampleFrame` (the sampled draws and their bound buffers) and `DepthFrame` (a depth grid), each with a timestamp |
| `analyzer.h/.cpp` | The core: hypotheses, candidates, scoring, ranking, the text report. Single-threaded and deterministic: time comes from the inputs, so the same inputs always give the same result |
| `discovery.h/.cpp` | What the addon runs: the analyzer on a low-priority worker thread, plus recording |
| `recording.h/.cpp` | The `.disc` format: writer and streaming reader |
| `replay.h/.cpp` | Runs a recording through a fresh analyzer and measures it against the expected camera |
| `tools/lidar_discover.cpp` | The command-line tool (below) |
| `tests/` | `lidar_discovery_tests`: math, synthetic games, recordings, real-game regressions |

It depends only on `lidar_camera` (profiles, camera math, the model-view solver, draw data) and
`lidar_common`. Nothing in it calls ReShade.

## How it decides
1. **Scan.** Every 16-byte-aligned 4×4 window of every bound buffer is classified as a view or
   inverse view (rigid), a projection, a view-projection or its inverse, row- or column-major.
   Each (buffer, offset, kind, major) is a hypothesis with a per-frame history.
2. **Candidates.** Hypotheses in one buffer combine into complete `CameraProfile`s (view + proj,
   inverse view-projection + proj, a single view-projection, camera-relative ones with a
   translation float3, per-draw world·view for model-view games). Each one decodes exactly as a
   saved profile would.
3. **Score.** Against depth: a candidate's view must stay put when the depth image is still and
   change when it moves, and reprojecting depth frame N into frame N+k with its matrices must land
   on frame N+k's depth (error under 0.2%; the right matrices give about 0). A candidate is
   *confident* after 5+ reprojection tests with at least 90% passing.

## Iterating on a game
1. **Record in the game.** In the overlay's Discovery section, press **Record**. It restarts
   discovery and records its inputs for 30 s to `lidar_discovery_<exe>_<date>.disc` next to the
   game's exe. Move and turn the camera the whole time, and stand still for a moment too. The
   capture has to be on, since discovery checks candidates against depth. Unattended: set
   `DiscoveryAutoStart=1` and `DiscoveryRecord=<seconds>` under `[LIDAR]` in `ReShade.ini`.
2. **Replay offline** as often as you like, in seconds, without the game:
   ```powershell
   build\bin\Release\lidar_discover.exe info   rec.disc
   build\bin\Release\lidar_discover.exe replay rec.disc --expect profiles\mgs_delta.toml --progress 5 --report report.txt
   ```
   `--expect` marks the right camera in the ranking and prints when it first became the best, from
   when on it stayed the best, and when it was the confident best. The exit code is 1 if it
   doesn't end as the confident best. `--report` writes the same report as the overlay's **Write
   report**: every hypothesis, the per-draw values and the draw censuses.
3. **Change the analyzer**, rebuild `lidar_discover`, and replay again. The replay is exact:
   the recording holds what the analyzer got, in the order it got it.
4. **Keep it as a regression case.** Once the right camera is known, copy the recording to
   `tests/recordings/<name>.disc` with its profile as `<name>.toml` (see
   [tests/recordings/README.md](tests/recordings/README.md)). `lidar_discover check` and the
   `discovery.recordings` test replay every case.

## Tests
```powershell
cmake --build build --config Release --target lidar_discovery_tests
ctest --test-dir build -C Release -R discovery
build\bin\Release\lidar_discovery_tests.exe analyzer          # one group
build\bin\Release\lidar_discovery_tests.exe analyzer.camera_relative
```
| Group | Covers |
|---|---|
| `math` | The classifier, view-projection decomposition, reprojection scoring |
| `analyzer` | The whole analyzer on synthetic games (`tests/synthetic.h`): one per camera layout engines use (view + proj, view-projection, inverse view-projection + proj as in UE, camera-relative with a translation, world·view·proj per draw), each next to decoys (last frame's matrix, a spot light's camera, per-object world matrices, other constants), plus a camera that never moves and a determinism check |
| `recording` | The `.disc` round trip, cut-short files, replay matching live analysis |
| `recordings` | Real-game regressions in `tests/recordings` (passes when there are none) |

To cover a new engine pattern, add a `GameLayout` (or a decoy) to `tests/synthetic.cpp` and a case
in `tests/analyzer.cpp`. A synthetic scenario runs in well under a second. Unlike the rest of the
unit tests, these don't use the shared-memory ring, so they can run while a game is open.
