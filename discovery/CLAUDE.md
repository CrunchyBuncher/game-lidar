# Discovery: working with .disc recordings

Notes for an LLM session working on discovery. The human-facing docs are in README.md.

## What a .disc is
A recording of everything the analyzer received during a live session in a game: sampled draws
with their bound constant buffers, and depth grids, each with a timestamp. Replaying one runs the
exact same analysis offline, and gives the same result every time. The format is in `recording.h`.

## Where they are
- New recordings: `lidar_discovery_<exe>_<date>.disc` next to the game's exe. The user makes
  these with **Record** in the overlay, or with `DiscoveryRecord=<s>` in ReShade.ini.
- Regression cases: `discovery/tests/recordings/<name>.disc` next to `<name>.toml`, the game's
  verified profile, which is the expected camera. `*.disc` is git-ignored, so the folder may be
  empty on a fresh checkout. Ask the user for recordings instead of assuming they exist.
- Never launch, record in, or modify the user's real games yourself. For a fresh recording, use
  `sandbox/fake_game` (below) or ask the user to make one.

## Build (PowerShell only, Git Bash breaks MSBuild)
```powershell
$cmake = "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
& $cmake --build build --config Release --target lidar_discover lidar_discovery_tests
```

## Loading a recording from the command line
```powershell
build\bin\Release\lidar_discover.exe info   <file.disc>        # note (game, API, settings), duration, frames, buffers
build\bin\Release\lidar_discover.exe replay <file.disc> --expect profiles\<game>.toml --progress 5 --top 10 --report out.txt
build\bin\Release\lidar_discover.exe check  [<dir>]            # every .disc with a .toml; default tests/recordings
```
`replay --expect` exits 1 if the expected camera doesn't end as the confident best candidate.
`--report` writes the full analyzer report: every hypothesis, per-draw values and draw censuses.
That report is the first place to look when a candidate is missing or mis-ranked.

## Loading a recording from code (tests, experiments)
Link `lidar_discovery`. To measure against an expected camera, use `replay()`:
```cpp
#include "replay.h"
disc::ReplayOptions o;
Profile p; std::string err;
load_profile("profiles/mgs_delta.toml", p, err);
o.expected = p.camera;                                   // optional
disc::ReplayResult r = disc::replay("rec.disc", o);      // r.ok, r.error, r.final (Status), r.found(),
                                                         // r.expected_rank, r.expected_confident_at, ...
```
To feed the analyzer yourself, read the inputs one by one:
```cpp
#include "recording.h"
disc::RecordingReader reader;
std::string err;
if (!reader.open("rec.disc", err)) { /* not a recording, or an old version */ }
disc::Analyzer a(reader.registers());
disc::RecordedInput in;
while (reader.next(in)) in.is_depth ? a.add_depth(in.depth) : a.add_samples(in.samples);
// reader.error(): non-empty if the file was cut short (what came before still loaded)
disc::Status s = a.status(64);                           // best 64 candidates
```
Scope the `RecordingReader` so it closes before you delete the file. On Windows, deleting an
open file throws, and the test process aborts with 0xC0000409 (see `read_back()` in
`tests/recording.cpp`).

## Making a recording without a real game
Record in the sandbox test game with the addon. It's safe, but first check that no game using the
addon is running, because they share the frame ring. Back up `sandbox\fake_game\ReShade.ini`, add
`DiscoveryAutoStart=1` and `DiscoveryRecord=15` under `[LIDAR]`, and run
`fake_game.exe --api d3d12 --no-publish --duration 22 --background` from that folder. Launch it
through `ProcessStartInfo` with `UseShellExecute=false` and `CreateNoWindow=true`, never
`Start-Process`, so it can't take the user's focus. Restore the ini afterwards. The expected
camera is `profiles/fake_game.toml`.

## Adding a case without a recording
Synthetic games in `tests/synthetic.h` generate the same inputs a recording holds
(`std::vector<disc::RecordedInput>`). Add a `GameLayout` or decoy there and a test in
`tests/analyzer.cpp`. `tests/recording.cpp` shows how to write them to a .disc with `Recorder`.

## Gotchas
- A recording only replays exactly from discovery's start. **Record** restarts discovery for that
  reason. Don't add a way to start recording mid-session without accounting for the missing
  history.
- Changing what the analyzer consumes (`input.h`) means changing the format: bump `kVersion` in
  `recording.cpp`. Old files then fail to open with a clear message instead of misreading.
- A recording keeps depth as the downsampled grid, not raw frames, so changes to
  `DepthGrid::build` don't show up in old recordings.
- Discovery tests don't touch the shared-memory ring, so they can run while a game is open.
  `lidar_tests` and e2e can't.
- `replay --expect` wants the exact place. `profiles/mgs_delta.toml` reads UE's previous-frame
  matrix (@ 1680): discovery rightly ranks it below the current one (vertex b0 viewproj @ 64), so
  replaying MGS recordings against that profile reports NOT FOUND.
- Before blaming the scoring, check the README's "What trips it up" table: garbage depth frames,
  scene motion, last-frame copies, degenerate projections and candidate floods each have a known
  signature in the report (`not depth`, `explains`, `LAST-FRAME`, `retired`).
