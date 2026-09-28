// Replays a recording through a fresh analyzer and measures how discovery did, optionally against
// the camera it should have found. lidar_discover and the recording regression tests both use it.
#pragma once
#include <cstdint>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>

#include "analyzer.h"

namespace lidar::disc {

struct ReplayOptions {
    std::optional<CameraProfile> expected;  // the right camera (a game's profile)
    double progress_every = 0;              // > 0: a status line to `log` every this many recorded seconds
    std::ostream* log = nullptr;
    std::ostream* report = nullptr;         // the full report, at the end
    size_t candidates = 64;                 // in the final status
};

struct ReplayResult {
    bool ok = false;     // the recording opened and replayed (maybe only up to a cut, see error)
    std::string error;   // why it didn't open, or that it was cut short
    std::string note;    // the recording's
    bool registers = false;
    double seconds = 0;  // recorded time
    uint64_t sample_frames = 0, depth_frames = 0;
    double wall_seconds = 0;  // replay time
    Status final;

    // With an expected camera (input time, seconds from the first input; -1: never):
    int expected_rank = -1;             // 0-based rank at the end, -1 if not a candidate
    double expected_first_top = -1;     // first time it was the best candidate
    double expected_top_since = -1;     // it stayed the best from then to the end
    double expected_confident_at = -1;  // first time it was the best and confident
    // The verdict: it ends as the confident best candidate.
    bool found() const {
        return expected_rank == 0 && !final.candidates.empty() && final.candidates.front().confident;
    }
};

ReplayResult replay(const std::filesystem::path& recording, const ReplayOptions& options = {});

}  // namespace lidar::disc
