// Regression cases from real games: every <name>.disc in tests/recordings with the game's profile
// next to it as <name>.toml must end with that camera as the confident best candidate. Recordings
// aren't in git (they're large, see tests/recordings/README.md), so this passes when there are none.
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <vector>

#include "replay.h"
#include "test.h"

using namespace lidar;

TEST_CASE(recordings, real_games) {
    const std::filesystem::path dir = LIDAR_DISCOVERY_RECORDINGS;
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec))
        if (e.path().extension() == ".disc" && std::filesystem::exists(std::filesystem::path(e.path()).replace_extension(".toml")))
            files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) std::printf("  no recordings with a profile in %s\n", dir.string().c_str());
    for (const std::filesystem::path& f : files) {
        Profile p;
        std::string error;
        const bool loaded = load_profile(std::filesystem::path(f).replace_extension(".toml"), p, error) && p.has_camera;
        EXPECT(loaded);
        if (!loaded) continue;
        disc::ReplayOptions o;
        o.expected = p.camera;
        const disc::ReplayResult r = disc::replay(f, o);
        std::printf("  %s: %s, rank %d, confident best at %.1f s (%.1f s recorded, replayed in %.1f s)\n",
                    f.filename().string().c_str(), r.found() ? "found" : "NOT FOUND", r.expected_rank + 1,
                    r.expected_confident_at, r.seconds, r.wall_seconds);
        EXPECT(r.ok);
        EXPECT(r.found());
    }
}
