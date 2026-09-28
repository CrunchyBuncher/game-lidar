// The whole analyzer on synthetic games: it must rank the right camera first, and be confident about
// it, for each way engines lay their camera out, next to the decoys real games have.
#include <cstdio>
#include <string>

#include "analyzer.h"
#include "synthetic.h"
#include "test.h"

using namespace lidar;
using namespace lidar::test;

namespace {

disc::Status run(const Scenario& s, size_t candidates = 8) {
    disc::Analyzer a(false);
    for (const disc::RecordedInput& in : generate(s)) {
        if (in.is_depth)
            a.add_depth(in.depth);
        else
            a.add_samples(in.samples);
    }
    return a.status(candidates);
}

void print_top(const char* name, const disc::Status& st, const CameraProfile& want) {
    std::printf("  %s: %s\n", name, disc::status_line(st).c_str());
    for (size_t i = 0; i < st.candidates.size() && i < 5; ++i)
        std::printf("    #%zu %s%s\n", i + 1, disc::candidate_line(st.candidates[i]).c_str(),
                    disc::same_camera(st.candidates[i].profile, want) ? "  <- expected" : "");
}

// The expected camera is the confident best candidate.
bool finds(const char* name, const Scenario& s) {
    const disc::Status st = run(s);
    const CameraProfile want = expected_camera(s);
    const bool ok = !st.candidates.empty() && disc::same_camera(st.candidates.front().profile, want) &&
                    st.candidates.front().confident;
    if (!ok) print_top(name, st, want);
    return ok;
}

}  // namespace

TEST_CASE(analyzer, view_and_proj) {
    for (bool col : {false, true})
        for (int mode : {0, 2}) {
            Scenario s;
            s.layout = GameLayout::ViewAndProj;
            s.column_major = col;
            s.depth_mode = mode;
            EXPECT(finds(("view+proj col " + std::to_string(col) + " mode " + std::to_string(mode)).c_str(), s));
        }
}

TEST_CASE(analyzer, view_proj) {
    Scenario s;
    s.layout = GameLayout::ViewProj;
    EXPECT(finds("viewproj", s));
}

TEST_CASE(analyzer, inv_view_proj_and_proj) {
    Scenario s;
    s.layout = GameLayout::InvViewProjAndProj;
    EXPECT(finds("invviewproj+proj", s));
}

TEST_CASE(analyzer, camera_relative) {
    Scenario s;
    s.layout = GameLayout::CameraRelative;
    EXPECT(finds("camera-relative viewproj + translation", s));
}

TEST_CASE(analyzer, world_view_proj) {
    Scenario s;
    s.layout = GameLayout::WorldViewProj;
    EXPECT(finds("world*view*proj per draw", s));
}

// Without the decoys too: the ranking mustn't depend on having something to beat.
TEST_CASE(analyzer, no_decoys) {
    Scenario s;
    s.decoys = false;
    EXPECT(finds("view+proj, no decoys", s));
}

// A camera that never moves can't be validated: nothing may claim confidence.
TEST_CASE(analyzer, still_camera) {
    Scenario s;
    s.moving = false;
    s.frames = 200;
    const disc::Status st = run(s);
    bool confident = false;
    for (const disc::CandidateInfo& c : st.candidates) confident = confident || c.confident;
    if (confident) print_top("still camera", st, expected_camera(s));
    EXPECT(!confident);
    EXPECT(st.reproj_rounds == 0);
}

// Depth frames that are another resource's memory (MGS Delta: NaN and stripes in up to half the
// frames) are left out, not taken for a camera jump.
TEST_CASE(analyzer, garbage_depth) {
    Scenario s;
    s.garbage_depth = 3;
    EXPECT(finds("every third depth frame garbage", s));
    EXPECT(run(s).depth_rejected > 0);
}

// Much of the scene moves on its own (MGS Delta's jungle): the right camera explains the static part
// exactly, but that's under half the pixels that change. Then last frame's camera (the decoy at c12)
// passes about as often as the camera, and only being a frame behind it gives it away.
TEST_CASE(analyzer, scene_motion) {
    for (GameLayout layout : {GameLayout::ViewAndProj, GameLayout::InvViewProjAndProj}) {
        Scenario s;
        s.layout = layout;
        s.scene_motion = true;
        EXPECT(finds(layout == GameLayout::ViewAndProj ? "view+proj, scene moves" : "invviewproj+proj, scene moves", s));
    }
}

// A still camera in a moving scene: the depth changes, but nothing may claim it explains that.
TEST_CASE(analyzer, still_camera_moving_scene) {
    Scenario s;
    s.moving = false;
    s.scene_motion = true;
    s.frames = 200;
    const disc::Status st = run(s);
    bool confident = false;
    for (const disc::CandidateInfo& c : st.candidates) confident = confident || c.confident;
    if (confident) print_top("still camera, scene moves", st, expected_camera(s));
    EXPECT(!confident);
    EXPECT(st.reproj_rounds > 0);
}

// More camera-like combinations than candidates kept at once (UE's View buffer, bound to several
// stages), and all of them there before the camera is: the ones that fail make room, so the camera
// still gets its turn.
TEST_CASE(analyzer, candidate_flood) {
    Scenario s;
    s.flood = 20;
    s.camera_from = 60;
    EXPECT(finds("20 buffers of other cameras, bound before the camera", s));
}

// Same inputs, same result: what makes replays and these tests meaningful.
TEST_CASE(analyzer, deterministic) {
    Scenario s;
    s.frames = 200;
    const disc::Status a = run(s, 32), b = run(s, 32);
    EXPECT(a.candidates.size() == b.candidates.size());
    EXPECT(a.reproj_rounds == b.reproj_rounds && a.still_frames == b.still_frames && a.moving_frames == b.moving_frames);
    for (size_t i = 0; i < a.candidates.size() && i < b.candidates.size(); ++i) {
        const disc::CandidateInfo &x = a.candidates[i], &y = b.candidates[i];
        EXPECT(disc::same_camera(x.profile, y.profile) && x.profile.latch == y.profile.latch);
        EXPECT(x.score == y.score && x.reproj_tests == y.reproj_tests && x.reproj_passes == y.reproj_passes &&
               x.temporal_checks == y.temporal_checks && x.temporal_agree == y.temporal_agree);
    }
}
