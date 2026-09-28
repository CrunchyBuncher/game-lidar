// The shared-memory ring: delivery order, overwrites, drops, clear requests, producer restarts.
#include "ring.h"

#include "test.h"

using namespace lidar;

static void write_frame(RingWriter& w, uint64_t index) {
    Slot* s = w.begin_frame();
    s->frame = {};
    s->frame.frame_index = index;
    s->frame.width = 4;
    s->frame.height = 2;
    s->frame.flags = kFlagPoseValid;
    for (int i = 0; i < 8; ++i) s->depth[i] = float(index) + i * 0.1f;
    w.commit();
}

TEST_CASE(ring, semantics) {
    const wchar_t* name = L"Local\\game_lidar_test_ring";
    RingWriter w;
    EXPECT(w.open(name));
    RingReader r;
    EXPECT(r.try_open(name));

    Frame f;
    EXPECT(!r.read_next(f));  // empty

    for (uint64_t i = 1; i <= 10; ++i) write_frame(w, i);
    // Only the last kSlotCount frames survive, delivered oldest first.
    for (uint64_t expect = 10 - kSlotCount + 1; expect <= 10; ++expect) {
        EXPECT(r.read_next(f));
        EXPECT(f.header.frame_index == expect);
        EXPECT(f.depth.size() == 8 && f.depth[7] == float(expect) + 0.7f);
    }
    EXPECT(!r.read_next(f));

    write_frame(w, 11);
    EXPECT(r.read_next(f) && f.header.frame_index == 11);

    // Reader falls behind by more than the ring: skipped frames count as dropped.
    const uint64_t dropped_before = r.dropped();
    for (uint64_t i = 12; i <= 20; ++i) write_frame(w, i);
    uint64_t first = 0;
    int got = 0;
    while (r.read_next(f)) {
        if (!got) first = f.header.frame_index;
        ++got;
    }
    EXPECT(got == int(kSlotCount) && first == 20 - kSlotCount + 1);
    EXPECT(r.dropped() - dropped_before == 9 - kSlotCount);

    // Clear requests: past everything written so far, plus the frame that may be in flight.
    EXPECT(r.clear_seq() == 0);
    const uint64_t written = w.latest_seq();
    w.request_clear();
    EXPECT(r.clear_seq() == written + 1);
    RingWriter closed;
    closed.request_clear();  // not open: a no-op

    // Producer restart: reader resyncs to the new session, and the clear request is gone.
    EXPECT(w.open(name));
    EXPECT(r.clear_seq() == 0);
    write_frame(w, 1);
    EXPECT(r.read_next(f) && f.header.frame_index == 1);
    RingReader unopened;
    EXPECT(unopened.clear_seq() == 0);
}
