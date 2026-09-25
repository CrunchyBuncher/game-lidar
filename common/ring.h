// Shared-memory ring writer/reader for protocol.h.
#pragma once
#include <cstdint>
#include <vector>

#include "protocol.h"

namespace lidar {

class RingWriter {
public:
    ~RingWriter();
    // Creates the mapping (or attaches to one a reader already holds) and
    // resets it with a fresh session id.
    bool open(const wchar_t* name = kFramesMappingName);
    void close();
    bool is_open() const { return hdr_ != nullptr; }

    // Returns the slot to fill for the next frame. Must be followed by commit().
    Slot* begin_frame();
    void commit();

    // For the writing thread to look at what it committed: the last committed seq (0: none yet)
    // and the slot a seq went to (valid until kSlotCount more frames are written).
    uint64_t latest_seq() const { return hdr_ ? hdr_->latest_seq : 0; }
    // Asks the viewer to drop its points and the frames written so far (or being written). Any
    // thread; nothing happens if no viewer is reading.
    void request_clear();
    const Slot* slot(uint64_t seq) const { return &slots_[seq % kSlotCount]; }

private:
    void* map_ = nullptr;
    uint8_t* base_ = nullptr;
    RingHeader* hdr_ = nullptr;
    Slot* slots_ = nullptr;
    uint64_t pending_seq_ = 0;
};

struct Frame {
    FrameHeader header{};
    std::vector<float> depth;     // width * height
    std::vector<uint32_t> color;  // width * height, empty if no color
    uint64_t seq = 0;
};

class RingReader {
public:
    ~RingReader();
    bool try_open(const wchar_t* name = kFramesMappingName);
    void close();
    bool is_open() const { return hdr_ != nullptr; }

    // Reads the next unread frame (oldest first). Returns false when caught up.
    // Frames the writer lapped are skipped and counted in dropped().
    bool read_next(Frame& out);
    uint64_t dropped() const { return dropped_; }
    // The producer's latest clear request (RingHeader::clear_seq), 0 if none or not open.
    uint64_t clear_seq() const;

private:
    void* map_ = nullptr;
    uint8_t* base_ = nullptr;
    RingHeader* hdr_ = nullptr;
    Slot* slots_ = nullptr;
    uint64_t session_ = 0;
    uint64_t last_seq_ = 0;
    uint64_t dropped_ = 0;
};

}  // namespace lidar
