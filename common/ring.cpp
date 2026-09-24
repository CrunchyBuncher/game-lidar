#include "ring.h"

#include <windows.h>

#include <atomic>
#include <cstring>

namespace lidar {
namespace {

uint64_t load_acquire(const uint64_t& v) {
    return std::atomic_ref<uint64_t>(const_cast<uint64_t&>(v)).load(std::memory_order_acquire);
}
void store_release(uint64_t& v, uint64_t x) {
    std::atomic_ref<uint64_t>(v).store(x, std::memory_order_release);
}

uint64_t make_session_id() {
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return (uint64_t(t.QuadPart) * 0x9E3779B97F4A7C15ull) ^ GetCurrentProcessId();
}

}  // namespace

// ---------------------------------------------------------------- writer

RingWriter::~RingWriter() { close(); }

bool RingWriter::open(const wchar_t* name) {
    close();
    const uint64_t size = kMappingSize;
    map_ = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, DWORD(size >> 32),
                              DWORD(size & 0xFFFFFFFF), name);
    if (!map_) return false;
    base_ = static_cast<uint8_t*>(MapViewOfFile(map_, FILE_MAP_ALL_ACCESS, 0, 0, size));
    if (!base_) {
        close();
        return false;
    }
    hdr_ = reinterpret_cast<RingHeader*>(base_);
    slots_ = reinterpret_cast<Slot*>(base_ + sizeof(RingHeader));

    // Invalidate first so a reader never sees a half-initialized header.
    std::atomic_ref<uint32_t>(hdr_->magic).store(0, std::memory_order_release);
    hdr_->version = kProtocolVersion;
    hdr_->slot_count = kSlotCount;
    hdr_->slot_size = sizeof(Slot);
    for (uint32_t i = 0; i < kSlotCount; ++i) {
        store_release(slots_[i].seq_begin, 0);
        store_release(slots_[i].seq_end, 0);
    }
    store_release(hdr_->latest_seq, 0);
    store_release(hdr_->session_id, make_session_id());
    std::atomic_ref<uint32_t>(hdr_->magic).store(kMagic, std::memory_order_release);
    pending_seq_ = 0;
    return true;
}

void RingWriter::close() {
    if (base_) UnmapViewOfFile(base_);
    if (map_) CloseHandle(map_);
    map_ = nullptr;
    base_ = nullptr;
    hdr_ = nullptr;
    slots_ = nullptr;
}

Slot* RingWriter::begin_frame() {
    pending_seq_ = hdr_->latest_seq + 1;
    Slot* s = &slots_[pending_seq_ % kSlotCount];
    std::atomic_ref<uint64_t>(s->seq_begin).store(pending_seq_, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    return s;
}

void RingWriter::commit() {
    Slot* s = &slots_[pending_seq_ % kSlotCount];
    store_release(s->seq_end, pending_seq_);
    store_release(hdr_->latest_seq, pending_seq_);
}

// ---------------------------------------------------------------- reader

RingReader::~RingReader() { close(); }

bool RingReader::try_open(const wchar_t* name) {
    close();
    map_ = OpenFileMappingW(FILE_MAP_READ, FALSE, name);
    if (!map_) return false;
    base_ = static_cast<uint8_t*>(MapViewOfFile(map_, FILE_MAP_READ, 0, 0, kMappingSize));
    if (!base_) {
        close();
        return false;
    }
    hdr_ = reinterpret_cast<RingHeader*>(base_);
    slots_ = reinterpret_cast<Slot*>(base_ + sizeof(RingHeader));
    const uint32_t magic =
        std::atomic_ref<uint32_t>(const_cast<uint32_t&>(hdr_->magic)).load(std::memory_order_acquire);
    if (magic != kMagic || hdr_->version != kProtocolVersion || hdr_->slot_size != sizeof(Slot) ||
        hdr_->slot_count != kSlotCount) {
        close();
        return false;
    }
    session_ = 0;
    last_seq_ = 0;
    return true;
}

void RingReader::close() {
    if (base_) UnmapViewOfFile(base_);
    if (map_) CloseHandle(map_);
    map_ = nullptr;
    base_ = nullptr;
    hdr_ = nullptr;
    slots_ = nullptr;
}

bool RingReader::read_next(Frame& out) {
    if (!hdr_) return false;
    const uint64_t session = load_acquire(hdr_->session_id);
    uint64_t latest = load_acquire(hdr_->latest_seq);
    if (session != session_ || latest < last_seq_) {
        // Producer restarted: start from whatever is still in the ring.
        session_ = session;
        last_seq_ = latest > kSlotCount ? latest - kSlotCount : 0;
    }

    while (last_seq_ < latest) {
        if (latest - last_seq_ > kSlotCount) {
            dropped_ += latest - last_seq_ - kSlotCount;
            last_seq_ = latest - kSlotCount;
        }
        const uint64_t seq = ++last_seq_;
        const Slot* s = &slots_[seq % kSlotCount];
        if (load_acquire(s->seq_end) != seq) {
            ++dropped_;
            continue;
        }
        out.header = s->frame;
        const FrameHeader& h = out.header;
        const bool dims_ok = h.width > 0 && h.height > 0 && h.width <= kMaxWidth && h.height <= kMaxHeight;
        if (dims_ok) {
            const size_t n = size_t(h.width) * h.height;
            out.depth.resize(n);
            std::memcpy(out.depth.data(), s->depth, n * sizeof(float));
            if (h.flags & kFlagHasColor) {
                out.color.resize(n);
                std::memcpy(out.color.data(), s->color, n * sizeof(uint32_t));
            } else {
                out.color.clear();
            }
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (std::atomic_ref<uint64_t>(const_cast<uint64_t&>(s->seq_begin)).load(std::memory_order_relaxed) !=
                seq ||
            !dims_ok) {
            ++dropped_;  // overwritten while copying
            latest = load_acquire(hdr_->latest_seq);
            continue;
        }
        out.seq = seq;
        return true;
    }
    return false;
}

}  // namespace lidar
