// Shared-memory frame protocol between a producer (ReShade addon / fake game)
// and the viewer.
//
// Conventions (v1):
//   * Matrices are 16 floats, row-major, ROW-VECTOR math (DirectXMath / D3D
//     style):  p_view = p_world * view,  p_clip = p_view * proj.
//   * Clip space is D3D: NDC x,y in [-1,1] (y up), z in [0,1]. Any depth
//     convention (standard, reversed, reversed-infinite) is fine; consumers
//     unproject with inverse(proj), so it's handled automatically.
//   * depth[] holds the raw depth-buffer value (NDC z) as float32.
//   * Stored pixel (u,v) was point-sampled from source depth-buffer pixel
//     sx = min(floor((u + 0.5) * src_width / width), src_width - 1)
//     (same for y). Consumers compute NDC from the SOURCE pixel center, so
//     downsampling introduces no geometric error.
//   * color[] is RGBA8 (R in the lowest byte), same layout as depth.
#pragma once
#include <cstddef>
#include <cstdint>

namespace lidar {

constexpr wchar_t kFramesMappingName[] = L"Local\\game_lidar_frames";
constexpr uint32_t kMagic = 0x4652444C;  // "LDRF"
constexpr uint32_t kProtocolVersion = 1;
constexpr uint32_t kSlotCount = 4;
constexpr uint32_t kMaxWidth = 1280;
constexpr uint32_t kMaxHeight = 720;

enum FrameFlags : uint32_t {
    kFlagPoseValid = 1u << 0,
    kFlagHasColor = 1u << 1,
    kFlagPaused = 1u << 2,
};

enum class DepthFormat : uint32_t {
    Float32Ndc = 1,
};

struct FrameHeader {
    uint64_t frame_index;    // producer's own frame counter
    uint64_t timestamp_qpc;  // QueryPerformanceCounter at capture
    uint32_t width, height;          // stored image size
    uint32_t src_width, src_height;  // depth buffer it was sampled from
    uint32_t depth_format;           // DepthFormat
    uint32_t flags;                  // FrameFlags
    float view[16];
    float proj[16];
};

// Seqlock per slot: writer stores seq_begin, writes the payload, then stores
// seq_end. A reader's copy is valid only if both equal the expected seq.
struct alignas(64) Slot {
    uint64_t seq_begin;
    uint64_t seq_end;
    uint8_t pad_[48];
    FrameHeader frame;
    float depth[kMaxWidth * kMaxHeight];
    uint32_t color[kMaxWidth * kMaxHeight];
};

struct alignas(64) RingHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t slot_count;
    uint32_t slot_size;
    uint64_t session_id;  // changes whenever a producer (re)initializes
    uint64_t latest_seq;  // last fully written seq; slot = seq % slot_count
};

constexpr size_t kMappingSize = sizeof(RingHeader) + size_t(kSlotCount) * sizeof(Slot);

}  // namespace lidar
