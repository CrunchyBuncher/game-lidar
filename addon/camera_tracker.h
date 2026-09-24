// Latches the camera cbuffer at the draws that go into each depth-stencil, so the frame's
// camera can be paired with whichever depth-stencil turns out to be the scene depth at present.
// API-agnostic: the bytes come from a CbufferSource.
//
// Discovery mode also samples some draws per depth-stencil and keeps everything bound at them
// (all constant buffers, up to a size), for the discovery scanner to analyze off the draw path.
#pragma once
#include <reshade.hpp>

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "cbuffer_source.h"
#include "profile.h"

namespace lidar::cam {

struct LatchRequest {
    CbufferKey key;
    uint32_t offset = 0, size = 0;  // byte window to read, relative to the bound range
    Latch latch = Latch::First;
};

// Per depth-stencil handle: the window read at its first (or last) draw that had the cbuffer bound,
// or for Latch::Common, the value most of its draws had.
using FrameLatches = std::unordered_map<uint64_t, CbufferRead>;

// Discovery: one sampled draw. `draw` is its index among the frame's draws into that depth-stencil.
struct DrawSample {
    uint32_t draw = 0;
    std::vector<BoundBuffer> buffers;  // all resolved
};
struct DepthSamples {
    uint32_t draws = 0;               // draws into the depth-stencil this frame
    std::vector<DrawSample> samples;  // in draw order
};
using FrameSamples = std::unordered_map<uint64_t, DepthSamples>;

void register_events();
void unregister_events();
// Sets up `dev` (idempotent). The init_device handler does it for devices created after
// register_events(); call it for a device created before.
void init_device(reshade::api::device* dev);

// Starts latching `req` on `dev` through `source`. Pass nullptr for either to stop.
// `source` must outlive the configuration.
void configure(reshade::api::device* dev, CbufferSource* source, const LatchRequest* req);

// Starts sampling up to `samples_per_frame` draws per depth-stencil (the first few, then spread
// over the frame by last frame's draw count), reading up to `max_bytes` of each bound buffer.
// samples_per_frame = 0 (or a null source) stops.
void configure_discovery(reshade::api::device* dev, CbufferSource* source, uint32_t samples_per_frame,
                         uint32_t max_bytes);

// Call once per present: this frame's latches from every graphics queue, and resets them. With
// `samples`, also this frame's discovery samples.
FrameLatches end_frame(reshade::api::device* dev, FrameSamples* samples = nullptr);

}  // namespace lidar::cam
