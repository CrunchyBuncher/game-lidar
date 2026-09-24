// Latches the camera cbuffer at the draws that go into each depth-stencil, so the frame's
// camera can be paired with whichever depth-stencil turns out to be the scene depth at present.
// API-agnostic: the bytes come from a CbufferSource.
#pragma once
#include <reshade.hpp>

#include <cstdint>
#include <unordered_map>

#include "cbuffer_source.h"

namespace lidar::cam {

struct LatchRequest {
    CbufferKey key;
    uint32_t offset = 0, size = 0;  // byte window to read, relative to the bound range
    bool last = false;              // keep the last matching draw of the frame instead of the first
};

// Per depth-stencil handle: the window read at its first (or last) draw that had the cbuffer bound.
using FrameLatches = std::unordered_map<uint64_t, CbufferRead>;

void register_events();
void unregister_events();

// Starts latching `req` on `dev` through `source`. Pass nullptr for either to stop.
// `source` must outlive the configuration.
void configure(reshade::api::device* dev, CbufferSource* source, const LatchRequest* req);

// Call once per present: this frame's latches from every graphics queue, and resets them.
FrameLatches end_frame(reshade::api::device* dev);

}  // namespace lidar::cam
