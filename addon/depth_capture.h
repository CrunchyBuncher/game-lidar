// The API-specific half of the depth path: snapshot the scene depth-stencil, point-sample it
// down to the capture size (protocol.h mapping) and read it back without stalling the game.
// Each graphics API gets one implementation (d3d11/, d3d12/). Everything around it (choosing the
// depth-stencil, pairing it with the camera, the ring) is shared.
#pragma once
#include <reshade.hpp>

#include <cstdint>
#include <memory>
#include <string>

#include "protocol.h"
#include "ring.h"

namespace lidar {

class DepthCapture {
public:
    virtual ~DepthCapture() = default;

    // Records the GPU work for one capture of `depth` on `queue`. `header` supplies everything
    // except the sizes and depth format, which are filled in here. Returns false (and sets
    // error()) if the depth buffer can't be captured; returns true but skips if the readback
    // pipeline is full.
    virtual bool capture(reshade::api::command_queue* queue, reshade::api::resource depth, uint32_t capture_width,
                         const FrameHeader& header) = 0;

    // Publishes every readback the GPU has finished. Never waits.
    virtual void publish(reshade::api::command_queue* queue, RingWriter& ring) = 0;

    const std::string& error() const { return error_; }
    uint32_t width() const { return cap_w_; }
    uint32_t height() const { return cap_h_; }
    uint64_t published() const { return published_; }
    uint64_t skipped() const { return skipped_; }

protected:
    std::string error_;
    uint32_t cap_w_ = 0, cap_h_ = 0;
    uint64_t published_ = 0, skipped_ = 0;
};

// The implementation for `dev`'s API. Returns nullptr and sets `error` if the API isn't
// supported or setup fails.
std::unique_ptr<DepthCapture> create_depth_capture(reshade::api::device* dev, std::string& error);

// Whether `dev`'s API has an implementation (D3D11, D3D12).
bool is_supported(reshade::api::device* dev);

// Registers the event handlers implementations need (D3D12 tracks depth-stencil states).
void register_capture_events();
void unregister_capture_events();
void init_capture_device(reshade::api::device* dev);  // for a device created before registration

}  // namespace lidar
