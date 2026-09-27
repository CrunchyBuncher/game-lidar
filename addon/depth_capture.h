// The API-specific half of the depth path: snapshot the scene depth-stencil, point-sample it
// down to the capture size (protocol.h mapping) and read it back without stalling the game.
// Each graphics API gets one implementation (d3d9/, d3d11/, d3d12/). Everything around it (choosing the
// depth-stencil, pairing it with the camera, the ring) is shared.
#pragma once
#include <reshade.hpp>

#include <algorithm>
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
    // except the sizes, depth format and color flag, which are filled in here. Returns false (and
    // sets error()) if the depth buffer can't be captured; returns true but skips if the readback
    // pipeline is full.
    // `color` (0: none) is the render target the scene was drawn into with `depth`. It's sampled at
    // the same pixels and published as kFlagHasColor, if it's the depth buffer's size and a format
    // that can be sampled (D3D12: and its state is known); otherwise the frame has no color.
    virtual bool capture(reshade::api::command_queue* queue, reshade::api::resource depth,
                         reshade::api::resource color, uint32_t capture_width, const FrameHeader& header) = 0;

    // Before a clear (depth::ClearHook), where the API allows it (D3D9): samples `depth` (and
    // `color`) down now, replacing this frame's earlier snapshot. The next capture() publishes the
    // snapshot instead of sampling again, unless drop_snapshot() comes first. Returns false if
    // nothing was taken.
    virtual bool snapshot(reshade::api::resource /*depth*/, reshade::api::resource /*color*/,
                          uint32_t /*capture_width*/) {
        return false;
    }
    virtual void drop_snapshot() {}

    // Publishes every readback the GPU has finished. Never waits.
    virtual void publish(reshade::api::command_queue* queue, RingWriter& ring) = 0;

    const std::string& error() const { return error_; }
    // Why the last capture has no color ("" if it has).
    const std::string& color_note() const { return color_note_; }
    // Fractions of the image, from each edge (left, top, right, bottom), whose color is dropped
    // (alpha 0, see protocol.h): games that draw their HUD into the scene's target.
    void set_color_crop(const float crop[4]) { std::copy_n(crop, 4, crop_); }
    uint32_t width() const { return cap_w_; }
    uint32_t height() const { return cap_h_; }
    uint64_t published() const { return published_; }
    uint64_t skipped() const { return skipped_; }

protected:
    // Applies the color crop to a published slot with color.
    void crop_color(Slot& slot) const {
        const uint32_t w = slot.frame.width, h = slot.frame.height;
        const uint32_t x0 = uint32_t(crop_[0] * w), y0 = uint32_t(crop_[1] * h);
        const uint32_t x1 = w - std::min(w, uint32_t(crop_[2] * w)), y1 = h - std::min(h, uint32_t(crop_[3] * h));
        for (uint32_t y = 0; y < h; ++y)
            for (uint32_t x = 0; x < w; ++x)
                if (y < y0 || y >= y1 || x < x0 || x >= x1) slot.color[y * w + x] &= 0x00FFFFFFu;
    }

    std::string error_;
    float crop_[4] = {};
    std::string color_note_ = "not captured";
    uint32_t cap_w_ = 0, cap_h_ = 0;
    uint64_t published_ = 0, skipped_ = 0;
};

// The implementation for `dev`'s API. Returns nullptr and sets `error` if the API isn't
// supported or setup fails.
std::unique_ptr<DepthCapture> create_depth_capture(reshade::api::device* dev, std::string& error);

// Whether `dev`'s API has an implementation (D3D9, D3D11, D3D12).
bool is_supported(reshade::api::device* dev);

// Registers the event handlers `api`'s implementation needs, once per API (D3D12 tracks
// depth-stencil states, D3D9 makes depth-stencils readable).
void register_capture_events(reshade::api::device_api api);
void init_capture_device(reshade::api::device* dev);  // for a device created before registration

}  // namespace lidar
