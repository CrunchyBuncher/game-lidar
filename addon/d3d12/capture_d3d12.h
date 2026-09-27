// D3D12 depth (and color) capture, all through the ReShade API on the queue's immediate command
// list (so none of it fires addon events): transition the depth buffer (and the scene's render
// target) from the state the game left it in to copy source, copy it, transition it back, point-sample
// it down with a compute shader and copy the result into one of a ring of persistently mapped
// readback buffers. A fence signaled after each capture says when its readback can be published.
//
// D3D12 has no "current state" to query, so the states of depth-stencils and render targets are
// tracked from the game's barriers: per command list at record time, applied to the device at
// execute_command_list. Swap chain back buffers aren't created through the device, so they're added
// at init_swapchain, in the present state.
#pragma once
#include <deque>

#include "depth_capture.h"

namespace lidar {

class D3D12Capture final : public DepthCapture {
public:
    ~D3D12Capture() override { release(); }
    bool init(reshade::api::device* dev);
    void release();

    bool capture(reshade::api::command_queue* queue, reshade::api::resource depth, reshade::api::resource color,
                 uint32_t capture_width, const FrameHeader& header) override;
    void publish(reshade::api::command_queue* queue, RingWriter& ring) override;

    // Tracks depth-stencil and render target states on D3D12 devices (see above).
    static void register_events();
    static void unregister_events();
    static void init_device(reshade::api::device* dev);  // idempotent, ignores other APIs

private:
    static constexpr int kReadback = 3;

    bool ensure_targets(const reshade::api::resource_desc& src, uint32_t capture_width);
    void destroy_targets();
    // Checks `color` against the depth buffer and (re)creates its targets. Returns false with
    // color_note_ set if this capture goes without color.
    bool color_source(reshade::api::resource color, const reshade::api::resource_desc& depth);
    bool ensure_color(const reshade::api::resource_desc& src);
    void destroy_color();
    void wait_idle();

    reshade::api::device* dev_ = nullptr;
    reshade::api::pipeline_layout layout_{0};  // root constants b0, SRV t0, UAV u0, SRV t1, UAV u1
    reshade::api::pipeline pipeline_{0}, color_pipeline_{0}, color_hdr_pipeline_{0};
    reshade::api::fence fence_{0};
    uint64_t fence_value_ = 0;

    // Snapshot of the depth buffer (typeless, shader-readable), the downsampled result and the
    // readback ring. Resting states: copy_ copy_dest, cap_ unordered_access.
    reshade::api::resource_desc src_desc_{};
    reshade::api::resource copy_{0};
    reshade::api::resource_view copy_srv_{0};
    reshade::api::resource cap_{0};
    reshade::api::resource_view cap_uav_{0};
    reshade::api::resource readback_[kReadback] = {};
    const uint8_t* readback_data_[kReadback] = {};
    uint32_t row_pitch_ = 0;  // the color readbacks' too: 4 bytes a pixel either way

    // The same for color, created on first use: copy in the render target's format, result RGBA8.
    reshade::api::resource_desc color_desc_{};
    bool color_hdr_ = false;  // a float format: cs_color_hdr
    uint32_t color_w_ = 0, color_h_ = 0;
    reshade::api::resource color_copy_{0};
    reshade::api::resource_view color_copy_srv_{0};
    reshade::api::resource color_cap_{0};
    reshade::api::resource_view color_cap_uav_{0};
    reshade::api::resource color_readback_[kReadback] = {};
    const uint8_t* color_readback_data_[kReadback] = {};

    struct Pending {
        int slot;
        uint64_t fence;
        FrameHeader header;
    };
    std::deque<Pending> pending_;
    int next_slot_ = 0;
};

}  // namespace lidar
