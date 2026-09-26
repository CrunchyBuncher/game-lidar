// D3D12 depth capture, all through the ReShade API on the queue's immediate command list (so none
// of it fires addon events): transition the depth buffer from the state the game left it in to
// copy source, copy it, transition it back, point-sample it down with a compute shader and copy
// the result into one of a ring of persistently mapped readback buffers. A fence signaled after
// each capture says when its readback can be published.
//
// D3D12 has no "current state" to query, so the depth buffer's state is tracked from the game's
// barriers: per command list at record time, applied to the device at execute_command_list.
#pragma once
#include <deque>

#include "depth_capture.h"

namespace lidar {

class D3D12Capture final : public DepthCapture {
public:
    ~D3D12Capture() override { release(); }
    bool init(reshade::api::device* dev);
    void release();

    // Depth only: `color` is ignored for now.
    bool capture(reshade::api::command_queue* queue, reshade::api::resource depth, reshade::api::resource color,
                 uint32_t capture_width, const FrameHeader& header) override;
    void publish(reshade::api::command_queue* queue, RingWriter& ring) override;

    // Tracks depth-stencil states on D3D12 devices (see above).
    static void register_events();
    static void unregister_events();
    static void init_device(reshade::api::device* dev);  // idempotent, ignores other APIs

private:
    static constexpr int kReadback = 3;

    bool ensure_targets(const reshade::api::resource_desc& src, uint32_t capture_width);
    void destroy_targets();
    void wait_idle();

    reshade::api::device* dev_ = nullptr;
    reshade::api::pipeline_layout layout_{0};  // root constants b0, SRV t0, UAV u0
    reshade::api::pipeline pipeline_{0};
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
    uint32_t row_pitch_ = 0;

    struct Pending {
        int slot;
        uint64_t fence;
        FrameHeader header;
    };
    std::deque<Pending> pending_;
    int next_slot_ = 0;
};

}  // namespace lidar
