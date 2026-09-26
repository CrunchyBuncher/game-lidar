// D3D11 depth capture: copy the depth-stencil, point-sample it down with a compute shader and
// read it back through a staging ring. The scene's color target, if given, goes the same way. All GPU work uses native calls, so it doesn't fire
// ReShade events (it would count as game draws otherwise), and restores the state it touches.
#pragma once
#include <d3d11.h>
#include <wrl/client.h>

#include <deque>

#include "depth_capture.h"

namespace lidar {

class D3D11Capture final : public DepthCapture {
public:
    ~D3D11Capture() override { release(); }
    bool init(ID3D11Device* dev);
    void release();

    bool capture(reshade::api::command_queue* queue, reshade::api::resource depth, reshade::api::resource color,
                 uint32_t capture_width, const FrameHeader& header) override;
    void publish(reshade::api::command_queue* queue, RingWriter& ring) override;

private:
    template <class T>
    using ComPtr = Microsoft::WRL::ComPtr<T>;
    static constexpr int kStaging = 3;

    bool ensure_targets(const D3D11_TEXTURE2D_DESC& src, uint32_t capture_width);
    // The color target, if it can be captured alongside `depth` (targets ready), else null with
    // color_note_ saying why.
    ComPtr<ID3D11Texture2D> color_source(reshade::api::resource color, const D3D11_TEXTURE2D_DESC& depth);
    bool ensure_color(const D3D11_TEXTURE2D_DESC& src);

    ComPtr<ID3D11Device> dev_;
    ComPtr<ID3D11ComputeShader> cs_, color_cs_;
    ComPtr<ID3D11Buffer> dims_cb_;

    // Snapshot of the depth buffer (typeless, shader-readable) and the downsampled result.
    D3D11_TEXTURE2D_DESC src_desc_{};
    ComPtr<ID3D11Texture2D> copy_;
    ComPtr<ID3D11ShaderResourceView> copy_srv_;
    ComPtr<ID3D11Texture2D> cap_;
    ComPtr<ID3D11UnorderedAccessView> cap_uav_;
    ComPtr<ID3D11Texture2D> stage_[kStaging];

    // The same for color: a copy of the render target, the downsampled RGBA8 and its staging ring.
    D3D11_TEXTURE2D_DESC color_desc_{};
    uint32_t color_w_ = 0, color_h_ = 0;
    ComPtr<ID3D11Texture2D> color_copy_;
    ComPtr<ID3D11ShaderResourceView> color_copy_srv_;
    ComPtr<ID3D11Texture2D> color_cap_;
    ComPtr<ID3D11UnorderedAccessView> color_cap_uav_;
    ComPtr<ID3D11Texture2D> color_stage_[kStaging];

    struct Pending {
        int stage;
        FrameHeader header;  // with kFlagHasColor, color_stage_[stage] holds its color
    };
    std::deque<Pending> pending_;
    int next_stage_ = 0;
};

}  // namespace lidar
