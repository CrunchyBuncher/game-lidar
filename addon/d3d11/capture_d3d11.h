// D3D11 depth capture: copy the depth-stencil, point-sample it down with a compute shader and
// read it back through a staging ring. All GPU work uses native calls, so it doesn't fire
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

    bool capture(reshade::api::command_queue* queue, reshade::api::resource depth, uint32_t capture_width,
                 const FrameHeader& header) override;
    void publish(reshade::api::command_queue* queue, RingWriter& ring) override;

private:
    template <class T>
    using ComPtr = Microsoft::WRL::ComPtr<T>;
    static constexpr int kStaging = 3;

    bool ensure_targets(const D3D11_TEXTURE2D_DESC& src, uint32_t capture_width);

    ComPtr<ID3D11Device> dev_;
    ComPtr<ID3D11ComputeShader> cs_;
    ComPtr<ID3D11Buffer> dims_cb_;

    // Snapshot of the depth buffer (typeless, shader-readable) and the downsampled result.
    D3D11_TEXTURE2D_DESC src_desc_{};
    ComPtr<ID3D11Texture2D> copy_;
    ComPtr<ID3D11ShaderResourceView> copy_srv_;
    ComPtr<ID3D11Texture2D> cap_;
    ComPtr<ID3D11UnorderedAccessView> cap_uav_;
    ComPtr<ID3D11Texture2D> stage_[kStaging];

    struct Pending {
        int stage;
        FrameHeader header;
    };
    std::deque<Pending> pending_;
    int next_stage_ = 0;
};

}  // namespace lidar
