// D3D11 depth capture: snapshot the selected depth-stencil, point-sample it down to the
// capture size on the GPU (protocol.h mapping), and read it back through a staging ring
// without ever stalling the game.
#pragma once
#include <d3d11.h>
#include <wrl/client.h>

#include <cstdint>
#include <deque>
#include <string>

#include "protocol.h"
#include "ring.h"

namespace lidar {

class D3D11Capture {
public:
    bool init(ID3D11Device* dev);
    void release();

    // Records the GPU work for one capture. `header` supplies everything except the
    // sizes, which are filled in here. Returns false (and sets error()) if the depth
    // buffer can't be captured; returns true but skips if all staging slots are busy.
    bool capture(ID3D11DeviceContext* ctx, ID3D11Resource* depth, uint32_t capture_width, const FrameHeader& header);

    // Publishes every readback the GPU has finished. Never waits.
    void publish(ID3D11DeviceContext* ctx, RingWriter& ring);

    const std::string& error() const { return error_; }
    uint32_t width() const { return cap_w_; }
    uint32_t height() const { return cap_h_; }
    uint64_t published() const { return published_; }
    uint64_t skipped() const { return skipped_; }

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
    uint32_t cap_w_ = 0, cap_h_ = 0;
    ComPtr<ID3D11Texture2D> cap_;
    ComPtr<ID3D11UnorderedAccessView> cap_uav_;
    ComPtr<ID3D11Texture2D> stage_[kStaging];

    struct Pending {
        int stage;
        FrameHeader header;
    };
    std::deque<Pending> pending_;
    int next_stage_ = 0;

    std::string error_;
    uint64_t published_ = 0, skipped_ = 0;
};

}  // namespace lidar
