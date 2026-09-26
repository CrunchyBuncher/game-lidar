#include "capture_d3d11.h"

#include <d3dcompiler.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <utility>

#include "downsample_hlsl.h"

namespace lidar {
namespace {

struct DimsCB {
    uint32_t src_w, src_h, dst_w, dst_h;
};

// Depth-stencil formats (typed or typeless) -> typeless copy format + depth-only SRV format.
bool depth_formats(DXGI_FORMAT f, DXGI_FORMAT& typeless, DXGI_FORMAT& srv) {
    switch (f) {
        case DXGI_FORMAT_D24_UNORM_S8_UINT:
        case DXGI_FORMAT_R24G8_TYPELESS:
        case DXGI_FORMAT_R24_UNORM_X8_TYPELESS:
            typeless = DXGI_FORMAT_R24G8_TYPELESS, srv = DXGI_FORMAT_R24_UNORM_X8_TYPELESS;
            return true;
        case DXGI_FORMAT_D32_FLOAT:
        case DXGI_FORMAT_R32_TYPELESS:
        case DXGI_FORMAT_R32_FLOAT:
            typeless = DXGI_FORMAT_R32_TYPELESS, srv = DXGI_FORMAT_R32_FLOAT;
            return true;
        case DXGI_FORMAT_D16_UNORM:
        case DXGI_FORMAT_R16_TYPELESS:
        case DXGI_FORMAT_R16_UNORM:
            typeless = DXGI_FORMAT_R16_TYPELESS, srv = DXGI_FORMAT_R16_UNORM;
            return true;
        case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
        case DXGI_FORMAT_R32G8X24_TYPELESS:
        case DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS:
            typeless = DXGI_FORMAT_R32G8X24_TYPELESS, srv = DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS;
            return true;
        default:
            return false;
    }
}

// Render target formats (typed or typeless) -> typeless copy format + the format to read it as.
// sRGB ones are read as UNORM: the viewer wants the stored (gamma-encoded) values, as displayed.
bool color_formats(DXGI_FORMAT f, DXGI_FORMAT& typeless, DXGI_FORMAT& srv) {
    switch (f) {
        case DXGI_FORMAT_R8G8B8A8_TYPELESS:
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
            typeless = DXGI_FORMAT_R8G8B8A8_TYPELESS, srv = DXGI_FORMAT_R8G8B8A8_UNORM;
            return true;
        case DXGI_FORMAT_B8G8R8A8_TYPELESS:
        case DXGI_FORMAT_B8G8R8A8_UNORM:
        case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
            typeless = DXGI_FORMAT_B8G8R8A8_TYPELESS, srv = DXGI_FORMAT_B8G8R8A8_UNORM;
            return true;
        case DXGI_FORMAT_B8G8R8X8_TYPELESS:
        case DXGI_FORMAT_B8G8R8X8_UNORM:
        case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
            typeless = DXGI_FORMAT_B8G8R8X8_TYPELESS, srv = DXGI_FORMAT_B8G8R8X8_UNORM;
            return true;
        case DXGI_FORMAT_R10G10B10A2_TYPELESS:
        case DXGI_FORMAT_R10G10B10A2_UNORM:
            typeless = DXGI_FORMAT_R10G10B10A2_TYPELESS, srv = DXGI_FORMAT_R10G10B10A2_UNORM;
            return true;
        case DXGI_FORMAT_R16G16B16A16_TYPELESS:
        case DXGI_FORMAT_R16G16B16A16_FLOAT:
            typeless = DXGI_FORMAT_R16G16B16A16_TYPELESS, srv = DXGI_FORMAT_R16G16B16A16_FLOAT;
            return true;
        case DXGI_FORMAT_R11G11B10_FLOAT:
            typeless = srv = DXGI_FORMAT_R11G11B10_FLOAT;
            return true;
        default:
            return false;
    }
}

// The game's compute state survives across frames in D3D11, so put back what we touch (slots 0-1).
struct ComputeStateGuard {
    ID3D11DeviceContext* ctx;
    ID3D11ComputeShader* cs = nullptr;
    ID3D11ShaderResourceView* srv[2] = {};
    ID3D11UnorderedAccessView* uav[2] = {};
    ID3D11Buffer* cb = nullptr;

    explicit ComputeStateGuard(ID3D11DeviceContext* c) : ctx(c) {
        ctx->CSGetShader(&cs, nullptr, nullptr);
        ctx->CSGetShaderResources(0, 2, srv);
        ctx->CSGetUnorderedAccessViews(0, 2, uav);
        ctx->CSGetConstantBuffers(0, 1, &cb);
    }
    ~ComputeStateGuard() {
        ctx->CSSetShader(cs, nullptr, 0);
        ctx->CSSetShaderResources(0, 2, srv);
        const UINT keep_counter[2] = {UINT(-1), UINT(-1)};
        ctx->CSSetUnorderedAccessViews(0, 2, uav, keep_counter);
        ctx->CSSetConstantBuffers(0, 1, &cb);
        for (IUnknown* p : {static_cast<IUnknown*>(cs), static_cast<IUnknown*>(srv[0]), static_cast<IUnknown*>(srv[1]),
                            static_cast<IUnknown*>(uav[0]), static_cast<IUnknown*>(uav[1]), static_cast<IUnknown*>(cb)})
            if (p) p->Release();
    }
};

}  // namespace

bool D3D11Capture::init(ID3D11Device* dev) {
    dev_ = dev;
    const std::pair<const char*, ComPtr<ID3D11ComputeShader>*> shaders[] = {{"cs_main", std::addressof(cs_)},
                                                                            {"cs_color", std::addressof(color_cs_)}};
    for (const auto& [entry, cs] : shaders) {
        ComPtr<ID3DBlob> code, errors;
        if (FAILED(D3DCompile(kDownsampleHlsl, std::strlen(kDownsampleHlsl), "lidar_downsample", nullptr, nullptr,
                              entry, "cs_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors))) {
            error_ = errors ? static_cast<const char*>(errors->GetBufferPointer()) : "D3DCompile failed";
            return false;
        }
        if (FAILED(dev->CreateComputeShader(code->GetBufferPointer(), code->GetBufferSize(), nullptr,
                                            cs->ReleaseAndGetAddressOf()))) {
            error_ = "CreateComputeShader failed";
            return false;
        }
    }
    D3D11_BUFFER_DESC d{};
    d.ByteWidth = 16;
    d.Usage = D3D11_USAGE_DEFAULT;
    d.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    if (FAILED(dev->CreateBuffer(&d, nullptr, &dims_cb_))) {
        error_ = "CreateBuffer failed";
        return false;
    }
    return true;
}

void D3D11Capture::release() {
    pending_.clear();
    for (auto& s : color_stage_) s.Reset();
    color_cap_uav_.Reset(), color_cap_.Reset(), color_copy_srv_.Reset(), color_copy_.Reset();
    color_desc_ = {};
    color_w_ = color_h_ = 0;
    for (auto& s : stage_) s.Reset();
    cap_uav_.Reset();
    cap_.Reset();
    copy_srv_.Reset();
    copy_.Reset();
    dims_cb_.Reset();
    color_cs_.Reset();
    cs_.Reset();
    dev_.Reset();
    src_desc_ = {};
    cap_w_ = cap_h_ = 0;
}

bool D3D11Capture::ensure_targets(const D3D11_TEXTURE2D_DESC& src, uint32_t capture_width) {
    const uint32_t cap_w = std::clamp(capture_width, 16u, std::min(kMaxWidth, uint32_t(src.Width)));
    const uint32_t cap_h = std::min(uint32_t(std::lround(double(cap_w) * src.Height / src.Width)), kMaxHeight);
    if (copy_ && src.Width == src_desc_.Width && src.Height == src_desc_.Height && src.Format == src_desc_.Format &&
        cap_w == cap_w_ && cap_h == cap_h_)
        return true;

    // Anything in flight refers to the old targets.
    pending_.clear();
    copy_.Reset(), copy_srv_.Reset(), cap_.Reset(), cap_uav_.Reset();
    for (auto& s : stage_) s.Reset();
    src_desc_ = {};

    DXGI_FORMAT typeless, srv_format;
    if (!depth_formats(src.Format, typeless, srv_format)) {
        error_ = "unsupported depth format " + std::to_string(int(src.Format));
        return false;
    }
    D3D11_TEXTURE2D_DESC d{};
    d.Width = src.Width;
    d.Height = src.Height;
    d.MipLevels = d.ArraySize = 1;
    d.Format = typeless;
    d.SampleDesc.Count = 1;
    d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    D3D11_SHADER_RESOURCE_VIEW_DESC sd{srv_format, D3D11_SRV_DIMENSION_TEXTURE2D};
    sd.Texture2D.MipLevels = 1;
    if (FAILED(dev_->CreateTexture2D(&d, nullptr, &copy_)) ||
        FAILED(dev_->CreateShaderResourceView(copy_.Get(), &sd, &copy_srv_))) {
        error_ = "failed to create depth copy";
        return false;
    }

    d.Width = cap_w;
    d.Height = cap_h;
    d.Format = DXGI_FORMAT_R32_FLOAT;
    d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
    if (FAILED(dev_->CreateTexture2D(&d, nullptr, &cap_)) ||
        FAILED(dev_->CreateUnorderedAccessView(cap_.Get(), nullptr, &cap_uav_))) {
        error_ = "failed to create capture target";
        return false;
    }
    d.BindFlags = 0;
    d.Usage = D3D11_USAGE_STAGING;
    d.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    for (auto& s : stage_)
        if (FAILED(dev_->CreateTexture2D(&d, nullptr, &s))) {
            error_ = "failed to create staging texture";
            return false;
        }

    src_desc_ = src;
    cap_w_ = cap_w;
    cap_h_ = cap_h;
    return true;
}

D3D11Capture::ComPtr<ID3D11Texture2D> D3D11Capture::color_source(reshade::api::resource color,
                                                                 const D3D11_TEXTURE2D_DESC& depth) {
    if (color == 0) {
        color_note_ = "no render target was bound with the depth buffer";
        return nullptr;
    }
    ComPtr<ID3D11Texture2D> tex;
    if (FAILED(reinterpret_cast<ID3D11Resource*>(color.handle)->QueryInterface(IID_PPV_ARGS(&tex)))) {
        color_note_ = "the render target is not a 2D texture";
        return nullptr;
    }
    D3D11_TEXTURE2D_DESC d;
    tex->GetDesc(&d);
    if (d.Width != depth.Width || d.Height != depth.Height || d.SampleDesc.Count > 1) {
        color_note_ = "the render target is " + std::to_string(d.Width) + "x" + std::to_string(d.Height) +
                      (d.SampleDesc.Count > 1 ? " MSAA" : "") + ", not the depth buffer's size";
        return nullptr;
    }
    DXGI_FORMAT typeless, srv;
    if (!color_formats(d.Format, typeless, srv)) {
        color_note_ = "unsupported render target format " + std::to_string(int(d.Format));
        return nullptr;
    }
    if (!ensure_color(d)) return nullptr;
    color_note_.clear();
    return tex;
}

bool D3D11Capture::ensure_color(const D3D11_TEXTURE2D_DESC& src) {
    if (color_copy_ && src.Width == color_desc_.Width && src.Height == color_desc_.Height &&
        src.Format == color_desc_.Format && color_w_ == cap_w_ && color_h_ == cap_h_)
        return true;

    pending_.clear();  // anything in flight may refer to the old targets
    for (auto& s : color_stage_) s.Reset();
    color_cap_uav_.Reset(), color_cap_.Reset(), color_copy_srv_.Reset(), color_copy_.Reset();
    color_desc_ = {};

    DXGI_FORMAT typeless, srv_format;
    color_formats(src.Format, typeless, srv_format);
    D3D11_TEXTURE2D_DESC d{};
    d.Width = src.Width;
    d.Height = src.Height;
    d.MipLevels = d.ArraySize = 1;
    d.Format = typeless;
    d.SampleDesc.Count = 1;
    d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    D3D11_SHADER_RESOURCE_VIEW_DESC sd{srv_format, D3D11_SRV_DIMENSION_TEXTURE2D};
    sd.Texture2D.MipLevels = 1;
    if (FAILED(dev_->CreateTexture2D(&d, nullptr, &color_copy_)) ||
        FAILED(dev_->CreateShaderResourceView(color_copy_.Get(), &sd, &color_copy_srv_))) {
        color_note_ = "failed to create the color copy";
        return false;
    }
    d.Width = cap_w_;
    d.Height = cap_h_;
    d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;  // protocol.h: R in the lowest byte
    d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
    if (FAILED(dev_->CreateTexture2D(&d, nullptr, &color_cap_)) ||
        FAILED(dev_->CreateUnorderedAccessView(color_cap_.Get(), nullptr, &color_cap_uav_))) {
        color_note_ = "failed to create the color capture target";
        return false;
    }
    d.BindFlags = 0;
    d.Usage = D3D11_USAGE_STAGING;
    d.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    for (auto& s : color_stage_)
        if (FAILED(dev_->CreateTexture2D(&d, nullptr, &s))) {
            color_note_ = "failed to create the color staging textures";
            return false;
        }
    color_desc_ = src;
    color_w_ = cap_w_;
    color_h_ = cap_h_;
    return true;
}

bool D3D11Capture::capture(reshade::api::command_queue* queue, reshade::api::resource depth_res,
                           reshade::api::resource color_res, uint32_t capture_width, const FrameHeader& header) {
    auto* ctx = reinterpret_cast<ID3D11DeviceContext*>(queue->get_native());
    auto* depth = reinterpret_cast<ID3D11Resource*>(depth_res.handle);
    ComPtr<ID3D11Texture2D> tex;
    if (FAILED(depth->QueryInterface(IID_PPV_ARGS(&tex)))) {
        error_ = "depth buffer is not a 2D texture";
        return false;
    }
    D3D11_TEXTURE2D_DESC src;
    tex->GetDesc(&src);
    if (src.SampleDesc.Count > 1) {
        error_ = "multisampled depth buffers are not supported yet";
        return false;
    }
    if (!ensure_targets(src, capture_width)) return false;
    error_.clear();
    if (pending_.size() >= kStaging) {
        ++skipped_;
        return true;
    }

    ComPtr<ID3D11Texture2D> color;
    if (color_res != 0)
        color = color_source(color_res, src);
    else
        color_note_ = "turned off";

    // Subresource 0 only: mip 0 of the first slice is the scene depth (and color).
    ctx->CopySubresourceRegion(copy_.Get(), 0, 0, 0, 0, tex.Get(), 0, nullptr);
    if (color) ctx->CopySubresourceRegion(color_copy_.Get(), 0, 0, 0, 0, color.Get(), 0, nullptr);
    {
        const ComputeStateGuard guard(ctx);
        const DimsCB dims{src.Width, src.Height, cap_w_, cap_h_};
        ctx->UpdateSubresource(dims_cb_.Get(), 0, nullptr, &dims, 0, 0);
        ctx->CSSetShader(cs_.Get(), nullptr, 0);
        ctx->CSSetShaderResources(0, 1, copy_srv_.GetAddressOf());
        ctx->CSSetUnorderedAccessViews(0, 1, cap_uav_.GetAddressOf(), nullptr);
        ctx->CSSetConstantBuffers(0, 1, dims_cb_.GetAddressOf());
        ctx->Dispatch((cap_w_ + 7) / 8, (cap_h_ + 7) / 8, 1);
        if (color) {
            ctx->CSSetShader(color_cs_.Get(), nullptr, 0);
            ctx->CSSetShaderResources(1, 1, color_copy_srv_.GetAddressOf());
            ctx->CSSetUnorderedAccessViews(1, 1, color_cap_uav_.GetAddressOf(), nullptr);
            ctx->Dispatch((cap_w_ + 7) / 8, (cap_h_ + 7) / 8, 1);
        }
    }
    const int s = next_stage_;
    next_stage_ = (next_stage_ + 1) % kStaging;
    ctx->CopyResource(stage_[s].Get(), cap_.Get());
    if (color) ctx->CopyResource(color_stage_[s].Get(), color_cap_.Get());

    Pending p{s, header};
    if (color) p.header.flags |= kFlagHasColor;
    p.header.width = cap_w_;
    p.header.height = cap_h_;
    p.header.src_width = src.Width;
    p.header.src_height = src.Height;
    p.header.depth_format = uint32_t(DepthFormat::Float32Ndc);
    pending_.push_back(p);
    return true;
}

void D3D11Capture::publish(reshade::api::command_queue* queue, RingWriter& ring) {
    auto* ctx = reinterpret_cast<ID3D11DeviceContext*>(queue->get_native());
    while (!pending_.empty()) {
        const Pending& p = pending_.front();
        D3D11_MAPPED_SUBRESOURCE m;
        if (ctx->Map(stage_[p.stage].Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m) != S_OK) break;
        const bool has_color = (p.header.flags & kFlagHasColor) != 0;
        D3D11_MAPPED_SUBRESOURCE cm{};
        if (has_color &&
            ctx->Map(color_stage_[p.stage].Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &cm) != S_OK) {
            ctx->Unmap(stage_[p.stage].Get(), 0);
            break;
        }
        if (ring.is_open()) {
            Slot* slot = ring.begin_frame();
            slot->frame = p.header;
            const uint32_t w = p.header.width, h = p.header.height;
            for (uint32_t y = 0; y < h; ++y)
                std::memcpy(&slot->depth[y * w], static_cast<const uint8_t*>(m.pData) + y * m.RowPitch,
                            w * sizeof(float));
            if (has_color)
                for (uint32_t y = 0; y < h; ++y)
                    std::memcpy(&slot->color[y * w], static_cast<const uint8_t*>(cm.pData) + y * cm.RowPitch,
                                w * sizeof(uint32_t));
            ring.commit();
            ++published_;
        }
        if (has_color) ctx->Unmap(color_stage_[p.stage].Get(), 0);
        ctx->Unmap(stage_[p.stage].Get(), 0);
        pending_.pop_front();
    }
}

}  // namespace lidar
