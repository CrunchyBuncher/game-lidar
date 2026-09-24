#include "capture_d3d9.h"

#include <d3dcompiler.h>

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace reshade::api;

namespace lidar {
namespace {

constexpr D3DFORMAT kIntz = static_cast<D3DFORMAT>(MAKEFOURCC('I', 'N', 'T', 'Z'));

// Same mapping as downsample_hlsl.h (protocol.h), in ps_3_0. SM3 has no integer math, so the
// integer division is done in floats: every value is an integer below 2^24 and exact, and the
// quotient, which the GPU computes as a reciprocal times a product, is corrected by one if needed.
const char* kDownsampleHlsl9 = R"(
sampler2D src_depth : register(s0);
float4 dims : register(c0);  // src width, src height, dst width, dst height

float4 vs_main(float4 p : POSITION) : POSITION { return p; }

float4 ps_main(float2 vpos : VPOS) : COLOR0 {
    float2 num = (2 * floor(vpos) + 1) * dims.xy, den = 2 * dims.zw;
    float2 s = floor(num / den);
    s += step((s + 1) * den, num);  // quotient one too small
    s -= step(num + 1, s * den);    // one too large
    s = min(s, dims.xy - 1);
    return tex2Dlod(src_depth, float4((s + 0.5) / dims.xy, 0, 0)).rrrr;
}
)";

// Everything the pass changes, restored afterwards. After ReShade's d3d9 state_block (Copyright (C)
// 2014 Patrick Mours, BSD-3-Clause): a state block for the pipeline state, plus the render targets,
// depth-stencil and viewport, which state blocks don't hold. Created per use, so it holds no
// references to the game's resources between frames (they'd block a Reset).
class StateGuard {
public:
    StateGuard(IDirect3DDevice9* dev, DWORD num_rts, DWORD vertex_processing)
        : dev_(dev), num_rts_(num_rts), mixed_(vertex_processing & D3DCREATE_MIXED_VERTEXPROCESSING) {
        if (SUCCEEDED(dev_->CreateStateBlock(D3DSBT_ALL, &block_))) block_->Capture();
        dev_->GetViewport(&viewport_);
        for (DWORD i = 0; i < num_rts_; ++i) dev_->GetRenderTarget(i, &rts_[i]);
        dev_->GetDepthStencilSurface(&ds_);
        if (mixed_) {  // shaders need hardware vertex processing
            software_ = dev_->GetSoftwareVertexProcessing();
            dev_->SetSoftwareVertexProcessing(FALSE);
        }
    }
    ~StateGuard() {
        if (block_) block_->Apply();
        if (mixed_) dev_->SetSoftwareVertexProcessing(software_);
        for (DWORD i = 0; i < num_rts_; ++i) dev_->SetRenderTarget(i, rts_[i].Get());
        dev_->SetDepthStencilSurface(ds_.Get());
        dev_->SetViewport(&viewport_);  // after the render targets, which reset it
    }
    bool ok() const { return block_ != nullptr && rts_[0] != nullptr; }

private:
    IDirect3DDevice9* dev_;
    DWORD num_rts_;
    bool mixed_;
    BOOL software_ = FALSE;
    Microsoft::WRL::ComPtr<IDirect3DStateBlock9> block_;
    D3DVIEWPORT9 viewport_{};
    Microsoft::WRL::ComPtr<IDirect3DSurface9> rts_[4];
    Microsoft::WRL::ComPtr<IDirect3DSurface9> ds_;
};

struct __declspec(uuid("3e8b1d74-a52c-4f09-b6e3-7c0d9f2a4b18")) DeviceData {
    bool intz = false;              // the driver can create (and shaders sample) INTZ depth textures
    D3D9Capture* capture = nullptr;  // resources to drop before a Reset
};

bool is_d3d9(device* dev) { return dev->get_api() == device_api::d3d9; }

// Replaces the game's scene depth-stencils with INTZ textures. Filters as generic_depth does:
// multisampled buffers can't be INTZ, depth textures in D16/D24 formats are likely sampled as PCF
// shadow maps (INTZ would break that), and small ones are likely shadow maps too.
bool on_create_resource(device* dev, resource_desc& desc, subresource_data*, resource_usage) {
    const DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr || !dd->intz) return false;
    if (desc.type != resource_type::surface && desc.type != resource_type::texture_2d) return false;
    if ((desc.usage & resource_usage::depth_stencil) == 0 || desc.texture.samples > 1) return false;
    if (desc.texture.format == format::s8_uint || desc.texture.format == format::intz) return false;
    if (desc.type == resource_type::texture_2d &&
        (desc.texture.format == format::d16_unorm || desc.texture.format == format::d24_unorm_x8_uint ||
         desc.texture.format == format::d24_unorm_s8_uint))
        return false;
    if (desc.texture.width <= 512) return false;
    desc.texture.format = format::intz;
    desc.usage |= resource_usage::shader_resource;
    return true;
}

void on_destroy_device(device* dev) {
    if (is_d3d9(dev)) dev->destroy_private_data<DeviceData>();
}

}  // namespace

// Before a Reset, ReShade destroys the device's queue (and creates it again after).
void D3D9Capture::on_destroy_command_queue(command_queue* q) {
    if (const DeviceData* dd = q->get_device()->get_private_data<DeviceData>(); dd != nullptr && dd->capture)
        dd->capture->release_targets();
}

void D3D9Capture::init_device(device* dev) {
    if (!is_d3d9(dev) || dev->get_private_data<DeviceData>() != nullptr) return;
    DeviceData* dd = dev->create_private_data<DeviceData>();
    auto* d = reinterpret_cast<IDirect3DDevice9*>(dev->get_native());
    Microsoft::WRL::ComPtr<IDirect3D9> d3d;
    D3DDEVICE_CREATION_PARAMETERS cp{};
    D3DDISPLAYMODE mode{};
    if (SUCCEEDED(d->GetDirect3D(&d3d)) && SUCCEEDED(d->GetCreationParameters(&cp)) &&
        SUCCEEDED(d3d->GetAdapterDisplayMode(cp.AdapterOrdinal, &mode)))
        dd->intz = SUCCEEDED(d3d->CheckDeviceFormat(cp.AdapterOrdinal, cp.DeviceType, mode.Format,
                                                    D3DUSAGE_DEPTHSTENCIL, D3DRTYPE_TEXTURE, kIntz));
    if (!dd->intz)
        reshade::log::message(reshade::log::level::warning,
                              "The driver doesn't support INTZ depth textures, so depth can't be captured.");
}

void D3D9Capture::register_events() {
    using reshade::addon_event;
    reshade::register_event<addon_event::init_device>(init_device);
    reshade::register_event<addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<addon_event::create_resource>(on_create_resource);
    reshade::register_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
}

void D3D9Capture::unregister_events() {
    using reshade::addon_event;
    reshade::unregister_event<addon_event::init_device>(init_device);
    reshade::unregister_event<addon_event::destroy_device>(on_destroy_device);
    reshade::unregister_event<addon_event::create_resource>(on_create_resource);
    reshade::unregister_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
}

D3D9Capture::~D3D9Capture() { release(); }

bool D3D9Capture::init(device* dev) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) {
        error_ = "device not set up";
        return false;
    }
    rdev_ = dev;
    dev_ = reinterpret_cast<IDirect3DDevice9*>(dev->get_native());
    D3DCAPS9 caps{};
    dev_->GetDeviceCaps(&caps);
    num_rts_ = std::clamp<DWORD>(caps.NumSimultaneousRTs, 1, 4);
    D3DDEVICE_CREATION_PARAMETERS cp{};
    dev_->GetCreationParameters(&cp);
    vertex_processing_ = cp.BehaviorFlags;

    for (const char* entry : {"vs_main", "ps_main"}) {
        const bool vs = entry[0] == 'v';
        ComPtr<ID3DBlob> code, errors;
        if (FAILED(D3DCompile(kDownsampleHlsl9, std::strlen(kDownsampleHlsl9), "lidar_downsample9", nullptr, nullptr,
                              entry, vs ? "vs_3_0" : "ps_3_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors))) {
            error_ = errors ? static_cast<const char*>(errors->GetBufferPointer()) : "D3DCompile failed";
            return false;
        }
        const auto* bytecode = static_cast<const DWORD*>(code->GetBufferPointer());
        if (vs ? FAILED(dev_->CreateVertexShader(bytecode, &vs_)) : FAILED(dev_->CreatePixelShader(bytecode, &ps_))) {
            error_ = "failed to create the downsample shaders (needs shader model 3)";
            return false;
        }
    }
    const D3DVERTEXELEMENT9 elems[] = {
        {0, 0, D3DDECLTYPE_FLOAT4, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_POSITION, 0},
        D3DDECL_END(),
    };
    if (FAILED(dev_->CreateVertexDeclaration(elems, &decl_))) {
        error_ = "CreateVertexDeclaration failed";
        return false;
    }
    dd->capture = this;
    return true;
}

void D3D9Capture::release() {
    release_targets();
    if (rdev_ != nullptr)
        if (DeviceData* dd = rdev_->get_private_data<DeviceData>(); dd != nullptr && dd->capture == this)
            dd->capture = nullptr;
    decl_.Reset();
    ps_.Reset();
    vs_.Reset();
    dev_.Reset();
    rdev_ = nullptr;
}

void D3D9Capture::release_targets() {
    pending_.clear();
    for (int i = 0; i < kReadback; ++i) {
        rt_[i].Reset();
        sys_[i].Reset();
        done_[i].Reset();
        copied_[i].Reset();
    }
    src_w_ = src_h_ = 0;
    cap_w_ = cap_h_ = 0;
}

bool D3D9Capture::ensure_targets(uint32_t src_w, uint32_t src_h, uint32_t capture_width) {
    const uint32_t cap_w = std::clamp(capture_width, 16u, std::min(kMaxWidth, src_w));
    const uint32_t cap_h = std::min(uint32_t(std::lround(double(cap_w) * src_h / src_w)), kMaxHeight);
    if (rt_[0] && src_w == src_w_ && src_h == src_h_ && cap_w == cap_w_ && cap_h == cap_h_) return true;

    release_targets();  // anything in flight refers to the old targets
    for (int i = 0; i < kReadback; ++i) {
        if (FAILED(dev_->CreateRenderTarget(cap_w, cap_h, D3DFMT_R32F, D3DMULTISAMPLE_NONE, 0, FALSE, &rt_[i],
                                            nullptr)) ||
            FAILED(dev_->CreateOffscreenPlainSurface(cap_w, cap_h, D3DFMT_R32F, D3DPOOL_SYSTEMMEM, &sys_[i],
                                                     nullptr)) ||
            FAILED(dev_->CreateQuery(D3DQUERYTYPE_EVENT, &done_[i])) ||
            FAILED(dev_->CreateQuery(D3DQUERYTYPE_EVENT, &copied_[i]))) {
            release_targets();
            error_ = "failed to create the capture targets (R32F render target)";
            return false;
        }
    }
    src_w_ = src_w;
    src_h_ = src_h;
    cap_w_ = cap_w;
    cap_h_ = cap_h;
    return true;
}

bool D3D9Capture::capture(command_queue*, resource depth, uint32_t capture_width, const FrameHeader& header) {
    auto* res = reinterpret_cast<IDirect3DResource9*>(depth.handle);
    D3DSURFACE_DESC src{};
    if (res->GetType() == D3DRTYPE_TEXTURE)
        static_cast<IDirect3DTexture9*>(res)->GetLevelDesc(0, &src);
    else if (res->GetType() == D3DRTYPE_SURFACE)
        static_cast<IDirect3DSurface9*>(res)->GetDesc(&src);
    if (src.Format != kIntz) {
        const DeviceData* dd = rdev_->get_private_data<DeviceData>();
        error_ = !dd->intz ? "the driver doesn't support INTZ, so D3D9 depth can't be read"
                 : src.MultiSampleType != D3DMULTISAMPLE_NONE
                     ? "multisampled depth buffers can't be read in D3D9 (turn off MSAA in the game)"
                     : "this depth buffer isn't readable: it wasn't created as INTZ (was it created before the addon "
                       "loaded, or is it a small one?)";
        return false;
    }
    if (!ensure_targets(src.Width, src.Height, capture_width)) return false;
    error_.clear();
    if (pending_.size() >= kReadback) {
        ++skipped_;
        return true;
    }

    const int s = next_slot_;
    {
        const StateGuard guard(dev_.Get(), num_rts_, vertex_processing_);
        if (!guard.ok()) {
            error_ = "failed to save the game's render state";
            return false;
        }
        dev_->SetRenderTarget(0, rt_[s].Get());
        for (DWORD i = 1; i < num_rts_; ++i) dev_->SetRenderTarget(i, nullptr);
        dev_->SetDepthStencilSurface(nullptr);
        const D3DVIEWPORT9 vp{0, 0, cap_w_, cap_h_, 0.0f, 1.0f};
        dev_->SetViewport(&vp);

        dev_->SetVertexDeclaration(decl_.Get());
        dev_->SetVertexShader(vs_.Get());
        dev_->SetPixelShader(ps_.Get());
        const float dims[4] = {float(src.Width), float(src.Height), float(cap_w_), float(cap_h_)};
        dev_->SetPixelShaderConstantF(0, dims, 1);
        dev_->SetTexture(0, static_cast<IDirect3DBaseTexture9*>(static_cast<IDirect3DTexture9*>(res)));
        dev_->SetSamplerState(0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
        dev_->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
        dev_->SetSamplerState(0, D3DSAMP_MAGFILTER, D3DTEXF_POINT);
        dev_->SetSamplerState(0, D3DSAMP_MINFILTER, D3DTEXF_POINT);
        dev_->SetSamplerState(0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
        dev_->SetSamplerState(0, D3DSAMP_MAXMIPLEVEL, 0);
        dev_->SetSamplerState(0, D3DSAMP_SRGBTEXTURE, FALSE);
        const std::pair<D3DRENDERSTATETYPE, DWORD> states[] = {
            {D3DRS_ZENABLE, D3DZB_FALSE},
            {D3DRS_ZWRITEENABLE, FALSE},
            {D3DRS_STENCILENABLE, FALSE},
            {D3DRS_FILLMODE, D3DFILL_SOLID},
            {D3DRS_CULLMODE, D3DCULL_NONE},
            {D3DRS_ALPHATESTENABLE, FALSE},
            {D3DRS_ALPHABLENDENABLE, FALSE},
            {D3DRS_SEPARATEALPHABLENDENABLE, FALSE},
            {D3DRS_COLORWRITEENABLE, 0xF},
            {D3DRS_SCISSORTESTENABLE, FALSE},
            {D3DRS_CLIPPLANEENABLE, 0},
            {D3DRS_SRGBWRITEENABLE, FALSE},
            {D3DRS_FOGENABLE, FALSE},
            {D3DRS_MULTISAMPLEANTIALIAS, FALSE},
        };
        for (const auto& [state, value] : states) dev_->SetRenderState(state, value);

        // One triangle covering the target. The pixel shader works from VPOS, so no texcoords and no
        // half-pixel concerns.
        const float tri[3][4] = {{-1, -1, 0, 1}, {-1, 3, 0, 1}, {3, -1, 0, 1}};
        if (FAILED(dev_->DrawPrimitiveUP(D3DPT_TRIANGLELIST, 1, tri, sizeof(tri[0])))) {
            error_ = "the downsample draw failed";
            return false;
        }
    }
    done_[s]->Issue(D3DISSUE_END);
    next_slot_ = (next_slot_ + 1) % kReadback;

    Pending p{s, header};
    p.header.width = cap_w_;
    p.header.height = cap_h_;
    p.header.src_width = src.Width;
    p.header.src_height = src.Height;
    p.header.depth_format = uint32_t(DepthFormat::Float32Ndc);
    pending_.push_back(p);
    return true;
}

// Two steps, neither of which waits: once done_ says the GPU has finished the capture's draw,
// GetRenderTargetData queues the copy to system memory with copied_ behind it; once copied_ has
// finished too (this or a later present), LockRect reads it. A copy that never finishes costs its
// frame, not the game.
void D3D9Capture::publish(command_queue*, RingWriter& ring) {
    while (!pending_.empty()) {
        Pending& p = pending_.front();
        if (!p.copied) {
            const HRESULT q = done_[p.slot]->GetData(nullptr, 0, 0);
            if (q == S_FALSE) break;  // still running
            if (q != S_OK || FAILED(dev_->GetRenderTargetData(rt_[p.slot].Get(), sys_[p.slot].Get())) ||
                FAILED(copied_[p.slot]->Issue(D3DISSUE_END))) {
                pending_.pop_front();  // lost with the device
                continue;
            }
            p.copied = true;
        }
        const HRESULT q = copied_[p.slot]->GetData(nullptr, 0, 0);
        if (q == S_FALSE) {
            if (++p.copy_waits < kMaxCopyWaits) break;
            ++skipped_;  // gave up on this one
            pending_.pop_front();
            continue;
        }
        D3DLOCKED_RECT lr{};
        if (q == S_OK && SUCCEEDED(sys_[p.slot]->LockRect(&lr, nullptr, D3DLOCK_READONLY | D3DLOCK_DONOTWAIT))) {
            if (ring.is_open()) {
                Slot* slot = ring.begin_frame();
                slot->frame = p.header;
                const uint32_t w = p.header.width, h = p.header.height;
                for (uint32_t y = 0; y < h; ++y)
                    std::memcpy(&slot->depth[y * w], static_cast<const uint8_t*>(lr.pBits) + y * lr.Pitch,
                                w * sizeof(float));
                ring.commit();
                ++published_;
            }
            sys_[p.slot]->UnlockRect();
        }
        pending_.pop_front();  // published, or lost with the device
    }
}

}  // namespace lidar
