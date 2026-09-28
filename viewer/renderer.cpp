#include "renderer.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "gpu.h"
#include "shaders.h"

using namespace DirectX;

namespace lidar {

namespace {

struct DrawCB {
    XMFLOAT4X4 view_proj;
    float px_to_ndc[2];
    float point_size;
    uint32_t color_mode;
    float height_min, height_max;
    uint32_t capacity;
    float px_per_m;  // 0: points are point_size pixels; else world_size metres, clamped to 1..max_px pixels
    float height_axis[4];  // a point's height as shown: dot((pos, 1), height_axis), for the tilt
    float cut_plane[4];    // hides points where dot((pos, 1), cut_plane) > 0; (0, 0, 0, -1) is off
    float cut_a[3], cut_r2;  // line-of-sight cylinder from cut_a to cut_a + cut_ab; radius 0 is off
    float cut_ab[3], cut_inv_ab2;
    float cut_base[4];  // the cylinder hides only where dot((pos, 1), cut_base) > 0
    float world_size, max_px, pad[2];
};

constexpr UINT kMaxLineVerts = 65536;

}  // namespace

DrawParams make_draw_params(const ViewParams& view, const ViewTilt& tilt, const PointStyle& style, float voxel,
                            const Cutaways& cut) {
    DrawParams p;
    XMStoreFloat4x4(&p.view_proj, tilt.matrix() * view.view * view.proj);
    p.fov_y = view.fov_y;
    tilt.height_axis(p.height_axis);
    p.voxel = voxel;
    p.style = style;
    p.cut = cut;
    return p;
}

void player_lines(const PlayerCamera& player, bool frustum, bool trail, std::vector<LineVertex>& out) {
    if (frustum && player.valid) {
        const XMMATRIX inv_proj = XMLoadFloat4x4(&player.inv_proj);
        const XMMATRIX inv_view = XMLoadFloat4x4(&player.inv_view);
        const XMVECTOR apex = XMVector3TransformCoord(XMVectorZero(), inv_view);
        XMVECTOR corners[4];
        const float cx[4] = {-1, 1, 1, -1}, cy[4] = {-1, -1, 1, 1};
        for (int k = 0; k < 4; ++k) {
            // Any depth works for a direction; scale the ray to 2 m of view depth.
            XMVECTOR v = XMVector3TransformCoord(XMVectorSet(cx[k], cy[k], 0.5f, 1), inv_proj);
            v = XMVectorScale(v, 2.0f / std::max(std::fabs(XMVectorGetZ(v)), 1e-6f));
            corners[k] = XMVector3TransformCoord(v, inv_view);
        }
        const uint32_t yellow = 0xFF00FFFF;
        auto add = [&](XMVECTOR a, XMVECTOR b) {
            LineVertex va{{}, yellow}, vb{{}, yellow};
            XMStoreFloat3(&va.pos, a);
            XMStoreFloat3(&vb.pos, b);
            out.push_back(va);
            out.push_back(vb);
        };
        for (int k = 0; k < 4; ++k) {
            add(apex, corners[k]);
            add(corners[k], corners[(k + 1) % 4]);
        }
    }
    const auto& t = player.trail;
    if (trail && t.size() > 1 && out.size() < kMaxLineVerts) {
        const size_t max_segments = (kMaxLineVerts - out.size()) / 2;
        const size_t start = t.size() - 1 > max_segments ? t.size() - 1 - max_segments : 0;
        for (size_t k = start; k + 1 < t.size(); ++k) {
            out.push_back({t[k], 0xFFFFFFFF});
            out.push_back({t[k + 1], 0xFFFFFFFF});
        }
    }
}

void Renderer::create(ID3D11Device* dev, ID3D11DeviceContext* ctx, int width, int height) {
    dev_ = dev;
    ctx_ = ctx;
    auto vsp_blob = compile_shader(shaders::kDraw, "vs_points", "vs_5_0");
    auto vsl_blob = compile_shader(shaders::kDraw, "vs_lines", "vs_5_0");
    auto ps_blob = compile_shader(shaders::kDraw, "ps_main", "ps_5_0");
    check(dev->CreateVertexShader(vsp_blob->GetBufferPointer(), vsp_blob->GetBufferSize(), nullptr, &vs_points_),
          "vs_points");
    check(dev->CreateVertexShader(vsl_blob->GetBufferPointer(), vsl_blob->GetBufferSize(), nullptr, &vs_lines_),
          "vs_lines");
    check(dev->CreatePixelShader(ps_blob->GetBufferPointer(), ps_blob->GetBufferSize(), nullptr, &ps_), "ps");

    const D3D11_INPUT_ELEMENT_DESC line_layout_desc[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
    };
    check(dev->CreateInputLayout(line_layout_desc, 2, vsl_blob->GetBufferPointer(), vsl_blob->GetBufferSize(),
                                 &line_layout_),
          "line layout");

    // Point quads, one batch's worth: 4 vertices each, drawn as two triangles. The draw repeats it
    // (one instance per batch) and the vertex shader finds the point from the instance and vertex ids.
    {
        std::vector<uint16_t> idx(size_t(shaders::kQuadsPerBatch) * 6);
        for (uint32_t q = 0; q < shaders::kQuadsPerBatch; ++q) {
            const uint16_t b = uint16_t(q * 4);
            const uint16_t tri[6] = {b, uint16_t(b + 1), uint16_t(b + 2), b, uint16_t(b + 2), uint16_t(b + 3)};
            std::memcpy(&idx[q * 6], tri, sizeof(tri));
        }
        D3D11_BUFFER_DESC d{UINT(idx.size() * sizeof(uint16_t)), D3D11_USAGE_IMMUTABLE, D3D11_BIND_INDEX_BUFFER};
        D3D11_SUBRESOURCE_DATA init{idx.data()};
        check(dev->CreateBuffer(&d, &init, &quad_ib_), "quad ib");
    }
    {
        D3D11_BUFFER_DESC d{kMaxLineVerts * sizeof(LineVertex), D3D11_USAGE_DYNAMIC, D3D11_BIND_VERTEX_BUFFER,
                            D3D11_CPU_ACCESS_WRITE};
        check(dev->CreateBuffer(&d, nullptr, &line_vb_), "line vb");
    }
    draw_cb_ = make_cbuffer<DrawCB>(dev);
    {
        D3D11_DEPTH_STENCIL_DESC d{};
        d.DepthEnable = TRUE;
        d.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
        d.DepthFunc = D3D11_COMPARISON_GREATER;  // reversed-Z
        check(dev->CreateDepthStencilState(&d, &dss_), "dss");
    }
    {
        D3D11_RASTERIZER_DESC d{};
        d.FillMode = D3D11_FILL_SOLID;
        d.CullMode = D3D11_CULL_NONE;
        d.DepthClipEnable = TRUE;
        check(dev->CreateRasterizerState(&d, &rs_), "rs");
    }
    resize(width, height);
}

void Renderer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    D3D11_TEXTURE2D_DESC d{};
    d.Width = width;
    d.Height = height;
    d.MipLevels = d.ArraySize = 1;
    d.SampleDesc.Count = 1;
    d.Format = DXGI_FORMAT_D32_FLOAT;
    d.BindFlags = D3D11_BIND_DEPTH_STENCIL;
    check(dev_->CreateTexture2D(&d, nullptr, &zbuf_), "zbuf");
    check(dev_->CreateDepthStencilView(zbuf_.Get(), nullptr, &zbuf_dsv_), "zbuf dsv");
}

void Renderer::begin(ID3D11RenderTargetView* rtv, const DrawParams& p, uint32_t capacity) {
    ID3D11DeviceContext* ctx = ctx_;
    DrawCB cb{};
    cb.view_proj = p.view_proj;
    std::memcpy(cb.height_axis, p.height_axis, sizeof(cb.height_axis));
    std::memcpy(cb.cut_plane, p.cut.plane, sizeof(cb.cut_plane));
    std::memcpy(cb.cut_a, p.cut.a, sizeof(cb.cut_a));
    std::memcpy(cb.cut_ab, p.cut.ab, sizeof(cb.cut_ab));
    std::memcpy(cb.cut_base, p.cut.base, sizeof(cb.cut_base));
    cb.cut_r2 = p.cut.r2;
    cb.cut_inv_ab2 = p.cut.inv_ab2;
    cb.px_to_ndc[0] = 2.0f / float(width_);
    cb.px_to_ndc[1] = 2.0f / float(height_);
    cb.point_size = p.style.point_size;
    if (p.style.world_points) {
        cb.px_per_m = 0.5f * float(height_) / std::tan(0.5f * p.fov_y);
        cb.world_size = p.style.world_scale * p.voxel;
        cb.max_px = p.style.world_max_px;
    }
    cb.color_mode = p.style.color_mode;
    cb.capacity = capacity;
    cb.height_min = p.style.height_min;
    cb.height_max = p.style.height_max;
    if (!(cb.height_max - cb.height_min > 0.01f)) cb.height_max = cb.height_min + 0.01f;  // flat cloud
    upload(ctx, draw_cb_.Get(), cb);

    const float bg[4] = {0.03f, 0.035f, 0.045f, 1.0f};
    ctx->ClearRenderTargetView(rtv, bg);
    ctx->ClearDepthStencilView(zbuf_dsv_.Get(), D3D11_CLEAR_DEPTH, 0.0f, 0);
    ctx->OMSetRenderTargets(1, &rtv, zbuf_dsv_.Get());
    ctx->OMSetDepthStencilState(dss_.Get(), 0);
    ctx->RSSetState(rs_.Get());
    D3D11_VIEWPORT vp{0, 0, float(width_), float(height_), 0, 1};
    ctx->RSSetViewports(1, &vp);
    ctx->VSSetConstantBuffers(0, 1, draw_cb_.GetAddressOf());
    ctx->PSSetShader(ps_.Get(), nullptr, 0);
}

void Renderer::draw_points(const PointCloud& cloud) {
    ID3D11DeviceContext* ctx = ctx_;
    ctx->IASetInputLayout(nullptr);
    ctx->IASetIndexBuffer(quad_ib_.Get(), DXGI_FORMAT_R16_UINT, 0);
    ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx->VSSetShader(vs_points_.Get(), nullptr, 0);
    ID3D11ShaderResourceView* point_srvs[2] = {cloud.points_srv(), cloud.counter_srv()};
    ctx->VSSetShaderResources(0, 2, point_srvs);
    ctx->DrawIndexedInstancedIndirect(cloud.args(), 0);
    ID3D11ShaderResourceView* null_srvs[2] = {};
    ctx->VSSetShaderResources(0, 2, null_srvs);
}

void Renderer::draw_lines(const std::vector<LineVertex>& lines) {
    if (lines.empty()) return;
    ID3D11DeviceContext* ctx = ctx_;
    const size_t n = std::min<size_t>(lines.size(), kMaxLineVerts);
    D3D11_MAPPED_SUBRESOURCE m;
    check(ctx->Map(line_vb_.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m), "Map lines");
    std::memcpy(m.pData, lines.data(), n * sizeof(LineVertex));
    ctx->Unmap(line_vb_.Get(), 0);
    const UINT stride = sizeof(LineVertex), offset = 0;
    ctx->IASetVertexBuffers(0, 1, line_vb_.GetAddressOf(), &stride, &offset);
    ctx->IASetInputLayout(line_layout_.Get());
    ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
    ctx->VSSetShader(vs_lines_.Get(), nullptr, 0);
    ctx->Draw(UINT(n), 0);
}

}  // namespace lidar
