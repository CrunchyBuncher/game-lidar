// CPU reference for turning a protocol frame pixel into a world-space point.
// The viewer's ingest compute shader mirrors this exactly.
#pragma once
#include <DirectXMath.h>

#include <cmath>
#include <cstdint>

#include "protocol.h"

namespace lidar {

struct Unprojector {
    DirectX::XMFLOAT4X4 inv_proj;
    DirectX::XMFLOAT4X4 inv_view;
    uint32_t width, height, src_width, src_height;

    explicit Unprojector(const FrameHeader& h)
        : width(h.width), height(h.height), src_width(h.src_width), src_height(h.src_height) {
        using namespace DirectX;
        XMMATRIX v = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(h.view));
        XMMATRIX p = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(h.proj));
        XMStoreFloat4x4(&inv_view, XMMatrixInverse(nullptr, v));
        XMStoreFloat4x4(&inv_proj, XMMatrixInverse(nullptr, p));
    }

    // NDC of the source pixel that stored pixel (u, v) was sampled from.
    void ndc(uint32_t u, uint32_t v, float& x, float& y) const {
        uint32_t sx = ((2 * u + 1) * src_width) / (2 * width);
        uint32_t sy = ((2 * v + 1) * src_height) / (2 * height);
        if (sx > src_width - 1) sx = src_width - 1;
        if (sy > src_height - 1) sy = src_height - 1;
        x = (float(sx) + 0.5f) / float(src_width) * 2.0f - 1.0f;
        y = 1.0f - (float(sy) + 0.5f) / float(src_height) * 2.0f;
    }

    // Returns false for samples that don't produce a usable point (sky / far
    // plane / infinite / outside [near_cut, max_range]).
    bool unproject(uint32_t u, uint32_t v, float z, float near_cut, float max_range, float out[3]) const {
        using namespace DirectX;
        float x, y;
        ndc(u, v, x, y);
        XMVECTOR p = XMVector4Transform(XMVectorSet(x, y, z, 1.0f), XMLoadFloat4x4(&inv_proj));
        const float w = XMVectorGetW(p);
        if (std::fabs(w) < 1e-20f) return false;
        p = XMVectorScale(p, 1.0f / w);
        const float d = XMVectorGetX(XMVector3Length(p));
        if (!(d > near_cut && d < max_range)) return false;
        p = XMVector4Transform(XMVectorSetW(p, 1.0f), XMLoadFloat4x4(&inv_view));
        out[0] = XMVectorGetX(p);
        out[1] = XMVectorGetY(p);
        out[2] = XMVectorGetZ(p);
        return true;
    }
};

}  // namespace lidar
