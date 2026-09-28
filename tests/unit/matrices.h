// Matrix helpers shared by the camera tests.
#pragma once
#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace lidar::test {

// 70-degree projections: 0 standard, 1 reversed, 2 reversed-infinite. Near 0.1, far 1000.
inline DirectX::XMMATRIX test_proj(int mode, float aspect) {
    using namespace DirectX;
    const float fov = XMConvertToRadians(70.0f), n = 0.1f, f = 1000.0f;
    if (mode == 0) return XMMatrixPerspectiveFovLH(fov, aspect, n, f);
    if (mode == 1) return XMMatrixPerspectiveFovLH(fov, aspect, f, n);
    const float ys = 1.0f / std::tan(fov * 0.5f), xs = ys / aspect;
    return XMMATRIX(xs, 0, 0, 0, 0, ys, 0, 0, 0, 0, 0, 1, 0, 0, n, 0);
}

// Writes m into a constant buffer image at offset, as a game would.
inline void put(uint8_t* buf, uint32_t offset, const DirectX::XMMATRIX& m, bool column_major) {
    DirectX::XMFLOAT4X4 f;
    DirectX::XMStoreFloat4x4(&f, column_major ? DirectX::XMMatrixTranspose(m) : m);
    std::memcpy(buf + offset, &f, sizeof(f));
}

inline void store_f(const DirectX::XMMATRIX& m, bool column_major, float f[16]) {
    DirectX::XMStoreFloat4x4(reinterpret_cast<DirectX::XMFLOAT4X4*>(f),
                             column_major ? DirectX::XMMatrixTranspose(m) : m);
}

inline float max_diff(const float a[16], const DirectX::XMMATRIX& b) {
    DirectX::XMFLOAT4X4 f;
    DirectX::XMStoreFloat4x4(&f, b);
    float d = 0;
    for (int i = 0; i < 16; ++i) d = std::max(d, std::abs(a[i] - (&f._11)[i]));
    return d;
}

}  // namespace lidar::test
