// Unprojection (the CPU reference in unproject.h) across depth conventions.
#include "unproject.h"

#include <DirectXMath.h>

#include <cmath>
#include <cstdio>

#include "matrices.h"
#include "test.h"

using namespace DirectX;
using namespace lidar;
using namespace lidar::test;

TEST_CASE(unproject, depth_modes) {
    const XMMATRIX view = XMMatrixLookToLH(XMVectorSet(3, 1.7f, -8, 1), XMVectorSet(0.3f, -0.1f, 1, 0),
                                           XMVectorSet(0, 1, 0, 0));
    for (int mode = 0; mode < 3; ++mode) {
        const XMMATRIX proj = test_proj(mode, 16.0f / 9.0f);
        const XMMATRIX view_proj = view * proj;
        FrameHeader h{};
        h.width = 480;
        h.height = 270;
        h.src_width = 1280;
        h.src_height = 720;
        XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(h.view), view);
        XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(h.proj), proj);
        const Unprojector up(h);
        const XMMATRIX inv_view = XMMatrixInverse(nullptr, view);
        const XMMATRIX inv_proj = XMMatrixInverse(nullptr, proj);

        const uint32_t pixels[][2] = {{0, 0}, {479, 269}, {240, 135}, {17, 200}};
        for (const auto& px : pixels) {
            for (float dist : {0.5f, 12.3f, 180.0f}) {
                // Build the true world point along this pixel's ray at view depth `dist`.
                float nx, ny;
                up.ndc(px[0], px[1], nx, ny);
                XMVECTOR dir = XMVector3TransformCoord(XMVectorSet(nx, ny, 0.5f, 1), inv_proj);
                dir = XMVectorScale(dir, dist / XMVectorGetZ(dir));
                const XMVECTOR world = XMVector3TransformCoord(dir, inv_view);
                const float z = XMVectorGetZ(XMVector3TransformCoord(world, view_proj));

                float p[3];
                const bool ok = up.unproject(px[0], px[1], z, 0.3f, 500.0f, p);
                if (dist < 0.3f) {
                    EXPECT(!ok);
                    continue;
                }
                EXPECT(ok);
                const float err = std::sqrt((p[0] - XMVectorGetX(world)) * (p[0] - XMVectorGetX(world)) +
                                            (p[1] - XMVectorGetY(world)) * (p[1] - XMVectorGetY(world)) +
                                            (p[2] - XMVectorGetZ(world)) * (p[2] - XMVectorGetZ(world)));
                // Float depth precision: allow 0.1% of distance.
                if (!(err <= 1e-3f * dist + 1e-4f))
                    std::printf("  mode %d px (%u,%u) dist %.1f err %.5f\n", mode, px[0], px[1], dist, err);
                EXPECT(err <= 1e-3f * dist + 1e-4f);
            }
        }

        // Sky / far plane must be rejected.
        float p[3];
        const float far_z = mode == 0 ? 1.0f : 0.0f;
        EXPECT(!up.unproject(10, 10, far_z, 0.3f, 500.0f, p));
    }
}
