// Unit tests: ring buffer semantics and unprojection across depth conventions.
#include <DirectXMath.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "ring.h"
#include "unproject.h"

using namespace DirectX;
using namespace lidar;

static int g_failures = 0;
#define EXPECT(cond)                                                    \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                               \
        }                                                               \
    } while (0)

static void write_frame(RingWriter& w, uint64_t index) {
    Slot* s = w.begin_frame();
    s->frame = {};
    s->frame.frame_index = index;
    s->frame.width = 4;
    s->frame.height = 2;
    s->frame.flags = kFlagPoseValid;
    for (int i = 0; i < 8; ++i) s->depth[i] = float(index) + i * 0.1f;
    w.commit();
}

static void test_ring() {
    const wchar_t* name = L"Local\\game_lidar_test_ring";
    RingWriter w;
    EXPECT(w.open(name));
    RingReader r;
    EXPECT(r.try_open(name));

    Frame f;
    EXPECT(!r.read_next(f));  // empty

    for (uint64_t i = 1; i <= 10; ++i) write_frame(w, i);
    // Only the last kSlotCount frames survive, delivered oldest first.
    for (uint64_t expect = 10 - kSlotCount + 1; expect <= 10; ++expect) {
        EXPECT(r.read_next(f));
        EXPECT(f.header.frame_index == expect);
        EXPECT(f.depth.size() == 8 && f.depth[7] == float(expect) + 0.7f);
    }
    EXPECT(!r.read_next(f));

    write_frame(w, 11);
    EXPECT(r.read_next(f) && f.header.frame_index == 11);

    // Reader falls behind by more than the ring: skipped frames count as dropped.
    const uint64_t dropped_before = r.dropped();
    for (uint64_t i = 12; i <= 20; ++i) write_frame(w, i);
    uint64_t first = 0;
    int got = 0;
    while (r.read_next(f)) {
        if (!got) first = f.header.frame_index;
        ++got;
    }
    EXPECT(got == int(kSlotCount) && first == 20 - kSlotCount + 1);
    EXPECT(r.dropped() - dropped_before == 9 - kSlotCount);

    // Producer restart: reader resyncs to the new session.
    EXPECT(w.open(name));
    write_frame(w, 1);
    EXPECT(r.read_next(f) && f.header.frame_index == 1);
}

static XMMATRIX make_proj(int mode, float fov, float aspect) {
    const float n = 0.1f, fa = 1000.0f;
    if (mode == 0) return XMMatrixPerspectiveFovLH(fov, aspect, n, fa);
    if (mode == 1) return XMMatrixPerspectiveFovLH(fov, aspect, fa, n);
    const float ys = 1.0f / std::tan(fov * 0.5f), xs = ys / aspect;
    return XMMATRIX(xs, 0, 0, 0, 0, ys, 0, 0, 0, 0, 0, 1, 0, 0, n, 0);
}

static void test_unproject() {
    const XMMATRIX view = XMMatrixLookToLH(XMVectorSet(3, 1.7f, -8, 1), XMVectorSet(0.3f, -0.1f, 1, 0),
                                           XMVectorSet(0, 1, 0, 0));
    for (int mode = 0; mode < 3; ++mode) {
        const XMMATRIX proj = make_proj(mode, XMConvertToRadians(70.0f), 16.0f / 9.0f);
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

int main() {
    test_ring();
    test_unproject();
    if (g_failures == 0) std::printf("all tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
