// Small D3D11 helpers shared by the viewer's modules.
#pragma once
#include <cstring>

#include "app.h"

namespace lidar {

template <class T>
ComPtr<ID3D11Buffer> make_cbuffer(ID3D11Device* dev) {
    D3D11_BUFFER_DESC d{};
    d.ByteWidth = (sizeof(T) + 15) & ~15u;
    d.Usage = D3D11_USAGE_DYNAMIC;
    d.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    d.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    ComPtr<ID3D11Buffer> b;
    check(dev->CreateBuffer(&d, nullptr, &b), "CreateBuffer(cb)");
    return b;
}

template <class T>
void upload(ID3D11DeviceContext* ctx, ID3D11Buffer* b, const T& data) {
    D3D11_MAPPED_SUBRESOURCE m;
    check(ctx->Map(b, 0, D3D11_MAP_WRITE_DISCARD, 0, &m), "Map(cb)");
    std::memcpy(m.pData, &data, sizeof(T));
    ctx->Unmap(b, 0);
}

// GPU time between kMarks points in the frame, read back a few frames late without stalling.
// begin(), then mark(0..kMarks-1) in order, then end(), once a frame.
struct GpuTimer {
    static constexpr int kFrames = 6, kMarks = 4;
    ComPtr<ID3D11Query> disjoint[kFrames], stamps[kFrames][kMarks];
    int frame = 0;
    float ms[kMarks - 1] = {};  // smoothed time from each mark to the next

    void create(ID3D11Device* dev) {
        D3D11_QUERY_DESC d{D3D11_QUERY_TIMESTAMP_DISJOINT};
        for (auto& q : disjoint) check(dev->CreateQuery(&d, &q), "timer query");
        d.Query = D3D11_QUERY_TIMESTAMP;
        for (auto& f : stamps)
            for (auto& q : f) check(dev->CreateQuery(&d, &q), "timer query");
    }
    void begin(ID3D11DeviceContext* ctx) { ctx->Begin(disjoint[frame % kFrames].Get()); }
    void mark(ID3D11DeviceContext* ctx, int i) { ctx->End(stamps[frame % kFrames][i].Get()); }
    void end(ID3D11DeviceContext* ctx) {
        ctx->End(disjoint[frame % kFrames].Get());
        if (++frame < kFrames) return;
        const int f = frame % kFrames;  // the oldest, reused next frame: read it now or never
        D3D11_QUERY_DATA_TIMESTAMP_DISJOINT dj;
        if (ctx->GetData(disjoint[f].Get(), &dj, sizeof(dj), D3D11_ASYNC_GETDATA_DONOTFLUSH) != S_OK || dj.Disjoint)
            return;
        UINT64 t[kMarks];
        for (int i = 0; i < kMarks; ++i)
            if (ctx->GetData(stamps[f][i].Get(), &t[i], sizeof(t[i]), D3D11_ASYNC_GETDATA_DONOTFLUSH) != S_OK) return;
        for (int i = 0; i + 1 < kMarks; ++i) {
            const float now = float(double(t[i + 1] - t[i]) * 1000.0 / double(dj.Frequency));
            ms[i] += (now - ms[i]) * 0.1f;
        }
    }
};

}  // namespace lidar
