#include "point_cloud.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <vector>

#include "gpu.h"
#include "shaders.h"

using namespace DirectX;

namespace lidar {

namespace {

struct FrameCB {
    XMFLOAT4X4 view, proj, inv_view, inv_proj;
    uint32_t dims[2], src_dims[2];
    float near_cut, max_range, voxel_size;
    uint32_t table_mask, capacity, has_color;
    float carve_margin_abs, carve_margin_rel;
    uint32_t carve_on, color_update;
};
struct HistCB {
    float height_axis[4];
    float lo, hi;
    uint32_t capacity, bins;
};
struct PointData {
    float pos[3];
    uint32_t color;
};

constexpr UINT kHistBytes = (2 + shaders::kHistBins) * 4;

// Inverse of the shaders' height_key().
float height_from_key(uint32_t key) {
    const uint32_t u = (key & 0x80000000u) ? (key & 0x7FFFFFFFu) : ~key;
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

ComPtr<ID3D11ComputeShader> make_cs(ID3D11Device* dev, const char* src, const char* entry) {
    auto blob = compile_shader(src, entry, "cs_5_0");
    ComPtr<ID3D11ComputeShader> cs;
    check(dev->CreateComputeShader(blob->GetBufferPointer(), blob->GetBufferSize(), nullptr, &cs), entry);
    return cs;
}

uint32_t read_counter_blocking(ID3D11Device* dev, ID3D11DeviceContext* ctx, ID3D11Buffer* counter) {
    D3D11_BUFFER_DESC d{16, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
    ComPtr<ID3D11Buffer> s;
    check(dev->CreateBuffer(&d, nullptr, &s), "staging");
    ctx->CopyResource(s.Get(), counter);
    D3D11_MAPPED_SUBRESOURCE m;
    check(ctx->Map(s.Get(), 0, D3D11_MAP_READ, 0, &m), "Map counter");
    const uint32_t n = *static_cast<const uint32_t*>(m.pData);
    ctx->Unmap(s.Get(), 0);
    return n;
}

}  // namespace

uint32_t table_bits_for(uint32_t capacity) {
    uint32_t bits = 16;
    while (bits < 28 && (1ull << bits) < 2ull * capacity) ++bits;
    return bits;
}

double pool_bytes(uint32_t capacity) {
    return double(capacity) * (sizeof(PointData) + 4) + double(1ull << table_bits_for(capacity)) * 4;
}

bool PointCloud::Pool::create(ID3D11Device* dev, uint32_t cap, uint32_t table_bits) {
    capacity = cap;
    table_size = 1u << table_bits;

    D3D11_BUFFER_DESC d{};
    d.ByteWidth = 16;
    d.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;  // the draw reads the count
    d.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
    if (FAILED(dev->CreateBuffer(&d, nullptr, &counter))) return false;
    D3D11_UNORDERED_ACCESS_VIEW_DESC u{DXGI_FORMAT_R32_TYPELESS, D3D11_UAV_DIMENSION_BUFFER};
    u.Buffer.NumElements = 4;
    u.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
    if (FAILED(dev->CreateUnorderedAccessView(counter.Get(), &u, &counter_uav))) return false;
    D3D11_SHADER_RESOURCE_VIEW_DESC sv{DXGI_FORMAT_R32_TYPELESS, D3D11_SRV_DIMENSION_BUFFEREX};
    sv.BufferEx.NumElements = 4;
    sv.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
    if (FAILED(dev->CreateShaderResourceView(counter.Get(), &sv, &counter_srv))) return false;

    d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
    d.ByteWidth = table_size * 4;
    d.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    d.StructureByteStride = 4;
    if (FAILED(dev->CreateBuffer(&d, nullptr, &table))) return false;
    if (FAILED(dev->CreateUnorderedAccessView(table.Get(), nullptr, &table_uav))) return false;

    d.ByteWidth = capacity * 4;
    if (FAILED(dev->CreateBuffer(&d, nullptr, &free_list))) return false;
    if (FAILED(dev->CreateUnorderedAccessView(free_list.Get(), nullptr, &free_list_uav))) return false;

    d.ByteWidth = capacity * sizeof(PointData);
    d.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    d.StructureByteStride = sizeof(PointData);
    if (FAILED(dev->CreateBuffer(&d, nullptr, &points))) return false;
    if (FAILED(dev->CreateUnorderedAccessView(points.Get(), nullptr, &points_uav))) return false;
    if (FAILED(dev->CreateShaderResourceView(points.Get(), nullptr, &points_srv))) return false;

    // [0..4] DrawIndexedInstancedIndirect, [5..7] DispatchIndirect (carving).
    d = {};
    d.ByteWidth = 32;
    d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
    d.MiscFlags = D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;
    if (FAILED(dev->CreateBuffer(&d, nullptr, &args))) return false;
    u = {DXGI_FORMAT_R32_UINT, D3D11_UAV_DIMENSION_BUFFER};
    u.Buffer.NumElements = 8;
    if (FAILED(dev->CreateUnorderedAccessView(args.Get(), &u, &args_uav))) return false;
    return true;
}

void PointCloud::Pool::clear(ID3D11DeviceContext* ctx) {
    const UINT zero[4] = {};
    ctx->ClearUnorderedAccessViewUint(counter_uav.Get(), zero);
    ctx->ClearUnorderedAccessViewUint(table_uav.Get(), zero);
    ctx->ClearUnorderedAccessViewUint(args_uav.Get(), zero);
}

bool PointCloud::create(ID3D11Device* dev, ID3D11DeviceContext* ctx, uint32_t capacity, uint32_t table_bits) {
    dev_ = dev;
    ctx_ = ctx;
    cs_min_dist_ = make_cs(dev, shaders::kCompute, "cs_min_dist");
    cs_carve_ = make_cs(dev, shaders::kCompute, "cs_carve");
    cs_ingest_ = make_cs(dev, shaders::kCompute, "cs_ingest");
    cs_args_ = make_cs(dev, shaders::kCompute, "cs_args");
    cs_hist_ = make_cs(dev, shaders::kHeightHist, "cs_height_hist");

    {
        D3D11_TEXTURE2D_DESC d{};
        d.Width = kMaxWidth;
        d.Height = kMaxHeight;
        d.MipLevels = d.ArraySize = 1;
        d.SampleDesc.Count = 1;
        d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        d.Format = DXGI_FORMAT_R32_FLOAT;
        check(dev->CreateTexture2D(&d, nullptr, &depth_tex_), "depth tex");
        check(dev->CreateShaderResourceView(depth_tex_.Get(), nullptr, &depth_srv_), "depth srv");
        d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        check(dev->CreateTexture2D(&d, nullptr, &color_tex_), "color tex");
        check(dev->CreateShaderResourceView(color_tex_.Get(), nullptr, &color_srv_), "color srv");

        d.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
        d.Format = DXGI_FORMAT_R32_FLOAT;
        check(dev->CreateTexture2D(&d, nullptr, &min_dist_tex_), "min dist tex");
        check(dev->CreateShaderResourceView(min_dist_tex_.Get(), nullptr, &min_dist_srv_), "min dist srv");
        check(dev->CreateUnorderedAccessView(min_dist_tex_.Get(), nullptr, &min_dist_uav_), "min dist uav");
    }

    for (auto& s : stat_stage_) {
        D3D11_BUFFER_DESC d{16, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
        check(dev->CreateBuffer(&d, nullptr, &s), "stat staging");
    }
    {
        D3D11_BUFFER_DESC d{kHistBytes, D3D11_USAGE_DEFAULT, D3D11_BIND_UNORDERED_ACCESS, 0,
                            D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS};
        check(dev->CreateBuffer(&d, nullptr, &hist_buf_), "hist");
        D3D11_UNORDERED_ACCESS_VIEW_DESC u{DXGI_FORMAT_R32_TYPELESS, D3D11_UAV_DIMENSION_BUFFER};
        u.Buffer.NumElements = kHistBytes / 4;
        u.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
        check(dev->CreateUnorderedAccessView(hist_buf_.Get(), &u, &hist_uav_), "hist uav");
        d = {kHistBytes, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
        check(dev->CreateBuffer(&d, nullptr, &hist_stage_), "hist staging");
    }
    frame_cb_ = make_cbuffer<FrameCB>(dev);
    hist_cb_ = make_cbuffer<HistCB>(dev);

    if (!pool_.create(dev, capacity, table_bits)) return false;
    pool_.clear(ctx);
    set_capacity_cb();
    return true;
}

// cs_args reads the capacity from the frame cbuffer, even before the first frame arrives.
void PointCloud::set_capacity_cb() {
    FrameCB cb{};
    cb.capacity = pool_.capacity;
    upload(ctx_, frame_cb_.Get(), cb);
}

bool PointCloud::resize(uint32_t capacity) {
    const uint32_t old_capacity = pool_.capacity, old_bits = table_bits_for(old_capacity);
    pool_ = Pool();
    ctx_->Flush();
    const bool ok = pool_.create(dev_, capacity, table_bits_for(capacity));
    if (!ok) {
        pool_ = Pool();
        if (!pool_.create(dev_, old_capacity, old_bits)) check(E_OUTOFMEMORY, "recreating the point pool");
    }
    set_capacity_cb();
    clear();
    return ok;
}

void PointCloud::clear() {
    pool_.clear(ctx_);
    update_args();
    point_count_ = free_count_ = 0;
    heights_.valid = false;
    hist_pending_ = false;  // counts the old points
    hist_timer_ = 1e9;      // measure the new ones right away
    stat_fresh_from_ = stat_frame_ + kStatStages;
}

// Binds the compute UAVs. The args buffer is only bound for cs_args, since
// it's also the source of DispatchIndirect/DrawInstancedIndirect.
void PointCloud::bind_uavs(bool with_args) {
    ID3D11UnorderedAccessView* uavs[5] = {pool_.counter_uav.Get(), pool_.table_uav.Get(), pool_.points_uav.Get(),
                                          pool_.free_list_uav.Get(), with_args ? pool_.args_uav.Get() : nullptr};
    ctx_->CSSetUnorderedAccessViews(0, 5, uavs, nullptr);
}

void PointCloud::unbind_compute() {
    ID3D11ShaderResourceView* null_srvs[3] = {};
    ID3D11UnorderedAccessView* null_uavs[6] = {};
    ctx_->CSSetShaderResources(0, 3, null_srvs);
    ctx_->CSSetUnorderedAccessViews(0, 6, null_uavs, nullptr);
}

void PointCloud::update_args() {
    ctx_->CSSetShader(cs_args_.Get(), nullptr, 0);
    ctx_->CSSetConstantBuffers(0, 1, frame_cb_.GetAddressOf());
    bind_uavs(true);
    ctx_->Dispatch(1, 1, 1);
    unbind_compute();
}

void PointCloud::ingest(const Frame& frame, FXMMATRIX view, CXMMATRIX proj, const CaptureSettings& s, bool snapshot,
                        bool& carved) {
    ID3D11DeviceContext* ctx = ctx_;
    const FrameHeader& h = frame.header;
    if (snapshot) pool_.clear(ctx);

    D3D11_BOX box{0, 0, 0, h.width, h.height, 1};
    ctx->UpdateSubresource(depth_tex_.Get(), 0, &box, frame.depth.data(), h.width * 4, 0);
    const bool has_color = (h.flags & kFlagHasColor) != 0;
    if (has_color) ctx->UpdateSubresource(color_tex_.Get(), 0, &box, frame.color.data(), h.width * 4, 0);

    FrameCB cb{};
    XMStoreFloat4x4(&cb.view, view);
    XMStoreFloat4x4(&cb.proj, proj);
    XMStoreFloat4x4(&cb.inv_view, XMMatrixInverse(nullptr, view));
    XMStoreFloat4x4(&cb.inv_proj, XMMatrixInverse(nullptr, proj));
    cb.dims[0] = h.width;
    cb.dims[1] = h.height;
    cb.src_dims[0] = h.src_width;
    cb.src_dims[1] = h.src_height;
    cb.near_cut = s.near_cut;
    cb.max_range = s.max_range;
    cb.voxel_size = s.voxel;
    cb.table_mask = pool_.table_size - 1;
    cb.capacity = pool_.capacity;
    cb.has_color = has_color;
    cb.carve_margin_abs = s.carve_margin;
    cb.carve_margin_rel = s.carve_rel;
    cb.carve_on = s.carve;
    cb.color_update = s.color_update;
    upload(ctx, frame_cb_.Get(), cb);

    ID3D11ShaderResourceView* srvs[2] = {depth_srv_.Get(), color_srv_.Get()};
    ctx->CSSetConstantBuffers(0, 1, frame_cb_.GetAddressOf());
    ctx->CSSetShaderResources(0, 2, srvs);
    // Carve first, so this frame's own points aren't tested against itself
    // and freed slots can be reused right away.
    if ((s.carve || has_color) && !carved) {  // has_color: the color refresh (even first-seen fills in)
        carved = true;
        ctx->CSSetShader(cs_min_dist_.Get(), nullptr, 0);
        ctx->CSSetUnorderedAccessViews(5, 1, min_dist_uav_.GetAddressOf(), nullptr);
        ctx->Dispatch((h.width + 7) / 8, (h.height + 7) / 8, 1);
        ID3D11UnorderedAccessView* null_uav = nullptr;
        ctx->CSSetUnorderedAccessViews(5, 1, &null_uav, nullptr);
        ctx->CSSetShaderResources(2, 1, min_dist_srv_.GetAddressOf());
        bind_uavs(false);
        ctx->CSSetShader(cs_carve_.Get(), nullptr, 0);
        ctx->DispatchIndirect(pool_.args.Get(), 20);
    }
    bind_uavs(false);
    ctx->CSSetShader(cs_ingest_.Get(), nullptr, 0);
    ctx->Dispatch((h.width + 7) / 8, (h.height + 7) / 8, 1);
    unbind_compute();
    update_args();
}

void PointCloud::poll_stats() {
    ctx_->CopyResource(stat_stage_[stat_frame_ % kStatStages].Get(), pool_.counter.Get());
    if (stat_frame_ >= kStatStages - 1 && stat_frame_ >= stat_fresh_from_) {
        D3D11_MAPPED_SUBRESOURCE m;
        ID3D11Buffer* s = stat_stage_[(stat_frame_ + 1) % kStatStages].Get();
        if (ctx_->Map(s, 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m) == S_OK) {
            const uint32_t* c = static_cast<const uint32_t*>(m.pData);
            point_count_ = c[0];
            free_count_ = c[1];
            ctx_->Unmap(s, 0);
        }
    }
    ++stat_frame_;
}

void PointCloud::update_heights(float dt, const float height_axis[4]) {
    if (hist_pending_) {
        D3D11_MAPPED_SUBRESOURCE m;
        if (ctx_->Map(hist_stage_.Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m) == S_OK) {
            const uint32_t* h = static_cast<const uint32_t*>(m.pData);
            const uint32_t* bins = h + 2;
            HeightStats& hs = heights_;
            hist_pending_ = false;
            hs.valid = h[1] != 0;
            if (hs.valid) {
                hs.low = height_from_key(~h[0]);
                hs.high = height_from_key(h[1]);
                hs.range_low = hs.low, hs.range_high = hs.high;
                if (hist_hi_ > hist_lo_) {
                    uint64_t total = 0;
                    for (uint32_t i = 0; i < shaders::kHistBins; ++i) total += bins[i];
                    const float bin_size = (hist_hi_ - hist_lo_) / float(shaders::kHistBins);
                    // The height below which a fraction q of the points lie, interpolated in its bin.
                    auto percentile = [&](double q) {
                        const double target = q * double(total);
                        double below = 0;
                        for (uint32_t i = 0; i < shaders::kHistBins; ++i) {
                            if (below + bins[i] >= target && bins[i] > 0)
                                return hist_lo_ + (float(i) + float((target - below) / bins[i])) * bin_size;
                            below += bins[i];
                        }
                        return hist_hi_;
                    };
                    hs.range_low = std::clamp(percentile(0.01), hs.low, hs.high);
                    hs.range_high = std::clamp(percentile(0.99), hs.low, hs.high);
                } else {
                    hist_timer_ = 1e9;  // that one only found the extremes: bin over them next
                }
            }
            ctx_->Unmap(hist_stage_.Get(), 0);
        }
    }
    hist_timer_ += dt;
    if (!hist_pending_ && hist_timer_ > 0.25) {
        hist_timer_ = 0;
        // Bin over the extremes last measured. The tilt or new points can move them; the next
        // histogram catches up.
        hist_lo_ = heights_.valid ? heights_.low : 0;
        hist_hi_ = heights_.valid ? heights_.high : 0;
        HistCB hcb{};
        std::memcpy(hcb.height_axis, height_axis, sizeof(hcb.height_axis));
        hcb.lo = hist_lo_;
        hcb.hi = hist_hi_;
        hcb.capacity = pool_.capacity;
        hcb.bins = shaders::kHistBins;
        upload(ctx_, hist_cb_.Get(), hcb);
        const UINT zero[4] = {};
        ctx_->ClearUnorderedAccessViewUint(hist_uav_.Get(), zero);
        ID3D11UnorderedAccessView* uavs[2] = {hist_uav_.Get(), pool_.counter_uav.Get()};
        ctx_->CSSetShader(cs_hist_.Get(), nullptr, 0);
        ctx_->CSSetConstantBuffers(0, 1, hist_cb_.GetAddressOf());
        ctx_->CSSetShaderResources(0, 1, pool_.points_srv.GetAddressOf());
        ctx_->CSSetUnorderedAccessViews(0, 2, uavs, nullptr);
        ctx_->DispatchIndirect(pool_.args.Get(), 20);  // one thread per slot, like carving
        unbind_compute();
        ctx_->CopyResource(hist_stage_.Get(), hist_buf_.Get());
        hist_pending_ = true;
    }
}

bool PointCloud::save_ply(const std::string& path) const {
    const uint32_t n = std::min(read_counter_blocking(dev_, ctx_, pool_.counter.Get()), pool_.capacity);
    std::vector<PointData> pts(n);
    if (n > 0) {
        D3D11_BUFFER_DESC d{UINT(n * sizeof(PointData)), D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
        ComPtr<ID3D11Buffer> s;
        check(dev_->CreateBuffer(&d, nullptr, &s), "staging points");
        D3D11_BOX box{0, 0, 0, d.ByteWidth, 1, 1};
        ctx_->CopySubresourceRegion(s.Get(), 0, 0, 0, 0, pool_.points.Get(), 0, &box);
        D3D11_MAPPED_SUBRESOURCE m;
        check(ctx_->Map(s.Get(), 0, D3D11_MAP_READ, 0, &m), "Map points");
        std::memcpy(pts.data(), m.pData, d.ByteWidth);
        ctx_->Unmap(s.Get(), 0);
    }
    // Drop carved (deleted) slots.
    std::erase_if(pts, [](const PointData& p) { return std::isnan(p.pos[0]); });
    const uint32_t live = uint32_t(pts.size());
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::printf("cannot write %s\n", path.c_str());
        return false;
    }
    std::fprintf(f,
                 "ply\nformat binary_little_endian 1.0\ncomment game-lidar scan\nelement vertex %u\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
                 live);
    for (const PointData& p : pts) {
        std::fwrite(p.pos, sizeof(float), 3, f);
        const uint8_t rgb[3] = {uint8_t(p.color), uint8_t(p.color >> 8), uint8_t(p.color >> 16)};
        std::fwrite(rgb, 1, 3, f);
    }
    std::fclose(f);
    std::printf("saved %u points to %s\n", live, path.c_str());
    return true;
}

}  // namespace lidar
