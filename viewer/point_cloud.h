// The GPU point cloud: a voxel-deduped point pool that depth frames are unprojected into, with
// free-space carving, a non-stalling readback of its size and height distribution, and .ply export.
// It knows nothing about how it's viewed: drawing reads its buffers (see Renderer).
#pragma once
#include <DirectXMath.h>

#include <algorithm>
#include <cstdint>
#include <string>

#include "app.h"
#include "ring.h"

namespace lidar {

// How frames become points. The panel edits these live (voxel only through PointCloud::resize).
struct CaptureSettings {
    float voxel = 0.1f;
    float near_cut = 0.3f;
    float max_range = 500.0f;
    bool carve = true;
    float carve_margin = 0.15f;  // meters
    float carve_rel = 0.02f;     // fraction of distance
    uint32_t color_update = 1;   // a point's color: 0 first sighting's, 1 closest's, 2 latest's
};

// Height statistics of the live points, as shown (along a height axis), from the last histogram.
struct HeightStats {
    bool valid = false;
    float low = 0, high = 0;          // the extremes
    float range_low = 0, range_high = 0;  // the 1st and 99th percentiles
};

// Hash table slots for a pool: at least twice the points, so probing stays short.
uint32_t table_bits_for(uint32_t capacity);

// GPU memory a pool of this many points takes, in bytes.
double pool_bytes(uint32_t capacity);

class PointCloud {
public:
    PointCloud() = default;
    PointCloud(const PointCloud&) = delete;
    PointCloud& operator=(const PointCloud&) = delete;

    // False if the GPU can't allocate the pool.
    bool create(ID3D11Device* dev, ID3D11DeviceContext* ctx, uint32_t capacity, uint32_t table_bits);
    // Rebuilds the pool empty. If the GPU can't allocate that much, the old size comes back and it's false.
    bool resize(uint32_t capacity);
    // Drops every point and resets the stats.
    void clear();

    // Adds a depth frame taken with the game camera's view and proj. snapshot: replace the points
    // instead of adding to them (an unposed, camera-relative frame). carved: carving (and the color
    // refresh that shares its pass) is a pass over the whole pool, so it runs for one frame per
    // viewer frame at most; ingest sets it, the caller resets it each viewer frame.
    void ingest(const Frame& frame, DirectX::FXMMATRIX view, DirectX::CXMMATRIX proj, const CaptureSettings& s,
                bool snapshot, bool& carved);

    // Once a frame: picks up the point count a few frames late, never stalling.
    void poll_stats();
    // Once a frame: reads the last height histogram if it's done and, a few times a second, starts
    // the next along height_axis (see ViewTilt::height_axis).
    void update_heights(float dt, const float height_axis[4]);

    // Blocks on the GPU.
    bool save_ply(const std::string& path) const;

    uint32_t capacity() const { return pool_.capacity; }
    uint32_t used() const { return std::min(point_count_, pool_.capacity); }  // slots allocated
    uint32_t live() const { return used() - std::min(free_count_, used()); }  // slots holding a point
    bool full() const { return point_count_ >= pool_.capacity && free_count_ == 0; }
    const HeightStats& heights() const { return heights_; }

    // For drawing: the points, the slot counter, and the indirect args ([0..4] DrawIndexedInstancedIndirect).
    ID3D11ShaderResourceView* points_srv() const { return pool_.points_srv.Get(); }
    ID3D11ShaderResourceView* counter_srv() const { return pool_.counter_srv.Get(); }
    ID3D11Buffer* args() const { return pool_.args.Get(); }

private:
    // Point pool + voxel hash table.
    struct Pool {
        uint32_t capacity = 0, table_size = 0;
        ComPtr<ID3D11Buffer> counter, table, points, free_list, args;
        ComPtr<ID3D11UnorderedAccessView> counter_uav, table_uav, points_uav, free_list_uav, args_uav;
        ComPtr<ID3D11ShaderResourceView> points_srv, counter_srv;

        bool create(ID3D11Device* dev, uint32_t cap, uint32_t table_bits);  // false if the GPU can't allocate it
        void clear(ID3D11DeviceContext* ctx);
    };
    void set_capacity_cb();
    void update_args();
    void bind_uavs(bool with_args);
    void unbind_compute();

    ID3D11Device* dev_ = nullptr;
    ID3D11DeviceContext* ctx_ = nullptr;
    Pool pool_;

    ComPtr<ID3D11ComputeShader> cs_min_dist_, cs_carve_, cs_ingest_, cs_args_, cs_hist_;
    ComPtr<ID3D11Buffer> frame_cb_, hist_cb_;
    // Frame upload textures (max protocol size; only a sub-rect is used).
    ComPtr<ID3D11Texture2D> depth_tex_, color_tex_;
    ComPtr<ID3D11ShaderResourceView> depth_srv_, color_srv_;
    // Per-pixel nearest observed distance over the 3x3 neighborhood, for carving.
    ComPtr<ID3D11Texture2D> min_dist_tex_;
    ComPtr<ID3D11ShaderResourceView> min_dist_srv_;
    ComPtr<ID3D11UnorderedAccessView> min_dist_uav_;

    // Async stats readback of the point counter.
    // Deep enough that the oldest copy is done even when the GPU runs several frames behind.
    static constexpr int kStatStages = 8;
    ComPtr<ID3D11Buffer> stat_stage_[kStatStages];
    int stat_frame_ = 0, stat_fresh_from_ = 0;    // copies made before stat_fresh_from_ predate a clear
    uint32_t point_count_ = 0, free_count_ = 0;  // slots ever allocated, slots freed by carving

    // Height histogram for the automatic color range, a few times a second, read back without
    // stalling. Coloring over the 1st to 99th percentile keeps a few stray points (sky, far
    // geometry) from squashing everything else into one end of the ramp.
    ComPtr<ID3D11Buffer> hist_buf_, hist_stage_;
    ComPtr<ID3D11UnorderedAccessView> hist_uav_;
    bool hist_pending_ = false;        // a copy in hist_stage_ waiting to be read
    float hist_lo_ = 0, hist_hi_ = 0;  // the bins of the pending histogram
    double hist_timer_ = 0;
    HeightStats heights_;
};

}  // namespace lidar
