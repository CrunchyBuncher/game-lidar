// HLSL for the viewer, compiled at startup.
#pragma once

namespace lidar::shaders {

// Points are drawn in batches of this many quads, one instance per batch (see vs_points).
inline constexpr unsigned kQuadsPerBatch = 16384;  // 4 vertices each: fits 16-bit indices

// Per-frame compute passes over the point pool:
//   cs_min_dist - per pixel, the nearest distance observed in its 3x3 neighborhood (for carving)
//   cs_carve    - free-space carving: delete points the current frame sees through; also
//                 refreshes the color of points it sees (color_update)
//   cs_ingest   - depth frame -> world points, deduplicated through a voxel hash
//   cs_args     - refresh indirect draw/dispatch arguments
// Unprojection mirrors common/unproject.h.
inline const char* kCompute = R"(
cbuffer FrameCB : register(b0) {
    row_major float4x4 view;
    row_major float4x4 proj;
    row_major float4x4 inv_view;
    row_major float4x4 inv_proj;
    uint2 dims;
    uint2 src_dims;
    float near_cut;
    float max_range;
    float voxel_size;
    uint table_mask;
    uint capacity;
    uint has_color;
    float carve_margin_abs;
    float carve_margin_rel;
    uint carve_on;
    uint color_update;  // 0 keep the first color, 1 closest sighting's, 2 latest sighting's
};

// pos.x = NaN marks a deleted point. color: RGB in the low bytes, dist_key() of the sighting it
// came from in the top one.
struct PointData { float3 pos; uint color; };

Texture2D<float> depth_tex : register(t0);
Texture2D<float4> color_tex : register(t1);
Texture2D<float> min_dist : register(t2);
// [0] slots ever allocated (may exceed capacity), [4] free-list size.
RWByteAddressBuffer counter : register(u0);
RWStructuredBuffer<uint> table : register(u1);
RWStructuredBuffer<PointData> points : register(u2);
RWStructuredBuffer<uint> free_list : register(u3);
RWBuffer<uint> args : register(u4);  // [0..4] DrawIndexedInstancedIndirect, [5..7] DispatchIndirect for carving
RWTexture2D<float> min_dist_out : register(u5);

static const uint kEmpty = 0;
static const uint kTombstone = 0xFFFFFFFFu;
static const uint kCarveGroup = 256;
static const uint kQuadsPerBatch = 16384;  // must match the C++ one

uint mix(uint h) {
    h ^= h >> 16; h *= 0x7feb352du;
    h ^= h >> 15; h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

uint voxel_hash(float3 w) {
    int3 k = (int3)floor(w / voxel_size);
    uint h = mix(asuint(k.x) + 0x9e3779b9u);
    h = mix(h ^ asuint(k.y));
    h = mix(h ^ asuint(k.z));
    return (h == kEmpty || h == kTombstone) ? 1 : h;
}

// A sighting distance as a byte: 0 at 0.1 m, then one step per 4.7% farther (255: ~13 km or more).
uint dist_key(float d) {
    return (uint)clamp(round(log2(max(d, 0.1) / 0.1) * 15.0), 0.0, 255.0);
}

uint pack_color(float4 c, float d) {
    uint3 u = (uint3)round(saturate(c.rgb) * 255.0);
    return u.r | (u.g << 8) | (u.b << 16) | (dist_key(d) << 24);
}

static const uint kNoColor = 0xFFFFFFFFu;  // white, "seen from infinitely far": any color replaces it

// View-space position of stored pixel p, sampled at its source pixel center.
float4 unproject_pixel(uint2 p) {
    uint2 s = min(((2 * p + 1) * src_dims) / (2 * dims), src_dims - 1);  // exact, see protocol.h
    float2 ndc = float2(((float)s.x + 0.5) / (float)src_dims.x * 2.0 - 1.0,
                        1.0 - ((float)s.y + 0.5) / (float)src_dims.y * 2.0);
    float4 v = mul(float4(ndc, depth_tex.Load(int3(p, 0)), 1), inv_proj);
    v.xyz /= v.w;
    return v;
}

// Distance the camera observed at pixel p. Sky / far plane / out of range count as infinitely far.
float observed_distance(uint2 p) {
    float4 v = unproject_pixel(p);
    float d = length(v.xyz);
    if (!(abs(v.w) >= 1e-20 && d < max_range)) return 1e30;  // also catches NaN / inf
    return d;
}

// Carving tests a point against a 3x3 pixel neighborhood; this does the per-pixel work once per
// frame instead of once per point. Edge pixels have no full neighborhood and get 0 (never carve).
[numthreads(8, 8, 1)]
void cs_min_dist(uint3 id : SV_DispatchThreadID) {
    if (any(id.xy >= dims)) return;
    float m = 0;
    if (all(id.xy >= 1) && all(id.xy < dims - 1)) {
        m = 1e30;
        [unroll] for (int y = -1; y <= 1; ++y)
            [unroll] for (int x = -1; x <= 1; ++x)
                m = min(m, observed_distance(uint2(int2(id.xy) + int2(x, y))));
    }
    min_dist_out[id.xy] = m;
}

[numthreads(kCarveGroup, 1, 1)]
void cs_carve(uint3 id : SV_DispatchThreadID) {
    uint n = min(counter.Load(0), capacity);
    if (id.x >= n) return;
    PointData pt = points[id.x];
    if (isnan(pt.pos.x)) return;

    float4 vp = mul(float4(pt.pos, 1), view);
    float d = length(vp.xyz);
    if (!(d > near_cut && d < max_range)) return;
    float4 clip = mul(vp, proj);
    if (clip.w <= 0) return;
    float2 ndc = clip.xy / clip.w;
    int2 pix = (int2)floor(float2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5) * (float2)dims);
    if (any(pix < 0) || any(pix >= (int2)dims)) return;

    // Color refresh, only where the pixel sees this point's own surface (not something in front of
    // it or behind it) and has color (alpha 0: cropped out). First: only a point without color yet
    // takes it. Closest: a sighting at the same distance step or nearer wins, so standing where a
    // passing effect was seen from also clears it. Latest: every sighting.
    float4 seen = color_tex.Load(int3(pix, 0));
    if (has_color && seen.a > 0.5 && abs(observed_distance(pix) - d) < max(2 * voxel_size, 0.02 * d)) {
        uint c = pack_color(seen, d);
        if (color_update == 2 || (color_update == 1 ? (c >> 24) <= (pt.color >> 24) : pt.color == kNoColor))
            points[id.x].color = c;
    }

    // Delete only if every neighboring ray clearly passes beyond the point. This is
    // conservative at silhouettes and at grazing angles. (A ray stopping before near_cut
    // also keeps it: need > d > near_cut.)
    float need = d + max(carve_margin_abs, carve_margin_rel * d);
    if (!carve_on || min_dist.Load(int3(pix, 0)) < need) return;

    // Free the voxel so it can be filled again, and recycle the point slot.
    uint h = voxel_hash(pt.pos);
    uint slot = h & table_mask;
    [loop] for (uint i = 0; i < 64; ++i) {
        uint e = table[slot];
        if (e == h) { table[slot] = kTombstone; break; }
        if (e == kEmpty) break;
        slot = (slot + 1) & table_mask;
    }
    points[id.x].pos = asfloat(0x7fc00000u).xxx;
    uint f;
    counter.InterlockedAdd(4, 1, f);
    free_list[f] = id.x;
}

[numthreads(8, 8, 1)]
void cs_ingest(uint3 id : SV_DispatchThreadID) {
    if (any(id.xy >= dims)) return;
    float4 v = unproject_pixel(id.xy);
    if (abs(v.w) < 1e-20) return;
    float d = length(v.xyz);
    if (!(d > near_cut && d < max_range)) return;  // also rejects NaN / inf
    float3 w = mul(float4(v.xyz, 1), inv_view).xyz;

    uint h = voxel_hash(w);
    uint slot = h & table_mask;
    [loop] for (uint i = 0; i < 64; ++i) {
        uint prev;
        InterlockedCompareExchange(table[slot], kEmpty, h, prev);
        if (prev == kEmpty) {
            // Reuse a carved slot if there is one, otherwise append.
            uint idx, avail;
            counter.InterlockedAdd(4, 0xFFFFFFFFu, avail);
            if ((int)avail > 0) {
                idx = free_list[avail - 1];
            } else {
                counter.InterlockedAdd(4, 1);
                counter.InterlockedAdd(0, 1, idx);
            }
            if (idx < capacity) {
                PointData p;
                p.pos = w;
                float4 c = color_tex.Load(int3(id.xy, 0));
                p.color = has_color && c.a > 0.5 ? pack_color(c, d) : kNoColor;
                points[idx] = p;
            }
            return;
        }
        if (prev == h) return;  // voxel already filled
        slot = (slot + 1) & table_mask;  // occupied by another voxel or a tombstone
    }
}

[numthreads(1, 1, 1)]
void cs_args() {
    uint n = min(counter.Load(0), capacity);
    args[0] = 6 * kQuadsPerBatch;  // indices per batch
    args[1] = (n + kQuadsPerBatch - 1) / kQuadsPerBatch;
    args[2] = 0;
    args[3] = 0;
    args[4] = 0;
    args[5] = (n + kCarveGroup - 1) / kCarveGroup;
    args[6] = 1;
    args[7] = 1;
}
)";

// Height histogram of the live points (height as shown, so tilted), for a color range that
// ignores a few stray points: [0] ~lowest and [4] highest height_key(), then kHistBins counts
// over [lo, hi] (heights outside land in the end bins).
inline constexpr unsigned kHistBins = 1024;
inline const char* kHeightHist = R"(
cbuffer HistCB : register(b0) {
    float4 height_axis;
    float lo;
    float hi;
    uint capacity;
    uint bins;
};

struct PointData { float3 pos; uint color; };
StructuredBuffer<PointData> points : register(t0);
RWByteAddressBuffer hist : register(u0);
RWByteAddressBuffer counter : register(u1);  // read only: [0] slots ever allocated

// A float as a uint with the same order, so atomics can track the extremes (never 0 for a real number).
uint height_key(float y) {
    uint u = asuint(y);
    return (u & 0x80000000u) ? ~u : (u | 0x80000000u);
}

[numthreads(256, 1, 1)]
void cs_height_hist(uint3 id : SV_DispatchThreadID) {
    if (id.x >= min(counter.Load(0), capacity)) return;
    float3 p = points[id.x].pos;
    if (isnan(p.x)) return;  // carved
    float h = dot(float4(p, 1), height_axis);
    uint key = height_key(h), ignored;
    hist.InterlockedMax(0, ~key, ignored);
    hist.InterlockedMax(4, key, ignored);
    uint bin = (uint)clamp((h - lo) / max(hi - lo, 1e-6) * bins, 0.0, bins - 1.0);
    hist.InterlockedAdd(8 + 4 * bin, 1);
}
)";

inline const char* kDraw = R"(
cbuffer Draw : register(b0) {
    row_major float4x4 view_proj;
    float2 px_to_ndc;
    float point_size;
    uint color_mode;  // 0 height, 1 captured color
    float height_min;
    float height_max;
    uint capacity;
    // 0: every point is point_size pixels wide. Otherwise points are world_size metres wide, so they
    // shrink with distance, clamped to 1..max_px pixels; this is pixels per metre at depth 1.
    float px_per_m;
    float4 height_axis;  // height as shown (tilted): dot(float4(pos, 1), height_axis)
    // Display-only cutaways, in the capture's frame. Disabled ones are set so they never hide:
    // the plane to (0, 0, 0, -1), the cylinder's radius to 0.
    float4 cut_plane;  // hides points where dot(float4(pos, 1), cut_plane) > 0
    // Cylinder along cut_a -> cut_a + cut_ab (a point above the player's camera -> the viewer's),
    // standing on the base plane through cut_a: only points on the viewer's side of it, where
    // dot(float4(pos, 1), cut_base) > 0, are hidden.
    float3 cut_a;
    float cut_r2;  // its radius squared
    float3 cut_ab;
    float cut_inv_ab2;  // 1 / dot(cut_ab, cut_ab)
    float4 cut_base;
    float world_size;
    float max_px;
    float2 pad_;
};

struct PointData { float3 pos; uint color; };
StructuredBuffer<PointData> points : register(t0);
ByteAddressBuffer counter : register(t1);  // [0] slots ever allocated
static const uint kQuadsPerBatch = 16384;  // must match the C++ one

float3 turbo(float x) {
    const float4 kr = float4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const float4 kg = float4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const float4 kb = float4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const float2 kr2 = float2(-152.94239396, 59.28637943);
    const float2 kg2 = float2(4.27729857, 2.82956604);
    const float2 kb2 = float2(-89.90310912, 27.34824973);
    x = saturate(x);
    float4 v4 = float4(1.0, x, x * x, x * x * x);
    float2 v2 = v4.zw * v4.z;
    return float3(dot(v4, kr) + dot(v2, kr2), dot(v4, kg) + dot(v2, kg2), dot(v4, kb) + dot(v2, kb2));
}

struct VSOut { float4 pos : SV_Position; float3 col : COLOR; };

// One instance per batch of kQuadsPerBatch points. The index buffer holds each batch's quads as
// 4 shared vertices (vid = 4 * quad + corner), so the vertex cache runs this ~4 times per point, not 6.
VSOut vs_points(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    const float2 corners[4] = { float2(-1,-1), float2(-1,1), float2(1,1), float2(1,-1) };
    uint idx = iid * kQuadsPerBatch + (vid >> 2);
    VSOut o;
    o.col = 0;
    o.pos = float4(2, 2, 2, 1);  // clipped
    if (idx >= min(counter.Load(0), capacity)) return o;  // past the last point of the last batch
    PointData p = points[idx];
    if (isnan(p.pos.x)) return o;  // deleted
    if (dot(float4(p.pos, 1), cut_plane) > 0) return o;  // above the cut plane
    float3 ap = p.pos - cut_a;
    float3 off = ap - cut_ab * min(dot(ap, cut_ab) * cut_inv_ab2, 1.0);  // the base plane ends it at the player
    if (dot(off, off) < cut_r2 && dot(float4(p.pos, 1), cut_base) > 0) return o;  // inside the line-of-sight cylinder
    o.pos = mul(float4(p.pos, 1), view_proj);
    float px = px_per_m > 0 ? clamp(world_size * px_per_m / max(o.pos.w, 1e-6), 1.0, max_px) : point_size;
    o.pos.xy += corners[vid & 3] * px_to_ndc * px * 0.5 * o.pos.w;
    if (color_mode == 0) {
        // Skip turbo's near-black bottom end so the lowest points (usually the floor) still show.
        float t = saturate((dot(float4(p.pos, 1), height_axis) - height_min) / (height_max - height_min));
        o.col = turbo(0.08 + 0.92 * t);
    } else {
        o.col = float3(p.color & 0xFF, (p.color >> 8) & 0xFF, (p.color >> 16) & 0xFF) / 255.0;
    }
    return o;
}

struct LineIn { float3 pos : POSITION; float4 col : COLOR; };
VSOut vs_lines(LineIn i) {
    VSOut o;
    o.pos = mul(float4(i.pos, 1), view_proj);
    o.col = i.col.rgb;
    return o;
}

float4 ps_main(VSOut i) : SV_Target { return float4(i.col, 1); }
)";

}  // namespace lidar::shaders
