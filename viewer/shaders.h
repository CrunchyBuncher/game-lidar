// HLSL for the viewer, compiled at startup.
#pragma once

namespace lidar::shaders {

// Per-frame compute passes over the point pool:
//   cs_carve  - free-space carving: delete points the current frame sees through
//   cs_ingest - depth frame -> world points, deduplicated through a voxel hash
//   cs_args   - refresh indirect draw/dispatch arguments
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
};

struct PointData { float3 pos; uint color; };  // pos.x = NaN marks a deleted point

Texture2D<float> depth_tex : register(t0);
Texture2D<float4> color_tex : register(t1);
// [0] slots ever allocated (may exceed capacity), [4] free-list size, [8] ~lowest and [12] highest
// point height as height_key()s (0 = no points yet; both only grow, so a clear resets them).
RWByteAddressBuffer counter : register(u0);
RWStructuredBuffer<uint> table : register(u1);
RWStructuredBuffer<PointData> points : register(u2);
RWStructuredBuffer<uint> free_list : register(u3);
RWBuffer<uint> args : register(u4);  // [0..3] DrawInstancedIndirect, [4..6] DispatchIndirect for carving

static const uint kEmpty = 0;
static const uint kTombstone = 0xFFFFFFFFu;
static const uint kCarveGroup = 256;

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

// A float as a uint with the same order, so atomics can track the extremes (never 0 for a real number).
uint height_key(float y) {
    uint u = asuint(y);
    return (u & 0x80000000u) ? ~u : (u | 0x80000000u);
}

uint pack_rgba(float4 c) {
    uint4 u = (uint4)round(saturate(c) * 255.0);
    return u.r | (u.g << 8) | (u.b << 16) | (u.a << 24);
}

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
    if (any(pix < 1) || any(pix >= (int2)dims - 1)) return;  // need a full 3x3 neighborhood

    // Delete only if every neighboring ray clearly passes beyond the point. This is
    // conservative at silhouettes and at grazing angles.
    float need = d + max(carve_margin_abs, carve_margin_rel * d);
    [unroll] for (int y = -1; y <= 1; ++y) {
        [unroll] for (int x = -1; x <= 1; ++x) {
            float o = observed_distance(uint2(pix + int2(x, y)));
            if (o < near_cut || o < need) return;
        }
    }

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
                p.color = has_color ? pack_rgba(color_tex.Load(int3(id.xy, 0))) : 0xFFFFFFFFu;
                points[idx] = p;
                uint key = height_key(w.y), ignored;
                counter.InterlockedMax(8, ~key, ignored);
                counter.InterlockedMax(12, key, ignored);
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
    args[0] = 6;  // vertices per point quad
    args[1] = n;
    args[2] = 0;
    args[3] = 0;
    args[4] = (n + kCarveGroup - 1) / kCarveGroup;
    args[5] = 1;
    args[6] = 1;
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
    float2 pad_;
    float4 height_axis;  // height as shown (tilted): dot(float4(pos, 1), height_axis)
};

struct PointData { float3 pos; uint color; };
StructuredBuffer<PointData> points : register(t0);

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

VSOut vs_points(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    const float2 corners[6] = { float2(-1,-1), float2(-1,1), float2(1,1), float2(-1,-1), float2(1,1), float2(1,-1) };
    PointData p = points[iid];
    VSOut o;
    o.col = 0;
    if (isnan(p.pos.x)) {  // deleted: emit a clipped vertex
        o.pos = float4(2, 2, 2, 1);
        return o;
    }
    o.pos = mul(float4(p.pos, 1), view_proj);
    o.pos.xy += corners[vid] * px_to_ndc * point_size * 0.5 * o.pos.w;
    if (color_mode == 0) {
        o.col = turbo((dot(float4(p.pos, 1), height_axis) - height_min) / (height_max - height_min));
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
