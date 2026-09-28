// Recognizing camera matrices in raw constant data, and checking them against depth. Pure (no
// ReShade calls), so the unit tests cover it. Row-vector convention throughout: a matrix read as
// "column-major" is transposed on load, and the tests below apply to the row-vector form.
#pragma once
#include <cstdint>
#include <vector>

#include "matrix.h"

namespace lidar {

// A rigid transform: last column (0,0,0,1), orthonormal upper 3x3 (a view, an inverse view, or
// an object's world matrix without scale).
bool is_rigid(const mat::Mat& m);

// A D3D perspective projection: the zero pattern, ±1 in the w column (m[2][3]), m[3][3] = 0, and a
// positive near/far (standard, reversed or infinite; either handedness). m[2][0] / m[2][1] may hold
// jitter or an off-center shift.
bool is_projection(const mat::Mat& m);

// Splits a perspective view * proj into its view (rigid) and projection. The w column of a
// view-projection is ±(the view's z axis), a unit vector, which is what makes this possible; the x
// and y columns then give the scales, jitter and the other two axes. Handedness is chosen so the view
// is a proper rotation (det +1). Fails for anything that isn't such a product, including a
// world * view * proj whose world scales, and a bare projection (view = identity).
bool decompose_view_proj(const mat::Mat& vp, mat::Mat& view, mat::Mat& proj);

// What a 64-byte window could be. Bit (kind * 2 + column_major). Translation isn't a matrix and has no
// bit: discovery's float3 next to a camera-relative view-projection (CameraProfile::has_translation).
enum class MatrixKind : uint32_t { Rigid = 0, Proj = 1, ViewProj = 2, InvViewProj = 3, Translation = 4 };
constexpr uint32_t kind_bit(MatrixKind k, bool column_major) { return 1u << (uint32_t(k) * 2 + (column_major ? 1 : 0)); }
uint32_t classify_matrix(const float f[16]);
const char* kind_name(MatrixKind k);

// A depth frame reduced to a grid for the checks below: raw NDC depth plus each column's/row's NDC.
struct DepthGrid {
    uint32_t w = 0, h = 0;
    std::vector<float> depth;  // w * h
    std::vector<float> ndc_x;  // w
    std::vector<float> ndc_y;  // h
    bool standard = true;      // most pixels near 1: standard depth (else reversed)

    // From a protocol frame (cap_w x cap_h sampled from src_w x src_h), keeping every step-th pixel.
    void build(const float* frame_depth, uint32_t cap_w, uint32_t cap_h, uint32_t src_w, uint32_t src_h, uint32_t step);
    // Fractional sample index of an NDC coordinate.
    double index_x(double x) const;
    double index_y(double y) const;
};

// Share of the grid that isn't depth at all: NaN, infinite or outside [0, 1]. A frame read from memory
// that held something else by then (a transient depth buffer whose memory another pass reused) is
// mostly this.
double invalid_depth(const DepthGrid& g);

// Fraction of pixels whose distance changed by more than `rel` between two frames (a sky/geometry
// flip counts as changed). Needs no projection: raw depth is about proportional to 1/distance
// (standard: 1 - d, reversed: d).
double depth_change(const DepthGrid& a, const DepthGrid& b, double rel = 1e-3);

// Reprojects frame a's depth into frame b with the candidate matrices of each frame and compares
// against b's depth. The right matrices give a near-zero median error on static geometry. Only
// pixels whose depth changed by over 1% count (on flat ground most pixels keep their distance, which
// a matrix that never moves would "predict" too).
struct ReprojStats {
    uint32_t tested = 0, landed = 0;  // informative pixels, and those that landed in frame b
    double median_rel = 1.0;  // median |predicted - measured| / measured view z, 1 = failure
    // Share of the compared pixels within `inlier_error`. The right matrices explain the static
    // geometry exactly, even where much of the image moves on its own (foliage, characters) or sits
    // on a depth edge that TAA jitter shifts, which is what spoils the median in such scenes.
    double explained = 0;
    // Enough informative pixels for a verdict. Too few says nothing about the matrices.
    bool conclusive() const { return tested >= 50; }
};
ReprojStats reproject(const DepthGrid& a, const mat::Mat& view_a, const mat::Mat& proj_a, const DepthGrid& b,
                      const mat::Mat& view_b, const mat::Mat& proj_b, uint32_t stride = 3,
                      double inlier_error = 0.005);

}  // namespace lidar
