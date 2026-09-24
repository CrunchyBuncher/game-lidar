#include "camera_math.h"

#include <algorithm>
#include <cmath>

#include "profile.h"

namespace lidar {
namespace {

using mat::Mat;

double dot3(const double a[4], const double b[4]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double len3(const double a[4]) { return std::sqrt(dot3(a, a)); }

void column(const Mat& m, int j, double out[4]) {
    for (int i = 0; i < 4; ++i) out[i] = m.m[i][j];
}

bool plausible_values(const Mat& m) {
    int nonzero = 0;
    for (const auto& row : m.m)
        for (double v : row) {
            if (!std::isfinite(v) || std::abs(v) > 1e7) return false;
            nonzero += v != 0;
        }
    return nonzero >= 4;
}

bool is_clear(float d) { return d <= 0.0f || d >= 1.0f; }

}  // namespace

bool is_rigid(const Mat& m) {
    if (!plausible_values(m)) return false;
    for (int i = 0; i < 3; ++i)
        if (std::abs(m.m[i][3]) > 1e-4) return false;
    if (std::abs(m.m[3][3] - 1) > 1e-4) return false;
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) {
            const double d = m.m[i][0] * m.m[j][0] + m.m[i][1] * m.m[j][1] + m.m[i][2] * m.m[j][2];
            if (std::abs(d - (i == j ? 1.0 : 0.0)) > 2e-3) return false;
        }
    return true;
}

bool is_projection(const Mat& m) {
    if (!plausible_values(m)) return false;
    constexpr double kZero = 1e-5;
    const double zeros[] = {m.m[0][1], m.m[0][2], m.m[0][3], m.m[1][0], m.m[1][2],
                            m.m[1][3], m.m[3][0], m.m[3][1], m.m[3][3]};
    for (double z : zeros)
        if (std::abs(z) > kZero) return false;
    if (std::abs(m.m[2][0]) >= 1 || std::abs(m.m[2][1]) >= 1) return false;
    const double sx = std::abs(m.m[0][0]), sy = std::abs(m.m[1][1]);
    if (sx < 0.05 || sx > 50 || sy < 0.05 || sy > 50 || sy / sx < 0.2 || sy / sx > 5) return false;
    float f[16];
    mat::store(m, f);
    return analyze_projection(f).valid;
}

bool decompose_view_proj(const Mat& vp, Mat& view, Mat& proj) {
    if (!plausible_values(vp)) return false;
    double c0[4], c1[4], c2[4], c3[4];
    column(vp, 0, c0);
    column(vp, 1, c1);
    column(vp, 2, c2);
    column(vp, 3, c3);

    // c3 = s * z (the view's z column, unit length in xyz), s = ±1.
    double s = len3(c3);
    if (std::abs(s - 1) > 5e-3) return false;
    double z[4];
    for (int i = 0; i < 4; ++i) z[i] = c3[i] / s;

    // c0 = sx * x + jx * z, c1 = sy * y + jy * z, with x, y, z orthonormal.
    double jx = dot3(c0, z), jy = dot3(c1, z);
    double x[4], y[4];
    for (int i = 0; i < 4; ++i) x[i] = c0[i] - jx * z[i], y[i] = c1[i] - jy * z[i];
    const double sx = len3(x), sy = len3(y);
    if (sx < 0.05 || sy < 0.05 || sx > 50 || sy > 50 || sy / sx < 0.2 || sy / sx > 5) return false;
    for (int i = 0; i < 4; ++i) x[i] /= sx, y[i] /= sy;
    if (std::abs(dot3(x, y)) > 5e-3) return false;

    // c2 = a * z + b * (0,0,0,1).
    double a = dot3(c2, z);
    const double r[3] = {c2[0] - a * z[0], c2[1] - a * z[1], c2[2] - a * z[2]};
    if (std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) > 5e-3 * std::max(1.0, std::abs(a))) return false;
    const double b = c2[3] - a * z[3];

    // A proper rotation: flip z (and with it s, the jitter and a) if the axes came out left-handed.
    const double det = x[0] * (y[1] * z[2] - y[2] * z[1]) - x[1] * (y[0] * z[2] - y[2] * z[0]) +
                       x[2] * (y[0] * z[1] - y[1] * z[0]);
    if (det < 0) {
        for (double& v : z) v = -v;
        s = -s, jx = -jx, jy = -jy, a = -a;
    }
    if (std::abs(jx) >= 1 || std::abs(jy) >= 1) return false;

    for (int i = 0; i < 4; ++i) {
        view.m[i][0] = x[i];
        view.m[i][1] = y[i];
        view.m[i][2] = z[i];
        view.m[i][3] = i == 3 ? 1.0 : 0.0;
    }
    proj = Mat{};
    proj.m[0][0] = sx;
    proj.m[1][1] = sy;
    proj.m[2][0] = jx;
    proj.m[2][1] = jy;
    proj.m[2][2] = a;
    proj.m[2][3] = s < 0 ? -1.0 : 1.0;
    proj.m[3][2] = b;
    float f[16];
    mat::store(proj, f);
    return analyze_projection(f).valid;
}

uint32_t classify_matrix(const float f[16]) {
    const Mat m[2] = {mat::load(f, false), mat::load(f, true)};
    if (!plausible_values(m[0])) return 0;
    // A clean rigid transform or projection in either major is that: e.g. a reversed projection,
    // transposed and inverted, also splits into a "view-projection".
    uint32_t bits = 0;
    for (int col = 0; col < 2; ++col) {
        if (is_rigid(m[col])) bits |= kind_bit(MatrixKind::Rigid, col);
        if (is_projection(m[col])) bits |= kind_bit(MatrixKind::Proj, col);
    }
    if (bits != 0) return bits;
    for (int col = 0; col < 2; ++col) {
        const auto& a = m[col].m;
        Mat v, p, inv;
        // Cheap tests first. A view-projection's w column is ±(the view's z axis): unit length.
        const double w_len = std::sqrt(a[0][3] * a[0][3] + a[1][3] * a[1][3] + a[2][3] * a[2][3]);
        if (std::abs(w_len - 1) < 0.01) {
            if (decompose_view_proj(m[col], v, p)) bits |= kind_bit(MatrixKind::ViewProj, col);
            continue;
        }
        // An inverse one's first two rows are the view's x and y axes (scaled): no w.
        const double r0 = std::abs(a[0][0]) + std::abs(a[0][1]) + std::abs(a[0][2]);
        const double r1 = std::abs(a[1][0]) + std::abs(a[1][1]) + std::abs(a[1][2]);
        if (std::abs(a[0][3]) > 1e-4 * r0 || std::abs(a[1][3]) > 1e-4 * r1) continue;
        if (mat::inverse(m[col], inv) && !is_projection(inv) && decompose_view_proj(inv, v, p))
            bits |= kind_bit(MatrixKind::InvViewProj, col);
    }
    return bits;
}

const char* kind_name(MatrixKind k) {
    switch (k) {
        case MatrixKind::Rigid: return "rigid";
        case MatrixKind::Proj: return "proj";
        case MatrixKind::ViewProj: return "viewproj";
        case MatrixKind::InvViewProj: return "invviewproj";
    }
    return "?";
}

void DepthGrid::build(const float* frame_depth, uint32_t cap_w, uint32_t cap_h, uint32_t src_w, uint32_t src_h,
                      uint32_t step) {
    step = std::max(step, 1u);
    w = (cap_w + step - 1) / step;
    h = (cap_h + step - 1) / step;
    depth.resize(size_t(w) * h);
    ndc_x.resize(w);
    ndc_y.resize(h);
    // Same source-pixel mapping as protocol.h / Unprojector::ndc.
    auto src = [](uint32_t u, uint32_t cap, uint32_t full) { return std::min(((2 * u + 1) * full) / (2 * cap), full - 1); };
    std::vector<uint32_t> us(w), vs(h);
    for (uint32_t i = 0; i < w; ++i) {
        us[i] = std::min(i * step + step / 2, cap_w - 1);
        ndc_x[i] = (float(src(us[i], cap_w, src_w)) + 0.5f) / float(src_w) * 2.0f - 1.0f;
    }
    for (uint32_t j = 0; j < h; ++j) {
        vs[j] = std::min(j * step + step / 2, cap_h - 1);
        ndc_y[j] = 1.0f - (float(src(vs[j], cap_h, src_h)) + 0.5f) / float(src_h) * 2.0f;
    }
    size_t high = 0;
    for (uint32_t j = 0; j < h; ++j)
        for (uint32_t i = 0; i < w; ++i) {
            const float d = frame_depth[size_t(vs[j]) * cap_w + us[i]];
            depth[size_t(j) * w + i] = d;
            high += d > 0.5f;
        }
    standard = high * 2 > depth.size();
}

// The samples are (nearly) evenly spaced, so the fractional index is linear in NDC.
double DepthGrid::index_x(double x) const {
    return w < 2 ? 0 : (x - ndc_x[0]) * (w - 1) / (double(ndc_x[w - 1]) - ndc_x[0]);
}
double DepthGrid::index_y(double y) const {
    return h < 2 ? 0 : (y - ndc_y[0]) * (h - 1) / (double(ndc_y[h - 1]) - ndc_y[0]);
}

double depth_change(const DepthGrid& a, const DepthGrid& b, double rel) {
    if (a.w != b.w || a.h != b.h || a.depth.empty()) return 1.0;
    size_t changed = 0;
    for (size_t i = 0; i < a.depth.size(); ++i) {
        const float da = a.depth[i], db = b.depth[i];
        if (is_clear(da) || is_clear(db)) {
            changed += da != db;
            continue;
        }
        const double qa = a.standard ? 1.0 - da : da, qb = b.standard ? 1.0 - db : db;
        changed += std::abs(qa - qb) > rel * std::max(qa, qb);
    }
    return double(changed) / double(a.depth.size());
}

// Distance from the camera of the point at NDC (x, y, depth), through an inverse projection. <= 0 if
// it doesn't unproject.
double view_distance(double x, double y, float depth, const Mat& inv_proj) {
    const double clip[4] = {x, y, depth, 1};
    double q[4];
    mat::transform(clip, inv_proj, q);
    if (!(std::abs(q[3]) > 1e-12)) return 0;
    const double d = len3(q) / std::abs(q[3]);
    return std::isfinite(d) ? d : 0;
}

ReprojStats reproject(const DepthGrid& a, const Mat& view_a, const Mat& proj_a, const DepthGrid& b, const Mat& view_b,
                      const Mat& proj_b, uint32_t stride) {
    ReprojStats st;
    Mat inv_a, inv_proj_a, inv_proj_b;
    if (!mat::inverse(mat::mul(view_a, proj_a), inv_a) || !mat::inverse(proj_a, inv_proj_a) ||
        !mat::inverse(proj_b, inv_proj_b))
        return st;
    const Mat vp_b = mat::mul(view_b, proj_b);
    const bool same_grid = a.w == b.w && a.h == b.h;
    std::vector<double> err;
    err.reserve(size_t(a.w / stride + 1) * (a.h / stride + 1));
    for (uint32_t j = stride / 2; j < a.h; j += stride) {
        for (uint32_t i = stride / 2; i < a.w; i += stride) {
            const float d = a.depth[size_t(j) * a.w + i];
            if (is_clear(d)) continue;
            // Only pixels whose depth changed tell anything: e.g. walking over flat ground leaves most
            // pixels at the same distance, which a matrix that never moves would "predict" too.
            if (same_grid) {
                const float same = b.depth[size_t(j) * b.w + i];
                const double da = view_distance(a.ndc_x[i], a.ndc_y[j], d, inv_proj_a);
                const double ds = view_distance(a.ndc_x[i], a.ndc_y[j], same, inv_proj_b);
                if (!is_clear(same) && da > 0 && ds > 0 && std::abs(da - ds) < 0.01 * ds) continue;
            }
            const double clip[4] = {a.ndc_x[i], a.ndc_y[j], d, 1};
            double world[4];
            mat::transform(clip, inv_a, world);
            if (!(std::abs(world[3]) > 1e-12)) continue;
            for (double& v : world) v /= world[3];
            ++st.tested;
            double pb[4];
            mat::transform(world, vp_b, pb);
            if (!(pb[3] > 1e-9)) continue;
            const double x = pb[0] / pb[3], y = pb[1] / pb[3];
            if (!(std::abs(x) <= 1 && std::abs(y) <= 1)) continue;
            ++st.landed;
            double pv[4];
            mat::transform(world, view_b, pv);
            const double predicted = std::abs(pv[2]);
            // Measured: view z at the landing point, from the 4 nearest samples. 1/z is linear in
            // screen space on a plane, so interpolating it makes sampling between pixels exact there.
            const double fx = std::clamp(b.index_x(x), 0.0, b.w - 1.0), fy = std::clamp(b.index_y(y), 0.0, b.h - 1.0);
            const int i0 = std::min(int(fx), int(b.w) - 2 < 0 ? 0 : int(b.w) - 2);
            const int j0 = std::min(int(fy), int(b.h) - 2 < 0 ? 0 : int(b.h) - 2);
            const double tx = std::clamp(fx - i0, 0.0, 1.0), ty = std::clamp(fy - j0, 0.0, 1.0);
            double inv_z = 0;
            int sky = 0;
            for (int dj = 0; dj < 2; ++dj)
                for (int di = 0; di < 2; ++di) {
                    const int ii = std::min(i0 + di, int(b.w) - 1), jj = std::min(j0 + dj, int(b.h) - 1);
                    const float db = b.depth[size_t(jj) * b.w + ii];
                    const double wgt = (di ? tx : 1 - tx) * (dj ? ty : 1 - ty);
                    const double clip_b[4] = {b.ndc_x[ii], b.ndc_y[jj], db, 1};
                    double q[4];
                    mat::transform(clip_b, inv_proj_b, q);
                    const double z = std::abs(q[3]) > 1e-12 ? std::abs(q[2] / q[3]) : 0;
                    if (is_clear(db) || !(z > 0) || !std::isfinite(z)) {
                        ++sky;
                        continue;
                    }
                    inv_z += wgt / z;
                }
            if (sky == 4) {
                err.push_back(1.0);  // predicted geometry, found sky
                continue;
            }
            if (sky > 0 || !(inv_z > 0)) continue;  // at a silhouette against the sky: no verdict
            const double measured = 1 / inv_z;
            err.push_back(std::min(1.0, std::abs(predicted - measured) / measured));
        }
    }
    if (st.tested == 0 || st.landed * 10 < st.tested || err.empty()) return st;
    const auto mid = err.begin() + ptrdiff_t(err.size() / 2);
    std::nth_element(err.begin(), mid, err.end());
    st.median_rel = *mid;
    return st;
}

}  // namespace lidar
