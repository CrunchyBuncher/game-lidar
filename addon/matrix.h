// 4x4 matrix helpers shared by profile decoding and discovery. Double precision, row-major,
// row-vector convention (p' = p * M), matching protocol.h.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

namespace lidar::mat {

struct Mat {
    double m[4][4];
};

inline Mat identity() {
    Mat r{};
    for (int i = 0; i < 4; ++i) r.m[i][i] = 1;
    return r;
}

// 16 floats as stored: row-major memory, or column-major (the transpose).
inline Mat load(const float f[16], bool column_major) {
    Mat r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) r.m[i][j] = column_major ? f[j * 4 + i] : f[i * 4 + j];
    return r;
}
inline Mat load(const uint8_t* p, bool column_major) {
    float f[16];
    std::memcpy(f, p, sizeof(f));
    return load(f, column_major);
}

inline void store(const Mat& a, float out[16]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out[i * 4 + j] = float(a.m[i][j]);
}

inline Mat mul(const Mat& a, const Mat& b) {
    Mat r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k) r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

// Gauss-Jordan with partial pivoting.
inline bool inverse(const Mat& a, Mat& out) {
    double w[4][8];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) w[i][j] = a.m[i][j], w[i][j + 4] = i == j ? 1.0 : 0.0;
    for (int c = 0; c < 4; ++c) {
        int p = c;
        for (int i = c + 1; i < 4; ++i)
            if (std::abs(w[i][c]) > std::abs(w[p][c])) p = i;
        if (std::abs(w[p][c]) < 1e-12) return false;
        if (p != c) std::swap(w[p], w[c]);
        const double inv = 1.0 / w[c][c];
        for (int j = 0; j < 8; ++j) w[c][j] *= inv;
        for (int i = 0; i < 4; ++i) {
            if (i == c) continue;
            const double f = w[i][c];
            for (int j = 0; j < 8; ++j) w[i][j] -= f * w[c][j];
        }
    }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out.m[i][j] = w[i][j + 4];
    return true;
}

inline bool finite(const Mat& a) {
    for (const auto& row : a.m)
        for (double v : row)
            if (!std::isfinite(v)) return false;
    return true;
}

// p (x, y, z, w) * M
inline void transform(const double p[4], const Mat& a, double out[4]) {
    for (int j = 0; j < 4; ++j) out[j] = p[0] * a.m[0][j] + p[1] * a.m[1][j] + p[2] * a.m[2][j] + p[3] * a.m[3][j];
}

}  // namespace lidar::mat
