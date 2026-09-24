// lidar_verify: checks captured geometry against the fake game's true scene.
//
//   lidar_verify ring [--frames 30] [--tol 0.02]   read live frames from the ring (CPU unprojection)
//   lidar_verify ply <file> [--tol 0.02]           check a viewer-saved .ply (GPU pipeline)
//
// Run the fake game with --no-npc, since the moving NPC isn't in the reference scene.
// Exits non-zero if fewer than 99% of points lie within tolerance of a surface.
#include <windows.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "ring.h"
#include "scene.h"
#include "unproject.h"

using namespace lidar;

namespace {

int report(std::vector<float>& err, float tol) {
    if (err.empty()) {
        std::printf("no points\n");
        return 2;
    }
    std::sort(err.begin(), err.end());
    const size_t within = size_t(std::lower_bound(err.begin(), err.end(), tol) - err.begin());
    const double frac = double(within) / double(err.size());
    auto pct = [&](double p) { return err[std::min(err.size() - 1, size_t(p * double(err.size())))]; };
    std::printf("points: %zu\n", err.size());
    std::printf("error  p50 %.4f m   p99 %.4f m   max %.4f m\n", pct(0.5), pct(0.99), err.back());
    std::printf("outliers (> %.3f m): %zu\n", tol, err.size() - within);
    std::printf("within %.3f m: %.2f%%  -> %s\n", tol, frac * 100.0, frac >= 0.99 ? "PASS" : "FAIL");
    return frac >= 0.99 ? 0 : 1;
}

int verify_ring(int frames_wanted, float tol) {
    RingReader ring;
    const auto boxes = scene::build();
    std::vector<float> err;
    Frame f;
    int frames = 0;
    const ULONGLONG deadline = GetTickCount64() + 15000;
    while (frames < frames_wanted && GetTickCount64() < deadline) {
        if (!ring.is_open() && !ring.try_open()) {
            Sleep(100);
            continue;
        }
        if (!ring.read_next(f)) {
            Sleep(5);
            continue;
        }
        const Unprojector up(f.header);
        for (uint32_t v = 0; v < f.header.height; v += 2) {
            for (uint32_t u = 0; u < f.header.width; u += 2) {
                float p[3];
                if (up.unproject(u, v, f.depth[size_t(v) * f.header.width + u], 0.3f, 500.0f, p))
                    err.push_back(scene::distance(boxes, p));
            }
        }
        ++frames;
    }
    std::printf("frames: %d\n", frames);
    if (frames == 0) {
        std::printf("no frames received (is fake_game running?)\n");
        return 2;
    }
    return report(err, tol);
}

int verify_ply(const char* path, float tol) {
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        std::printf("cannot open %s\n", path);
        return 2;
    }
    char line[256];
    unsigned n = 0;
    while (std::fgets(line, sizeof(line), f)) {
        std::sscanf(line, "element vertex %u", &n);
        if (std::strncmp(line, "end_header", 10) == 0) break;
    }
    const auto boxes = scene::build();
    std::vector<float> err;
    err.reserve(n);
    for (unsigned i = 0; i < n; ++i) {
        float p[3];
        uint8_t rgb[3];
        if (std::fread(p, sizeof(float), 3, f) != 3 || std::fread(rgb, 1, 3, f) != 3) break;
        err.push_back(scene::distance(boxes, p));
    }
    std::fclose(f);
    return report(err, tol);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: lidar_verify ring [--frames N] [--tol m] | ply <file> [--tol m]\n");
        return 2;
    }
    const std::string mode = argv[1];
    float tol = 0.02f;
    int frames = 30;
    const char* file = nullptr;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--tol" && i + 1 < argc) tol = float(std::atof(argv[++i]));
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else file = argv[i];
    }
    if (mode == "ring") return verify_ring(frames, tol);
    if (mode == "ply" && file) return verify_ply(file, tol);
    std::printf("bad arguments\n");
    return 2;
}
