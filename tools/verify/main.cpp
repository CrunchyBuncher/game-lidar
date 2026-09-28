// lidar_verify: checks captured geometry against the fake game's true scene.
//
//   lidar_verify ring [--frames 30] [--tol 0.02]   read live frames from the ring (CPU unprojection)
//   lidar_verify ply <file> [--tol 0.02] [--min-within 0.99]
//                                                  check a viewer-saved .ply (GPU pipeline)
//       --min-within: the fraction of points that must lie within --tol. With the NPC on, its points
//       aren't in the (static) true scene, so a scan of it needs some room.
//   lidar_verify addon [--frames 30] [--tol 0.02] [--depth-tol 0]
//                                                  check the ReShade addon's frames against a reference
//       Needs two fake_games' worth of setup: the ReShade-injected one run with
//       `--no-npc --no-publish --freeze T`, plus a plain `fake_game --no-npc --freeze T
//       --ring Local\game_lidar_ref` as reference. Same T, same window size.
//       --depth-tol: largest per-pixel depth difference that still counts as a match (default 0,
//       exact). D3D9 against a `--d24` reference needs 6e-8 in reversed modes: one 24-bit step.
//   lidar_verify compare <a> <b> [--depth-tol 0] [--matrix-tol 0]
//                                                  check that two frames saved with --dump agree
//       (the end-to-end tests compare what each graphics API captured of the same frozen scene).
//
// `ring` and `addon` take --dump <file>: save the last frame received, for `compare`.
// `ring` needs every frame to carry a pose (fake_game itself, or the addon with a profile).
// Run the fake game with --no-npc, since the moving NPC isn't in the reference scene.
// Exits non-zero if fewer than 99% of points (ply: --min-within) lie within tolerance of a surface.
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "ring.h"
#include "scene.h"
#include "unproject.h"

using namespace lidar;

namespace {

int report(std::vector<float>& err, float tol, double min_within = 0.99) {
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
    const bool pass = frac >= min_within;
    std::printf("within %.3f m: %.2f%% (needs %.2f%%)  -> %s\n", tol, frac * 100.0, min_within * 100.0,
                pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

// A frame on disk, for `compare`: magic, header, depth, then color if the header's flags have it.
constexpr uint32_t kDumpMagic = 0x4450444C;  // "LDPD"

bool save_frame(const char* path, const Frame& f) {
    FILE* out = std::fopen(path, "wb");
    if (!out) {
        std::printf("cannot write %s\n", path);
        return false;
    }
    std::fwrite(&kDumpMagic, sizeof(kDumpMagic), 1, out);
    std::fwrite(&f.header, sizeof(f.header), 1, out);
    std::fwrite(f.depth.data(), sizeof(float), f.depth.size(), out);
    if (f.header.flags & kFlagHasColor) std::fwrite(f.color.data(), sizeof(uint32_t), f.color.size(), out);
    std::fclose(out);
    return true;
}

bool load_frame(const char* path, Frame& f) {
    FILE* in = std::fopen(path, "rb");
    uint32_t magic = 0;
    bool ok = in && std::fread(&magic, sizeof(magic), 1, in) == 1 && magic == kDumpMagic &&
              std::fread(&f.header, sizeof(f.header), 1, in) == 1 && f.header.width <= kMaxWidth &&
              f.header.height <= kMaxHeight;
    if (ok) {
        const size_t n = size_t(f.header.width) * f.header.height;
        f.depth.resize(n);
        ok = std::fread(f.depth.data(), sizeof(float), n, in) == n;
        f.color.clear();
        if (ok && (f.header.flags & kFlagHasColor)) {
            f.color.resize(n);
            ok = std::fread(f.color.data(), sizeof(uint32_t), n, in) == n;
        }
    }
    if (in) std::fclose(in);
    if (!ok) std::printf("cannot read frame dump %s\n", path);
    return ok;
}

// Pixels whose color is more than kColorTol steps away in some channel (rasterization differs
// slightly between APIs). Pixels without color (alpha 0: cropped) aren't compared.
constexpr int kColorTol = 8;
bool color_differs(uint32_t a, uint32_t b) {
    for (int c = 0; c < 24; c += 8)
        if (std::abs(int((a >> c) & 0xFF) - int((b >> c) & 0xFF)) > kColorTol) return true;
    return false;
}

float matrix_diff(const float a[16], const float b[16]) {
    float d = 0;
    for (int i = 0; i < 16; ++i) d = std::max(d, std::abs(a[i] - b[i]));
    return d;
}

// Two captures of the same frozen scene (e.g. from different graphics APIs) must agree: size, pose,
// matrices, depth within depth_tol, and color where both have it.
int compare_frames(const char* path_a, const char* path_b, float depth_tol, float matrix_tol) {
    Frame a, b;
    if (!load_frame(path_a, a) || !load_frame(path_b, b)) return 2;
    if (a.header.width != b.header.width || a.header.height != b.header.height ||
        a.header.src_width != b.header.src_width || a.header.src_height != b.header.src_height) {
        std::printf("size mismatch: %ux%u of %ux%u vs %ux%u of %ux%u -> FAIL\n", a.header.width, a.header.height,
                    a.header.src_width, a.header.src_height, b.header.width, b.header.height, b.header.src_width,
                    b.header.src_height);
        return 1;
    }
    bool fail = false;
    const bool pose_a = a.header.flags & kFlagPoseValid, pose_b = b.header.flags & kFlagPoseValid;
    const float view_diff = matrix_diff(a.header.view, b.header.view);
    const float proj_diff = matrix_diff(a.header.proj, b.header.proj);
    std::printf("pose: %s / %s\n", pose_a ? "yes" : "no", pose_b ? "yes" : "no");
    std::printf("view max |diff| %.3g, projection max |diff| %.3g (tolerance %.3g)\n", view_diff, proj_diff,
                matrix_tol);
    fail |= pose_a != pose_b || proj_diff > matrix_tol || (pose_a && view_diff > matrix_tol);

    float max_diff = 0;
    size_t differ = 0, mismatched = 0;
    for (size_t i = 0; i < a.depth.size(); ++i) {
        const float d = std::abs(a.depth[i] - b.depth[i]);
        max_diff = std::max(max_diff, d);
        differ += d != 0;
        mismatched += d > depth_tol;
    }
    std::printf("depth: %zu of %zu pixels differ, max |diff| %.3g, %zu beyond %.3g\n", differ, a.depth.size(),
                max_diff, mismatched, depth_tol);
    fail |= mismatched != 0;

    if (!a.color.empty() && !b.color.empty()) {
        size_t compared = 0, off = 0;
        for (size_t i = 0; i < a.color.size(); ++i) {
            if ((a.color[i] >> 24) == 0 || (b.color[i] >> 24) == 0) continue;
            ++compared;
            off += color_differs(a.color[i], b.color[i]);
        }
        const bool bad = off * 100 > compared;  // more than 1% of pixels
        std::printf("color: %zu of %zu pixels off by more than %d steps%s\n", off, compared, kColorTol,
                    bad ? " (over 1%)" : "");
        fail |= bad;
    } else if (a.color.empty() != b.color.empty()) {
        std::printf("color: only one frame has it\n");
        fail = true;
    }
    std::printf("-> %s\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}

int verify_ring(int frames_wanted, float tol, const char* dump) {
    RingReader ring;
    const auto boxes = scene::build();
    std::vector<float> err;
    Frame f;
    int frames = 0, poseless = 0;
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
        if ((f.header.flags & kFlagPoseValid) == 0) {  // camera-relative: can't be placed in the world
            ++poseless;
            ++frames;
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
    std::printf("frames: %d (%d without a pose)\n", frames, poseless);
    if (frames == 0) {
        std::printf("no frames received (is fake_game running?)\n");
        return 2;
    }
    if (dump && !save_frame(dump, f)) return 2;
    const int result = report(err, tol);
    if (poseless > 0) std::printf("FAIL: %d frames had no pose\n", poseless);
    return poseless > 0 ? 1 : result;
}

// Both fake_games are frozen at the same camera, so the addon's depth must match the reference
// pixel for pixel. Frames with a pose (the addon's camera sniffer) are unprojected with their own
// matrices, which must also match the reference's. Pose-less frames borrow the reference's view,
// so the addon's fallback projection is what gets checked. Color, where both frames have it, must
// match within a few steps per channel (rasterization differs slightly between APIs).
int verify_addon(int frames_wanted, float tol, float depth_tol, const char* dump) {
    RingReader ref_ring, ring;
    Frame ref, f, last;
    const ULONGLONG deadline = GetTickCount64() + 15000;
    bool have_ref = false;
    while (!have_ref && GetTickCount64() < deadline) {
        if (ref_ring.is_open() || ref_ring.try_open(L"Local\\game_lidar_ref"))
            have_ref = ref_ring.read_next(ref);
        if (!have_ref) Sleep(20);
    }
    if (!have_ref) {
        std::printf("no reference frame (run fake_game --ring Local\\game_lidar_ref)\n");
        return 2;
    }

    const auto boxes = scene::build();
    std::vector<float> err;
    int frames = 0;
    float max_diff = 0, view_diff = 0;
    size_t differ = 0, mismatched = 0, compared = 0;
    int with_pose = 0;
    int with_color = 0;
    size_t color_compared = 0, color_off = 0;  // off: a channel more than kColorTol steps away
    size_t color_none = 0;                     // alpha 0: no color (cropped), not compared
    while (frames < frames_wanted && GetTickCount64() < deadline) {
        if (!ring.is_open() && !ring.try_open()) {
            Sleep(100);
            continue;
        }
        if (!ring.read_next(f)) {
            Sleep(5);
            continue;
        }
        if (f.header.width != ref.header.width || f.header.height != ref.header.height ||
            f.header.src_width != ref.header.src_width || f.header.src_height != ref.header.src_height) {
            std::printf("size mismatch: addon %ux%u of %ux%u, reference %ux%u of %ux%u\n", f.header.width,
                        f.header.height, f.header.src_width, f.header.src_height, ref.header.width, ref.header.height,
                        ref.header.src_width, ref.header.src_height);
            return 2;
        }
        if (dump) last = f;  // as captured: pose-less frames get the reference's view below
        for (size_t i = 0; i < f.depth.size(); ++i) {
            const float d = std::abs(f.depth[i] - ref.depth[i]);
            max_diff = std::max(max_diff, d);
            differ += d != 0;
            mismatched += d > depth_tol;
            ++compared;
        }
        if ((f.header.flags & kFlagHasColor) && (ref.header.flags & kFlagHasColor)) {
            ++with_color;
            for (size_t i = 0; i < size_t(f.header.width) * f.header.height; ++i) {
                if ((f.color[i] >> 24) == 0) {
                    ++color_none;
                    continue;
                }
                color_off += color_differs(f.color[i], ref.color[i]);
                ++color_compared;
            }
        }
        if (f.header.flags & kFlagPoseValid) {
            ++with_pose;
            for (int i = 0; i < 16; ++i) view_diff = std::max(view_diff, std::abs(f.header.view[i] - ref.header.view[i]));
        } else {
            std::memcpy(f.header.view, ref.header.view, sizeof(f.header.view));
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
        std::printf("no addon frames received (is the ReShade-injected fake_game running?)\n");
        return 2;
    }
    if (dump && !save_frame(dump, last)) return 2;
    float proj_diff = 0;
    for (int i = 0; i < 16; ++i) proj_diff = std::max(proj_diff, std::abs(f.header.proj[i] - ref.header.proj[i]));
    std::printf("depth vs reference: %zu of %zu pixels differ, max |diff| %.3g", differ, compared, max_diff);
    if (depth_tol > 0) std::printf(", %zu beyond %.3g", mismatched, depth_tol);
    std::printf("\n");
    std::printf("projection vs reference: max |diff| %.3g\n", proj_diff);
    std::printf("frames with the addon's pose: %d of %d", with_pose, frames);
    if (with_pose > 0) std::printf(", view vs reference: max |diff| %.3g", view_diff);
    std::printf("\n");
    const bool color_bad = color_off * 100 > color_compared;  // more than 1% of pixels
    if (with_color > 0)
        std::printf("frames with color: %d of %d, %zu of %zu pixels off by more than %d steps%s, %zu without color "
                    "(cropped)\n", with_color, frames, color_off, color_compared, kColorTol,
                    color_bad ? " (over 1%: FAIL)" : "", color_none);
    else
        std::printf("frames with color: 0 of %d%s\n", frames,
                    ref.header.flags & kFlagHasColor ? "" : " (the reference has no color either)");
    return report(err, tol) != 0 || mismatched != 0 || color_bad ? 1 : 0;
}

int verify_ply(const char* path, float tol, double min_within) {
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
    return report(err, tol, min_within);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: lidar_verify ring [--frames N] [--tol m] [--dump file] |\n"
                    "                    ply <file> [--tol m] [--min-within f] |\n"
                    "                    addon [--frames N] [--tol m] [--depth-tol d] [--dump file] |\n"
                    "                    compare <a> <b> [--depth-tol d] [--matrix-tol m]\n");
        return 2;
    }
    const std::string mode = argv[1];
    float tol = 0.02f, depth_tol = 0, matrix_tol = 0;
    double min_within = 0.99;
    int frames = 30;
    const char* dump = nullptr;
    std::vector<const char*> files;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--tol" && i + 1 < argc) tol = float(std::atof(argv[++i]));
        else if (a == "--depth-tol" && i + 1 < argc) depth_tol = float(std::atof(argv[++i]));
        else if (a == "--matrix-tol" && i + 1 < argc) matrix_tol = float(std::atof(argv[++i]));
        else if (a == "--min-within" && i + 1 < argc) min_within = std::atof(argv[++i]);
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (a == "--dump" && i + 1 < argc) dump = argv[++i];
        else files.push_back(argv[i]);
    }
    if (mode == "ring") return verify_ring(frames, tol, dump);
    if (mode == "ply" && files.size() == 1) return verify_ply(files[0], tol, min_within);
    if (mode == "addon") return verify_addon(frames, tol, depth_tol, dump);
    if (mode == "compare" && files.size() == 2) return compare_frames(files[0], files[1], depth_tol, matrix_tol);
    std::printf("bad arguments\n");
    return 2;
}
