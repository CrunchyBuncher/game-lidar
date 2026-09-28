// lidar_capture.addon64: ReShade addon that captures the scene depth buffer and the game's
// camera matrices and publishes them to the shared-memory ring for the viewer.
//
// Depth path: depth_tracker picks the scene depth-stencil, a DepthCapture reads it back.
// Camera path: camera_tracker latches the profile's cbuffer window at the draws into each
// depth-stencil (bytes from a CbufferSource), and at present the latch of the captured
// depth-stencil becomes the frame's pose. Without a profile or a latch, frames go out
// camera-relative (no pose flag) with a projection from the overlay settings.
//
// Only the DepthCapture and CbufferSource implementations are API-specific (backends.cpp).
//
// Discovery mode (discovery/, its own module) runs on top of both paths: the camera tracker samples draws, the
// published depth frames are tapped from the ring, and its candidates can be previewed (used as an
// unsaved profile) or saved as lidar_profile.toml.
#include <imgui.h>
#include <reshade.hpp>

#include <DirectXMath.h>
#include <windows.h>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "camera_tracker.h"
#include "cbuffer_source.h"
#include "depth_capture.h"
#include "depth_tracker.h"
#include "discovery.h"
#include "modelview.h"
#include "profile.h"
#include "ring.h"
#include "watchdog.h"

using namespace reshade::api;
using namespace lidar;

extern "C" __declspec(dllexport) const char* NAME = "game-lidar capture";
extern "C" __declspec(dllexport) const char* DESCRIPTION =
    "Captures depth and camera matrices and streams them to lidar_viewer.";

namespace {

constexpr char kSection[] = "LIDAR";
constexpr char kDefaultProfile[] = "lidar_profile.toml";
constexpr char kDiscoveryReport[] = "lidar_discovery.txt";
constexpr uint32_t kDiscoveryBytes = 8192;  // per bound buffer at a sampled draw
constexpr double kRecordSeconds = 30;       // discovery recordings (discovery/README.md)
constexpr uint64_t kRecordMaxBytes = 2ull << 30;

enum class DepthMode : int { Standard = 0, Reversed = 1, ReversedInfinite = 2 };
constexpr const char* kCropKeys[4] = {"ColorCropLeft", "ColorCropTop", "ColorCropRight", "ColorCropBottom"};

struct Settings {
    bool enabled = true;
    bool color = true;  // also capture the scene's color, where the backend can
    float color_crop[4] = {};  // percent of the image from the left, top, right, bottom without color
    int capture_width = 480;
    std::string profile = kDefaultProfile;  // relative to the game's folder
    // Fallback projection for frames without a camera pose. Defaults match fake_game.
    float fov_y_deg = 70.0f;
    float near_z = 0.1f;
    float far_z = 1000.0f;
    int depth_mode = int(DepthMode::Reversed);
    // Discovery, read from the ini only (for unattended runs): start with the game, save the best
    // candidate after this many seconds once it's confident (0 = never), draws sampled per frame,
    // record the session for this many seconds (0 = don't).
    bool discovery_autostart = false;
    int discovery_autosave = 0;
    int discovery_samples = 48;
    int discovery_record = 0;

    void load() {
        reshade::get_config_value(nullptr, kSection, "DiscoveryAutoStart", discovery_autostart);
        reshade::get_config_value(nullptr, kSection, "DiscoveryAutoSave", discovery_autosave);
        reshade::get_config_value(nullptr, kSection, "DiscoverySamples", discovery_samples);
        discovery_samples = std::clamp(discovery_samples, 4, 512);
        reshade::get_config_value(nullptr, kSection, "DiscoveryRecord", discovery_record);
        discovery_record = std::clamp(discovery_record, 0, 600);
        reshade::get_config_value(nullptr, kSection, "Enabled", enabled);
        reshade::get_config_value(nullptr, kSection, "Color", color);
        for (int i = 0; i < 4; ++i) {
            reshade::get_config_value(nullptr, kSection, kCropKeys[i], color_crop[i]);
            color_crop[i] = std::clamp(color_crop[i], 0.0f, 45.0f);
        }
        reshade::get_config_value(nullptr, kSection, "CaptureWidth", capture_width);
        char buf[1024];
        size_t n = sizeof(buf);
        if (reshade::get_config_value(nullptr, kSection, "Profile", buf, &n) && buf[0] != '\0') profile = buf;
        reshade::get_config_value(nullptr, kSection, "FovY", fov_y_deg);
        reshade::get_config_value(nullptr, kSection, "Near", near_z);
        reshade::get_config_value(nullptr, kSection, "Far", far_z);
        reshade::get_config_value(nullptr, kSection, "DepthMode", depth_mode);
    }
    void save() const {
        reshade::set_config_value(nullptr, kSection, "Enabled", enabled);
        reshade::set_config_value(nullptr, kSection, "Color", color);
        for (int i = 0; i < 4; ++i) reshade::set_config_value(nullptr, kSection, kCropKeys[i], color_crop[i]);
        reshade::set_config_value(nullptr, kSection, "CaptureWidth", capture_width);
        reshade::set_config_value(nullptr, kSection, "Profile", profile.c_str());
        reshade::set_config_value(nullptr, kSection, "FovY", fov_y_deg);
        reshade::set_config_value(nullptr, kSection, "Near", near_z);
        reshade::set_config_value(nullptr, kSection, "Far", far_z);
        reshade::set_config_value(nullptr, kSection, "DepthMode", depth_mode);
    }
};

// What happened to the camera on the last captured frame (overlay readout).
struct CameraStatus {
    enum State { NoProfile, NoLatch, Rejected, Latched } state = NoProfile;
    std::string why;  // for Rejected
    float view[16] = {}, proj[16] = {};
    ProjectionInfo info;
    uint64_t with_pose = 0, without_pose = 0;
    uint64_t recent = 0;  // the last 64 captured frames, bit set: no pose (newest in bit 0)
};

// The poses of the last second, logged as ranges: a profile that latches more than one camera (another
// view's buffer at the same register, a camera-relative matrix) shows up as FOV/aspect ranges or
// position jumps.
struct PoseStats {
    uint32_t posed = 0, unposed = 0;
    float fov_min = 0, fov_max = 0, aspect_min = 0, aspect_max = 0;
    double max_step = 0;          // meters between consecutive posed frames
    double pos[3] = {}, last[3] = {};
    bool have_last = false;
    std::chrono::steady_clock::time_point since = std::chrono::steady_clock::now();
};
PoseStats g_pose_stats;

// Touched from the game's render thread (present) and the overlay, which ReShade draws on the
// same thread; the mutex covers games that differ.
std::mutex g_mutex;
Settings g_settings;
device* g_device = nullptr;  // the device we capture from (first supported one created)
std::unique_ptr<DepthCapture> g_capture;
std::unique_ptr<cam::CbufferSource> g_source;
RingWriter g_ring;
std::string g_init_error;
uint64_t g_frame = 0;

Profile g_profile;
std::filesystem::path g_profile_path;
std::string g_profile_error;
CameraStatus g_camera;

// A model-view camera (plan_modelview.md): the solver over every draw into the captured depth buffer.
mv::Solver g_solver;
mv::Result g_mv;                    // last frame's
std::vector<mv::Draw> g_mv_input;   // reused
double g_mv_us = 0;                 // smoothed solve time

std::vector<depth::Candidate> g_candidates;  // last frame's depth-stencils, best first
uint64_t g_selected = 0;                     // handle captured last frame
uint64_t g_override = 0;                     // manual pick from the overlay, 0 = auto
// A snapshot of the captured depth-stencil taken before a clear this frame (games that reuse it for a
// later pass), and the pass it holds.
struct DepthSnapshot {
    uint64_t ds = 0;
    depth::DrawStats pass;
    uint32_t pass_index = 0;  // cam::PassKey::pass
};
DepthSnapshot g_snapshot;
std::optional<DepthSnapshot> g_snapshot_used;  // last capture's, for the overlay (nullopt: at present)
double g_cpu_us = 0;                         // smoothed CPU cost of our present work

disc::Discovery g_discovery;
bool g_discovering = false;
uint64_t g_ring_seen = 0;                // last ring seq handed to discovery
std::optional<CameraProfile> g_preview;  // a discovery candidate in use instead of the profile (not saved)
disc::Status g_disc_status;              // refreshed at most every half second
std::chrono::steady_clock::time_point g_disc_polled{}, g_disc_logged{};
bool g_disc_autosaved = false;
std::string g_disc_message;

std::filesystem::path game_dir() {
    wchar_t buf[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    return std::filesystem::path(std::wstring(buf, n)).parent_path();
}

// Which of a pass' cameras the pose follows (other views' buffers can share the register, e.g. UE's
// cube captures and a second camera): the one near the last pose. A camera far from it only takes over
// after kSwitchFrames frames without a near one (a cut), so a view that wins a frame or two doesn't
// move the pose.
struct Follow {
    bool have = false;
    double pos[3] = {};  // game units
    uint64_t frame = 0;  // g_frame of the last pose
    uint32_t far_frames = 0;
};
Follow g_follow;
constexpr uint32_t kSwitchFrames = 15;
constexpr double kFollowMeters = 3;     // per frame since the last pose
constexpr uint64_t kFollowFrames = 30;  // a last pose older than this is forgotten

// Whether `proj` fits a depth buffer of aspect ratio `target` (0: unknown, anything does). Cube captures
// and shadow maps are square; a scene camera has the depth buffer's shape.
bool aspect_fits(const float proj[16], float target) {
    return target <= 0 || std::abs(analyze_projection(proj).aspect / target - 1) < 0.1f;
}

// The camera in use: a previewed discovery candidate, or the profile's.
const CameraProfile* active_camera() {
    if (g_preview) return &*g_preview;
    return g_profile.has_camera ? &g_profile.camera : nullptr;
}

// Points the camera tracker at the active camera. Caller holds g_mutex.
void configure_camera() {
    g_camera = {};
    g_follow = {};
    g_solver = mv::Solver();
    g_mv = {};
    const CameraProfile* c = active_camera();
    if (c != nullptr && g_source != nullptr && c->model_view()) {
        // The constant projection is latched like any camera; the per-draw window is recorded at every draw.
        cam::LatchRequest req{c->key, c->proj_offset, 64, Latch::First};
        req.accept = [camera = *c](const std::vector<uint8_t>& w, float aspect) {
            float proj[16];
            return decode_projection(camera, w.data(), w.size(), proj) && aspect_fits(proj, aspect);
        };
        cam::configure(g_device, g_source.get(), &req);
        const cam::DrawRequest draws{c->key, c->view_offset};
        cam::configure_draws(g_device, g_source.get(), &draws);
    } else if (c != nullptr && g_source != nullptr) {
        cam::LatchRequest req{c->key, c->window_offset(), c->window_size(), c->latch};
        req.accept = [camera = *c](const std::vector<uint8_t>& w, float aspect) {
            float view[16], proj[16];
            return decode_camera(camera, w.data(), w.size(), view, proj) && aspect_fits(proj, aspect);
        };
        cam::configure(g_device, g_source.get(), &req);
        cam::configure_draws(g_device, nullptr, nullptr);
    } else {
        cam::configure(g_device, nullptr, nullptr);
        cam::configure_draws(g_device, nullptr, nullptr);
    }
}

// (Re)loads the profile and points the camera tracker at it. Ends any preview. Caller holds g_mutex.
void reload_profile() {
    g_profile = {};
    g_profile_error.clear();
    g_preview.reset();
    g_profile_path = std::filesystem::path(std::u8string(g_settings.profile.begin(), g_settings.profile.end()));
    if (g_profile_path.is_relative()) g_profile_path = game_dir() / g_profile_path;
    if (g_source == nullptr)
        g_profile_error = "camera sniffing isn't supported for this graphics API yet";
    else
        load_profile(g_profile_path, g_profile, g_profile_error);
    configure_camera();
}

void log_info(const std::string& msg) { reshade::log::message(reshade::log::level::info, msg.c_str()); }

// Microseconds since `t0`, a QueryPerformanceCounter reading.
double us_since(const LARGE_INTEGER& t0) {
    LARGE_INTEGER t1, freq;
    QueryPerformanceCounter(&t1);
    QueryPerformanceFrequency(&freq);
    return double(t1.QuadPart - t0.QuadPart) * 1e6 / double(freq.QuadPart);
}

double dist3(const double a[3], const double b[3]) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

// Where the camera is in the world, in the game's units (view = [R 0; t 1], row vectors: p = -t * R^T).
void camera_position(const float view[16], double pos[3]) {
    for (int i = 0; i < 3; ++i)
        pos[i] = -(double(view[12]) * view[i * 4] + double(view[13]) * view[i * 4 + 1] + double(view[14]) * view[i * 4 + 2]);
}

// ---- Discovery ------------------------------------------------------------------------------

const char* api_text(device_api api) {
    switch (api) {
        case device_api::d3d9: return "d3d9";
        case device_api::d3d11: return "d3d11";
        case device_api::d3d12: return "d3d12";
        default: return "other";
    }
}

std::string exe_stem() {
    wchar_t buf[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    return std::filesystem::path(std::wstring(buf, n)).stem().string();
}

// lidar_discovery_<exe>_<date>.disc next to the exe.
std::filesystem::path recording_path() {
    const std::time_t now = std::time(nullptr);
    char date[32];
    std::strftime(date, sizeof(date), "%Y%m%d_%H%M%S", std::localtime(&now));
    return game_dir() / ("lidar_discovery_" + exe_stem() + "_" + date + ".disc");
}

// `record_seconds` > 0: also records the session from its start (a recording replays exactly only
// from the analyzer's first input).
void start_discovery(double record_seconds = 0) {
    if (g_source == nullptr || g_discovering) return;
    g_discovery.start(g_device->get_api() == device_api::d3d9);
    if (record_seconds > 0) {
        const std::filesystem::path path = recording_path();
        char note[256];
        std::snprintf(note, sizeof(note), "game=%s\napi=%s\nsamples_per_frame=%d\nbuffer_bytes=%u\ncapture_width=%d\n",
                      exe_stem().c_str(), api_text(g_device->get_api()), g_settings.discovery_samples, kDiscoveryBytes,
                      g_settings.capture_width);
        g_discovery.start_recording(path, note, record_seconds, kRecordMaxBytes);
        log_info("Discovery recording to " + path.string());
    }
    cam::configure_discovery(g_device, g_source.get(), uint32_t(g_settings.discovery_samples), kDiscoveryBytes);
    g_discovering = true;
    g_ring_seen = g_ring.latest_seq();
    g_disc_autosaved = false;
    g_disc_message.clear();
    g_disc_logged = std::chrono::steady_clock::now();
    log_info("Discovery started (" + std::to_string(g_settings.discovery_samples) + " sampled draws per frame).");
}

void stop_discovery() {
    if (!g_discovering) return;
    cam::configure_discovery(g_device, nullptr, 0, 0);
    g_discovery.stop();
    g_disc_status = g_discovery.status();
    g_discovering = false;
    log_info("Discovery stopped.");
}

using disc::candidate_line;

void log_discovery(const disc::Status& s) {
    log_info("Discovery " + disc::status_line(s));
    for (size_t i = 0; i < s.candidates.size() && i < 5; ++i)
        log_info("  #" + std::to_string(i + 1) + " " + candidate_line(s.candidates[i]));
}

// Sets `key = value` in a profile file, keeping the rest of it (comments too). Profiles have one
// section, [camera], so the line goes at the end.
void write_profile_key(const std::filesystem::path& path, std::string_view key, const std::string& value) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return;
    std::string text, line;
    while (std::getline(in, line)) {
        const size_t start = line.find_first_not_of(" \t");
        const bool match = start != std::string::npos && line.compare(start, key.size(), key) == 0 &&
                           (line.size() == start + key.size() || line[start + key.size()] == ' ' ||
                            line[start + key.size()] == '\t' || line[start + key.size()] == '=');
        if (!match) text += line + "\n";
    }
    in.close();
    while (text.size() >= 2 && text.ends_with("\n\n")) text.pop_back();
    const std::string set = std::string(key) + " = " + value + "\n";
    std::ofstream(path, std::ios::binary) << text << set;
    log_info("Profile: " + set);
}

// Writes the candidate as lidar_profile.toml next to the exe (keeping the old one as .bak), makes
// it the profile and reloads. Caller holds g_mutex.
bool save_candidate(const disc::CandidateInfo& c) {
    const std::filesystem::path path = game_dir() / kDefaultProfile;
    std::error_code ec;
    if (std::filesystem::exists(path, ec))
        std::filesystem::rename(path, std::filesystem::path(path).concat(".bak"), ec);
    const std::time_t now = std::time(nullptr);
    char date[32];
    std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M", std::localtime(&now));
    std::string comment = std::string("Written by game-lidar discovery mode, ") + date + ".\n" + candidate_line(c);
    CameraProfile camera = c.profile;
    if (const CameraProfile* active = active_camera()) camera.units_per_meter = active->units_per_meter;
    std::ofstream f(path, std::ios::binary);
    f << format_profile(camera, comment);
    f.close();
    if (!f) {
        g_disc_message = "Couldn't write " + path.string();
        return false;
    }
    g_settings.profile = kDefaultProfile;
    g_settings.save();
    reload_profile();
    g_disc_message = g_profile.has_camera ? "Saved " + path.string() + " and loaded it."
                                          : "Saved " + path.string() + ", but it didn't load: " + g_profile_error;
    log_info(g_disc_message);
    if (g_discovering) g_discovery.request_report(game_dir() / kDiscoveryReport);
    return g_profile.has_camera;
}

// Once per present while discovering: refresh the status, log it now and then, auto-save.
void discovery_tick() {
    const auto now = std::chrono::steady_clock::now();
    if (now - g_disc_polled < std::chrono::milliseconds(500)) return;
    g_disc_polled = now;
    g_disc_status = g_discovery.status();
    if (now - g_disc_logged >= std::chrono::seconds(10)) {
        g_disc_logged = now;
        log_discovery(g_disc_status);
    }
    const int autosave = g_settings.discovery_autosave;
    if (autosave > 0 && !g_disc_autosaved && g_disc_status.seconds >= autosave && !g_disc_status.candidates.empty() &&
        g_disc_status.candidates.front().confident) {
        g_disc_autosaved = true;
        log_discovery(g_disc_status);
        log_info("Discovery auto-save: " + candidate_line(g_disc_status.candidates.front()));
        save_candidate(g_disc_status.candidates.front());
    }
}

// Hands this frame's samples (for the captured depth-stencil) and newly published depth frames to
// discovery.
void feed_discovery(cam::FrameSamples& samples, uint64_t captured_ds) {
    const uint64_t latest = g_ring.latest_seq();
    for (uint64_t seq = std::max(g_ring_seen, latest > kSlotCount ? latest - kSlotCount : 0) + 1; seq <= latest; ++seq) {
        const Slot* s = g_ring.slot(seq);
        g_discovery.submit_depth(s->frame, s->depth);
    }
    g_ring_seen = latest;
    if (const auto it = samples.find(captured_ds); it != samples.end())
        g_discovery.submit_samples(g_frame, std::move(it->second));
}

// Row-major, row-vector (D3D) projection, as protocol.h expects. Same math as fake_game.
void make_proj(const Settings& s, float aspect, float out[16]) {
    using namespace DirectX;
    const float fov = XMConvertToRadians(s.fov_y_deg);
    XMMATRIX p;
    switch (DepthMode(s.depth_mode)) {
        case DepthMode::Standard:
            p = XMMatrixPerspectiveFovLH(fov, aspect, s.near_z, s.far_z);
            break;
        case DepthMode::ReversedInfinite: {
            const float ys = 1.0f / std::tan(fov * 0.5f), xs = ys / aspect;
            p = XMMATRIX(xs, 0, 0, 0, 0, ys, 0, 0, 0, 0, 0, 1, 0, 0, s.near_z, 0);
            break;
        }
        default:
            p = XMMatrixPerspectiveFovLH(fov, aspect, s.far_z, s.near_z);
            break;
    }
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(out), p);
}

// A model-view camera's pose: the latched projection, and the view the solver finds in this frame's
// draws into the captured depth-stencil.
bool solve_model_view(const CameraProfile& camera, const cam::CbufferRead& proj_read, const cam::FrameDraws& draws,
                      uint64_t depth_stencil, float view[16], float proj[16]) {
    if (!decode_projection(camera, proj_read.bytes.data(), proj_read.bytes.size(), proj, &g_camera.why)) return false;
    const auto d = draws.find(depth_stencil);
    if (d == draws.end() || d->second.empty()) {
        g_camera.why = "no draws recorded (model-view cameras need Direct3D 9 for now)";
        return false;
    }
    LARGE_INTEGER t0;
    QueryPerformanceCounter(&t0);
    cam::solver_draws(d->second, camera.column_major, g_mv_input);
    g_mv = g_solver.solve(g_mv_input);
    if (g_mv.new_segment)
        log_info("Camera: model-view segment " + std::to_string(g_mv.segment) +
                 (g_mv.anchor_shared != 0 ? ", in the game's world frame (" + std::to_string(g_mv.anchor_shared) +
                                                " objects share the anchor matrix)"
                                          : ", in one object's frame (nothing shares a matrix: may be tilted)"));
    g_mv_us = g_mv_us * 0.9 + us_since(t0) * 0.1;
    if (!g_mv.posed) {
        g_camera.why = "the draws don't agree on a camera this frame (" + std::to_string(g_mv.inliers) + " of " +
                       std::to_string(g_mv.known) + " known objects agree)";
        return false;
    }
    mat::store(g_mv.view, view);
    return true;
}

// Decodes the pass' latched cameras and picks the one to follow (see Follow). Returns false, with
// g_camera.why set, if there's none this frame.
bool pick_camera(const CameraProfile& camera, const std::vector<cam::LatchedRead>& latched, float aspect,
                 float view[16], float proj[16]) {
    struct Option {
        float view[16], proj[16];
        double pos[3];
        uint32_t draws;
    };
    std::vector<Option> options;
    for (const cam::LatchedRead& l : latched) {
        Option o;
        if (!decode_camera(camera, l.read.bytes.data(), l.read.bytes.size(), o.view, o.proj, &g_camera.why)) continue;
        if (!l.accepted) {  // decodes, so it's the shape
            char buf[128];
            std::snprintf(buf, sizeof(buf), "the projection's aspect ratio (%.2f) isn't the depth buffer's (%.2f)",
                          analyze_projection(o.proj).aspect, aspect);
            g_camera.why = buf;
            continue;
        }
        camera_position(o.view, o.pos);
        o.draws = l.draws;
        options.push_back(o);
    }
    if (options.empty()) return false;
    size_t chosen = 0;  // without a pose to follow: the one most draws had
    for (size_t i = 1; i < options.size(); ++i)
        if (options[i].draws > options[chosen].draws) chosen = i;
    if (g_follow.have && g_frame - g_follow.frame <= kFollowFrames) {
        auto dist = [&](const Option& o) { return dist3(o.pos, g_follow.pos); };
        size_t nearest = 0;
        for (size_t i = 1; i < options.size(); ++i)
            if (dist(options[i]) < dist(options[nearest])) nearest = i;
        const double limit =
            kFollowMeters * camera.units_per_meter * double(std::max<uint64_t>(1, g_frame - g_follow.frame));
        if (dist(options[nearest]) <= limit) {
            chosen = nearest;
            g_follow.far_frames = 0;
        } else if (++g_follow.far_frames < kSwitchFrames) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "the camera jumped %.1f m (followed if it stays there)",
                          dist(options[nearest]) / camera.units_per_meter);
            g_camera.why = buf;
            return false;
        } else {
            g_follow.far_frames = 0;  // it stayed: a cut
        }
    }
    const Option& o = options[chosen];
    std::memcpy(view, o.view, sizeof(o.view));
    std::memcpy(proj, o.proj, sizeof(o.proj));
    g_follow.have = true;
    std::copy_n(o.pos, 3, g_follow.pos);
    g_follow.frame = g_frame;
    return true;
}

// Fills the header's pose from this frame's latch (or model-view solve) for the captured depth-stencil.
// The camera latched in `pass`: the one the captured depth comes from, whose aspect ratio is `aspect`.
bool apply_camera(const cam::FrameLatches& latches, const cam::FrameDraws& draws, cam::PassKey pass, float aspect,
                  FrameHeader& h) {
    const uint64_t depth_stencil = pass.ds;
    const CameraProfile* camera = active_camera();
    if (camera == nullptr) {
        g_camera.state = CameraStatus::NoProfile;
        return false;
    }
    const auto it = latches.find(pass);
    if (it == latches.end() || it->second.empty()) {
        g_camera.state = CameraStatus::NoLatch;
        return false;
    }
    float view[16], proj[16];
    const bool ok = camera->model_view()
                        ? solve_model_view(*camera, it->second.front().read, draws, depth_stencil, view, proj)
                        : pick_camera(*camera, it->second, aspect, view, proj);
    if (!ok) {
        g_camera.state = CameraStatus::Rejected;
        return false;
    }
    g_camera.state = CameraStatus::Latched;
    g_camera.info = analyze_projection(proj);  // in the game's units and handedness
    normalize_pose(view, proj, g_camera.info.right_handed, camera->units_per_meter, camera->z_up);
    std::memcpy(g_camera.view, view, sizeof(view));
    std::memcpy(g_camera.proj, proj, sizeof(proj));
    std::memcpy(h.view, view, sizeof(h.view));
    std::memcpy(h.proj, proj, sizeof(h.proj));
    h.flags |= kFlagPoseValid;
    return true;
}

// Logs the camera status when it changes (at most once a second), so ReShade.log tells why frames
// go out without a pose even when nobody looks at the overlay.
void log_camera_changes() {
    static CameraStatus::State last_state = CameraStatus::NoProfile;
    static std::string last_why;
    static std::chrono::steady_clock::time_point last_time{};
    const std::string why = g_camera.state == CameraStatus::Rejected ? g_camera.why : std::string();
    const auto now = std::chrono::steady_clock::now();
    if ((g_camera.state == last_state && why == last_why) || now - last_time < std::chrono::seconds(1)) return;
    last_state = g_camera.state;
    last_why = why;
    last_time = now;
    static const char* const kStates[] = {"no profile", "no latch", "rejected", "pose"};
    std::string msg = std::string("Camera: ") + kStates[g_camera.state];
    if (!why.empty()) msg += ": " + why;
    if (const CameraProfile* c = active_camera(); c != nullptr && c->model_view())
        msg += " (model-view: segment " + std::to_string(g_mv.segment) + ", " + std::to_string(g_mv.draws) +
               " draws, " + std::to_string(g_mv.known) + " known, " + std::to_string(g_mv.inliers) + " agree)";
    log_info(msg);
}

// Counts this frame into g_pose_stats, and logs them once a second.
void track_pose(bool posed) {
    PoseStats& s = g_pose_stats;
    if (posed) {
        double p[3];
        camera_position(g_camera.view, p);
        if (s.have_last) s.max_step = std::max(s.max_step, dist3(p, s.last));
        std::copy_n(p, 3, s.last);
        std::copy_n(p, 3, s.pos);
        s.have_last = true;
        const float fov = g_camera.info.fov_y_deg, aspect = g_camera.info.aspect;
        if (s.posed++ == 0) {
            s.fov_min = s.fov_max = fov;
            s.aspect_min = s.aspect_max = aspect;
        }
        s.fov_min = std::min(s.fov_min, fov), s.fov_max = std::max(s.fov_max, fov);
        s.aspect_min = std::min(s.aspect_min, aspect), s.aspect_max = std::max(s.aspect_max, aspect);
    } else {
        ++s.unposed;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - s.since < std::chrono::seconds(1)) return;
    if (s.posed + s.unposed > 0 && active_camera() != nullptr) {
        char buf[256];
        if (s.posed > 0)
            std::snprintf(buf, sizeof(buf),
                          "Camera, last second: %u posed, %u not; FOV %.1f-%.1f, aspect %.2f-%.2f, at (%.1f, %.1f, "
                          "%.1f) m, largest step %.2f m",
                          s.posed, s.unposed, s.fov_min, s.fov_max, s.aspect_min, s.aspect_max, s.pos[0], s.pos[1],
                          s.pos[2], s.max_step);
        else
            std::snprintf(buf, sizeof(buf), "Camera, last second: 0 posed, %u not", s.unposed);
        log_info(buf);
    }
    const bool keep = s.have_last;
    const double last[3] = {s.last[0], s.last[1], s.last[2]};
    s = {};
    s.have_last = keep;  // steps span the seconds
    std::copy_n(last, 3, s.last);
}

const char* api_name(device_api api) {
    switch (api) {
        case device_api::d3d9: return "Direct3D 9";
        case device_api::d3d10: return "Direct3D 10";
        case device_api::d3d11: return "Direct3D 11";
        case device_api::d3d12: return "Direct3D 12";
        case device_api::opengl: return "OpenGL";
        case device_api::vulkan: return "Vulkan";
    }
    return "this graphics API";
}

void register_active_handlers();  // below, next to the handlers it registers
void on_depth_clear(command_list* cmd, resource ds, resource color, const depth::DrawStats& pass, uint32_t pass_index);

// Until a supported device shows up, the addon's only handler is init_device. Registering an
// event can change how ReShade hooks the game (on D3D9, map events wrap every buffer Lock), so on
// an API we don't support the addon must not register anything else: the game has to run exactly
// as it does without the addon. On the first supported device, the shared modules register, plus
// that API's backend (only it: D3D11's map events would cost a D3D9 game for nothing). That device
// was created before the modules' handlers existed, so it's set up by hand. On D3D9 this runs
// before the device's auto depth-stencil is created, so the backend can still make it readable.
void attach_modules(device* dev) {
    static bool registered = false;
    if (!registered) {
        depth::register_events();
        cam::register_events();
        register_active_handlers();
        registered = true;
    }
    static std::vector<device_api> backends;
    if (std::find(backends.begin(), backends.end(), dev->get_api()) == backends.end()) {
        cam::register_source_events(dev->get_api());
        register_capture_events(dev->get_api());
        backends.push_back(dev->get_api());
    }
    depth::init_device(dev);
    cam::init_device(dev);
    cam::init_source_device(dev);
    init_capture_device(dev);
}

void on_init_device(device* dev) {
    const watchdog::Step step("init_device");
    const std::lock_guard lock(g_mutex);
    if (!is_supported(dev)) {
        static bool logged = false;
        if (!logged) {
            const std::string msg = std::string("Inactive: the game created a ") + api_name(dev->get_api()) +
                                    " device, which isn't supported (D3D9, D3D11 and D3D12 are). Nothing else is hooked.";
            reshade::log::message(reshade::log::level::warning, msg.c_str());
            logged = true;
        }
        return;
    }
    // Later supported devices get this from the modules' own init_device handlers too, but those
    // run after this one.
    attach_modules(dev);
    if (g_device != nullptr) return;
    std::string error;
    std::unique_ptr<DepthCapture> capture = create_depth_capture(dev, error);
    if (capture == nullptr && g_init_error.empty()) {
        g_init_error = error;  // keep looking: a later device may be supported
        return;
    }
    if (capture == nullptr) return;
    g_device = dev;
    watchdog::start();
    g_init_error.clear();
    g_capture = std::move(capture);
    depth::set_clear_hook(on_depth_clear);
    g_source = cam::create_cbuffer_source(dev);
    g_settings.load();
    if (!g_ring.open()) g_init_error = "failed to create the shared-memory ring";
    reload_profile();
    if (g_settings.discovery_autostart) start_discovery(g_settings.discovery_record);
}

// Before a depth clear: if it's the captured depth-stencil and this pass is its busiest so far this
// frame, snapshot it (the frame's final contents may be a later pass that isn't the scene).
void on_depth_clear(command_list* cmd, resource ds, resource color, const depth::DrawStats& pass, uint32_t pass_index) {
    const std::lock_guard lock(g_mutex);
    if (cmd->get_device() != g_device || g_capture == nullptr || !g_init_error.empty() || !g_settings.enabled ||
        ds.handle != g_selected)
        return;
    if (g_snapshot.ds == ds.handle && !pass.better_than(g_snapshot.pass)) return;
    const watchdog::Step step("depth clear: snapshot");
    if (g_capture->snapshot(ds, g_settings.color ? color : resource{0}, uint32_t(g_settings.capture_width))) g_snapshot = {ds.handle, pass, pass_index};
}

void on_destroy_device(device* dev) {
    const watchdog::Step step("destroy_device");
    const std::lock_guard lock(g_mutex);
    if (dev != g_device) return;
    depth::set_clear_hook(nullptr);
    g_snapshot = {};
    stop_discovery();
    watchdog::stop();
    cam::configure(dev, nullptr, nullptr);
    g_source.reset();
    g_capture.reset();
    g_ring.close();
    g_candidates.clear();
    g_device = nullptr;
}

void on_present(command_queue* queue, swapchain* sc, const rect*, const rect*, uint32_t, const rect*) {
    device* const dev = sc->get_device();
    watchdog::heartbeat();
    const watchdog::Step step("present: waiting for the addon lock");
    const std::lock_guard lock(g_mutex);
    if (dev != g_device || !g_init_error.empty()) return;

    LARGE_INTEGER t0;
    QueryPerformanceCounter(&t0);

    watchdog::exchange_step("present: camera end_frame");
    cam::FrameSamples samples;
    cam::FrameDraws draws;
    const CameraProfile* camera = active_camera();
    const cam::FrameLatches latches = cam::end_frame(dev, g_discovering ? &samples : nullptr,
                                                     camera != nullptr && camera->model_view() ? &draws : nullptr);
    watchdog::exchange_step("present: depth end_frame");
    const resource_desc bb = dev->get_resource_desc(sc->get_current_back_buffer());
    std::vector<depth::Candidate> candidates = depth::end_frame(dev, bb.texture.width, bb.texture.height);
    const DepthSnapshot snapshot = std::exchange(g_snapshot, {});
    if (candidates.empty()) {  // e.g. a second present without any rendering in between
        g_capture->drop_snapshot();
        return;
    }
    g_candidates = std::move(candidates);

    watchdog::exchange_step("present: publish");
    float crop[4];
    for (int i = 0; i < 4; ++i) crop[i] = g_settings.color_crop[i] / 100;
    g_capture->set_color_crop(crop);
    g_capture->publish(queue, g_ring);
    watchdog::exchange_step("present: capture");

    const depth::Candidate* pick = nullptr;
    for (const auto& c : g_candidates)
        if (c.resource.handle == g_override) pick = &c;
    if (pick == nullptr && g_candidates.front().fits_frame) pick = &g_candidates.front();
    g_selected = pick ? pick->resource.handle : 0;
    // The snapshot, unless the pass still in the depth-stencil drew more.
    const bool use_snapshot = pick != nullptr && g_settings.enabled && snapshot.ds == pick->resource.handle &&
                              !pick->last_segment.better_than(snapshot.pass);
    if (!use_snapshot) g_capture->drop_snapshot();

    if (g_settings.enabled && pick != nullptr) {
        FrameHeader h{};
        h.frame_index = g_frame;
        h.timestamp_qpc = uint64_t(t0.QuadPart);
        const cam::PassKey pass{pick->resource.handle, use_snapshot ? snapshot.pass_index : pick->last_pass};
        const float aspect = float(pick->desc.texture.width) / float(std::max(1u, pick->desc.texture.height));
        const bool posed = apply_camera(latches, draws, pass, aspect, h);
        log_camera_changes();
        track_pose(posed);
        g_camera.recent = (g_camera.recent << 1) | (posed ? 0 : 1);
        if (posed) {
            ++g_camera.with_pose;
        } else {
            ++g_camera.without_pose;
            const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            std::memcpy(h.view, identity, sizeof(h.view));
            make_proj(g_settings, float(pick->desc.texture.width) / float(pick->desc.texture.height), h.proj);
        }
        // With a camera, a frame it couldn't pose is dropped: the viewer would take it as camera-relative
        // and start over. Without one, unposed frames are all there is (a live view).
        if (posed || active_camera() == nullptr) {
            g_capture->capture(queue, pick->resource, g_settings.color ? pick->color : resource{0},
                               uint32_t(g_settings.capture_width), h);
            g_snapshot_used = use_snapshot ? std::optional(snapshot) : std::nullopt;
        } else {
            g_capture->drop_snapshot();
        }
    }
    if (g_discovering) {
        watchdog::exchange_step("present: discovery");
        feed_discovery(samples, g_selected);
        discovery_tick();
    }
    ++g_frame;

    g_cpu_us = g_cpu_us * 0.95 + us_since(t0) * 0.05;
}

const char* format_name(format f) {
    switch (f) {
        case format::d16_unorm: return "D16";
        case format::r16_typeless: return "R16 typeless";
        case format::d24_unorm_s8_uint: return "D24S8";
        case format::r24_g8_typeless: return "R24G8 typeless";
        case format::d32_float: return "D32F";
        case format::r32_typeless: return "R32 typeless";
        case format::d32_float_s8_uint: return "D32FS8";
        case format::r32_g8_typeless: return "R32G8 typeless";
        case format::d24_unorm_x8_uint: return "D24X8";
        case format::intz: return "INTZ";
        default: return "other";
    }
}

const ImVec4 kOk(0.3f, 1, 0.4f, 1);
const ImVec4 kWarn(1, 0.8f, 0.2f, 1);
const ImVec4 kError(1, 0.3f, 0.3f, 1);

// A "(?)" after the last item, showing `text` on hover.
void help_marker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", text);
}

// FOV, aspect, near, far, depth direction and handedness, after `prefix`.
void projection_text(const char* prefix, const ProjectionInfo& p) {
    char far_text[32] = "infinite";
    if (!std::isinf(p.far_z)) std::snprintf(far_text, sizeof(far_text), "%.6g", p.far_z);
    ImGui::Text("%sFOV %.2f deg, aspect %.3f, near %.4g, far %s, %s, %s-handed", prefix, p.fov_y_deg, p.aspect,
                p.near_z, far_text, p.reversed ? "reversed depth" : "standard depth", p.right_handed ? "right" : "left");
}

void matrix_table(const char* id, const float m[16]) {
    if (!ImGui::BeginTable(id, 4, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit)) return;
    for (int r = 0; r < 4; ++r) {
        ImGui::TableNextRow();
        for (int c = 0; c < 4; ++c) {
            ImGui::TableNextColumn();
            ImGui::Text("%10.4f", m[r * 4 + c]);
        }
    }
    ImGui::EndTable();
}

bool draw_camera_section() {
    bool changed = false;
    if (!ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) return changed;

    char path[1024];
    strncpy_s(path, g_settings.profile.c_str(), _TRUNCATE);
    if (ImGui::InputText("Profile", path, sizeof(path), ImGuiInputTextFlags_EnterReturnsTrue)) {
        g_settings.profile = path;
        changed = true;
        reload_profile();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reload")) reload_profile();
    // Shown without a profile too: it's what tells whether the game's camera is in constants at all.
    const bool d3d9 = g_device->get_api() == device_api::d3d9;
    if (g_source)
        ImGui::TextDisabled(d3d9 ? "Tracking %zu constant registers" : "Tracking %zu constant buffers",
                            g_source->tracked_buffers());
    if (g_preview) {
        ImGui::TextColored(kWarn, "Previewing a discovery candidate (not saved)");
        ImGui::SameLine();
        if (ImGui::Button("End preview")) {
            g_preview.reset();
            configure_camera();
            g_ring.request_clear();
        }
    }
    if (!g_preview && !g_profile_error.empty()) {
        ImGui::TextColored(kError, "%s", g_profile_error.c_str());
        return changed;
    }
    // None after ending a preview with no profile to go back to.
    const CameraProfile* active = active_camera();
    if (active == nullptr) return changed;
    const CameraProfile& c = *active;
    // D3D9: one register file per stage, no buffers: offsets are registers (16 bytes each).
    auto at = [&](uint32_t offset) {
        return d3d9 ? "c" + std::to_string(offset / 16) + (offset % 16 ? " (unaligned)" : "") : std::to_string(offset);
    };
    std::string where = c.single_matrix() ? at(c.view_offset) : at(c.view_offset) + " / " + at(c.proj_offset);
    if (c.has_translation) where += (c.translation_subtract ? " - " : " + ") + at(c.translation_offset);
    if (d3d9)
        ImGui::TextDisabled("%s constants, %s at %s, %s-major, %s-handed, %s latch", stage_name(c.key.stage),
                            layout_name(c.layout), where.c_str(), c.column_major ? "column" : "row",
                            c.right_handed ? "right" : "left", latch_name(c.latch));
    else
        ImGui::TextDisabled("%s b%u%s%s, %s at %s, %s-major, %s-handed, %s latch", stage_name(c.key.stage), c.key.slot,
                            c.key.space ? (" space" + std::to_string(c.key.space)).c_str() : "",
                            c.key.size ? (" (" + std::to_string(c.key.size) + " bytes)").c_str() : "",
                            layout_name(c.layout), where.c_str(), c.column_major ? "column" : "row",
                            c.right_handed ? "right" : "left", latch_name(c.latch));

    // Scale: live, and written to the profile file unless a preview is in use (Save carries it over).
    float upm = c.units_per_meter;
    ImGui::SetNextItemWidth(120);
    if (ImGui::InputFloat("Game units per meter", &upm, 0, 0, "%.6g", ImGuiInputTextFlags_EnterReturnsTrue) &&
        upm > 0 && std::isfinite(upm)) {
        (g_preview ? *g_preview : g_profile.camera).units_per_meter = upm;
        if (!g_preview) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.9g", double(upm));
            write_profile_key(g_profile_path, "units_per_meter", buf);
        }
        g_ring.request_clear();  // the viewer's points are in the old units
    }
    help_marker("How many of the game's world units make one meter. The viewer assumes meters (voxel\n"
                "size, range, height colors). Changing it clears the viewer's points.");

    // Up axis: live too, like the scale.
    bool z_up = c.z_up;
    if (ImGui::Checkbox("World is Z-up", &z_up)) {
        (g_preview ? *g_preview : g_profile.camera).z_up = z_up;
        if (!g_preview) write_profile_key(g_profile_path, "up", z_up ? "\"z\"" : "\"y\"");
        g_ring.request_clear();
    }
    help_marker("The game's world has z up (IW, Unreal, Source), not y: the capture shows on its side\n"
                "without this. Discovery suggests it once the camera has turned around. Changing it\n"
                "clears the viewer's points.");

    if (ImGui::Button("Clear viewer points")) g_ring.request_clear();
    help_marker("Like C in the viewer, from here. Also done when the scale, the up axis or the previewed\n"
                "camera changes. Does nothing if the viewer isn't running.");

    switch (g_camera.state) {
        case CameraStatus::NoProfile: break;
        case CameraStatus::NoLatch:
            ImGui::TextColored(kWarn,
                               d3d9 ? "No latch: those registers were never set before a draw into the captured depth "
                                      "buffer (or slot isn't 0)."
                                    : "No latch: no draw into the captured depth buffer had that cbuffer bound.");
            break;
        case CameraStatus::Rejected:
        case CameraStatus::Latched: {
            // Over the last 64 frames, so a camera that's rejected now and then reads as one steady line.
            const int missed = std::popcount(g_camera.recent);
            const char* posed = c.model_view() ? "Pose solved" : "Pose latched";
            if (missed == 0)
                ImGui::TextColored(kOk, "%s", posed);
            else if (missed < 64)
                ImGui::TextColored(kWarn, "%s in %d of the last 64 frames (the others: %s)", posed,
                                   64 - missed, g_camera.why.c_str());
            else
                ImGui::TextColored(kError, "Latched, but rejected: %s", g_camera.why.c_str());
            break;
        }
    }
    if (c.model_view()) {
        ImGui::Text("Model-view: segment %u, %u draws, %u known objects, %u agree; %u static, %u provisional, "
                    "%u dynamic; %.0f us",
                    g_mv.segment, g_mv.draws, g_mv.known, g_mv.inliers, g_mv.statics, g_mv.provisional, g_mv.dynamic,
                    g_mv_us);
        if (g_mv.segment != 0 && g_mv.anchor_shared != 0)
            ImGui::TextDisabled("World frame: the game's (%u objects share its matrix)", g_mv.anchor_shared);
        else if (g_mv.segment != 0)
            ImGui::TextColored(kWarn, "World frame: one object's (may be tilted)");
    }
    ImGui::Text("Frames with pose %llu, without %llu", static_cast<unsigned long long>(g_camera.with_pose),
                static_cast<unsigned long long>(g_camera.without_pose));

    if (g_camera.with_pose > 0) {  // the last pose, kept through rejected frames
        const ProjectionInfo& p = g_camera.info;
        projection_text("From proj: ", p);
        if (p.right_handed != c.right_handed)
            ImGui::TextColored(kWarn, "Hint: the projection is %s-handed but the profile says %s.",
                               p.right_handed ? "right" : "left", c.right_handed ? "right" : "left");
        if (ImGui::TreeNode("Matrices")) {
            ImGui::TextUnformatted("view");
            matrix_table("view", g_camera.view);
            ImGui::TextUnformatted("proj");
            matrix_table("proj", g_camera.proj);
            ImGui::TreePop();
        }
    }
    return changed;
}

void draw_discovery_section() {
    if (!ImGui::CollapsingHeader("Discovery", g_profile.has_camera ? 0 : ImGuiTreeNodeFlags_DefaultOpen)) return;
    if (g_source == nullptr) {
        ImGui::TextDisabled("Not available for this graphics API.");
        return;
    }
    ImGui::TextWrapped("Finds the camera in the game's constants. Start it, then move and turn the camera for a few "
                       "seconds (stand still for a moment too): candidates are checked against how the depth image "
                       "moves. Use one to preview it in the viewer, then save it as the profile.");
    if (ImGui::Button(g_discovering ? "Stop" : "Start")) {
        if (g_discovering)
            stop_discovery();
        else
            start_discovery();
    }
    ImGui::SameLine();
    if (ImGui::Button("Write report")) {
        g_discovery.request_report(game_dir() / kDiscoveryReport);
        g_disc_message = g_discovering ? "Writing " + (game_dir() / kDiscoveryReport).string()
                                       : "Start discovery first: the report is written by it.";
    }
    ImGui::SameLine();
    const disc::RecordingStatus rec = g_discovery.recording();
    if (rec.active) {
        if (ImGui::Button("Stop recording")) g_discovery.stop_recording();
    } else if (ImGui::Button("Record")) {
        // From a fresh start, so the recording holds everything the analyzer saw.
        stop_discovery();
        start_discovery(kRecordSeconds);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Restarts discovery and records its inputs for %.0f s to a .disc file next to the game, for\n"
                          "lidar_discover to replay offline (discovery/README.md). Move and turn the camera meanwhile.",
                          kRecordSeconds);
    if (!rec.path.empty()) {
        if (!rec.error.empty())
            ImGui::TextColored(kWarn, "Recording failed: %s", rec.error.c_str());
        else
            ImGui::TextDisabled("%s %s: %.0f s, %.1f MB, %llu sample / %llu depth frames", rec.active ? "Recording" : "Recorded",
                                rec.path.filename().string().c_str(), rec.seconds, rec.bytes / 1048576.0,
                                (unsigned long long)rec.sample_frames, (unsigned long long)rec.depth_frames);
    }
    if (!g_disc_message.empty()) ImGui::TextDisabled("%s", g_disc_message.c_str());

    const disc::Status& s = g_disc_status;
    if (s.frames_analyzed == 0 && !g_discovering) return;
    ImGui::Text("%.0f s: %llu frames analyzed (%llu dropped, %.2f ms each). Last frame: %u draws, %u sampled, %u "
                "buffers",
                s.seconds, (unsigned long long)s.frames_analyzed, (unsigned long long)s.frames_dropped, s.analyze_ms,
                s.last_draws, s.last_samples, s.last_buffers);
    ImGui::Text("Matrices found: %zu rigid, %zu projection, %zu view-projection, %zu inverse; %zu translations",
                s.hypotheses[0], s.hypotheses[1], s.hypotheses[2], s.hypotheses[3], s.hypotheses[4]);
    ImGui::Text("Depth: %llu frames (%llu still, %llu moving), %llu reprojection rounds",
                (unsigned long long)s.depth_frames, (unsigned long long)s.still_frames,
                (unsigned long long)s.moving_frames, (unsigned long long)s.reproj_rounds);
    if (g_discovering && s.frames_analyzed == 0 && g_selected == 0)
        ImGui::TextColored(kWarn, "No depth buffer is captured, so no draws are sampled.");
    else if (g_discovering && s.depth_frames == 0 && s.seconds > 3)
        ImGui::TextColored(kWarn, "No depth frames: turn Capture on (candidates are validated with depth).");
    else if (g_discovering && s.reproj_rounds == 0 && s.seconds > 3)
        ImGui::TextColored(kWarn, "Move and turn the camera: validation needs the view to change.");
    if (s.candidates.empty()) {
        if (s.frames_analyzed > 0) ImGui::TextDisabled("No candidates yet.");
        return;
    }

    static int expanded = -1;
    if (ImGui::BeginTable("candidates", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("#");
        ImGui::TableSetupColumn("Layout");
        ImGui::TableSetupColumn("Where");
        ImGui::TableSetupColumn("Score");
        ImGui::TableSetupColumn("Reprojection");
        ImGui::TableSetupColumn("Still/moving");
        ImGui::TableSetupColumn("");
        ImGui::TableHeadersRow();
        for (int i = 0; i < int(s.candidates.size()); ++i) {
            const disc::CandidateInfo& c = s.candidates[size_t(i)];
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            // AllowOverlap: without it the row-wide selectable swallows clicks on the Use/Save buttons.
            if (ImGui::Selectable(std::to_string(i + 1).c_str(), expanded == i,
                                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
                expanded = expanded == i ? -1 : i;
            ImGui::TableNextColumn();
            ImGui::Text("%s%s%s", layout_name(c.profile.layout), c.profile.column_major ? " (column)" : "",
                        c.profile.latch == Latch::Common ? " (common)" : "");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(c.where.c_str());
            ImGui::TableNextColumn();
            if (c.confident)
                ImGui::TextColored(kOk, "%.2f", c.score);
            else
                ImGui::Text("%.2f", c.score);
            ImGui::TableNextColumn();
            if (c.reproj_tests)
                ImGui::Text("%u/%u, err %.2f%%", c.reproj_passes, c.reproj_tests, c.reproj_error * 100);
            else
                ImGui::TextDisabled("-");
            ImGui::TableNextColumn();
            ImGui::Text("%u/%u", c.temporal_agree, c.temporal_checks);
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("Use")) {
                // The scale and up axis in use carry over (discovery can only suggest z-up).
                const CameraProfile* now = active_camera();
                const float upm = now ? now->units_per_meter : 1;
                const bool z_up = c.profile.z_up || (now && now->z_up);
                g_preview = c.profile;
                g_preview->units_per_meter = upm;
                g_preview->z_up = z_up;
                configure_camera();
                g_ring.request_clear();
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Save")) save_candidate(c);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (expanded >= 0 && expanded < int(s.candidates.size())) {
        const disc::CandidateInfo& c = s.candidates[size_t(expanded)];
        ImGui::Text("#%d, live values:", expanded + 1);
        if (!c.have_values) {
            ImGui::TextDisabled("(no values in the latest frame)");
        } else {
            if (c.info.valid) projection_text("", c.info);
            ImGui::TextUnformatted("view");
            matrix_table("cand_view", c.view);
            ImGui::TextUnformatted("proj");
            matrix_table("cand_proj", c.proj);
        }
    }
}

// A multisampled depth-stencil drawing more than the captured one: the scene is likely there, and
// neither capture nor discovery can use it.
bool msaa_hides_scene() {
    uint32_t captured = 0;
    for (const auto& c : g_candidates)
        if (c.resource.handle == g_selected) captured = c.stats.drawcalls;
    for (const auto& c : g_candidates)
        if (c.desc.texture.samples > 1 && c.stats.drawcalls > captured) return true;
    return false;
}

void draw_overlay(effect_runtime*) {
    const watchdog::Step step("overlay");
    const std::lock_guard lock(g_mutex);
    if (g_device == nullptr) {
        ImGui::TextColored(kWarn, "No supported device: %s",
                           g_init_error.empty() ? "none created yet" : g_init_error.c_str());
        return;
    }
    if (!g_init_error.empty()) {
        ImGui::TextColored(kError, "%s", g_init_error.c_str());
        return;
    }

    bool changed = ImGui::Checkbox("Capture", &g_settings.enabled);
    ImGui::SameLine();
    ImGui::TextDisabled("(%.1f fps, addon CPU %.0f us/frame)", ImGui::GetIO().Framerate, g_cpu_us);
    if (!g_capture->error().empty()) ImGui::TextColored(kError, "%s", g_capture->error().c_str());
    if (msaa_hides_scene())
        ImGui::TextColored(kError,
                           "The game renders with MSAA, which can't be captured: turn anti-aliasing (MSAA) off in the\n"
                           "game's video settings. Until then depth and discovery only see a secondary buffer.");
    ImGui::Text("Published %llu frames at %ux%u, skipped %llu (GPU readback busy)",
                static_cast<unsigned long long>(g_capture->published()), g_capture->width(), g_capture->height(),
                static_cast<unsigned long long>(g_capture->skipped()));
    changed |= ImGui::SliderInt("Capture width", &g_settings.capture_width, 64, int(kMaxWidth));
    changed |= ImGui::Checkbox("Color", &g_settings.color);
    ImGui::SameLine();
    if (!g_settings.color)
        ImGui::TextDisabled("(off: the viewer colors by height)");
    else if (g_capture->color_note().empty())
        ImGui::TextDisabled("(the scene's render target, sampled with depth)");
    else
        ImGui::TextColored(kWarn, "none: %s", g_capture->color_note().c_str());
    if (g_settings.color && ImGui::TreeNode("Color crop (HUD)")) {
        ImGui::TextDisabled("Percent of the screen from each edge whose color is dropped (depth is kept).\n"
                            "The outline on screen is the area that keeps its color.");
        static const char* const kLabels[4] = {"Left", "Top", "Right", "Bottom"};
        for (int i = 0; i < 4; ++i)
            changed |= ImGui::SliderFloat(kLabels[i], &g_settings.color_crop[i], 0.0f, 45.0f, "%.0f%%");
        if (ImGui::Button("Reset crop")) {
            std::fill_n(g_settings.color_crop, 4, 0.0f);
            changed = true;
        }
        const ImVec2 size = ImGui::GetIO().DisplaySize;
        const float* c = g_settings.color_crop;
        ImGui::GetForegroundDrawList()->AddRect(ImVec2(size.x * c[0] / 100, size.y * c[1] / 100),
                                                ImVec2(size.x * (1 - c[2] / 100), size.y * (1 - c[3] / 100)),
                                                IM_COL32(255, 200, 40, 255), 0.0f, 0, 3.0f);
        ImGui::TreePop();
    }

    changed |= draw_camera_section();
    draw_discovery_section();

    if (ImGui::CollapsingHeader("Fallback projection (frames without a pose)")) {
        changed |= ImGui::SliderFloat("Vertical FOV", &g_settings.fov_y_deg, 20.0f, 120.0f, "%.1f deg");
        changed |= ImGui::InputFloat("Near", &g_settings.near_z, 0.01f, 0.1f, "%.3f");
        changed |= ImGui::InputFloat("Far", &g_settings.far_z, 10.0f, 100.0f, "%.1f");
        changed |= ImGui::Combo("Depth mode", &g_settings.depth_mode, "Standard\0Reversed\0Reversed infinite\0");
        for (const auto& c : g_candidates)
            if (c.resource.handle == g_selected) {
                const bool reversed = g_settings.depth_mode != int(DepthMode::Standard);
                if (c.reversed_clear != reversed)
                    ImGui::TextColored(kWarn, "Hint: the game clears depth to %s, which suggests %s.",
                                       c.reversed_clear ? "0" : "1", c.reversed_clear ? "reversed" : "standard");
            }
    }
    g_settings.near_z = std::max(g_settings.near_z, 1e-4f);
    g_settings.far_z = std::max(g_settings.far_z, g_settings.near_z * 2);

    if (ImGui::CollapsingHeader("Depth buffer", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::RadioButton("Automatic", g_override == 0)) g_override = 0;
        if (ImGui::BeginTable("ds", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Use");
            ImGui::TableSetupColumn("Size");
            ImGui::TableSetupColumn("Format");
            ImGui::TableSetupColumn("Draws");
            ImGui::TableSetupColumn("Vertices");
            ImGui::TableHeadersRow();
            for (const auto& c : g_candidates) {
                ImGui::PushID(reinterpret_cast<void*>(c.resource.handle));
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::RadioButton("##use", g_override == c.resource.handle)) g_override = c.resource.handle;
                if (c.resource.handle == g_selected) {
                    ImGui::SameLine();
                    ImGui::TextColored(kOk, "captured");
                }
                ImGui::TableNextColumn();
                ImGui::Text("%ux%u%s", c.desc.texture.width, c.desc.texture.height,
                            c.desc.texture.samples > 1 ? " MSAA" : (c.fits_frame ? "" : " (off-size)"));
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(format_name(c.desc.texture.format));
                ImGui::TableNextColumn();
                ImGui::Text("%u", c.stats.drawcalls);
                ImGui::TableNextColumn();
                ImGui::Text("%u", c.stats.vertices);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        if (g_snapshot_used)
            ImGui::TextDisabled("Captured before a clear: that pass drew %u draws (the depth buffer is reused later "
                                "in the frame).",
                                g_snapshot_used->pass.drawcalls);
    }
    if (changed) g_settings.save();
}

void register_active_handlers() {
    reshade::register_event<reshade::addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<reshade::addon_event::present>(on_present);
    reshade::register_overlay("LiDAR", draw_overlay);
}

}  // namespace

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID) {
    switch (reason) {
        case DLL_PROCESS_ATTACH:
            if (!reshade::register_addon(module)) return FALSE;
            // Everything else registers in attach_modules(), for supported devices only.
            reshade::register_event<reshade::addon_event::init_device>(on_init_device);
            break;
        case DLL_PROCESS_DETACH:
            reshade::unregister_addon(module);
            break;
    }
    return TRUE;
}
