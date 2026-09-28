// lidar_viewer: live point-cloud viewer. Reads depth frames + camera matrices
// from the shared-memory ring, unprojects them on the GPU into a voxel-deduped
// point pool, and renders it with a free-fly camera. Points that a newer frame
// sees straight through (things that moved away) are carved out.
//
// Usage: lidar_viewer [--voxel 0.1] [--capacity-m 50] [--table-bits auto]
//                     [--near-cut 0.3] [--max-range 500] [--height-range -1 20]
//                     [--no-carve] [--carve-margin 0.15] [--carve-rel 0.02]
//                     [--color-update first|closest|latest]
//                     [--size 1600x900] [--out file.ply] [--save-after s] [--exit-after s]
// Keys:  right-drag look, WASD move, Q/E down/up, Shift fast, wheel speed,
//        F follow player (right-drag orbits it, wheel zooms, R resets), V attach to the player's camera, H color mode, T trail,
//        M carving, U hide above the player, O hide between you and the player,
//        +/- point size, arrows tilt the view, L level it, C clear,
//        P save .ply, Space pause ingest, F1 settings panel, Esc unfocus.
// The settings panel changes voxel size, capacity, range, carving and colors while it runs.
//
// The pieces: point_cloud.h (the GPU point pool and what goes into it), camera.h (the game's
// camera, the viewer's, and the display tilt), cutaway.h (hiding what's in front of the player),
// renderer.h (drawing it all). This file reads the ring, handles input and the settings panel,
// and wires them together.
#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

#include "app.h"
#include "camera.h"
#include "cutaway.h"
#include "gpu.h"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "point_cloud.h"
#include "renderer.h"
#include "ring.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);

using namespace DirectX;
using namespace lidar;

namespace {

// Feeds the settings panel; keeps what it's using (typing, clicks, the wheel) from the viewer's
// own controls. Key and button releases always go through so nothing sticks.
bool ui_message(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp, LRESULT& result) {
    if ((result = ImGui_ImplWin32_WndProcHandler(hwnd, msg, wp, lp)) != 0) return true;
    const ImGuiIO& io = ImGui::GetIO();
    switch (msg) {
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN:
        case WM_CHAR: return io.WantCaptureKeyboard;
        case WM_LBUTTONDOWN:
        case WM_RBUTTONDOWN:
        case WM_MOUSEWHEEL: return io.WantCaptureMouse;
    }
    return false;
}

struct Options {
    CaptureSettings capture;
    uint32_t capacity = 50u << 20;
    uint32_t table_bits = 0;  // 0: sized for the capacity
    float height_min = -1.0f, height_max = 20.0f;
    int width = 1600, height = 900;
    std::string out;
    float save_after = 0, exit_after = 0;
};

Options parse(int argc, char** argv) {
    Options o;
    CaptureSettings& c = o.capture;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (a == "--voxel") c.voxel = float(std::atof(next()));
        else if (a == "--capacity-m") o.capacity = uint32_t(std::atof(next()) * (1 << 20));
        else if (a == "--table-bits") o.table_bits = uint32_t(std::clamp(std::atoi(next()), 16, 28));
        else if (a == "--near-cut") c.near_cut = float(std::atof(next()));
        else if (a == "--max-range") c.max_range = float(std::atof(next()));
        else if (a == "--height-range") {
            o.height_min = float(std::atof(next()));
            o.height_max = float(std::atof(next()));
        } else if (a == "--size") std::sscanf(next(), "%dx%d", &o.width, &o.height);
        else if (a == "--out") o.out = next();
        else if (a == "--save-after") o.save_after = float(std::atof(next()));
        else if (a == "--exit-after") o.exit_after = float(std::atof(next()));
        else if (a == "--no-carve") c.carve = false;
        else if (a == "--carve-margin") c.carve_margin = float(std::atof(next()));
        else if (a == "--carve-rel") c.carve_rel = float(std::atof(next()));
        else if (a == "--color-update") {
            const std::string m = next();
            c.color_update = m == "first" ? 0 : m == "latest" ? 2 : 1;
        }
        else std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
    }
    return o;
}

std::string default_scan_path() {
    std::time_t t = std::time(nullptr);
    std::tm tm;
    localtime_s(&tm, &t);
    char buf[64];
    std::strftime(buf, sizeof(buf), "scans/scan_%Y%m%d_%H%M%S.ply", &tm);
    return buf;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt = parse(argc, argv);  // the panel edits it live
    CaptureSettings& capture = opt.capture;
    App app;
    if (!app.create(L"lidar_viewer", opt.width, opt.height)) return 1;
    ID3D11Device* dev = app.dev.Get();
    ID3D11DeviceContext* ctx = app.ctx.Get();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;  // nothing to remember between runs
    ImGui::StyleColorsDark();
    {
        const float dpi = ImGui_ImplWin32_GetDpiScaleForHwnd(app.hwnd);
        ImGui::GetStyle().ScaleAllSizes(dpi);
        ImGui::GetStyle().FontScaleDpi = dpi;
    }
    ImGui_ImplWin32_Init(app.hwnd);
    ImGui_ImplDX11_Init(dev, ctx);
    app.message_hook = ui_message;

    PointCloud cloud;
    if (!cloud.create(dev, ctx, opt.capacity, opt.table_bits != 0 ? opt.table_bits : table_bits_for(opt.capacity))) {
        std::fprintf(stderr, "can't allocate a %u-point pool: try a smaller --capacity-m\n", opt.capacity);
        return 1;
    }
    Renderer renderer;
    renderer.create(dev, ctx, app.width, app.height);

    // Marks: 0 capture (carve + ingest) 1 ... 2 point draw 3.
    GpuTimer gpu_timer;
    gpu_timer.create(dev);

    RingReader ring;
    Frame frame;
    PlayerCamera player;
    ViewerCamera camera;
    ViewTilt tilt;
    CutawaySettings cutaways;
    PointStyle style;
    style.height_min = opt.height_min;
    style.height_max = opt.height_max;

    bool show_trail = true, show_player_cam = true, paused = false, saved = false;
    // Settings panel (F1). Voxel size and capacity only apply with the button: both rebuild the pool.
    bool show_ui = true, auto_height = true;
    float ui_voxel = capture.voxel, ui_capacity_m = float(opt.capacity) / float(1 << 20);
    std::string pool_message;

    auto clear_all = [&]() {
        cloud.clear();
        player.trail.clear();
    };
    // Rebuilds the pool with the panel's voxel size and capacity. If the GPU can't allocate that
    // much, the old size comes back.
    auto apply_pool = [&]() {
        if (cloud.resize(uint32_t(double(ui_capacity_m) * (1 << 20)))) {
            pool_message.clear();
        } else {
            pool_message = "Couldn't allocate " + std::to_string(int(ui_capacity_m)) + "M points: kept the old size.";
            ui_capacity_m = float(cloud.capacity()) / float(1 << 20);
        }
        capture.voxel = ui_voxel;
        opt.capacity = cloud.capacity();
        player.trail.clear();
    };
    auto save = [&]() { cloud.save_ply(opt.out.empty() ? default_scan_path() : opt.out); };

    LARGE_INTEGER qpf, t0, now;
    QueryPerformanceFrequency(&qpf);
    QueryPerformanceCounter(&t0);
    double last = 0, title_timer = 0, reconnect_timer = 0;
    double last_posed = -1;  // when the last posed frame came in
    uint64_t ingested = 0;
    uint64_t seen_clear = 0;  // the producer's last clear request handled
    int fps_frames = 0;
    double fps = 0;

    while (app.pump()) {
        if (app.resized) renderer.resize(app.width, app.height);
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - t0.QuadPart) / double(qpf.QuadPart);
        const float dt = float(std::min(t - last, 0.1));
        last = t;

        gpu_timer.begin(ctx);
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // Input.
        if (app.key_pressed(VK_F1)) show_ui = !show_ui;
        if (app.key_pressed('F')) camera.toggle_follow();
        if (app.key_pressed('V')) camera.toggle_attach();
        if (app.key_pressed('H')) style.color_mode ^= 1;
        if (app.key_pressed('T')) show_trail = !show_trail;
        if (app.key_pressed('M')) capture.carve = !capture.carve;
        if (app.key_pressed('U')) cutaways.plane_on = !cutaways.plane_on;
        if (app.key_pressed('O')) cutaways.sight_on = !cutaways.sight_on;
        if (app.key_pressed(VK_SPACE)) paused = !paused;
        if (app.key_pressed(VK_OEM_PLUS) || app.key_pressed(VK_ADD))
            style.point_size = std::min(style.point_size + 1, 16.0f);
        if (app.key_pressed(VK_OEM_MINUS) || app.key_pressed(VK_SUBTRACT))
            style.point_size = std::max(style.point_size - 1, 1.0f);
        if (app.key_pressed('C')) clear_all();
        if (app.key_pressed('P')) save();
        if (opt.save_after > 0 && !saved && t > opt.save_after) {
            save();
            saved = true;
        }
        if (opt.exit_after > 0 && t > opt.exit_after) break;

        // Ingest new frames.
        if (!ring.is_open()) {
            reconnect_timer -= dt;
            if (reconnect_timer <= 0) {
                ring.try_open();
                reconnect_timer = 0.5;
            }
        }
        // The producer asked for a clear (its pose's frame changed). A restarted producer starts at 0:
        // that keeps the points.
        if (const uint64_t cs = ring.clear_seq(); cs != seen_clear) {
            if (cs != 0) clear_all();
            seen_clear = cs;
        }
        bool carved = false;  // see PointCloud::ingest
        gpu_timer.mark(ctx, 0);
        for (uint32_t i = 0; i < kSlotCount && ring.is_open() && ring.read_next(frame); ++i) {
            const FrameHeader& h = frame.header;
            if (h.flags & kFlagPaused) continue;
            if (frame.seq <= seen_clear) continue;  // posed before the clear
            // Without a pose (addon before M2) the frame is camera-relative: show it as a live
            // snapshot at the origin instead of accumulating it.
            const bool posed = (h.flags & kFlagPoseValid) != 0;
            // An unposed frame among posed ones is a camera that failed for a moment: skip it rather
            // than start over.
            if (posed) last_posed = t;
            else if (last_posed >= 0 && t - last_posed < 2.0) continue;
            const XMMATRIX view =
                posed ? XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(h.view)) : XMMatrixIdentity();
            const XMMATRIX proj = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(h.proj));
            player.observe(view, proj, posed);
            if (paused) continue;
            if (!posed) player.trail.clear();
            cloud.ingest(frame, view, proj, capture, !posed, carved);
            ++ingested;
        }
        gpu_timer.mark(ctx, 1);
        cloud.poll_stats();

        // Settings panel.
        const HeightStats& hs = cloud.heights();
        if (show_ui) {
            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::Begin("lidar_viewer (F1 hides)", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("%s, %llu frames (%llu dropped)", ring.is_open() ? "Connected" : "Waiting for the game",
                        (unsigned long long)ingested, (unsigned long long)ring.dropped());
            char overlay[64];
            std::snprintf(overlay, sizeof(overlay), "%.2fM / %.1fM points%s", cloud.live() / 1e6,
                          cloud.capacity() / 1e6, cloud.full() ? " (FULL)" : "");
            ImGui::ProgressBar(float(cloud.used()) / float(cloud.capacity()), ImVec2(-FLT_MIN, 0), overlay);
            ImGui::TextDisabled("%.0f fps, GPU: capture %.2f ms, points %.2f ms", fps, gpu_timer.ms[0],
                                gpu_timer.ms[2]);
            if (ImGui::Button("Clear points (C)")) clear_all();
            ImGui::SameLine();
            if (ImGui::Button("Save .ply (P)")) save();
            ImGui::SameLine();
            ImGui::Checkbox("Pause (Space)", &paused);

            ImGui::SeparatorText("Point pool");
            ImGui::SetNextItemWidth(160);
            ImGui::InputFloat("Voxel size (m)", &ui_voxel, 0.01f, 0.1f, "%.3f");
            ui_voxel = std::clamp(ui_voxel, 0.001f, 100.0f);
            ImGui::SetNextItemWidth(160);
            ImGui::InputFloat("Capacity (M points)", &ui_capacity_m, 4, 16, "%.0f");
            ui_capacity_m = std::clamp(std::round(ui_capacity_m), 1.0f, 128.0f);
            const uint32_t ui_capacity = uint32_t(double(ui_capacity_m) * (1 << 20));
            ImGui::TextDisabled("%.0f MB of GPU memory", pool_bytes(ui_capacity) / (1 << 20));
            const bool pool_changed = ui_voxel != capture.voxel || ui_capacity != cloud.capacity();
            ImGui::BeginDisabled(!pool_changed);
            if (ImGui::Button("Apply (clears points)")) apply_pool();
            ImGui::SameLine();
            if (ImGui::Button("Revert")) {
                ui_voxel = capture.voxel;
                ui_capacity_m = float(cloud.capacity()) / float(1 << 20);
            }
            ImGui::EndDisabled();
            if (!pool_message.empty()) ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "%s", pool_message.c_str());

            ImGui::SeparatorText("Capture");
            ImGui::SetNextItemWidth(160);
            ImGui::SliderFloat("Max range (m)", &capture.max_range, 10, 20000, "%.0f", ImGuiSliderFlags_Logarithmic);
            ImGui::SetNextItemWidth(160);
            ImGui::SliderFloat("Near cut (m)", &capture.near_cut, 0, 10, "%.2f");
            ImGui::Checkbox("Carve out moved things (M)", &capture.carve);
            if (capture.carve) {
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Carve margin (m)", &capture.carve_margin, 0.01f, 5, "%.2f",
                                   ImGuiSliderFlags_Logarithmic);
            }

            ImGui::SeparatorText("Color (H)");
            int mode = int(style.color_mode);
            ImGui::RadioButton("Height", &mode, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Captured color", &mode, 1);
            style.color_mode = uint32_t(mode);
            int update = int(capture.color_update);
            ImGui::TextUnformatted("Keep the color");
            ImGui::SameLine();
            ImGui::RadioButton("first seen", &update, 0);
            ImGui::SameLine();
            ImGui::RadioButton("seen closest", &update, 1);
            ImGui::SameLine();
            ImGui::RadioButton("seen last", &update, 2);
            capture.color_update = uint32_t(update);
            ImGui::Checkbox("Height range from the cloud", &auto_height);
            if (auto_height) {
                if (hs.valid)
                    ImGui::TextDisabled("coloring %.1f to %.1f m (lowest %.1f, highest %.1f)", hs.range_low,
                                        hs.range_high, hs.low, hs.high);
                else
                    ImGui::TextDisabled("(no points yet)");
            } else {
                ImGui::SetNextItemWidth(220);
                ImGui::DragFloatRange2("Height range (m)", &opt.height_min, &opt.height_max, 0.5f, -100000, 100000,
                                       "%.1f", "%.1f");
                if (hs.valid && ImGui::Button("Set to the cloud's")) {
                    opt.height_min = hs.range_low;
                    opt.height_max = hs.range_high;
                }
            }

            ImGui::SeparatorText("View");
            ImGui::SetNextItemWidth(160);
            ImGui::Checkbox("Scale points with distance", &style.world_points);
            ImGui::SetNextItemWidth(160);
            if (style.world_points) {
                ImGui::SliderFloat("Point size (x voxel)", &style.world_scale, 0.25f, 4.0f, "%.2f");
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Max point size (px)", &style.world_max_px, 2, 256, "%.0f",
                                   ImGuiSliderFlags_Logarithmic);
            } else {
                ImGui::SliderFloat("Point size (+/-)", &style.point_size, 1, 16, "%.0f");
            }
            ImGui::Checkbox("Follow the player (F)", &camera.follow);
            ImGui::SameLine();
            ImGui::Checkbox("Trail (T)", &show_trail);
            ImGui::SameLine();
            ImGui::Checkbox("Show game camera", &show_player_cam);
            ImGui::SameLine();
            if (ImGui::Checkbox("Attach to camera (V)", &camera.attach) && camera.attach) camera.follow = false;
            if (camera.follow) camera.attach = false;
            if (camera.follow)
                ImGui::TextDisabled("Right-drag orbit the player, wheel zoom (%.1f m), R reset", camera.follow_dist);
            else
                ImGui::TextDisabled("Right-drag look, WASD/arrows move, Q/E down/up, Shift fast, wheel speed");
            ImGui::TextDisabled("Z/X roll, PgUp/PgDn tilt the view, L levels it (display only)");

            ImGui::SeparatorText("Clear the view (display only)");
            ImGui::Checkbox("Hide above the player (U)", &cutaways.plane_on);
            if (cutaways.plane_on) {
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Offset above camera (m)", &cutaways.plane_offset, -5, 20, "%.2f");
            }
            ImGui::Checkbox("Hide between you and the player (O)", &cutaways.sight_on);
            if (cutaways.sight_on) {
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Radius (m)", &cutaways.sight_radius, 0.1f, 20, "%.2f",
                                   ImGuiSliderFlags_Logarithmic);
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Offset toward you (m)", &cutaways.sight_offset, -5, 20, "%.2f");
                if (camera.attach) ImGui::TextDisabled("(off while attached to the camera)");
            }
            if (cutaways.plane_on || cutaways.sight_on) {  // the orientation of both bases
                int up = cutaways.camera_up ? 1 : 0;
                ImGui::TextUnformatted("Base");
                ImGui::SameLine();
                ImGui::RadioButton("level", &up, 0);
                ImGui::SameLine();
                ImGui::RadioButton("follows camera's up", &up, 1);
                cutaways.camera_up = up == 1;
            }
            ImGui::End();
        }

        // View: the tilt pivots about where the player was when it left level (or the viewer, before
        // any frame), then the camera moves.
        tilt.update(app, dt,
                    player.valid ? player.position() : XMFLOAT3{camera.pos[0], camera.pos[1], camera.pos[2]});
        const XMMATRIX tilt_m = tilt.matrix();
        const ViewParams view = camera.update(app, dt, player, tilt_m);

        style.height_min = auto_height && hs.valid ? hs.range_low : opt.height_min;
        style.height_max = auto_height && hs.valid ? hs.range_high : opt.height_max;
        DrawParams draw = make_draw_params(view, tilt, style, capture.voxel, {});
        draw.cut = make_cutaways(cutaways, player, camera, tilt_m, draw.height_axis);

        // Auto color range, for the next frames.
        cloud.update_heights(dt, draw.height_axis);

        std::vector<LineVertex> lines;
        player_lines(player, show_player_cam && !camera.attach,  // attached, it would just frame the screen
                     show_trail, lines);

        // Render.
        renderer.begin(app.back_rtv.Get(), draw, cloud.capacity());
        gpu_timer.mark(ctx, 2);
        renderer.draw_points(cloud);
        gpu_timer.mark(ctx, 3);
        renderer.draw_lines(lines);
        ImGui::Render();
        ctx->OMSetRenderTargets(1, app.back_rtv.GetAddressOf(), nullptr);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        gpu_timer.end(ctx);
        app.present(true);

        ++fps_frames;
        title_timer += dt;
        if (title_timer > 0.5) {
            fps = fps_frames / title_timer;
            fps_frames = 0;
            title_timer = 0;
            wchar_t buf[256];
            swprintf(buf, 256,
                     L"lidar_viewer  %.0f fps  |  %s  |  frames %llu (dropped %llu)  |  points %.2fM / %.1fM%s  |  "
                     L"carving %s%s",
                     fps, ring.is_open() ? L"connected" : L"waiting for producer", ingested, ring.dropped(),
                     cloud.live() / 1e6, cloud.capacity() / 1e6, cloud.full() ? L" FULL" : L"",
                     capture.carve ? L"on" : L"off", paused ? L"  PAUSED" : L"");
            app.set_title(buf);
        }
    }
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    return 0;
}
