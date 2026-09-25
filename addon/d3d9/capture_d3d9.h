// D3D9 depth capture. D3D9 depth-stencils can't be copied or sampled, except in the INTZ format
// (a depth format drivers let shaders read), so the addon has ReShade create the game's
// screen-sized depth-stencils as INTZ textures (create_resource), like ReShade's generic_depth.
// At capture, a ps_3_0 pass point-samples the INTZ texture into a small R32F render target, and
// once an event query says the GPU is done with it, GetRenderTargetData + LockRect read it back.
// Nothing here waits on the GPU: LockRect of a surface with a copy in flight hung Sonic Adventure 2
// for good while it loaded a level (D3DLOCK_DONOTWAIT didn't help: NVIDIA's driver blocks anyway),
// so a second event query follows the copy, and the lock only happens once that one has finished.
//
// Native calls on the unwrapped device fire no addon events. The game's state is captured in a
// state block and restored (render targets and viewport by hand, which state blocks don't cover).
// Everything in D3DPOOL_DEFAULT goes before a device Reset (destroy_command_queue) and is
// recreated at the next capture.
#pragma once
#include <d3d9.h>
#include <wrl/client.h>

#include <deque>

#include "depth_capture.h"

namespace lidar {

class D3D9Capture final : public DepthCapture {
public:
    ~D3D9Capture() override;
    bool init(reshade::api::device* dev);
    void release();

    bool capture(reshade::api::command_queue* queue, reshade::api::resource depth, uint32_t capture_width,
                 const FrameHeader& header) override;
    void publish(reshade::api::command_queue* queue, RingWriter& ring) override;
    bool snapshot(reshade::api::resource depth, uint32_t capture_width) override;
    void drop_snapshot() override { staged_ = -1; }

    // INTZ replacement of depth-stencils, and releasing GPU resources before a Reset.
    static void register_events();
    static void unregister_events();
    static void init_device(reshade::api::device* dev);  // idempotent, ignores other APIs

private:
    template <class T>
    using ComPtr = Microsoft::WRL::ComPtr<T>;
    static constexpr int kReadback = 3;

    bool ensure_targets(uint32_t src_w, uint32_t src_h, uint32_t capture_width);
    // Samples `depth` down into slot next_slot_ (without claiming it). False, with error_, if it can't.
    bool downsample(reshade::api::resource depth, uint32_t capture_width, D3DSURFACE_DESC& src);
    void release_targets();
    static void on_destroy_command_queue(reshade::api::command_queue* q);

    reshade::api::device* rdev_ = nullptr;
    ComPtr<IDirect3DDevice9> dev_;
    ComPtr<IDirect3DVertexShader9> vs_;
    ComPtr<IDirect3DPixelShader9> ps_;
    ComPtr<IDirect3DVertexDeclaration9> decl_;
    DWORD num_rts_ = 1;
    DWORD vertex_processing_ = 0;  // creation flags, for mixed vertex processing

    // D3DPOOL_DEFAULT: per readback slot, the capture render target, a system-memory copy, and event
    // queries behind the capture draw and behind the copy.
    uint32_t src_w_ = 0, src_h_ = 0;
    ComPtr<IDirect3DSurface9> rt_[kReadback];
    ComPtr<IDirect3DSurface9> sys_[kReadback];
    ComPtr<IDirect3DQuery9> done_[kReadback];
    ComPtr<IDirect3DQuery9> copied_[kReadback];

    // Presents a copy may stay unfinished before its capture is dropped.
    static constexpr int kMaxCopyWaits = 30;

    struct Pending {
        int slot;
        FrameHeader header;
        bool copied = false;  // GetRenderTargetData issued into sys_[slot], copied_[slot] behind it
        int copy_waits = 0;   // presents copied_[slot] was found still running
    };
    std::deque<Pending> pending_;
    int next_slot_ = 0;
    int staged_ = -1;  // next_slot_ holds a snapshot taken before a clear this frame
    uint32_t staged_w_ = 0, staged_h_ = 0;  // its source size
};

}  // namespace lidar
