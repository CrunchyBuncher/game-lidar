// The fake game's D3D12 renderer, laid out the way D3D12 games usually handle per-frame
// constants, so the addon's D3D12 sniffer has something realistic to find:
//  * One persistently mapped upload buffer, never unmapped. Frame f % kFrames writes its camera
//    and per-draw object constants into its own region, after waiting for that region's fence,
//    so up to kFrames frames are in flight and the memory changes without any API event.
//  * Root CBVs (default): root parameter 0 = b1 (object), 1 = b0 (camera), so the parameter
//    index isn't the register. Root signature version 1.1.
//  * --cbv-tables: one descriptor table [b1 at offset 0, b0 appended]. The CBVs are created once
//    in a CPU-only heap and copied into a shader-visible ring every frame, as engines do.
//    Root signature version 1.0.
//  * The depth buffer ends each frame in PIXEL_SHADER_RESOURCE (as if post-processing read it),
//    so a capture can't assume it's still in DEPTH_WRITE at present.
#include <d3d12.h>
#include <dxgi1_4.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "fake_game.h"

using namespace DirectX;

namespace lidar::fake {
namespace {

constexpr UINT kFrames = 3;  // frames in flight, and swap chain buffers
constexpr UINT kDraws = 2;   // static level + NPC
constexpr UINT kCbAlign = D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT;  // 256
// Per frame region: camera, then one object block per draw.
constexpr UINT kFrameStride = kCbAlign * (1 + kDraws);
constexpr DXGI_FORMAT kColorFormat = DXGI_FORMAT_R8G8B8A8_UNORM;

UINT cb_size(size_t n) { return UINT((n + kCbAlign - 1) & ~size_t(kCbAlign - 1)); }

D3D12_RESOURCE_BARRIER transition(ID3D12Resource* r, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
    D3D12_RESOURCE_BARRIER b{D3D12_RESOURCE_BARRIER_TYPE_TRANSITION};
    b.Transition.pResource = r;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    b.Transition.StateBefore = before;
    b.Transition.StateAfter = after;
    return b;
}

ComPtr<ID3D12Resource> make_buffer(ID3D12Device* dev, UINT64 size, D3D12_HEAP_TYPE heap,
                                   D3D12_RESOURCE_STATES state) {
    D3D12_HEAP_PROPERTIES hp{heap};
    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    d.Width = size;
    d.Height = d.DepthOrArraySize = d.MipLevels = 1;
    d.SampleDesc.Count = 1;
    d.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    ComPtr<ID3D12Resource> r;
    check(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, state, nullptr, IID_PPV_ARGS(&r)), "buffer");
    return r;
}

ComPtr<ID3D12DescriptorHeap> make_heap(ID3D12Device* dev, D3D12_DESCRIPTOR_HEAP_TYPE type, UINT count,
                                       bool shader_visible) {
    D3D12_DESCRIPTOR_HEAP_DESC d{type, count,
                                 shader_visible ? D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE : D3D12_DESCRIPTOR_HEAP_FLAG_NONE};
    ComPtr<ID3D12DescriptorHeap> h;
    check(dev->CreateDescriptorHeap(&d, IID_PPV_ARGS(&h)), "descriptor heap");
    return h;
}

ComPtr<ID3D12RootSignature> make_root_signature(ID3D12Device* dev, bool tables) {
    ComPtr<ID3DBlob> blob, errors;
    HRESULT hr;
    if (tables) {
        const D3D12_DESCRIPTOR_RANGE ranges[2] = {
            {D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 1, 0, 0},                                     // b1 at offset 0
            {D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0, 0, D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND},  // b0 at offset 1
        };
        D3D12_ROOT_PARAMETER p{D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE};
        p.DescriptorTable = {2, ranges};
        p.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
        const D3D12_ROOT_SIGNATURE_DESC d{1, &p, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT};
        hr = D3D12SerializeRootSignature(&d, D3D_ROOT_SIGNATURE_VERSION_1_0, &blob, &errors);
    } else {
        D3D12_ROOT_PARAMETER1 p[2] = {};
        p[0].ParameterType = p[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        p[0].Descriptor = {1, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE};  // b1: object
        p[1].Descriptor = {0, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE};  // b0: camera
        p[0].ShaderVisibility = p[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
        D3D12_VERSIONED_ROOT_SIGNATURE_DESC d{D3D_ROOT_SIGNATURE_VERSION_1_1};
        d.Desc_1_1 = {2, p, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT};
        hr = D3D12SerializeVersionedRootSignature(&d, &blob, &errors);
    }
    if (FAILED(hr)) {
        std::fprintf(stderr, "root signature: %s\n", errors ? static_cast<const char*>(errors->GetBufferPointer()) : "?");
        std::exit(1);
    }
    ComPtr<ID3D12RootSignature> rs;
    check(dev->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&rs)),
          "CreateRootSignature");
    return rs;
}

// Prints (and clears) what the debug layer found since the last call. Returns false on errors.
bool drain_debug_messages(ID3D12InfoQueue* q) {
    if (q == nullptr) return true;
    bool ok = true;
    const UINT64 n = q->GetNumStoredMessages();
    for (UINT64 i = 0; i < n; ++i) {
        SIZE_T size = 0;
        q->GetMessage(i, nullptr, &size);
        std::vector<char> buf(size);
        auto* m = reinterpret_cast<D3D12_MESSAGE*>(buf.data());
        if (FAILED(q->GetMessage(i, m, &size))) continue;
        if (m->Severity <= D3D12_MESSAGE_SEVERITY_WARNING) {
            std::fprintf(stderr, "D3D12 %s: %.*s\n", m->Severity <= D3D12_MESSAGE_SEVERITY_ERROR ? "error" : "warning",
                         int(m->DescriptionByteLength), m->pDescription);
            ok &= m->Severity > D3D12_MESSAGE_SEVERITY_ERROR;
        }
    }
    q->ClearStoredMessages();
    return ok;
}

struct Renderer {
    const Options& opt;
    App& app;

    ComPtr<ID3D12Device> dev;
    ComPtr<ID3D12CommandQueue> queue;
    ComPtr<IDXGISwapChain3> swap;
    ComPtr<ID3D12InfoQueue> info;  // with --d3d12-debug

    ComPtr<ID3D12CommandAllocator> alloc[kFrames];
    ComPtr<ID3D12GraphicsCommandList> cmd;
    ComPtr<ID3D12Fence> fence;
    UINT64 fence_value = 0, frame_fence[kFrames] = {};
    HANDLE fence_event = nullptr;

    ComPtr<ID3D12DescriptorHeap> rtv_heap, dsv_heap;
    UINT rtv_size = 0;
    ComPtr<ID3D12Resource> back_buffers[kFrames];
    ComPtr<ID3D12Resource> depth;

    ComPtr<ID3D12RootSignature> root_sig;
    ComPtr<ID3D12PipelineState> pso;
    ComPtr<ID3D12Resource> vb;
    D3D12_VERTEX_BUFFER_VIEW vbv{};
    Geometry geo;

    ComPtr<ID3D12Resource> constants;  // upload heap, persistently mapped
    uint8_t* constants_ptr = nullptr;

    // --cbv-tables: CBVs per frame (camera, object per draw) in a CPU-only heap, and a
    // shader-visible ring of tables (per frame, per draw: [object, camera]).
    ComPtr<ID3D12DescriptorHeap> cbv_cpu, cbv_gpu;
    UINT cbv_size = 0;

    Renderer(const Options& o, App& a) : opt(o), app(a) {}

    ~Renderer() {
        if (queue) wait_idle();
        if (fence_event) CloseHandle(fence_event);
    }

    void wait_idle() {
        check(queue->Signal(fence.Get(), ++fence_value), "Signal");
        wait_for(fence_value);
    }
    void wait_for(UINT64 value) {
        if (fence->GetCompletedValue() >= value) return;
        check(fence->SetEventOnCompletion(value, fence_event), "SetEventOnCompletion");
        WaitForSingleObject(fence_event, INFINITE);
    }

    void create() {
        if (opt.d3d12_debug) {
            ComPtr<ID3D12Debug> debug;
            if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debug))))
                debug->EnableDebugLayer();
            else
                std::fprintf(stderr, "D3D12 debug layer not available (install Graphics Tools)\n");
        }
        check(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&dev)), "D3D12CreateDevice");
        if (opt.d3d12_debug && SUCCEEDED(dev.As(&info))) info->SetMuteDebugOutput(FALSE);

        D3D12_COMMAND_QUEUE_DESC qd{D3D12_COMMAND_LIST_TYPE_DIRECT};
        check(dev->CreateCommandQueue(&qd, IID_PPV_ARGS(&queue)), "CreateCommandQueue");

        ComPtr<IDXGIFactory4> factory;
        check(CreateDXGIFactory2(0, IID_PPV_ARGS(&factory)), "CreateDXGIFactory2");
        DXGI_SWAP_CHAIN_DESC1 sd{};
        sd.Width = app.width;
        sd.Height = app.height;
        sd.Format = kColorFormat;
        sd.SampleDesc.Count = 1;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.BufferCount = kFrames;
        sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        ComPtr<IDXGISwapChain1> sc1;
        check(factory->CreateSwapChainForHwnd(queue.Get(), app.hwnd, &sd, nullptr, nullptr, &sc1), "CreateSwapChain");
        check(sc1.As(&swap), "IDXGISwapChain3");
        factory->MakeWindowAssociation(app.hwnd, DXGI_MWA_NO_ALT_ENTER);

        for (auto& a : alloc)
            check(dev->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&a)), "allocator");
        check(dev->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, alloc[0].Get(), nullptr, IID_PPV_ARGS(&cmd)),
              "CreateCommandList");
        check(cmd->Close(), "Close");
        check(dev->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)), "CreateFence");
        fence_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);

        rtv_heap = make_heap(dev.Get(), D3D12_DESCRIPTOR_HEAP_TYPE_RTV, kFrames, false);
        dsv_heap = make_heap(dev.Get(), D3D12_DESCRIPTOR_HEAP_TYPE_DSV, 1, false);
        rtv_size = dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
        create_targets();

        root_sig = make_root_signature(dev.Get(), opt.cbv_tables);
        create_pipeline();

        // Vertices live in an upload buffer too: small, static, and simplest.
        geo = build_geometry();
        const UINT vb_bytes = UINT(geo.verts.size() * sizeof(Vertex));
        vb = make_buffer(dev.Get(), vb_bytes, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
        void* p = nullptr;
        const D3D12_RANGE none{0, 0};
        check(vb->Map(0, &none, &p), "Map(vb)");
        std::memcpy(p, geo.verts.data(), vb_bytes);
        vb->Unmap(0, nullptr);
        vbv = {vb->GetGPUVirtualAddress(), vb_bytes, sizeof(Vertex)};

        constants = make_buffer(dev.Get(), kFrameStride * kFrames, D3D12_HEAP_TYPE_UPLOAD,
                                D3D12_RESOURCE_STATE_GENERIC_READ);
        check(constants->Map(0, &none, reinterpret_cast<void**>(&constants_ptr)), "Map(constants)");

        if (opt.cbv_tables) {
            cbv_size = dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
            cbv_cpu = make_heap(dev.Get(), D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, kFrames * (1 + kDraws), false);
            cbv_gpu = make_heap(dev.Get(), D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, kFrames * kDraws * 2, true);
            D3D12_CPU_DESCRIPTOR_HANDLE h = cbv_cpu->GetCPUDescriptorHandleForHeapStart();
            for (UINT i = 0; i < kFrames * (1 + kDraws); ++i, h.ptr += cbv_size) {
                const UINT size = i % (1 + kDraws) == 0 ? cb_size(sizeof(CameraCB)) : cb_size(sizeof(ObjectCB));
                const D3D12_CONSTANT_BUFFER_VIEW_DESC cbv{constants->GetGPUVirtualAddress() + UINT64(i) * kCbAlign, size};
                dev->CreateConstantBufferView(&cbv, h);
            }
        }
    }

    void create_pipeline() {
        auto vs = compile_shader(kSceneHlsl, "vs_main", "vs_5_0");
        auto ps = compile_shader(kSceneHlsl, "ps_main", "ps_5_0");
        const D3D12_INPUT_ELEMENT_DESC layout[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        };
        D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
        d.pRootSignature = root_sig.Get();
        d.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
        d.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
        d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        d.SampleMask = UINT_MAX;
        d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        d.RasterizerState.DepthClipEnable = TRUE;
        d.DepthStencilState.DepthEnable = TRUE;
        d.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        d.DepthStencilState.DepthFunc =
            opt.depth != DepthMode::Standard ? D3D12_COMPARISON_FUNC_GREATER : D3D12_COMPARISON_FUNC_LESS;
        d.InputLayout = {layout, 3};
        d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        d.NumRenderTargets = 1;
        d.RTVFormats[0] = kColorFormat;
        d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        d.SampleDesc.Count = 1;
        check(dev->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)), "CreateGraphicsPipelineState");
    }

    // Back buffer RTVs and the depth buffer, (re)created at the window size.
    void create_targets() {
        D3D12_CPU_DESCRIPTOR_HANDLE rtv = rtv_heap->GetCPUDescriptorHandleForHeapStart();
        for (UINT i = 0; i < kFrames; ++i, rtv.ptr += rtv_size) {
            check(swap->GetBuffer(i, IID_PPV_ARGS(&back_buffers[i])), "GetBuffer");
            dev->CreateRenderTargetView(back_buffers[i].Get(), nullptr, rtv);
        }

        D3D12_HEAP_PROPERTIES hp{D3D12_HEAP_TYPE_DEFAULT};
        D3D12_RESOURCE_DESC d{};
        d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        d.Width = UINT64(app.width);
        d.Height = UINT(app.height);
        d.DepthOrArraySize = d.MipLevels = 1;
        d.Format = DXGI_FORMAT_R32_TYPELESS;
        d.SampleDesc.Count = 1;
        d.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
        D3D12_CLEAR_VALUE clear{DXGI_FORMAT_D32_FLOAT};
        clear.DepthStencil.Depth = opt.depth != DepthMode::Standard ? 0.0f : 1.0f;
        check(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                                           &clear, IID_PPV_ARGS(&depth)),
              "depth");
        D3D12_DEPTH_STENCIL_VIEW_DESC dsv{DXGI_FORMAT_D32_FLOAT, D3D12_DSV_DIMENSION_TEXTURE2D};
        dev->CreateDepthStencilView(depth.Get(), &dsv, dsv_heap->GetCPUDescriptorHandleForHeapStart());
    }

    void resize() {
        wait_idle();
        for (auto& b : back_buffers) b.Reset();
        depth.Reset();
        check(swap->ResizeBuffers(0, UINT(app.width), UINT(app.height), DXGI_FORMAT_UNKNOWN, 0), "ResizeBuffers");
        create_targets();
    }

    void render(UINT64 frame_index, const CameraCB& cam, double t) {
        const UINT f = UINT(frame_index % kFrames);
        wait_for(frame_fence[f]);  // the GPU is done with this frame's allocator and constants

        // Constants for this frame's region: nothing tells the runtime (or ReShade) about these writes.
        uint8_t* region = constants_ptr + f * kFrameStride;
        std::memcpy(region, &cam, sizeof(cam));
        const ObjectCB objects[kDraws] = {static_object(), npc_object(t)};
        for (UINT d = 0; d < kDraws; ++d) std::memcpy(region + kCbAlign * (1 + d), &objects[d], sizeof(ObjectCB));

        check(alloc[f]->Reset(), "allocator Reset");
        check(cmd->Reset(alloc[f].Get(), pso.Get()), "Reset");

        const UINT bb = swap->GetCurrentBackBufferIndex();
        const D3D12_RESOURCE_BARRIER begin[2] = {
            transition(back_buffers[bb].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET),
            transition(depth.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE),
        };
        cmd->ResourceBarrier(2, begin);

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = rtv_heap->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += SIZE_T(bb) * rtv_size;
        const D3D12_CPU_DESCRIPTOR_HANDLE dsv = dsv_heap->GetCPUDescriptorHandleForHeapStart();
        const float sky[4] = {0.55f, 0.7f, 0.9f, 1.0f};
        cmd->ClearRenderTargetView(rtv, sky, 0, nullptr);
        cmd->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, opt.depth != DepthMode::Standard ? 0.0f : 1.0f, 0, 0,
                                   nullptr);
        cmd->OMSetRenderTargets(1, &rtv, FALSE, &dsv);
        const D3D12_VIEWPORT vp{0, 0, float(app.width), float(app.height), 0, 1};
        const D3D12_RECT scissor{0, 0, app.width, app.height};
        cmd->RSSetViewports(1, &vp);
        cmd->RSSetScissorRects(1, &scissor);
        cmd->SetGraphicsRootSignature(root_sig.Get());
        cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        cmd->IASetVertexBuffers(0, 1, &vbv);

        const D3D12_GPU_VIRTUAL_ADDRESS base = constants->GetGPUVirtualAddress() + UINT64(f) * kFrameStride;
        if (opt.cbv_tables) {
            ID3D12DescriptorHeap* heaps[] = {cbv_gpu.Get()};
            cmd->SetDescriptorHeaps(1, heaps);
        } else {
            cmd->SetGraphicsRootConstantBufferView(1, base);  // b0: camera
        }
        const UINT draw_count = opt.npc ? kDraws : 1;
        for (UINT d = 0; d < draw_count; ++d) {
            if (opt.cbv_tables) {
                // Table [object, camera], copied from this frame's CPU-only CBVs.
                const UINT slot = (f * kDraws + d) * 2;
                D3D12_CPU_DESCRIPTOR_HANDLE dst = cbv_gpu->GetCPUDescriptorHandleForHeapStart();
                D3D12_CPU_DESCRIPTOR_HANDLE src = cbv_cpu->GetCPUDescriptorHandleForHeapStart();
                dst.ptr += SIZE_T(slot) * cbv_size;
                const SIZE_T frame_cbvs = src.ptr + SIZE_T(f) * (1 + kDraws) * cbv_size;
                src.ptr = frame_cbvs + SIZE_T(1 + d) * cbv_size;
                dev->CopyDescriptorsSimple(1, dst, src, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
                dst.ptr += cbv_size;
                src.ptr = frame_cbvs;
                dev->CopyDescriptorsSimple(1, dst, src, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
                D3D12_GPU_DESCRIPTOR_HANDLE table = cbv_gpu->GetGPUDescriptorHandleForHeapStart();
                table.ptr += UINT64(slot) * cbv_size;
                cmd->SetGraphicsRootDescriptorTable(0, table);
            } else {
                cmd->SetGraphicsRootConstantBufferView(0, base + kCbAlign * (1 + d));  // b1: object
            }
            if (d == 0)
                cmd->DrawInstanced(geo.static_count, 1, 0, 0);
            else
                cmd->DrawInstanced(UINT(geo.verts.size()) - geo.static_count, 1, geo.static_count, 0);
        }

        const D3D12_RESOURCE_BARRIER end[2] = {
            transition(back_buffers[bb].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT),
            transition(depth.Get(), D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE),
        };
        cmd->ResourceBarrier(2, end);
        check(cmd->Close(), "Close");
        ID3D12CommandList* lists[] = {cmd.Get()};
        queue->ExecuteCommandLists(1, lists);
        check(swap->Present(1, 0), "Present");
        check(queue->Signal(fence.Get(), ++fence_value), "Signal");
        frame_fence[f] = fence_value;
    }
};

}  // namespace

int run_d3d12(const Options& opt) {
    App app;
    if (!app.create(L"fake_game", opt.width, opt.height, /*d3d11=*/false)) return 1;
    Renderer r(opt, app);
    r.create();

    LARGE_INTEGER qpf, t0, now;
    QueryPerformanceFrequency(&qpf);
    QueryPerformanceCounter(&t0);
    double last = 0, title_timer = 0;
    uint64_t frame_index = 0;
    CameraRig rig;
    int exit_code = 0;

    while (app.pump()) {
        if (app.resized) r.resize();
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - t0.QuadPart) / double(qpf.QuadPart);
        const float dt = float(std::min(t - last, 0.1));
        last = t;
        if (opt.duration > 0 && t > opt.duration) break;

        rig.update(app, opt, dt);
        r.render(frame_index, rig.constants(opt, float(app.width) / float(app.height)), t);
        if (!drain_debug_messages(r.info.Get())) exit_code = 3;
        ++frame_index;

        title_timer += dt;
        if (title_timer > 0.5) {
            title_timer = 0;
            const wchar_t* modes[] = {L"standard", L"reversed", L"reversed-infinite"};
            wchar_t buf[256];
            swprintf(buf, 256, L"fake_game D3D12 (%s)  [%s]  depth=%s  frame %llu",
                     opt.cbv_tables ? L"CBV tables" : L"root CBVs", rig.manual ? L"manual" : L"auto",
                     modes[int(opt.depth)], frame_index);
            app.set_title(buf);
        }
    }
    return exit_code;
}

}  // namespace lidar::fake
