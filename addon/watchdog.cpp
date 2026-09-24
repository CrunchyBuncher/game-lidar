#include "watchdog.h"

#include <reshade.hpp>

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>

namespace lidar::watchdog {
namespace {

constexpr ULONGLONG kFirstReportMs = 3000;
constexpr ULONGLONG kSecondReportMs = 8000;  // same stack twice: stuck; different: looping
constexpr int kMaxFrames = 40;
constexpr uintptr_t kMaxScanBytes = 64 * 1024;

std::atomic<const char*> g_step{nullptr};
std::atomic<DWORD> g_thread{0};
std::atomic<uint64_t> g_beats{0};
std::atomic<ULONGLONG> g_last_beat{0};
HANDLE g_worker = nullptr;
HANDLE g_stop = nullptr;

bool executable(const MEMORY_BASIC_INFORMATION& m) {
    return m.State == MEM_COMMIT && m.Type == MEM_IMAGE &&
           (m.Protect & (PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY)) != 0;
}

// Heuristic: a value on the stack is a return address if it points into module code right after
// a call instruction (E8 rel32, or FF /2 in its 2, 3, 6 and 7 byte forms). Only VirtualQuery,
// which takes no user-mode locks: the render thread is suspended and may hold any of them.
bool is_return_address(uintptr_t a, uintptr_t& module) {
    MEMORY_BASIC_INFORMATION m{};
    if (a < 0x10000 || VirtualQuery(reinterpret_cast<const void*>(a), &m, sizeof(m)) == 0 || !executable(m))
        return false;
    if (a - 7 < uintptr_t(m.BaseAddress)) {
        MEMORY_BASIC_INFORMATION before{};
        if (VirtualQuery(reinterpret_cast<const void*>(a - 7), &before, sizeof(before)) == 0 || !executable(before))
            return false;
    }
    const auto* p = reinterpret_cast<const uint8_t*>(a);
    const auto call_ff = [&](int len) { return p[-len] == 0xFF && (p[-len + 1] & 0x38) == 0x10; };
    if (p[-5] != 0xE8 && !call_ff(2) && !call_ff(3) && !call_ff(6) && !call_ff(7)) return false;
    module = uintptr_t(m.AllocationBase);
    return true;
}

std::string describe(uintptr_t addr, uintptr_t module) {
    char path[MAX_PATH] = "?";
    if (module != 0) GetModuleFileNameA(reinterpret_cast<HMODULE>(module), path, MAX_PATH);
    const char* name = path;
    for (const char* c = path; *c; ++c)
        if (*c == '\\' || *c == '/') name = c + 1;
    char buf[MAX_PATH + 32];
    std::snprintf(buf, sizeof(buf), "%s+0x%llx", name, static_cast<unsigned long long>(addr - module));
    return buf;
}

void report(ULONGLONG stalled_ms) {
    const DWORD tid = g_thread.load();
    const char* const step = g_step.load();
    struct Frame {
        uintptr_t addr, module;
    } frames[kMaxFrames];
    int n = 0;
    uintptr_t pc = 0, pc_module = 0;
    bool suspended = false;

    // While the thread is suspended: no allocation, no logging, no loader calls.
    if (HANDLE t = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE, tid)) {
        if (SuspendThread(t) != DWORD(-1)) {
            suspended = true;
            CONTEXT ctx{};
            ctx.ContextFlags = CONTEXT_CONTROL;
            if (GetThreadContext(t, &ctx)) {
#ifdef _WIN64
                pc = uintptr_t(ctx.Rip);
                const uintptr_t sp = uintptr_t(ctx.Rsp);
#else
                pc = uintptr_t(ctx.Eip);
                const uintptr_t sp = uintptr_t(ctx.Esp);
#endif
                MEMORY_BASIC_INFORMATION m{};
                if (VirtualQuery(reinterpret_cast<const void*>(pc), &m, sizeof(m)) != 0 && m.Type == MEM_IMAGE)
                    pc_module = uintptr_t(m.AllocationBase);
                if (VirtualQuery(reinterpret_cast<const void*>(sp), &m, sizeof(m)) != 0) {
                    const uintptr_t end = std::min(uintptr_t(m.BaseAddress) + uintptr_t(m.RegionSize), sp + kMaxScanBytes);
                    for (uintptr_t a = sp; a + sizeof(uintptr_t) <= end && n < kMaxFrames; a += sizeof(uintptr_t)) {
                        const uintptr_t v = *reinterpret_cast<const uintptr_t*>(a);
                        if (uintptr_t module = 0; is_return_address(v, module)) frames[n++] = {v, module};
                    }
                }
            }
            ResumeThread(t);
        }
        CloseHandle(t);
    }

    char head[256];
    std::snprintf(head, sizeof(head), "Watchdog: no present for %.1f s (after %llu frames). Render thread %lu is %s%s%s.",
                  double(stalled_ms) / 1000.0, static_cast<unsigned long long>(g_beats.load()), tid,
                  step ? "in the addon, step \"" : "outside the addon", step ? step : "", step ? "\"" : "");
    std::string msg = head;
    if (!suspended) {
        msg += " (couldn't suspend it to read its stack)";
    } else {
        msg += "\n  pc: " + describe(pc, pc_module);
        for (int i = 0; i < n; ++i) msg += "\n  ret: " + describe(frames[i].addr, frames[i].module);
    }
    reshade::log::message(reshade::log::level::warning, msg.c_str());
}

// Free address space. 32-bit games that aren't large-address-aware live in 2 GB, and running
// out (or out of contiguous room) tends to show up as crashes and hangs inside the driver.
struct AddressSpace {
    uintptr_t free = 0, largest = 0;
};
AddressSpace address_space() {
    AddressSpace s;
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    MEMORY_BASIC_INFORMATION m{};
    for (auto a = uintptr_t(si.lpMinimumApplicationAddress); a < uintptr_t(si.lpMaximumApplicationAddress);
         a = uintptr_t(m.BaseAddress) + uintptr_t(m.RegionSize)) {
        if (VirtualQuery(reinterpret_cast<const void*>(a), &m, sizeof(m)) == 0) break;
        if (m.State == MEM_FREE) {
            s.free += uintptr_t(m.RegionSize);
            s.largest = std::max(s.largest, uintptr_t(m.RegionSize));
        }
    }
    return s;
}

void log_address_space(const char* what) {
    const AddressSpace s = address_space();
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s: %llu MB of address space free, largest free block %llu MB.", what,
                  static_cast<unsigned long long>(s.free >> 20), static_cast<unsigned long long>(s.largest >> 20));
    reshade::log::message(reshade::log::level::info, buf);
}

// Logs the first few access violations anywhere in the process (first chance, so before the game
// or driver handles them, or the process dies).
std::atomic<int> g_faults{0};
LONG CALLBACK on_exception(EXCEPTION_POINTERS* e) {
    const EXCEPTION_RECORD& r = *e->ExceptionRecord;
    if (r.ExceptionCode != EXCEPTION_ACCESS_VIOLATION || g_faults.fetch_add(1) >= 3) return EXCEPTION_CONTINUE_SEARCH;
    const auto addr = uintptr_t(r.ExceptionAddress);
    MEMORY_BASIC_INFORMATION m{};
    const uintptr_t module =
        VirtualQuery(r.ExceptionAddress, &m, sizeof(m)) != 0 && m.Type == MEM_IMAGE ? uintptr_t(m.AllocationBase) : 0;
    const AddressSpace s = address_space();
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "Access violation at %s (%s address 0x%llx) on thread %lu, after %llu frames. Address space: %llu MB "
                  "free, largest free block %llu MB.",
                  describe(addr, module).c_str(), r.ExceptionInformation[0] == 1 ? "writing" : "reading",
                  static_cast<unsigned long long>(r.ExceptionInformation[1]), GetCurrentThreadId(),
                  static_cast<unsigned long long>(g_beats.load()), static_cast<unsigned long long>(s.free >> 20),
                  static_cast<unsigned long long>(s.largest >> 20));
    reshade::log::message(reshade::log::level::error, buf);
    return EXCEPTION_CONTINUE_SEARCH;
}
PVOID g_handler = nullptr;

DWORD WINAPI run(void*) {
    int reports = 0;
    uint64_t beats_seen = 0;
    log_address_space("Start");
    uintptr_t logged_largest = address_space().largest;
    for (int tick = 0; WaitForSingleObject(g_stop, 500) == WAIT_TIMEOUT; ++tick) {
        if (tick % 4 == 0) {  // log each time the largest free block shrinks by a quarter
            if (address_space().largest < logged_largest - logged_largest / 4) {
                log_address_space("Address space shrinking");
                logged_largest = address_space().largest;
            }
        }
        const uint64_t beats = g_beats.load();
        if (beats == 0) continue;  // still loading before the first frame
        if (beats != beats_seen) {
            beats_seen = beats;
            reports = 0;
            continue;
        }
        const ULONGLONG stalled = GetTickCount64() - g_last_beat.load();
        if ((reports == 0 && stalled >= kFirstReportMs) || (reports == 1 && stalled >= kSecondReportMs)) {
            report(stalled);
            ++reports;
        }
    }
    return 0;
}

}  // namespace

const char* exchange_step(const char* step) { return g_step.exchange(step); }

void start() {
    if (g_worker != nullptr) return;
    g_thread = GetCurrentThreadId();
    g_last_beat = GetTickCount64();
    g_stop = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    g_worker = g_stop ? CreateThread(nullptr, 0, run, nullptr, 0, nullptr) : nullptr;
    g_handler = AddVectoredExceptionHandler(1, on_exception);
    reshade::log::message(reshade::log::level::info,
                          g_worker ? "Watchdog running: stalls over 3 s get logged." : "Watchdog failed to start.");
}

void stop() {
    if (g_worker == nullptr) return;
    if (g_handler) RemoveVectoredExceptionHandler(g_handler);
    g_handler = nullptr;
    SetEvent(g_stop);
    WaitForSingleObject(g_worker, INFINITE);
    CloseHandle(g_worker);
    CloseHandle(g_stop);
    g_worker = g_stop = nullptr;
}

void heartbeat() {
    g_thread = GetCurrentThreadId();
    g_last_beat = GetTickCount64();
    ++g_beats;
}

}  // namespace lidar::watchdog
