// Hang diagnostics. A background thread notices when the game stops presenting and writes to
// ReShade.log where the render thread is: the addon step it's in (if any) and the return
// addresses found on its stack, as module+offset. A freeze then leaves evidence behind. It also
// logs free address space as it shrinks, and the first access violations, for crashes.
#pragma once

namespace lidar::watchdog {

void start();      // from the render thread; idempotent
void stop();       // waits for the thread to exit; not from DllMain
void heartbeat();  // once per present, from the render thread

// Names the addon step the render thread is in, for as long as the scope lives.
const char* exchange_step(const char* step);
class Step {
public:
    explicit Step(const char* step) : prev_(exchange_step(step)) {}
    ~Step() { exchange_step(prev_); }
    Step(const Step&) = delete;
    Step& operator=(const Step&) = delete;

private:
    const char* prev_;
};

}  // namespace lidar::watchdog
