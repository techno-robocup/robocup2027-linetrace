// Scripted, dependency-free gamepad: a sinusoidal steer sweep at a steady
// forward throttle with recording enabled shortly after start. Used for the
// headless station<->collector integration test.
#include <chrono>
#include <cmath>

#include "gamepad.h"

namespace rc {

namespace {
class ScriptedInput : public GamepadInput {
 public:
  bool open() override {
    t0_ = std::chrono::steady_clock::now();
    return true;
  }
  void poll(GamepadState& out) override {
    const double t =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    out.throttle = 0.4f;
    out.steer = 0.5f * static_cast<float>(std::sin(t * 1.5));
    out.recording = t > 0.2;  // start recording after warm-up
    out.estop = false;
    out.quit = false;
  }

 private:
  std::chrono::steady_clock::time_point t0_;
};
}  // namespace

std::unique_ptr<GamepadInput> makeScriptedInput() {
  return std::make_unique<ScriptedInput>();
}

}  // namespace rc
