// Gamepad input abstraction for the control station. Backends:
//   * SdlGamepad   - real USB gamepad via SDL2 (compiled when HAVE_SDL2).
//   * ScriptedInput - deterministic throttle/steer sweep with recording on, for
//     headless integration testing (no dependencies).
#pragma once

#include <memory>
#include <string>

namespace rc {

struct GamepadState {
  float throttle = 0.0f;  // [-1,1], + = forward
  float steer = 0.0f;     // [-1,1], + = right
  bool recording = false;  // latched
  bool estop = false;
  bool quit = false;
};

class GamepadInput {
 public:
  virtual ~GamepadInput() = default;
  virtual bool open() = 0;
  virtual void poll(GamepadState& out) = 0;  // fills latest state
};

// kind: "sdl" (real pad) or "scripted" (headless test). Falls back to scripted
// if SDL2 is unavailable.
std::unique_ptr<GamepadInput> makeGamepad(const std::string& kind);

}  // namespace rc
