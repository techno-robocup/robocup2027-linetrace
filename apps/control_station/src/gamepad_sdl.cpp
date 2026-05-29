// SDL2 gamepad backend (compiled only with HAVE_SDL2). Left stick = throttle
// (Y) / steer (X); A toggles recording; B / either bumper is e-stop.
#include "gamepad.h"

#ifdef HAVE_SDL2

#include <SDL2/SDL.h>

#include <cstdint>
#include <iostream>

#include "common/mixing.h"

namespace rc {
namespace {

class SdlGamepad : public GamepadInput {
 public:
  bool open() override {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
      std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
      return false;
    }
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
      if (SDL_IsGameController(i)) {
        pad_ = SDL_GameControllerOpen(i);
        if (pad_) break;
      }
    }
    if (!pad_) {
      std::cerr << "no game controller found\n";
      return false;
    }
    return true;
  }

  void poll(GamepadState& out) override {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_CONTROLLERBUTTONDOWN) {
        if (e.cbutton.button == SDL_CONTROLLER_BUTTON_A) recording_ = !recording_;
      }
    }
    auto axis = [&](SDL_GameControllerAxis a) {
      return applyDeadzone(SDL_GameControllerGetAxis(pad_, a) / 32767.0f);
    };
    out.throttle = -axis(SDL_CONTROLLER_AXIS_LEFTY);  // stick up = forward
    out.steer = axis(SDL_CONTROLLER_AXIS_LEFTX);
    out.estop = SDL_GameControllerGetButton(pad_, SDL_CONTROLLER_BUTTON_B) ||
                SDL_GameControllerGetButton(pad_, SDL_CONTROLLER_BUTTON_LEFTSHOULDER) ||
                SDL_GameControllerGetButton(pad_, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
    out.recording = recording_ && !out.estop;
    out.quit = false;
  }

  ~SdlGamepad() override {
    if (pad_) SDL_GameControllerClose(pad_);
    SDL_Quit();
  }

 private:
  SDL_GameController* pad_ = nullptr;
  bool recording_ = false;
};

}  // namespace

std::unique_ptr<GamepadInput> makeSdlGamepad() { return std::make_unique<SdlGamepad>(); }

}  // namespace rc

#endif  // HAVE_SDL2
