#include "gamepad.h"

#include <iostream>

namespace rc {

std::unique_ptr<GamepadInput> makeScriptedInput();
#ifdef HAVE_SDL2
std::unique_ptr<GamepadInput> makeSdlGamepad();
#endif
#ifdef HAVE_HIGHGUI
std::unique_ptr<GamepadInput> makeMouseInput();
#endif

std::unique_ptr<GamepadInput> makeGamepad(const std::string& kind) {
#ifdef HAVE_SDL2
  if (kind == "sdl") return makeSdlGamepad();
#endif
#ifdef HAVE_HIGHGUI
  if (kind == "mouse") return makeMouseInput();
#endif
  if (kind != "scripted")
    std::cerr << "input '" << kind << "' unavailable; using scripted\n";
  return makeScriptedInput();
}

}  // namespace rc
