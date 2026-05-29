#include "gamepad.h"

namespace rc {

std::unique_ptr<GamepadInput> makeScriptedInput();
#ifdef HAVE_SDL2
std::unique_ptr<GamepadInput> makeSdlGamepad();
#endif

std::unique_ptr<GamepadInput> makeGamepad(const std::string& kind) {
#ifdef HAVE_SDL2
  if (kind == "sdl") return makeSdlGamepad();
#endif
  return makeScriptedInput();
}

}  // namespace rc
