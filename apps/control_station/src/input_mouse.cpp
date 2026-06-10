// Mouse "virtual joystick" backend (compiled with HAVE_HIGHGUI): hold the left
// button on the preview window and drag — the drag vector from the press point
// maps continuously to throttle (up = forward) / steer (right = right), and
// releasing the button stops the robot. Continuous like a real stick: key
// presses are on/off and would quantize the dataset into discrete maneuvers
// (the 2026 keyboard data problem), so keys are only used for the discrete
// actions, delivered via onKey() from the main loop's cv::waitKey:
//   r = record toggle, space = e-stop toggle (q = quit, handled in main).
#include "gamepad.h"

#ifdef HAVE_HIGHGUI

#include <algorithm>
#include <iostream>

#include <opencv2/highgui.hpp>

#include "common/mixing.h"

namespace rc {
namespace {

constexpr char kWindow[] = "preview";       // must match main.cpp's imshow
constexpr float kPixelsFullScale = 120.0f;  // drag distance for full deflection

class MouseInput : public GamepadInput {
 public:
  bool open() override {
    // Create the window up front so the callback can attach before the first
    // preview frame arrives; main.cpp's imshow("preview") reuses it.
    cv::namedWindow(kWindow, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(kWindow, &MouseInput::onMouseThunk, this);
    std::cerr << "mouse input: hold the left button on the preview window and drag\n"
                 "  up/down = throttle, left/right = steer, release = stop\n"
                 "  keys: r = record on/off, space = e-stop on/off, q = quit\n";
    return true;
  }

  void poll(GamepadState& out) override {
    out.throttle = throttle_;
    out.steer = steer_;
    out.recording = recording_ && !estop_;
    out.estop = estop_;
    out.quit = false;
  }

  void onKey(int key) override {
    if (key == 'r') recording_ = !recording_;
    else if (key == ' ') estop_ = !estop_;
  }

 private:
  static void onMouseThunk(int ev, int x, int y, int, void* self) {
    static_cast<MouseInput*>(self)->onMouse(ev, x, y);
  }

  void onMouse(int ev, int x, int y) {
    switch (ev) {
      case cv::EVENT_LBUTTONDOWN:
        dragging_ = true;
        ox_ = x;
        oy_ = y;
        break;
      case cv::EVENT_LBUTTONUP:
        dragging_ = false;
        throttle_ = 0.0f;
        steer_ = 0.0f;
        break;
      case cv::EVENT_MOUSEMOVE:
        if (dragging_) {
          const float dz = 0.04f;  // small: allow driving exactly straight
          steer_ = applyDeadzone(
              std::clamp((x - ox_) / kPixelsFullScale, -1.0f, 1.0f), dz);
          throttle_ = applyDeadzone(
              std::clamp((oy_ - y) / kPixelsFullScale, -1.0f, 1.0f), dz);
        }
        break;
      default:
        break;
    }
  }

  // The highgui mouse callback and waitKey run on the same thread as poll()
  // (events are pumped inside main's cv::waitKey), so no locking is needed.
  bool dragging_ = false;
  int ox_ = 0, oy_ = 0;
  float throttle_ = 0.0f, steer_ = 0.0f;
  bool recording_ = false;
  bool estop_ = false;
};

}  // namespace

std::unique_ptr<GamepadInput> makeMouseInput() {
  return std::make_unique<MouseInput>();
}

}  // namespace rc

#endif  // HAVE_HIGHGUI
