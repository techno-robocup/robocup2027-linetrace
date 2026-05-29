// Abstract camera frame source shared by the collector and the executor.
//
// Two backends:
//   * OpenCvFrameSource  - cv::VideoCapture (device index, GStreamer/libcamera
//     pipeline, video file) or a built-in "synthetic" moving-line generator.
//     Always available; used for desktop testing and as a portable fallback.
//   * LibcameraFrameSource - native libcamera with a request-completed callback
//     mirroring the 2026 pre_callback. Compiled only when libcamera is found
//     (HAVE_LIBCAMERA); preferred on the Raspberry Pi.
//
// Frames are delivered already oriented (rotation applied here if configured),
// so downstream preprocessing never rotates.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace rc {

struct FrameSourceConfig {
  // "synthetic" | "libcamera" | "<index>" | "<gstreamer pipeline>" | "<file>"
  std::string spec = "synthetic";
  int width = 576;
  int height = 324;
  double fps = 30.0;
  bool rotate180 = true;  // robot camera is mounted upside down (2026 behaviour)
};

class FrameSource {
 public:
  // Called from the capture thread with a BGR frame (width x height) and a
  // monotonic timestamp in nanoseconds.
  using Callback = std::function<void(const cv::Mat& bgr, uint64_t tsNs)>;

  virtual ~FrameSource() = default;
  virtual bool start(Callback cb) = 0;
  virtual void stop() = 0;
};

// Builds the appropriate backend for cfg.spec. Falls back to OpenCV if a native
// libcamera source is requested but not compiled in.
std::unique_ptr<FrameSource> makeFrameSource(const FrameSourceConfig& cfg);

}  // namespace rc
