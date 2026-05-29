#include "camera/frame_source.h"

namespace rc {

std::unique_ptr<FrameSource> makeOpenCvFrameSource(const FrameSourceConfig& cfg);
#ifdef HAVE_LIBCAMERA
std::unique_ptr<FrameSource> makeLibcameraFrameSource(const FrameSourceConfig& cfg);
#endif

std::unique_ptr<FrameSource> makeFrameSource(const FrameSourceConfig& cfg) {
#ifdef HAVE_LIBCAMERA
  if (cfg.spec == "libcamera") return makeLibcameraFrameSource(cfg);
#endif
  // OpenCV backend also covers libcamera on the Pi via a GStreamer pipeline
  // spec, e.g. "libcamerasrc ! video/x-raw,width=576,height=324 ! videoconvert ! appsink".
  return makeOpenCvFrameSource(cfg);
}

}  // namespace rc
