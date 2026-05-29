// Native libcamera frame source: a single low-resolution viewfinder stream with
// a request-completed callback, mirroring the 2026 Picamera2 pre_callback.
//
// Compiled only when libcamera is found (HAVE_LIBCAMERA). It cannot be built or
// run off-device, so it must be verified on the Raspberry Pi during collector
// bring-up. The one empirical unknown (flagged in the plan) is the lores stream
// pixel format: this handles packed RGB/BGR888 and planar YUV420; confirm which
// the installed pipeline yields and adjust kPreferred below if needed.
#ifdef HAVE_LIBCAMERA

#include <sys/mman.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <libcamera/libcamera.h>
#include <opencv2/imgproc.hpp>

#include "camera/frame_source.h"

namespace rc {
namespace {

using namespace libcamera;

class LibcameraFrameSource : public FrameSource {
 public:
  explicit LibcameraFrameSource(FrameSourceConfig cfg) : cfg_(std::move(cfg)) {}
  ~LibcameraFrameSource() override { stop(); }

  bool start(Callback cb) override {
    cb_ = std::move(cb);
    cm_ = std::make_unique<CameraManager>();
    if (cm_->start() != 0) {
      std::cerr << "libcamera: CameraManager start failed\n";
      return false;
    }
    if (cm_->cameras().empty()) {
      std::cerr << "libcamera: no cameras\n";
      return false;
    }
    camera_ = cm_->cameras().front();
    if (camera_->acquire() != 0) {
      std::cerr << "libcamera: acquire failed\n";
      return false;
    }

    config_ = camera_->generateConfiguration({StreamRole::Viewfinder});
    StreamConfiguration& sc = config_->at(0);
    sc.size = Size(cfg_.width, cfg_.height);
    sc.pixelFormat = formats::RGB888;  // see note at top of file
    sc.bufferCount = 4;
    if (config_->validate() == CameraConfiguration::Invalid) {
      std::cerr << "libcamera: invalid configuration\n";
      return false;
    }
    if (camera_->configure(config_.get()) != 0) {
      std::cerr << "libcamera: configure failed\n";
      return false;
    }
    stream_ = sc.stream();
    fmt_ = sc.pixelFormat;
    stride_ = sc.stride;

    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0) {
      std::cerr << "libcamera: buffer allocation failed\n";
      return false;
    }
    // mmap each plane of each buffer up front.
    for (const std::unique_ptr<FrameBuffer>& buffer : allocator_->buffers(stream_)) {
      const FrameBuffer::Plane& plane0 = buffer->planes().front();
      size_t length = 0;
      for (const auto& p : buffer->planes()) length += p.length;
      void* mem = mmap(nullptr, length, PROT_READ, MAP_SHARED, plane0.fd.get(), 0);
      maps_[buffer.get()] = {mem, length};
      std::unique_ptr<Request> req = camera_->createRequest();
      req->addBuffer(stream_, buffer.get());
      requests_.push_back(std::move(req));
    }

    camera_->requestCompleted.connect(this, &LibcameraFrameSource::onComplete);

    ControlList controls(controls::controls);
    const int64_t frameUs = static_cast<int64_t>(1e6 / std::max(1.0, cfg_.fps));
    controls.set(controls::FrameDurationLimits, Span<const int64_t, 2>({frameUs, frameUs}));
    controls.set(controls::AfMode, controls::AfModeManual);
    controls.set(controls::AwbEnable, false);

    if (camera_->start(&controls) != 0) {
      std::cerr << "libcamera: camera start failed\n";
      return false;
    }
    for (auto& req : requests_) camera_->queueRequest(req.get());
    running_.store(true);
    return true;
  }

  void stop() override {
    if (!running_.exchange(false)) return;
    if (camera_) {
      camera_->stop();
      camera_->requestCompleted.disconnect(this);
    }
    for (auto& [buf, m] : maps_)
      if (m.first) munmap(m.first, m.second);
    maps_.clear();
    requests_.clear();
    if (allocator_) allocator_->free(stream_);
    allocator_.reset();
    if (camera_) {
      camera_->release();
      camera_.reset();
    }
    if (cm_) {
      cm_->stop();
      cm_.reset();
    }
  }

 private:
  void onComplete(Request* request) {
    if (request->status() == Request::RequestCancelled) return;
    FrameBuffer* buffer = request->buffers().at(stream_);
    auto it = maps_.find(buffer);
    if (it != maps_.end() && cb_) {
      uint8_t* data = static_cast<uint8_t*>(it->second.first);
      cv::Mat bgr;
      if (fmt_ == formats::RGB888 || fmt_ == formats::BGR888) {
        // Packed 24-bit; honour stride. 2026 treated this buffer as BGR.
        cv::Mat wrapped(cfg_.height, cfg_.width, CV_8UC3, data, stride_);
        wrapped.copyTo(bgr);
      } else {
        // Planar YUV420 (I420): Y plane is height rows of stride, then U,V.
        cv::Mat yuv(cfg_.height * 3 / 2, cfg_.width, CV_8UC1, data, stride_);
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
      }
      if (cfg_.rotate180) cv::rotate(bgr, bgr, cv::ROTATE_180);
      const uint64_t ts = request->metadata().contains(controls::SensorTimestamp.id())
                              ? request->metadata().get(controls::SensorTimestamp).value_or(0)
                              : 0;
      cb_(bgr, ts);
    }
    if (running_.load()) {
      request->reuse(Request::ReuseBuffers);
      camera_->queueRequest(request);
    }
  }

  FrameSourceConfig cfg_;
  Callback cb_;
  std::unique_ptr<CameraManager> cm_;
  std::shared_ptr<Camera> camera_;
  std::unique_ptr<CameraConfiguration> config_;
  std::unique_ptr<FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<Request>> requests_;
  std::map<FrameBuffer*, std::pair<void*, size_t>> maps_;
  Stream* stream_ = nullptr;
  PixelFormat fmt_;
  unsigned int stride_ = 0;
  std::atomic<bool> running_{false};
};

}  // namespace

std::unique_ptr<FrameSource> makeLibcameraFrameSource(const FrameSourceConfig& cfg) {
  return std::make_unique<LibcameraFrameSource>(cfg);
}

}  // namespace rc

#endif  // HAVE_LIBCAMERA
