// OpenCV-backed frame source (+ synthetic generator). Pull-based capture run on
// a dedicated thread that invokes the callback, so it matches the push-style
// FrameSource interface.
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "camera/frame_source.h"

namespace rc {

namespace {

uint64_t monoNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

class OpenCvFrameSource : public FrameSource {
 public:
  explicit OpenCvFrameSource(FrameSourceConfig cfg) : cfg_(std::move(cfg)) {}
  ~OpenCvFrameSource() override { stop(); }

  bool start(Callback cb) override {
    cb_ = std::move(cb);
    synthetic_ = (cfg_.spec == "synthetic" || cfg_.spec.empty());
    if (!synthetic_) {
      // Numeric spec => camera index; otherwise a pipeline/file path.
      bool numeric = !cfg_.spec.empty();
      for (char c : cfg_.spec) numeric = numeric && std::isdigit((unsigned char)c);
      if (numeric)
        cap_.open(std::stoi(cfg_.spec));
      else
        cap_.open(cfg_.spec);
      if (!cap_.isOpened()) return false;
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, cfg_.width);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    }
    run_.store(true);
    thread_ = std::thread([this] { loop(); });
    return true;
  }

  void stop() override {
    if (run_.exchange(false)) {
      if (thread_.joinable()) thread_.join();
      if (cap_.isOpened()) cap_.release();
    }
  }

 private:
  void loop() {
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cfg_.fps));
    int frame = 0;
    while (run_.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      cv::Mat bgr;
      if (synthetic_) {
        bgr = makeSynthetic(frame++);
      } else {
        if (!cap_.read(bgr) || bgr.empty()) {
          // For file sources, loop; for cameras, just retry.
          cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
          continue;
        }
        if (bgr.cols != cfg_.width || bgr.rows != cfg_.height)
          cv::resize(bgr, bgr, cv::Size(cfg_.width, cfg_.height));
      }
      if (cfg_.rotate180) cv::rotate(bgr, bgr, cv::ROTATE_180);
      if (cb_) cb_(bgr, monoNs());

      std::this_thread::sleep_until(t0 + std::chrono::duration_cast<
                                             std::chrono::steady_clock::duration>(period));
    }
  }

  // A white field with a wandering black line, for end-to-end testing.
  cv::Mat makeSynthetic(int frame) {
    cv::Mat img(cfg_.height, cfg_.width, CV_8UC3, cv::Scalar(235, 235, 235));
    const double phase = frame * 0.05;
    const int x = static_cast<int>(cfg_.width * (0.5 + 0.3 * std::sin(phase)));
    cv::line(img, {x, 0}, {x, cfg_.height}, cv::Scalar(20, 20, 20), 24);
    return img;
  }

  FrameSourceConfig cfg_;
  Callback cb_;
  cv::VideoCapture cap_;
  std::thread thread_;
  std::atomic<bool> run_{false};
  bool synthetic_ = true;
};

}  // namespace

std::unique_ptr<FrameSource> makeOpenCvFrameSource(const FrameSourceConfig& cfg) {
  return std::make_unique<OpenCvFrameSource>(cfg);
}

}  // namespace rc
