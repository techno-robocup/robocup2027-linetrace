// Latest-frame-wins preview JPEG sender (fragmented UDP to the station's
// preview port; protocol in common/proto.h). Shared by the collector (which
// learns the station address from incoming control packets) and the executor
// (which is given a station host on the command line).
//
// Header-only on top of netutil, but requires OpenCV (imencode/resize) — only
// include from targets that link OpenCV.
#pragma once

#include <netinet/in.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "common/proto.h"
#include "netutil/net.h"

namespace rc::net {

class PreviewSender {
 public:
  PreviewSender(int fd, int previewPort, int quality, double fps)
      : fd_(fd), port_(previewPort), quality_(quality), fps_(fps) {}

  void setStation(const in_addr& a) {
    std::lock_guard<std::mutex> lk(mx_);
    stationAddr_ = a;
    hasStation_ = true;
  }
  // Fixed station by hostname/IP (executor's --preview-to).
  bool resolveStation(const std::string& host) {
    UdpTarget t;
    if (!t.resolve(host, port_)) return false;
    setStation(t.addr.sin_addr);
    return true;
  }
  void offer(const cv::Mat& bgr) {
    std::lock_guard<std::mutex> lk(mx_);
    latest_ = bgr.clone();
  }
  void start() {
    run_.store(true);
    thread_ = std::thread([this] { loop(); });
  }
  void stop() {
    if (run_.exchange(false) && thread_.joinable()) thread_.join();
  }

 private:
  void loop() {
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, fps_));
    uint32_t frameId = 0;
    while (run_.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      cv::Mat frame;
      UdpTarget tgt;
      bool ready = false;
      {
        std::lock_guard<std::mutex> lk(mx_);
        if (hasStation_ && !latest_.empty()) {
          frame = latest_;
          latest_.release();
          tgt.addr.sin_family = AF_INET;
          tgt.addr.sin_addr = stationAddr_;
          tgt.addr.sin_port = htons(static_cast<uint16_t>(port_));
          ready = true;
        }
      }
      if (ready) sendFrame(frame, tgt, frameId++);
      std::this_thread::sleep_until(
          t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
    }
  }

  void sendFrame(const cv::Mat& bgr, const UdpTarget& tgt, uint32_t frameId) {
    cv::Mat small;
    cv::resize(bgr, small, cv::Size(384, 216));
    std::vector<uchar> jpg;
    cv::imencode(".jpg", small, jpg, {cv::IMWRITE_JPEG_QUALITY, quality_});

    const size_t payloadMax = 1400;
    const uint16_t fragCount =
        static_cast<uint16_t>((jpg.size() + payloadMax - 1) / payloadMax);
    std::vector<uint8_t> pkt(kPreviewHeaderSize + payloadMax);
    for (uint16_t i = 0; i < fragCount; ++i) {
      const size_t off = i * payloadMax;
      const size_t n = std::min(payloadMax, jpg.size() - off);
      PreviewFragHeader h{frameId, i, fragCount, static_cast<uint32_t>(jpg.size())};
      encodePreviewHeader(h, pkt.data());
      std::memcpy(pkt.data() + kPreviewHeaderSize, jpg.data() + off, n);
      net::sendTo(fd_, tgt, pkt.data(), kPreviewHeaderSize + n);
    }
  }

  int fd_;
  int port_;
  int quality_;
  double fps_;
  std::mutex mx_;
  cv::Mat latest_;
  in_addr stationAddr_{};
  bool hasStation_ = false;
  std::thread thread_;
  std::atomic<bool> run_{false};
};

}  // namespace rc::net
