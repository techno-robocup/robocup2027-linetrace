// Bounded-queue background writer: encodes frames to JPEG and appends rows to a
// labels.csv whose header matches the 2026 dataset schema (so existing tooling
// and the existing linetrace_data sessions stay compatible).
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "esp32_driver/protocol.h"

namespace rc {

struct FrameJob {
  cv::Mat image;  // BGR, already oriented
  uint64_t unixMs = 0;
  int left = 1500;
  int right = 1500;
  esp32::Bno bno;
  esp32::Ultrasonic uson;
  bool button = false;
};

class DatasetWriter {
 public:
  DatasetWriter(std::string root, int jpegQuality = 90, size_t maxQueue = 256);
  ~DatasetWriter();

  // Creates <root>/session_<stamp>/{linetrace, labels.csv, session.json}.
  bool startSession(const std::string& stamp, const std::string& meta);
  std::string sessionDir() const { return sessionDir_; }

  void start();
  void stop();  // flush + join

  // Non-blocking; drops the oldest job if the queue is full (counted).
  void enqueue(FrameJob job);

  uint64_t written() const { return written_.load(); }
  uint64_t dropped() const { return dropped_.load(); }

 private:
  void loop();

  std::string root_;
  int quality_;
  size_t maxQueue_;
  std::string sessionDir_;
  std::string imageDir_;
  FILE* csv_ = nullptr;

  std::deque<FrameJob> q_;
  std::mutex mx_;
  std::condition_variable cv_;
  std::thread thread_;
  std::atomic<bool> run_{false};
  std::atomic<uint64_t> written_{0};
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace rc
