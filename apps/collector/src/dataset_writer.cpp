#include "dataset_writer.h"

#include <cstdio>
#include <filesystem>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

namespace rc {

DatasetWriter::DatasetWriter(std::string root, int jpegQuality, size_t maxQueue)
    : root_(std::move(root)), quality_(jpegQuality), maxQueue_(maxQueue) {}

DatasetWriter::~DatasetWriter() { stop(); }

bool DatasetWriter::startSession(const std::string& stamp, const std::string& meta) {
  sessionDir_ = (fs::path(root_) / ("session_" + stamp)).string();
  imageDir_ = (fs::path(sessionDir_) / "linetrace").string();
  std::error_code ec;
  fs::create_directories(imageDir_, ec);
  if (ec) return false;

  csv_ = std::fopen((fs::path(sessionDir_) / "labels.csv").string().c_str(), "w");
  if (!csv_) return false;
  std::fprintf(csv_,
               "timestamp,motor_left,motor_right,yaw,roll,pitch,acc_x,acc_y,acc_z,"
               "usonic_l,usonic_m,usonic_r,linetrace_file,rescue_file\n");
  std::fflush(csv_);

  FILE* sj = std::fopen((fs::path(sessionDir_) / "session.json").string().c_str(), "w");
  if (sj) {
    std::fprintf(sj, "%s\n", meta.c_str());
    std::fclose(sj);
  }
  return true;
}

void DatasetWriter::start() {
  if (run_.exchange(true)) return;
  thread_ = std::thread([this] { loop(); });
}

void DatasetWriter::stop() {
  if (!run_.exchange(false)) return;
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  if (csv_) {
    std::fclose(csv_);
    csv_ = nullptr;
  }
}

void DatasetWriter::enqueue(FrameJob job) {
  {
    std::lock_guard<std::mutex> lk(mx_);
    if (q_.size() >= maxQueue_) {
      q_.pop_front();  // drop oldest under back-pressure
      dropped_.fetch_add(1);
    }
    q_.push_back(std::move(job));
  }
  cv_.notify_one();
}

void DatasetWriter::loop() {
  const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality_};
  while (true) {
    FrameJob job;
    {
      std::unique_lock<std::mutex> lk(mx_);
      cv_.wait(lk, [this] { return !run_.load() || !q_.empty(); });
      if (!run_.load() && q_.empty()) break;
      job = std::move(q_.front());
      q_.pop_front();
    }

    const double ts = job.unixMs / 1000.0;
    char name[64];
    std::snprintf(name, sizeof(name), "%.3f.jpg", ts);
    const std::string path = (std::filesystem::path(imageDir_) / name).string();
    if (!cv::imwrite(path, job.image, params)) {
      dropped_.fetch_add(1);
      continue;
    }
    if (csv_) {
      std::fprintf(csv_,
                   "%.3f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%s,\n",
                   ts, job.left, job.right, job.bno.heading, job.bno.roll, job.bno.pitch,
                   job.bno.ax, job.bno.ay, job.bno.az, job.uson.l, job.uson.m, job.uson.r,
                   name);
      written_.fetch_add(1);
      if ((written_.load() % 50) == 0) std::fflush(csv_);
    }
  }
  if (csv_) std::fflush(csv_);
}

}  // namespace rc
