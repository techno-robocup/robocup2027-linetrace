#include "dataset.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>

#include <opencv2/imgcodecs.hpp>

#include "augment.h"

namespace fs = std::filesystem;

namespace rc {

namespace {

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string field;
  std::stringstream ss(line);
  while (std::getline(ss, field, ',')) out.push_back(field);
  return out;
}

float parseFloatOr(const std::string& s, float dflt) {
  if (s.empty()) return dflt;
  try {
    return std::stof(s);
  } catch (...) {
    return dflt;
  }
}

// Scale raw sensors to roughly [-1,1] for fusion input.
std::array<float, kSensorDim> normalizeSensors(const std::array<float, kSensorDim>& s) {
  return {s[0] / 180.f, s[1] / 180.f, s[2] / 180.f,
          s[3] / 10.f,  s[4] / 10.f,  s[5] / 10.f,
          s[6] / 100.f, s[7] / 100.f, s[8] / 100.f};
}

}  // namespace

std::vector<std::string> findSessionDirs(const std::string& root) {
  std::vector<std::string> dirs;
  if (!fs::is_directory(root)) return dirs;
  for (const auto& e : fs::directory_iterator(root)) {
    if (e.is_directory() && e.path().filename().string().rfind("session_", 0) == 0)
      dirs.push_back(e.path().string());
  }
  std::sort(dirs.begin(), dirs.end());  // chronological (timestamped names)
  return dirs;
}

std::vector<Sample> scanSessions(const std::vector<std::string>& sessionDirs,
                                 bool skipStopped) {
  std::vector<Sample> out;
  int sessionId = 0;
  for (const auto& dir : sessionDirs) {
    const fs::path csvPath = fs::path(dir) / "labels.csv";
    std::ifstream f(csvPath);
    if (!f) {
      std::cerr << "warn: no labels.csv in " << dir << "\n";
      ++sessionId;
      continue;
    }
    std::string header;
    if (!std::getline(f, header)) {
      ++sessionId;
      continue;
    }
    std::unordered_map<std::string, int> col;
    {
      auto names = splitCsv(header);
      for (int i = 0; i < static_cast<int>(names.size()); ++i) col[names[i]] = i;
    }
    auto idx = [&](const char* name) -> int {
      auto it = col.find(name);
      return it == col.end() ? -1 : it->second;
    };
    const int cLeft = idx("motor_left"), cRight = idx("motor_right"),
              cFile = idx("linetrace_file"), cTs = idx("timestamp");
    if (cLeft < 0 || cRight < 0 || cFile < 0) {
      std::cerr << "warn: missing required columns in " << csvPath << "\n";
      ++sessionId;
      continue;
    }
    const int cYaw = idx("yaw"), cRoll = idx("roll"), cPitch = idx("pitch"),
              cAx = idx("acc_x"), cAy = idx("acc_y"), cAz = idx("acc_z"),
              cUl = idx("usonic_l"), cUm = idx("usonic_m"), cUr = idx("usonic_r");

    std::string line;
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      auto v = splitCsv(line);
      const int need = std::max({cLeft, cRight, cFile});
      if (static_cast<int>(v.size()) <= need) continue;
      const std::string file = v[cFile];
      if (file.empty()) continue;

      Sample s;
      s.cmd.left = clampMotor(static_cast<int>(parseFloatOr(v[cLeft], kMotorStop)));
      s.cmd.right = clampMotor(static_cast<int>(parseFloatOr(v[cRight], kMotorStop)));
      if (skipStopped && s.cmd.left == kMotorStop && s.cmd.right == kMotorStop) continue;

      auto sens = [&](int c) {
        return (c >= 0 && c < static_cast<int>(v.size())) ? parseFloatOr(v[c], 0.f) : 0.f;
      };
      s.sensors = {sens(cYaw),  sens(cRoll), sens(cPitch), sens(cAx), sens(cAy),
                   sens(cAz),   sens(cUl),   sens(cUm),     sens(cUr)};
      s.imagePath = (fs::path(dir) / "linetrace" / file).string();
      s.sessionId = sessionId;
      if (cTs >= 0 && cTs < static_cast<int>(v.size()) && !v[cTs].empty()) {
        try {
          s.ts = std::stod(v[cTs]);  // double: epoch seconds exceed float precision
        } catch (...) {
        }
      }
      out.push_back(std::move(s));
    }
    ++sessionId;
  }
  return out;
}

LineDataset::LineDataset(std::vector<Sample> samples, DatasetConfig cfg)
    : samples_(std::move(samples)), cfg_(cfg) {
  if (cfg_.seqLen > 1) {
    // A clip ending at i is valid if the previous seqLen-1 samples are from the
    // same session with no recording gap (skipStopped and REC pauses create
    // jumps; 1s at the 10 Hz record rate tolerates a few dropped frames).
    constexpr double kMaxGapSec = 1.0;
    for (size_t i = cfg_.seqLen - 1; i < samples_.size(); ++i) {
      bool ok = true;
      for (size_t j = i - cfg_.seqLen + 2; j <= i && ok; ++j) {
        ok = samples_[j].sessionId == samples_[j - 1].sessionId &&
             (samples_[j].ts <= 0 || samples_[j - 1].ts <= 0 ||
              (samples_[j].ts - samples_[j - 1].ts) < kMaxGapSec);
      }
      if (ok) clipEnds_.push_back(i);
    }
  }
}

namespace {
cv::Mat readFrameOr(const std::string& path, int h, int w) {
  cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
  // Missing/corrupt frame: feed a black image so the batch shape is preserved.
  if (img.empty()) img = cv::Mat::zeros(h, w, CV_8UC3);
  return img;
}
}  // namespace

torch::data::Example<> LineDataset::get(size_t index) {
  if (cfg_.seqLen > 1) {
    const size_t end = clipEnds_[index];
    std::vector<cv::Mat> frames;
    frames.reserve(cfg_.seqLen);
    for (size_t i = end - cfg_.seqLen + 1; i <= end; ++i)
      frames.push_back(readFrameOr(samples_[i].imagePath, cfg_.preproc.h, cfg_.preproc.w));

    MotorCmd cmd = samples_[end].cmd;
    if (cfg_.augment) {
      augmentClipBrightness(frames);
      maybeFlipClip(frames, cmd);
      augmentClipNoise(frames);  // after brightness so noise isn't scaled by it
    }

    std::vector<torch::Tensor> ts;
    ts.reserve(frames.size());
    for (auto& f : frames) ts.push_back(preprocess(f, cfg_.preproc, /*addBatch=*/false));
    torch::Tensor data = torch::stack(ts);  // {T,C,H,W}

    const auto y = encode(cmd, cfg_.targetSpace);
    return {data, torch::tensor({y[0], y[1]}, torch::kFloat32)};
  }

  const Sample& s = samples_[index];
  cv::Mat img = readFrameOr(s.imagePath, cfg_.preproc.h, cfg_.preproc.w);

  MotorCmd cmd = s.cmd;
  if (cfg_.augment) {
    augmentBrightness(img);
    maybeFlip(img, cmd);
    augmentNoise(img);  // after brightness so noise isn't scaled by it
  }

  torch::Tensor data = preprocess(img, cfg_.preproc, /*addBatch=*/false);

  const auto y = encode(cmd, cfg_.targetSpace);
  torch::Tensor label = torch::tensor({y[0], y[1]}, torch::kFloat32);

  if (cfg_.useSensors) {
    auto ns = normalizeSensors(s.sensors);
    torch::Tensor sensors = torch::from_blob(ns.data(), {kSensorDim}, torch::kFloat32).clone();
    return {data, torch::cat({label, sensors})};  // target tail carries sensors
  }
  return {data, label};
}

}  // namespace rc
