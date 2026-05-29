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
              cFile = idx("linetrace_file");
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
      out.push_back(std::move(s));
    }
    ++sessionId;
  }
  return out;
}

LineDataset::LineDataset(std::vector<Sample> samples, DatasetConfig cfg)
    : samples_(std::move(samples)), cfg_(cfg) {}

torch::data::Example<> LineDataset::get(size_t index) {
  const Sample& s = samples_[index];
  cv::Mat img = cv::imread(s.imagePath, cv::IMREAD_COLOR);
  if (img.empty()) {
    // Missing/corrupt frame: feed a black image so the batch shape is preserved.
    img = cv::Mat::zeros(cfg_.preproc.h, cfg_.preproc.w, CV_8UC3);
  }

  MotorCmd cmd = s.cmd;
  if (cfg_.augment) {
    augmentBrightness(img);
    maybeFlip(img, cmd);
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
