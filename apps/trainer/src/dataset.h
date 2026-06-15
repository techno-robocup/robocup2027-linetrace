// Dataset for the line-trace trainer: reads the 2026/2027 labels.csv schema and
// the per-session linetrace JPEGs, lazily decoding images per access.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <torch/torch.h>

#include "common/label_codec.h"
#include "common_torch/preprocess.h"

namespace rc {

// kSensorDim (9) and the sensor column order are defined in common/config.h.

struct Sample {
  std::string imagePath;
  MotorCmd cmd;
  std::array<float, kSensorDim> sensors{};
  int sessionId = 0;
  double ts = 0.0;  // labels.csv timestamp (seconds); 0 if absent
};

struct DatasetConfig {
  PreprocConfig preproc;
  TargetSpace targetSpace = TargetSpace::LR;
  bool augment = false;     // enable on the training split only
  bool useSensors = false;  // include the sensor vector in each Example's target tail
  int seqLen = 1;           // >1: temporal clips ending at each sample (memory training)
  uint64_t seed = 1234;
};

// Scans session directories, parses each labels.csv (header-driven, tolerant of
// extra/missing columns), and returns the flat sample list. Rows with an empty
// linetrace_file are skipped. If skipStopped, drops neutral (0/0) frames.
std::vector<Sample> scanSessions(const std::vector<std::string>& sessionDirs,
                                 bool skipStopped);

// Finds session_* subdirectories under a dataset root.
std::vector<std::string> findSessionDirs(const std::string& root);

// With cfg.seqLen == 1, one example per sample: image {C,H,W} -> label {2}.
// With cfg.seqLen > 1, one example per *valid clip end*: a clip of seqLen
// consecutive frames (same session, no recording gap) {T,C,H,W} -> the last
// frame's label. Augmentation decisions are made once per clip.
class LineDataset : public torch::data::Dataset<LineDataset> {
 public:
  LineDataset(std::vector<Sample> samples, DatasetConfig cfg);

  torch::data::Example<> get(size_t index) override;
  torch::optional<size_t> size() const override {
    return cfg_.seqLen > 1 ? clipEnds_.size() : samples_.size();
  }

  const DatasetConfig& config() const { return cfg_; }
  const std::vector<Sample>& samples() const { return samples_; }

 private:
  std::vector<Sample> samples_;
  std::vector<size_t> clipEnds_;  // indices of samples that end a valid clip
  DatasetConfig cfg_;
};

}  // namespace rc
