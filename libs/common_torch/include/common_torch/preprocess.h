// Image -> tensor preprocessing shared by the trainer and the executor so the
// model always sees identically-prepared input. This is the single contract
// that keeps "trained on" and "run on" in sync.
#pragma once

#include <opencv2/core.hpp>
#include <torch/torch.h>

#include "common/config.h"

namespace rc {

struct PreprocConfig {
  int w = kModelW;
  int h = kModelH;
  float cropRatio = kCropRatio;
  bool rotate180 = false;  // executor: true; trainer: false (stored frames already oriented)
  Channels channels = Channels::BGR;
};

// Returns a float tensor in [0,1], shape {C,H,W} (addBatch=false) or {1,C,H,W}.
// The returned tensor owns its memory (the cv::Mat may be freed afterwards).
torch::Tensor preprocess(const cv::Mat& bgr, const PreprocConfig& cfg, bool addBatch = true);

}  // namespace rc
