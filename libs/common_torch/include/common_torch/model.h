// LineNet: the line-trace CNN, shared by the trainer (defines + trains it) and
// the executor (reconstructs the identical module and torch::load's the
// weights). Keeping one definition is what lets us avoid TorchScript export and
// stay fully C++.
//
// Architecture: NVIDIA PilotNet backbone (5 conv + 4 fc), sigmoid head emitting
// two values in [0,1] (interpreted by label_codec as (left,right) or
// (throttle,steer)). An optional sensor-fusion branch concatenates an IMU /
// ultrasonic feature vector before the fully-connected stack.
//
// Temporal memory (seq_len > 1, GRU over per-frame features) is declared in the
// options for forward-compatibility but not yet implemented; single-frame is
// the pipeline bring-up baseline.
#pragma once

#include <torch/torch.h>

#include "common/config.h"

namespace rc {

struct LineNetOptions {
  int inChannels = 3;
  int inputH = kModelH;
  int inputW = kModelW;
  int sensorDim = 0;       // 0 => no sensor fusion
  float dropout = 0.3f;
  // Temporal memory (not yet implemented):
  bool useMemory = false;
  int seqLen = 1;
};

class LineNetImpl : public torch::nn::Module {
 public:
  explicit LineNetImpl(const LineNetOptions& opt = {});

  // Single-frame forward. x: {N, C, H, W}. Returns {N, 2} in [0,1].
  torch::Tensor forward(torch::Tensor x);

  // Sensor-fusion forward. sensors: {N, sensorDim}.
  torch::Tensor forward(torch::Tensor x, torch::Tensor sensors);

  const LineNetOptions& options() const { return opt_; }

 private:
  torch::Tensor convFeatures(torch::Tensor x);  // conv stack -> flat {N, flatDim}

  LineNetOptions opt_;
  int flatDim_ = 0;

  torch::nn::Conv2d conv1_{nullptr}, conv2_{nullptr}, conv3_{nullptr}, conv4_{nullptr},
      conv5_{nullptr};
  torch::nn::Dropout drop_{nullptr};
  torch::nn::Linear fc1_{nullptr}, fc2_{nullptr}, fc3_{nullptr}, head_{nullptr};
};

TORCH_MODULE(LineNet);

}  // namespace rc
