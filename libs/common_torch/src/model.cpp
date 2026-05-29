#include "common_torch/model.h"

namespace rc {

namespace nn = torch::nn;

LineNetImpl::LineNetImpl(const LineNetOptions& opt) : opt_(opt) {
  // PilotNet conv stack.
  conv1_ = register_module(
      "conv1", nn::Conv2d(nn::Conv2dOptions(opt_.inChannels, 24, 5).stride(2)));
  conv2_ = register_module("conv2", nn::Conv2d(nn::Conv2dOptions(24, 36, 5).stride(2)));
  conv3_ = register_module("conv3", nn::Conv2d(nn::Conv2dOptions(36, 48, 5).stride(2)));
  conv4_ = register_module("conv4", nn::Conv2d(nn::Conv2dOptions(48, 64, 3)));
  conv5_ = register_module("conv5", nn::Conv2d(nn::Conv2dOptions(64, 64, 3)));

  // Determine the flattened conv-output size with a dummy forward (eager mode
  // needs a concrete in_features for the first Linear layer).
  {
    torch::NoGradGuard ng;
    torch::Tensor dummy = torch::zeros({1, opt_.inChannels, opt_.inputH, opt_.inputW});
    flatDim_ = static_cast<int>(convFeatures(dummy).size(1));
  }

  const int fc1In = flatDim_ + opt_.sensorDim;
  drop_ = register_module("drop", nn::Dropout(opt_.dropout));
  fc1_ = register_module("fc1", nn::Linear(fc1In, 100));
  fc2_ = register_module("fc2", nn::Linear(100, 50));
  fc3_ = register_module("fc3", nn::Linear(50, 10));
  head_ = register_module("head", nn::Linear(10, 2));
}

torch::Tensor LineNetImpl::convFeatures(torch::Tensor x) {
  x = torch::relu(conv1_->forward(x));
  x = torch::relu(conv2_->forward(x));
  x = torch::relu(conv3_->forward(x));
  x = torch::relu(conv4_->forward(x));
  x = torch::relu(conv5_->forward(x));
  return x.flatten(1);
}

torch::Tensor LineNetImpl::forward(torch::Tensor x) {
  x = convFeatures(x);
  x = drop_->forward(x);
  x = torch::relu(fc1_->forward(x));
  x = torch::relu(fc2_->forward(x));
  x = torch::relu(fc3_->forward(x));
  return torch::sigmoid(head_->forward(x));
}

torch::Tensor LineNetImpl::forward(torch::Tensor x, torch::Tensor sensors) {
  x = convFeatures(x);
  x = torch::cat({x, sensors}, /*dim=*/1);
  x = drop_->forward(x);
  x = torch::relu(fc1_->forward(x));
  x = torch::relu(fc2_->forward(x));
  x = torch::relu(fc3_->forward(x));
  return torch::sigmoid(head_->forward(x));
}

}  // namespace rc
