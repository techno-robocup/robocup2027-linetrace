// Loads a trained LineNet (torch::load) and runs it on a single image, printing
// the predicted motor command. This exercises exactly the executor's
// load+infer path, so it verifies train -> save -> load -> infer on the PC
// without any camera.
//
//   model_probe <model.pt> <image.jpg> [--grayscale] [--sensors] [--rotate] [--ts]
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <torch/torch.h>

#include "common/label_codec.h"
#include "common_torch/model.h"
#include "common_torch/preprocess.h"

using namespace rc;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: model_probe <model.pt> <image.jpg> "
                 "[--grayscale] [--sensors] [--memory] [--rotate] [--ts]\n";
    return 2;
  }
  const std::string modelPath = argv[1];
  const std::string imagePath = argv[2];
  bool grayscale = false, sensors = false, memory = false, rotate = false;
  TargetSpace space = TargetSpace::LR;
  for (int i = 3; i < argc; ++i) {
    std::string s = argv[i];
    if (s == "--grayscale") grayscale = true;
    else if (s == "--sensors") sensors = true;
    else if (s == "--memory") memory = true;
    else if (s == "--rotate") rotate = true;
    else if (s == "--ts") space = TargetSpace::ThrottleSteer;
  }

  LineNetOptions opt;
  opt.inChannels = grayscale ? 1 : 3;
  opt.sensorDim = sensors ? kSensorDim : 0;
  opt.useMemory = memory;  // single-frame probe runs one step from zero state
  LineNet net(opt);
  try {
    torch::load(net, modelPath);
  } catch (const std::exception& e) {
    std::cerr << "load failed: " << e.what() << "\n";
    return 1;
  }
  net->eval();

  cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);
  if (img.empty()) {
    std::cerr << "cannot read image: " << imagePath << "\n";
    return 1;
  }
  PreprocConfig pc;
  pc.channels = grayscale ? Channels::Gray : Channels::BGR;
  pc.rotate180 = rotate;
  torch::Tensor x = preprocess(img, pc);

  torch::NoGradGuard ng;
  torch::Tensor y;
  if (sensors) {
    y = net->forward(x, torch::zeros({1L, static_cast<long>(kSensorDim)}));
  } else {
    y = net->forward(x);
  }
  const float a0 = y[0][0].item<float>();
  const float a1 = y[0][1].item<float>();
  MotorCmd cmd = decode({a0, a1}, space);
  std::cout << "raw output: [" << a0 << ", " << a1 << "]\n";
  std::cout << "motor: left=" << cmd.left << " right=" << cmd.right << "\n";
  return 0;
}
