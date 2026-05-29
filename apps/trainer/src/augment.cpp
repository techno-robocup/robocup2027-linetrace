#include "augment.h"

#include <random>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace rc {

namespace {
// Augmentation randomness need not be reproducible; a per-thread RNG keeps
// concurrent DataLoader workers race-free.
std::mt19937& rng() {
  thread_local std::mt19937 gen(std::random_device{}());
  return gen;
}
}  // namespace

void augmentBrightness(cv::Mat& bgr) {
  std::uniform_real_distribution<float> dist(0.6f, 1.4f);
  bgr.convertTo(bgr, -1, dist(rng()), 0.0);  // saturating scale, clamps to [0,255]
}

void maybeFlip(cv::Mat& bgr, MotorCmd& cmd) {
  std::bernoulli_distribution coin(0.5);
  if (coin(rng())) {
    cv::flip(bgr, bgr, /*flipCode=*/1);  // horizontal
    std::swap(cmd.left, cmd.right);
  }
}

}  // namespace rc
