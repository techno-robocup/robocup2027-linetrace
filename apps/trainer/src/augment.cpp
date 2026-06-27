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

void augmentNoise(cv::Mat& bgr) {
  std::uniform_real_distribution<float> dist(0.0f, 12.0f);  // stddev on the 8-bit scale
  const double sigma = dist(rng());
  if (sigma < 0.5) return;  // negligible noise: skip the work
  // Promote to signed 16-bit so negative noise doesn't wrap the unsigned input,
  // then saturate back to 8-bit. cv::randn fills via the thread-local cv::theRNG,
  // so concurrent DataLoader workers stay race-free (like rng() above).
  cv::Mat wide;
  bgr.convertTo(wide, CV_16S);
  cv::Mat noise(wide.size(), wide.type());
  cv::randn(noise, cv::Scalar::all(0.0), cv::Scalar::all(sigma));
  wide += noise;
  wide.convertTo(bgr, CV_8U);  // saturating cast clamps to [0,255]
}

void maybeFlip(cv::Mat& bgr, MotorCmd& cmd) {
  std::bernoulli_distribution coin(0.5);
  if (coin(rng())) {
    cv::flip(bgr, bgr, /*flipCode=*/1);  // horizontal
    std::swap(cmd.left, cmd.right);
  }
}

void augmentClipBrightness(std::vector<cv::Mat>& frames) {
  std::uniform_real_distribution<float> dist(0.6f, 1.4f);
  const float k = dist(rng());
  for (auto& f : frames) f.convertTo(f, -1, k, 0.0);
}

void maybeFlipClip(std::vector<cv::Mat>& frames, MotorCmd& cmd) {
  std::bernoulli_distribution coin(0.5);
  if (!coin(rng())) return;
  for (auto& f : frames) cv::flip(f, f, /*flipCode=*/1);
  std::swap(cmd.left, cmd.right);
}

void augmentClipNoise(std::vector<cv::Mat>& frames) {
  for (auto& f : frames) augmentNoise(f);  // independent draw per frame
}

}  // namespace rc
