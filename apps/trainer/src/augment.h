// Label-aware data augmentation for line-trace training. Operates on the BGR
// cv::Mat before preprocessing.
#pragma once

#include <opencv2/core.hpp>

#include "common/label_codec.h"

namespace rc {

// Random brightness scaling in [0.6, 1.4] (matches the 2026 prototype).
void augmentBrightness(cv::Mat& bgr);

// With probability 0.5, horizontally flip the frame and swap the motor command
// (left<->right). Swapping the command is correct for both target spaces:
// in ThrottleSteer it negates steer while preserving throttle.
void maybeFlip(cv::Mat& bgr, MotorCmd& cmd);

}  // namespace rc
