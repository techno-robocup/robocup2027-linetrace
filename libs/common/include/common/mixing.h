// Gamepad-axis -> motor-command mixing, shared by the control station (which
// sends the command) and any on-robot mixer so they always agree.
//
// Arcade drive: a throttle axis and a steer axis combine into a differential
// (left,right) pair. Header-only and dependency-free.
#pragma once

#include <algorithm>
#include <cmath>

#include "common/config.h"
#include "common/label_codec.h"

namespace rc {

// throttle, steer in [-1, 1]:
//   throttle > 0 => forward, steer > 0 => turn right (left wheel faster).
// gain is the maximum offset from stop applied to either wheel (<= kMotorSpan).
inline MotorCmd mixArcade(float throttle, float steer, float gain = kMotorSpan) {
  throttle = std::max(-1.0f, std::min(1.0f, throttle));
  steer = std::max(-1.0f, std::min(1.0f, steer));
  gain = std::min<float>(gain, static_cast<float>(kMotorSpan));

  float l = throttle + steer;  // left wheel offset (normalized, may exceed 1)
  float r = throttle - steer;  // right wheel offset

  // Scale the pair down together so a "fast forward + turn" doesn't saturate
  // into a pivot; preserves the throttle:steer ratio.
  const float peak = std::max(std::fabs(l), std::fabs(r));
  if (peak > 1.0f) {
    l /= peak;
    r /= peak;
  }

  MotorCmd cmd;
  cmd.left = clampMotor(kMotorStop + static_cast<int>(std::lround(l * gain)));
  cmd.right = clampMotor(kMotorStop + static_cast<int>(std::lround(r * gain)));
  return cmd;
}

// Apply a symmetric dead zone to a raw stick axis in [-1, 1].
inline float applyDeadzone(float axis, float dz = 0.08f) {
  if (std::fabs(axis) < dz) return 0.0f;
  const float sign = axis < 0 ? -1.0f : 1.0f;
  return sign * (std::fabs(axis) - dz) / (1.0f - dz);
}

}  // namespace rc
