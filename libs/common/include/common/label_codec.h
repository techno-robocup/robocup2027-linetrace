// Invertible conversions between motor commands (PWM microseconds) and the
// normalized [0,1] targets a sigmoid-headed network emits.
//
// Two target spaces are supported:
//   * LR            : the two outputs are (left, right) directly.
//   * ThrottleSteer : the two outputs are (throttle, steer), a decorrelated
//                     reparameterization that makes horizontal-flip
//                     augmentation exact (flip <=> negate steer).
//
// Header-only and dependency-free so the laptop control station can use it
// without pulling in LibTorch.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "common/config.h"

namespace rc {

struct MotorCmd {
  int left = kMotorStop;
  int right = kMotorStop;
};

enum class TargetSpace { LR, ThrottleSteer };

inline int clampMotor(int v) { return std::max(kMotorMin, std::min(kMotorMax, v)); }

inline float clamp01(float v) { return std::max(0.0f, std::min(1.0f, v)); }

// PWM (1000..2000) -> normalized [0,1].
inline float pwmToNorm(int pwm) {
  return clamp01(static_cast<float>(clampMotor(pwm) - kMotorMin) /
                 static_cast<float>(kMotorMax - kMotorMin));
}

// normalized [0,1] -> PWM (1000..2000).
inline int normToPwm(float n) {
  return clampMotor(static_cast<int>(std::lround(
      kMotorMin + clamp01(n) * static_cast<float>(kMotorMax - kMotorMin))));
}

// --- (left,right) target space ---
inline std::array<float, 2> encodeLR(const MotorCmd& m) {
  return {pwmToNorm(m.left), pwmToNorm(m.right)};
}
inline MotorCmd decodeLR(const std::array<float, 2>& y) {
  return {normToPwm(y[0]), normToPwm(y[1])};
}

// --- (throttle,steer) target space ---
// throttle/steer live in [-kMotorSpan, kMotorSpan] then map to [0,1].
inline std::array<float, 2> encodeTS(const MotorCmd& m) {
  const float lOff = static_cast<float>(clampMotor(m.left) - kMotorStop);
  const float rOff = static_cast<float>(clampMotor(m.right) - kMotorStop);
  const float throttle = 0.5f * (lOff + rOff);  // [-span, span]
  const float steer = 0.5f * (rOff - lOff);      // [-span, span] (>0 => left faster)
  const float s = static_cast<float>(kMotorSpan);
  return {clamp01(0.5f * (throttle / s + 1.0f)), clamp01(0.5f * (steer / s + 1.0f))};
}
inline MotorCmd decodeTS(const std::array<float, 2>& y) {
  const float s = static_cast<float>(kMotorSpan);
  const float throttle = (2.0f * clamp01(y[0]) - 1.0f) * s;
  const float steer = (2.0f * clamp01(y[1]) - 1.0f) * s;
  const int lOff = static_cast<int>(std::lround(throttle - steer));
  const int rOff = static_cast<int>(std::lround(throttle + steer));
  return {clampMotor(kMotorStop + lOff), clampMotor(kMotorStop + rOff)};
}

inline std::array<float, 2> encode(const MotorCmd& m, TargetSpace space) {
  return space == TargetSpace::LR ? encodeLR(m) : encodeTS(m);
}
inline MotorCmd decode(const std::array<float, 2>& y, TargetSpace space) {
  return space == TargetSpace::LR ? decodeLR(y) : decodeTS(y);
}

}  // namespace rc
