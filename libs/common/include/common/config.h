// Shared configuration constants for the RoboCup 2027 line-trace system.
//
// These values are the single source of truth shared by the trainer, the
// executor and the collector so that preprocessing and label handling can
// never drift between "what the model was trained on" and "what the robot
// feeds the model at run time".
#pragma once

namespace rc {

// --- Motor command range (matches the ESP32 firmware / 2026 program) ---
// Each wheel command is a servo-style pulse width in microseconds.
//   1500 = stop, 2000 = full forward, 1000 = full reverse (natural convention;
//   the ESP32 firmware applies its own right-wheel inversion internally).
inline constexpr int kMotorMin = 1000;
inline constexpr int kMotorMax = 2000;
inline constexpr int kMotorStop = 1500;
inline constexpr int kMotorSpan = (kMotorMax - kMotorMin) / 2;  // 500 either side of stop

// --- Model input geometry (matches the 2026 prototype's model_info.json) ---
inline constexpr int kModelW = 160;
inline constexpr int kModelH = 90;

// Center-width crop applied before resize (matches 2026 LINETRACE_CROP_WIDTH_RATIO).
// 1.0 disables cropping.
inline constexpr float kCropRatio = 0.6f;

// The camera is mounted upside-down on the 2026 chassis, so frames are rotated
// 180 degrees before processing. The collector stores already-rotated frames,
// so the trainer leaves this off and the executor turns it on.
inline constexpr bool kRotate180Default = true;

// Channel order used end-to-end. OpenCV's imread / libcamera buffers are BGR,
// so we keep BGR everywhere to avoid needless conversions.
enum class Channels { BGR, Gray };

// Sensor vector recorded per frame and optionally fed to the model, in a fixed
// order: yaw, roll, pitch, acc_x, acc_y, acc_z, usonic_l, usonic_m, usonic_r.
inline constexpr int kSensorDim = 9;

}  // namespace rc
