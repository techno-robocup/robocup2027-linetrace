// Threaded driver for the ESP32 motor/sensor board.
//
// A dedicated I/O thread owns the serial port and performs all (blocking)
// request/response transactions: it sends the latest motor command every
// cycle, polls the button / IMU / ultrasonics in rotation, and reconnects on
// error. Callers interact only through atomics-backed setters/getters, so a
// slow or stalled UART never blocks the camera, control, or recording paths.
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "esp32_driver/protocol.h"
#include "esp32_driver/serial_port.h"

namespace rc::esp32 {

struct Config {
  std::string port;       // empty => auto-detect
  int baud = 115200;
  int timeoutMs = 1000;
  int sensorPollEvery = 4;  // poll one sensor every N motor cycles
  int reconnectDelayMs = 200;
};

class Esp32Driver {
 public:
  explicit Esp32Driver(Config cfg = {});
  ~Esp32Driver();

  bool start();   // opens the port and spawns the I/O thread
  void stop();    // commands neutral, joins the thread
  bool connected() const { return connected_.load(); }

  // --- control-loop-safe (non-blocking) ---
  void setMotors(int left, int right);  // clamped to [1000,2000]
  void rescue(int pos, int wire);        // queued (latest wins)

  // --- cached reads (instant) ---
  Bno bno() const;
  Ultrasonic ultrasonic() const;
  bool button() const;  // true = ON
  std::chrono::steady_clock::time_point lastSensorTime() const;

 private:
  void ioLoop();
  std::optional<std::string> transact(const std::string& payload);  // I/O thread only
  bool ensureConnected();

  Config cfg_;
  SerialPort port_;
  std::thread thread_;
  std::atomic<bool> run_{false};
  std::atomic<bool> connected_{false};
  long long nextId_ = 1;

  std::atomic<int> left_{1500};
  std::atomic<int> right_{1500};

  std::mutex cmdMx_;
  std::optional<std::pair<int, int>> pendingRescue_;

  mutable std::mutex cacheMx_;
  Bno bno_;
  Ultrasonic uson_;
  bool button_ = false;
  std::chrono::steady_clock::time_point sensorTime_{};
};

}  // namespace rc::esp32
