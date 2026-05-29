#include "esp32_driver/esp32_driver.h"

#include <algorithm>
#include <iostream>

#include "esp32_driver/port_scan.h"

namespace rc::esp32 {

namespace {
int clampMotor(int v) { return std::max(1000, std::min(2000, v)); }
}  // namespace

Esp32Driver::Esp32Driver(Config cfg) : cfg_(std::move(cfg)) {}

Esp32Driver::~Esp32Driver() { stop(); }

bool Esp32Driver::start() {
  if (run_.load()) return true;
  run_.store(true);
  thread_ = std::thread([this] { ioLoop(); });
  return true;
}

void Esp32Driver::stop() {
  if (!run_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void Esp32Driver::setMotors(int left, int right) {
  left_.store(clampMotor(left));
  right_.store(clampMotor(right));
}

void Esp32Driver::rescue(int pos, int wire) {
  std::lock_guard<std::mutex> lk(cmdMx_);
  pendingRescue_ = std::make_pair(pos, wire);
}

Bno Esp32Driver::bno() const {
  std::lock_guard<std::mutex> lk(cacheMx_);
  return bno_;
}
Ultrasonic Esp32Driver::ultrasonic() const {
  std::lock_guard<std::mutex> lk(cacheMx_);
  return uson_;
}
bool Esp32Driver::button() const {
  std::lock_guard<std::mutex> lk(cacheMx_);
  return button_;
}
std::chrono::steady_clock::time_point Esp32Driver::lastSensorTime() const {
  std::lock_guard<std::mutex> lk(cacheMx_);
  return sensorTime_;
}

std::optional<std::string> Esp32Driver::transact(const std::string& payload) {
  const long long id = nextId_++;
  if (!port_.writeLine(buildFrame(id, payload))) return std::nullopt;

  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(cfg_.timeoutMs);
  while (clock::now() < deadline) {
    const int remain = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now())
            .count());
    std::string line;
    if (!port_.readLine(line, std::max(1, remain))) return std::nullopt;
    long long rid = 0;
    std::string resp;
    if (!parseResponse(line, rid, resp)) continue;  // skip noise
    if (rid < id) continue;                          // stale reply
    if (rid > id) return std::nullopt;               // out of sync -> reconnect
    return resp;
  }
  return std::nullopt;
}

bool Esp32Driver::ensureConnected() {
  if (connected_.load() && port_.isOpen()) return true;

  const std::string path = cfg_.port.empty() ? autodetectPort() : cfg_.port;
  if (path.empty()) return false;
  if (!port_.open(path, cfg_.baud)) return false;

  // Confirm the link with a healthcheck (mirrors robot.py reconnect logic).
  auto resp = transact("healthcheck");
  if (resp && *resp == "OK") {
    connected_.store(true);
    std::cerr << "esp32: connected on " << path << "\n";
    return true;
  }
  port_.close();
  return false;
}

void Esp32Driver::ioLoop() {
  int cycle = 0;
  while (run_.load()) {
    if (!ensureConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnectDelayMs));
      continue;
    }

    // 1) Always push the latest motor command (keeps the 500 ms ESP32 watchdog fed).
    if (!transact("MOTOR " + std::to_string(left_.load()) + " " +
                  std::to_string(right_.load()))) {
      connected_.store(false);
      continue;
    }

    // 2) Flush a pending rescue command, if any.
    std::optional<std::pair<int, int>> rescueCmd;
    {
      std::lock_guard<std::mutex> lk(cmdMx_);
      rescueCmd.swap(pendingRescue_);
    }
    if (rescueCmd) {
      if (!transact("Rescue " + std::to_string(rescueCmd->first) + " " +
                    std::to_string(rescueCmd->second))) {
        connected_.store(false);
        continue;
      }
    }

    // 3) Poll one sensor in rotation every sensorPollEvery cycles.
    if (cfg_.sensorPollEvery > 0 && cycle % cfg_.sensorPollEvery == 0) {
      const int which = (cycle / cfg_.sensorPollEvery) % 3;
      bool ok = true;
      if (which == 0) {
        if (auto r = transact("GET button")) {
          bool b;
          if (parseButton(*r, b)) {
            std::lock_guard<std::mutex> lk(cacheMx_);
            button_ = b;
            sensorTime_ = std::chrono::steady_clock::now();
          }
        } else ok = false;
      } else if (which == 1) {
        if (auto r = transact("GET bno")) {
          Bno b;
          if (parseBno(*r, b)) {
            std::lock_guard<std::mutex> lk(cacheMx_);
            bno_ = b;
            sensorTime_ = std::chrono::steady_clock::now();
          }
        } else ok = false;
      } else {
        if (auto r = transact("GET usonic")) {
          Ultrasonic u;
          if (parseUltrasonic(*r, u)) {
            std::lock_guard<std::mutex> lk(cacheMx_);
            uson_ = u;
            sensorTime_ = std::chrono::steady_clock::now();
          }
        } else ok = false;
      }
      if (!ok) {
        connected_.store(false);
        continue;
      }
    }
    ++cycle;
  }

  // On shutdown, best-effort neutral.
  if (port_.isOpen()) transact("MOTOR 1500 1500");
  port_.close();
  connected_.store(false);
}

}  // namespace rc::esp32
