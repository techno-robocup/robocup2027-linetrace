// Lock-free-ish shared state between the collector's threads (camera callback,
// control receiver, ESP32 I/O, dataset writer, preview sender).
#pragma once

#include <atomic>
#include <cstdint>

#include "common/config.h"

namespace rc {

struct SharedState {
  // Latest control command (from the station, or failsafe neutral).
  std::atomic<int> left{kMotorStop};
  std::atomic<int> right{kMotorStop};
  std::atomic<uint32_t> ctrlSeq{0};
  std::atomic<uint64_t> ctrlTsMs{0};        // station clock at send
  std::atomic<int64_t> lastRxMonoNs{0};     // robot monotonic at receive

  std::atomic<bool> recording{false};
  std::atomic<bool> estop{false};
  std::atomic<bool> linkLost{true};

  std::atomic<uint64_t> framesWritten{0};
  std::atomic<uint64_t> framesDropped{0};
};

}  // namespace rc
