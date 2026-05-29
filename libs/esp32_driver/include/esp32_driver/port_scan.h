#pragma once

#include <string>
#include <vector>

namespace rc::esp32 {

// Returns candidate serial device paths, preferring /dev/ttyUSB* then
// /dev/ttyACM* (matches the 2026 robot.py auto-detection order).
std::vector<std::string> scanSerialPorts();

// First candidate, or "" if none found.
std::string autodetectPort();

}  // namespace rc::esp32
