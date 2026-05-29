#include "esp32_driver/port_scan.h"

#include <glob.h>

namespace rc::esp32 {

namespace {
void globInto(const char* pattern, std::vector<std::string>& out) {
  glob_t g;
  if (glob(pattern, 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i) out.emplace_back(g.gl_pathv[i]);
  }
  globfree(&g);
}
}  // namespace

std::vector<std::string> scanSerialPorts() {
  std::vector<std::string> out;
  globInto("/dev/ttyUSB*", out);
  globInto("/dev/ttyACM*", out);
  return out;
}

std::string autodetectPort() {
  auto ports = scanSerialPorts();
  return ports.empty() ? std::string{} : ports.front();
}

}  // namespace rc::esp32
