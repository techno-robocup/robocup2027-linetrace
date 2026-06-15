// Interactive ESP32 link probe (C++ counterpart of the 2026 uart_raw.py).
//
//   uart_probe [port] [baud]
//
// Type a command (e.g. "MOTOR 2000 -2000", "GET bno", "healthcheck"); it is sent
// as "<id> <command>" and the matching reply is printed. Ctrl-D to quit.
#include <iostream>
#include <string>

#include "esp32_driver/port_scan.h"
#include "esp32_driver/protocol.h"
#include "esp32_driver/serial_port.h"

using namespace rc::esp32;

int main(int argc, char** argv) {
  std::string port = argc > 1 ? argv[1] : autodetectPort();
  const int baud = argc > 2 ? std::stoi(argv[2]) : 115200;
  if (port.empty()) {
    std::cerr << "no serial port found (pass one explicitly)\n";
    return 1;
  }
  SerialPort sp;
  if (!sp.open(port, baud)) {
    std::cerr << "failed to open " << port << "\n";
    return 1;
  }
  std::cerr << "connected to " << port << " @ " << baud << " baud\n"
            << "examples: MOTOR 2000 -2000 | GET bno | GET usonic | GET button | healthcheck\n";

  long long id = 1;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    if (!sp.writeLine(buildFrame(id, line))) {
      std::cerr << "write failed\n";
      return 1;
    }
    // Collect replies until the matching id arrives or we time out.
    std::string resp;
    bool matched = false;
    while (sp.readLine(resp, 1000)) {
      long long rid = 0;
      std::string payload;
      if (!parseResponse(resp, rid, payload)) continue;
      if (rid < id) continue;
      std::cout << "<= [" << rid << "] " << payload << "\n";
      matched = true;
      break;
    }
    if (!matched) std::cout << "(timeout, no reply for id " << id << ")\n";
    ++id;
  }
  return 0;
}
