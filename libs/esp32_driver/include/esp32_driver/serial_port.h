// Minimal blocking serial port (termios, raw mode) with a line reader that
// honours a millisecond timeout via poll().
#pragma once

#include <string>

namespace rc::esp32 {

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  bool open(const std::string& path, int baud);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  bool writeLine(const std::string& line);              // appends '\n'
  bool readLine(std::string& out, int timeoutMs);       // up to '\n' (stripped)

 private:
  int fd_ = -1;
  std::string rxbuf_;
};

}  // namespace rc::esp32
