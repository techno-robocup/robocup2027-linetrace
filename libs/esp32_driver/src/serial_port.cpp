#include "esp32_driver/serial_port.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>

namespace rc::esp32 {

namespace {
// Maps an integer baud to the matching termios constant. Only the rates we use.
speed_t toSpeed(int baud) {
  switch (baud) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}
}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& path, int baud) {
  close();
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "serial open(" << path << ") failed: " << std::strerror(errno) << "\n";
    return false;
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    std::cerr << "tcgetattr failed: " << std::strerror(errno) << "\n";
    close();
    return false;
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, toSpeed(baud));
  cfsetospeed(&tio, toSpeed(baud));
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;             // no hardware flow control
  tio.c_cflag &= ~CSTOPB;              // 1 stop bit
  tio.c_cflag &= ~PARENB;              // no parity
  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
  tio.c_cc[VMIN] = 0;                  // non-blocking reads; we gate with poll()
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    std::cerr << "tcsetattr failed: " << std::strerror(errno) << "\n";
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  rxbuf_.clear();
  return true;
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rxbuf_.clear();
}

bool SerialPort::writeLine(const std::string& line) {
  if (fd_ < 0) return false;
  std::string msg = line;
  msg.push_back('\n');
  size_t sent = 0;
  while (sent < msg.size()) {
    const ssize_t n = ::write(fd_, msg.data() + sent, msg.size() - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool SerialPort::readLine(std::string& out, int timeoutMs) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeoutMs);

  while (true) {
    // Emit a complete line already buffered.
    const size_t nl = rxbuf_.find('\n');
    if (nl != std::string::npos) {
      out = rxbuf_.substr(0, nl);
      rxbuf_.erase(0, nl + 1);
      if (!out.empty() && out.back() == '\r') out.pop_back();
      return true;
    }

    const auto now = clock::now();
    if (now >= deadline) return false;
    const int remainMs = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

    pollfd pfd{fd_, POLLIN, 0};
    const int pr = ::poll(&pfd, 1, remainMs);
    if (pr < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (pr == 0) return false;  // timeout

    char chunk[512];
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      return false;
    }
    if (n == 0) return false;  // closed
    rxbuf_.append(chunk, static_cast<size_t>(n));
  }
}

}  // namespace rc::esp32
