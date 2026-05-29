// Minimal UDP helpers for the control/preview links.
#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <string>

namespace rc::net {

struct UdpTarget {
  sockaddr_in addr{};
  bool resolve(const std::string& host, int port);  // host may be IP or name
};

int openUdpSend();                     // unconnected sender socket, or -1
int openUdpRecv(int port);             // bound to 0.0.0.0:port, or -1
bool sendTo(int fd, const UdpTarget& t, const void* data, size_t len);

// Returns bytes received (>0), 0 on timeout, -1 on error.
int recvWithTimeout(int fd, void* buf, size_t len, int timeoutMs);

void closeFd(int fd);

}  // namespace rc::net
