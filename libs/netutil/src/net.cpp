#include "netutil/net.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace rc::net {

bool UdpTarget::resolve(const std::string& host, int port) {
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) == 1) return true;

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 || !res) return false;
  addr.sin_addr = reinterpret_cast<sockaddr_in*>(res->ai_addr)->sin_addr;
  freeaddrinfo(res);
  return true;
}

int openUdpSend() { return socket(AF_INET, SOCK_DGRAM, 0); }

int openUdpRecv(int port) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;
  int opt = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

bool sendTo(int fd, const UdpTarget& t, const void* data, size_t len) {
  const ssize_t n = sendto(fd, data, len, 0, reinterpret_cast<const sockaddr*>(&t.addr),
                           sizeof(t.addr));
  return n == static_cast<ssize_t>(len);
}

int recvWithTimeout(int fd, void* buf, size_t len, int timeoutMs) {
  pollfd pfd{fd, POLLIN, 0};
  const int pr = ::poll(&pfd, 1, timeoutMs);
  if (pr < 0) return -1;
  if (pr == 0) return 0;
  const ssize_t n = recv(fd, buf, len, 0);
  return n < 0 ? -1 : static_cast<int>(n);
}

void closeFd(int fd) {
  if (fd >= 0) ::close(fd);
}

}  // namespace rc::net
