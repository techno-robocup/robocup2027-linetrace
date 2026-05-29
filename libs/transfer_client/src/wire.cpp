#include "transfer_client/wire.h"

#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace rc::transfer {

int connectTcp(const std::string& host, int port) {
  struct addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo* res = nullptr;
  const std::string portStr = std::to_string(port);
  const int status = getaddrinfo(host.c_str(), portStr.c_str(), &hints, &res);
  if (status != 0) {
    std::cerr << "getaddrinfo failed: " << gai_strerror(status) << "\n";
    return -1;
  }

  int fd = -1;
  for (struct addrinfo* p = res; p != nullptr; p = p->ai_next) {
    fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (fd < 0) continue;
    if (connect(fd, p->ai_addr, p->ai_addrlen) == 0) break;
    close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  return fd;
}

bool sendAll(int fd, const void* data, size_t len) {
  const char* p = static_cast<const char*>(data);
  size_t sent = 0;
  while (sent < len) {
    const ssize_t n = send(fd, p + sent, len - sent, 0);
    if (n <= 0) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool recvAll(int fd, void* data, size_t len) {
  char* p = static_cast<char*>(data);
  size_t got = 0;
  while (got < len) {
    const ssize_t n = recv(fd, p + got, len - got, 0);
    if (n <= 0) return false;
    got += static_cast<size_t>(n);
  }
  return true;
}

bool sendLine(int fd, const std::string& line) {
  std::string msg = line;
  msg.push_back('\n');
  return sendAll(fd, msg.data(), msg.size());
}

bool recvLine(int fd, std::string& out) {
  out.clear();
  char c = '\0';
  while (true) {
    const ssize_t n = recv(fd, &c, 1, 0);
    if (n <= 0) return false;
    if (c == '\n') return true;
    out.push_back(c);
    if (out.size() > 8192) return false;
  }
}

void closeFd(int fd) {
  if (fd >= 0) close(fd);
}

}  // namespace rc::transfer
