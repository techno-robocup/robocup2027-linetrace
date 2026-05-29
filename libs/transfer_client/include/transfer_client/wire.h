// Low-level blocking socket helpers for the transfer protocol. These mirror the
// primitives in robocup2027-transfer-server so the client speaks the exact same
// wire format (newline-delimited text headers + raw binary payload).
#pragma once

#include <cstddef>
#include <string>

namespace rc::transfer {

// Connects to host:port over TCP (IPv4). Returns a socket fd, or -1 on failure.
int connectTcp(const std::string& host, int port);

bool sendAll(int fd, const void* data, size_t len);
bool recvAll(int fd, void* data, size_t len);
bool sendLine(int fd, const std::string& line);   // appends '\n'
bool recvLine(int fd, std::string& out);           // reads up to '\n' (max 8192)

void closeFd(int fd);

}  // namespace rc::transfer
