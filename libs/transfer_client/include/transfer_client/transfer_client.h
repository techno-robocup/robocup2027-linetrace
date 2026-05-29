// Reusable client for the RoboCup transfer protocol (compatible with the
// transfer_node server in robocup2027-transfer-server). Used by the collector
// to bulk-ship recorded sessions to the training PC.
#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace rc::transfer {

struct Result {
  bool ok = false;
  std::string error;
  explicit operator bool() const { return ok; }
};

class TransferClient {
 public:
  TransferClient(std::string host, int port) : host_(std::move(host)), port_(port) {}

  // Uploads a single local file to remotePath (relative to the server root).
  // Each call opens its own connection (matches the server's one-shot model).
  Result uploadFile(const std::filesystem::path& local, const std::string& remotePath);

  // Downloads remotePath into localDirOrFile (if a directory, keeps the basename).
  Result downloadFile(const std::string& remotePath,
                      const std::filesystem::path& localDirOrFile);

  // Lists server-side paths matching a glob pattern (relative to server root).
  Result listFiles(const std::string& pattern, std::vector<std::string>& out);

  // Recursively uploads every regular file under localDir, preserving the tree
  // beneath remotePrefix. Returns the first failure (if any).
  Result uploadDir(const std::filesystem::path& localDir, const std::string& remotePrefix);

  const std::string& host() const { return host_; }
  int port() const { return port_; }

 private:
  std::string host_;
  int port_;
};

}  // namespace rc::transfer
