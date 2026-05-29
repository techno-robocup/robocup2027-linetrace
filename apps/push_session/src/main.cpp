// push_session: bulk-upload a recorded session directory to the training PC via
// the transfer protocol.
//
//   push_session <host> <port> <session_dir> [remote_prefix]
//
// Default remote_prefix is "datasets/<session_basename>".
#include <filesystem>
#include <iostream>
#include <string>

#include "transfer_client/transfer_client.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: push_session <host> <port> <session_dir> [remote_prefix]\n";
    return 2;
  }
  const std::string host = argv[1];
  const int port = std::stoi(argv[2]);
  const fs::path sessionDir = argv[3];
  std::string remotePrefix =
      argc > 4 ? argv[4] : ("datasets/" + sessionDir.filename().string());

  if (!fs::is_directory(sessionDir)) {
    std::cerr << "not a directory: " << sessionDir << "\n";
    return 1;
  }

  rc::transfer::TransferClient client(host, port);
  std::cout << "uploading " << sessionDir << " -> " << host << ":" << port << "/"
            << remotePrefix << "\n";
  auto r = client.uploadDir(sessionDir, remotePrefix);
  if (!r) {
    std::cerr << "upload failed: " << r.error << "\n";
    return 1;
  }
  std::cout << "upload complete\n";
  return 0;
}
