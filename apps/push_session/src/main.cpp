// push_session: bulk-upload a recorded session directory to the training PC via
// the transfer protocol.
//
//   push_session <host> <port> <session_dir> [remote_prefix]
//
// Default remote_prefix is "datasets/<session_basename>".
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>

#include "transfer_client/transfer_client.h"

namespace fs = std::filesystem;

namespace {
// "[##########----------]  123/433 files   12.3/41.7 MB (29%)"
void drawBar(size_t filesDone, size_t filesTotal, uint64_t bytesDone, uint64_t bytesTotal) {
  constexpr int kWidth = 24;
  const double frac = bytesTotal > 0 ? static_cast<double>(bytesDone) / bytesTotal
                                     : (filesTotal > 0 ? static_cast<double>(filesDone) / filesTotal : 1.0);
  const int fill = static_cast<int>(frac * kWidth + 0.5);
  std::string bar(static_cast<size_t>(fill), '#');
  bar.append(static_cast<size_t>(kWidth - fill), '-');
  std::fprintf(stderr, "\r[%s] %zu/%zu files  %.1f/%.1f MB (%d%%)", bar.c_str(),
               filesDone, filesTotal, bytesDone / 1048576.0, bytesTotal / 1048576.0,
               static_cast<int>(frac * 100 + 0.5));
  std::fflush(stderr);
}
}  // namespace

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
  auto r = client.uploadDir(sessionDir, remotePrefix,
                            [](size_t fd, size_t ft, uint64_t bd, uint64_t bt,
                               const std::string&) { drawBar(fd, ft, bd, bt); });
  std::fputc('\n', stderr);
  if (!r) {
    std::cerr << "upload failed: " << r.error << "\n";
    return 1;
  }
  std::cout << "upload complete\n";
  return 0;
}
