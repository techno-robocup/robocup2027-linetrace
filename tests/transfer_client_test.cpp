// End-to-end test for TransferClient against a running transfer_node server.
//
//   transfer_client_test <host> <port> <workdir>
//
// Creates files under <workdir>/src, uploads the tree, lists it, downloads it
// back to <workdir>/dst, and verifies byte-for-byte. Returns non-zero on
// failure.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "transfer_client/transfer_client.h"

namespace fs = std::filesystem;
using rc::transfer::TransferClient;

static std::string readAll(const fs::path& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

int main(int argc, char** argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: transfer_client_test <host> <port> <workdir>\n");
    return 2;
  }
  const std::string host = argv[1];
  const int port = std::stoi(argv[2]);
  const fs::path work = argv[3];
  const fs::path src = work / "src";
  const fs::path dst = work / "dst";
  fs::create_directories(src / "sub");
  fs::create_directories(dst);

  // Known files, incl. a >64KB one to exercise multi-chunk transfer.
  std::vector<std::pair<std::string, std::string>> files = {
      {"a.txt", "hello transfer\n"},
      {"sub/b.bin", std::string(200000, 'Z')},
      {"sub/c.txt", "third file"},
  };
  for (auto& [rel, content] : files) {
    std::ofstream o(src / rel, std::ios::binary);
    o.write(content.data(), static_cast<std::streamsize>(content.size()));
  }

  TransferClient client(host, port);
  int failures = 0;

  auto r = client.uploadDir(src, "tctest");
  if (!r) { std::printf("FAIL uploadDir: %s\n", r.error.c_str()); ++failures; }

  std::vector<std::string> listed;
  r = client.listFiles("tctest/*", listed);
  if (!r) { std::printf("FAIL listFiles: %s\n", r.error.c_str()); ++failures; }
  // server globs one level; just require the top-level entries are present
  if (listed.empty()) { std::printf("FAIL listFiles empty\n"); ++failures; }

  for (auto& [rel, content] : files) {
    r = client.downloadFile("tctest/" + rel, dst / rel);
    if (!r) { std::printf("FAIL download %s: %s\n", rel.c_str(), r.error.c_str()); ++failures; continue; }
    if (readAll(dst / rel) != content) {
      std::printf("FAIL content mismatch: %s\n", rel.c_str());
      ++failures;
    }
  }

  // negative: missing file should error cleanly
  r = client.downloadFile("tctest/does_not_exist", dst / "nope");
  if (r) { std::printf("FAIL: expected error for missing file\n"); ++failures; }

  if (failures == 0) {
    std::printf("transfer_client_test: ALL PASSED (%zu files round-tripped)\n", files.size());
    return 0;
  }
  std::printf("transfer_client_test: %d FAILURE(S)\n", failures);
  return 1;
}
