#include "transfer_client/transfer_client.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <sstream>

#include "transfer_client/wire.h"

namespace fs = std::filesystem;

namespace rc::transfer {

namespace {
constexpr size_t kChunk = 64 * 1024;
Result err(std::string msg) { return {false, std::move(msg)}; }
Result ok() { return {true, {}}; }
}  // namespace

Result TransferClient::uploadFile(const fs::path& local, const std::string& remotePath) {
  std::error_code ec;
  const uint64_t size = static_cast<uint64_t>(fs::file_size(local, ec));
  if (ec) return err("local file_size failed: " + local.string());

  std::ifstream in(local, std::ios::binary);
  if (!in) return err("cannot open local file: " + local.string());

  const int fd = connectTcp(host_, port_);
  if (fd < 0) return err("connect failed");

  std::ostringstream header;
  header << "PUT " << remotePath << " " << size;
  if (!sendLine(fd, header.str())) {
    closeFd(fd);
    return err("send header failed");
  }

  std::string buf(kChunk, '\0');
  uint64_t remaining = size;
  while (remaining > 0) {
    in.read(buf.data(), static_cast<std::streamsize>(std::min<uint64_t>(kChunk, remaining)));
    const std::streamsize got = in.gcount();
    if (got <= 0) {
      closeFd(fd);
      return err("local read short");
    }
    if (!sendAll(fd, buf.data(), static_cast<size_t>(got))) {
      closeFd(fd);
      return err("send payload failed");
    }
    remaining -= static_cast<uint64_t>(got);
  }

  std::string resp;
  const bool gotResp = recvLine(fd, resp);
  closeFd(fd);
  if (!gotResp) return err("no response");
  if (resp != "OK") return err("server: " + resp);
  return ok();
}

Result TransferClient::downloadFile(const std::string& remotePath,
                                    const fs::path& localDirOrFile) {
  const int fd = connectTcp(host_, port_);
  if (fd < 0) return err("connect failed");

  if (!sendLine(fd, "GET " + remotePath)) {
    closeFd(fd);
    return err("send request failed");
  }
  std::string resp;
  if (!recvLine(fd, resp)) {
    closeFd(fd);
    return err("no response");
  }
  if (resp.rfind("ERR ", 0) == 0) {
    closeFd(fd);
    return err("server: " + resp);
  }
  std::istringstream iss(resp);
  std::string tag;
  uint64_t size = 0;
  iss >> tag >> size;
  if (!iss || tag != "SIZE") {
    closeFd(fd);
    return err("bad response: " + resp);
  }

  std::error_code ec;
  fs::path out = localDirOrFile;
  if (fs::is_directory(out, ec)) out /= fs::path(remotePath).filename();
  if (out.has_parent_path()) fs::create_directories(out.parent_path(), ec);

  std::ofstream of(out, std::ios::binary | std::ios::trunc);
  if (!of) {
    closeFd(fd);
    return err("cannot open local output: " + out.string());
  }

  std::string buf(kChunk, '\0');
  uint64_t remaining = size;
  while (remaining > 0) {
    const size_t want = static_cast<size_t>(std::min<uint64_t>(kChunk, remaining));
    if (!recvAll(fd, buf.data(), want)) {
      closeFd(fd);
      return err("recv payload failed");
    }
    of.write(buf.data(), static_cast<std::streamsize>(want));
    remaining -= want;
  }
  closeFd(fd);
  return of.good() ? ok() : err("local write failed");
}

Result TransferClient::listFiles(const std::string& pattern, std::vector<std::string>& outv) {
  outv.clear();
  const int fd = connectTcp(host_, port_);
  if (fd < 0) return err("connect failed");
  if (!sendLine(fd, "LIST " + pattern)) {
    closeFd(fd);
    return err("send request failed");
  }
  std::string resp;
  if (!recvLine(fd, resp) || resp.rfind("FILES ", 0) != 0) {
    closeFd(fd);
    return err("bad list response: " + resp);
  }
  const size_t count = std::stoull(resp.substr(6));
  for (size_t i = 0; i < count; ++i) {
    std::string f;
    if (!recvLine(fd, f)) break;
    outv.push_back(f);
  }
  closeFd(fd);
  return ok();
}

Result TransferClient::uploadDir(const fs::path& localDir, const std::string& remotePrefix,
                                 const Progress& progress) {
  std::error_code ec;
  if (!fs::is_directory(localDir, ec)) return err("not a directory: " + localDir.string());

  // Pre-scan so progress can report totals.
  std::vector<std::pair<fs::path, uint64_t>> files;
  uint64_t bytesTotal = 0;
  for (const auto& entry : fs::recursive_directory_iterator(localDir, ec)) {
    if (!entry.is_regular_file()) continue;
    const uint64_t size = static_cast<uint64_t>(fs::file_size(entry.path(), ec));
    if (ec) return err("local file_size failed: " + entry.path().string());
    files.emplace_back(entry.path(), size);
    bytesTotal += size;
  }

  uint64_t bytesDone = 0;
  for (size_t i = 0; i < files.size(); ++i) {
    const fs::path rel = fs::relative(files[i].first, localDir, ec);
    if (ec) return err("relative path failed");
    if (progress) progress(i, files.size(), bytesDone, bytesTotal, rel.generic_string());
    Result r = uploadFile(files[i].first, remotePrefix + "/" + rel.generic_string());
    if (!r.ok) return r;
    bytesDone += files[i].second;
  }
  if (progress) progress(files.size(), files.size(), bytesDone, bytesTotal, "");
  return ok();
}

}  // namespace rc::transfer
