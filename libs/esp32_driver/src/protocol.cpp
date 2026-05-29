#include "esp32_driver/protocol.h"

#include <cctype>
#include <sstream>

namespace rc::esp32 {

std::string buildFrame(long long id, const std::string& payload) {
  return std::to_string(id) + " " + payload;
}

bool parseResponse(const std::string& line, long long& id, std::string& payload) {
  std::string s = line;
  if (!s.empty() && s.back() == '\r') s.pop_back();
  size_t i = 0;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
  const size_t start = i;
  if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
  size_t digits = 0;
  while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) {
    ++i;
    ++digits;
  }
  if (digits == 0) return false;
  try {
    id = std::stoll(s.substr(start, i - start));
  } catch (...) {
    return false;
  }
  // skip the single separating space, keep the rest as payload
  if (i < s.size() && s[i] == ' ') ++i;
  payload = s.substr(i);
  return true;
}

bool isError(const std::string& payload) { return payload.rfind("ERR", 0) == 0; }

bool parseBno(const std::string& payload, Bno& out) {
  if (isError(payload)) return false;
  std::istringstream iss(payload);
  Bno b;
  if (iss >> b.heading >> b.roll >> b.pitch >> b.ax >> b.ay >> b.az) {
    out = b;
    return true;
  }
  return false;
}

bool parseUltrasonic(const std::string& payload, Ultrasonic& out) {
  if (isError(payload)) return false;
  std::istringstream iss(payload);
  Ultrasonic u;
  if (iss >> u.l >> u.m >> u.r) {
    out = u;
    return true;
  }
  return false;
}

bool parseButton(const std::string& payload, bool& out) {
  if (payload == "ON") {
    out = true;
    return true;
  }
  if (payload == "OFF") {
    out = false;
    return true;
  }
  return false;
}

}  // namespace rc::esp32
