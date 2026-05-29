// Wire-protocol helpers for the ESP32 link.
//
// Frames are ASCII "<id> <payload>\n". The Pi sends "<id> <command>" and the
// firmware replies "<id> <response>" with the same id (request/response is
// matched by id). See robocup2026-esp32-program/src/main.cpp.
#pragma once

#include <string>

namespace rc::esp32 {

struct Bno {
  float heading = 0, roll = 0, pitch = 0, ax = 0, ay = 0, az = 0;
};
struct Ultrasonic {
  float l = -1, m = -1, r = -1;  // cm; -1 = unavailable
};

// "<id> <payload>"
std::string buildFrame(long long id, const std::string& payload);

// Splits a received line into id + payload. Strips a trailing '\r'. Returns
// false if there is no leading integer id.
bool parseResponse(const std::string& line, long long& id, std::string& payload);

bool isError(const std::string& payload);  // payload begins with "ERR"

bool parseBno(const std::string& payload, Bno& out);          // 6 floats
bool parseUltrasonic(const std::string& payload, Ultrasonic& out);  // 3 floats
// "ON" -> true, "OFF" -> false; returns false (and leaves out unchanged) otherwise.
bool parseButton(const std::string& payload, bool& out);

}  // namespace rc::esp32
