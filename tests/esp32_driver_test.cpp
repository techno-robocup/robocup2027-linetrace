// Verifies the ESP32 protocol parsing and the threaded driver against a fake
// firmware running over a pseudo-terminal pair (no real hardware needed).
#include <pty.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>

#include "esp32_driver/esp32_driver.h"
#include "esp32_driver/protocol.h"

using namespace rc::esp32;

static int g_fail = 0;
#define EXPECT(c, m)                                                    \
  do {                                                                  \
    if (!(c)) { std::printf("FAIL: %s (%s:%d)\n", m, __FILE__, __LINE__); ++g_fail; } \
  } while (0)

// Minimal fake firmware: read "<id> <cmd>\n" from master, reply "<id> <resp>\n".
static void fakeFirmware(int master, std::atomic<bool>& run) {
  std::string buf;
  char chunk[256];
  while (run.load()) {
    const ssize_t n = ::read(master, chunk, sizeof(chunk));
    if (n <= 0) {
      if (run.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    buf.append(chunk, static_cast<size_t>(n));
    size_t nl;
    while ((nl = buf.find('\n')) != std::string::npos) {
      std::string line = buf.substr(0, nl);
      buf.erase(0, nl + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      std::istringstream iss(line);
      long long id;
      std::string cmd, arg;
      iss >> id >> cmd;
      std::string resp;
      if (cmd == "MOTOR") {
        int a, b; iss >> a >> b;
        std::ostringstream o; o << "OK " << a << " " << b << " " << a << " " << b;
        resp = o.str();
      } else if (cmd == "GET") {
        iss >> arg;
        if (arg == "button") resp = "ON";
        else if (arg == "bno") resp = "1.0 2.0 3.0 4.0 5.0 6.0";
        else if (arg == "usonic") resp = "10 20 30";
        else resp = "ERR: unknown";
      } else if (cmd == "healthcheck") {
        resp = "OK";
      } else if (cmd == "Rescue") {
        int p, w; iss >> p >> w;
        std::ostringstream o; o << "OK " << p << " " << w;
        resp = o.str();
      } else {
        resp = "ERR: unknown";
      }
      std::string out = std::to_string(id) + " " + resp + "\n";
      ssize_t w = ::write(master, out.data(), out.size());
      (void)w;
    }
  }
}

int main() {
  // --- pure protocol checks ---
  {
    long long id; std::string p;
    EXPECT(parseResponse("7 OK 1 2 3 4\r", id, p) && id == 7 && p == "OK 1 2 3 4", "parseResponse");
    EXPECT(!parseResponse("garbage", id, p), "parseResponse rejects non-id");
    Bno b; EXPECT(parseBno("1 2 3 4 5 6", b) && b.heading == 1 && b.az == 6, "parseBno");
    Ultrasonic u; EXPECT(parseUltrasonic("10 20 30", u) && u.m == 20, "parseUltrasonic");
    bool on; EXPECT(parseButton("ON", on) && on, "parseButton ON");
    EXPECT(parseButton("OFF", on) && !on, "parseButton OFF");
    EXPECT(isError("ERR: MOTOR"), "isError");
    EXPECT(buildFrame(3, "MOTOR 1 2") == "3 MOTOR 1 2", "buildFrame");
  }

  // --- threaded driver over a pty loopback ---
  int master = -1, slave = -1;
  char name[256];
  if (openpty(&master, &slave, name, nullptr, nullptr) != 0) {
    std::printf("openpty failed; skipping loopback portion\n");
    return g_fail == 0 ? 0 : 1;
  }
  std::string slavePath = name;
  std::atomic<bool> run{true};
  std::thread fw(fakeFirmware, master, std::ref(run));

  {
    Config cfg;
    cfg.port = slavePath;
    cfg.sensorPollEvery = 1;  // poll aggressively so the cache fills fast
    Esp32Driver drv(cfg);
    drv.start();
    drv.setMotors(1600, 1400);

    // give the I/O thread time to connect + cycle through all three sensors
    for (int i = 0; i < 200 && !drv.connected(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    EXPECT(drv.connected(), "driver connected via healthcheck");

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT(drv.button(), "cached button == ON");
    EXPECT(drv.bno().heading == 1.0f && drv.bno().az == 6.0f, "cached bno");
    EXPECT(drv.ultrasonic().l == 10.0f && drv.ultrasonic().r == 30.0f, "cached usonic");

    drv.stop();
  }

  run.store(false);
  ::close(master);
  ::close(slave);
  if (fw.joinable()) fw.join();

  if (g_fail == 0) { std::printf("esp32_driver_test: ALL PASSED\n"); return 0; }
  std::printf("esp32_driver_test: %d FAILURE(S)\n", g_fail);
  return 1;
}
