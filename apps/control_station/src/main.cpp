// Control station: reads a gamepad, streams ControlPackets to the robot at
// 50 Hz, and displays the low-res preview streamed back from the collector.
//
//   control_station [options]
//     --robot <host>        robot IP/hostname (default 127.0.0.1)
//     --ctrl-port <p>       robot control port (default 9101)
//     --preview-port <p>    local preview listen port (default 9102)
//     --input sdl|mouse|scripted  input backend (default sdl; mouse = drag on
//                           the preview window; scripted for testing)
//     --gain <n>            max motor offset from stop (default 500)
//     --no-display          don't open a preview window (headless)
//     --view-only           just display the preview stream (e.g. from the
//                           executor's --preview-to); no input, no control TX
//     --run-seconds <n>     auto-exit (0 = forever)
#include <netinet/in.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "common/label_codec.h"
#include "common/mixing.h"
#include "common/proto.h"
#include "gamepad.h"
#include "netutil/net.h"

using namespace rc;

namespace {
std::atomic<bool> g_run{true};
void onSignal(int) { g_run.store(false); }

uint64_t nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

struct Args {
  std::string robot = "127.0.0.1";
  int ctrlPort = 9101;
  int previewPort = 9102;
  std::string input = "sdl";
  float gain = 500.0f;
  bool display = true;
  bool viewOnly = false;
  int runSeconds = 0;
};

Args parseArgs(int argc, char** argv) {
  Args a;
  auto nx = [&](int& i) { return std::string(argv[++i]); };
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    if (s == "--robot") a.robot = nx(i);
    else if (s == "--ctrl-port") a.ctrlPort = std::stoi(nx(i));
    else if (s == "--preview-port") a.previewPort = std::stoi(nx(i));
    else if (s == "--input") a.input = nx(i);
    else if (s == "--gain") a.gain = std::stof(nx(i));
    else if (s == "--no-display") a.display = false;
    else if (s == "--view-only") a.viewOnly = true;
    else if (s == "--run-seconds") a.runSeconds = std::stoi(nx(i));
    else { std::cerr << "unknown arg: " << s << "\n"; std::exit(2); }
  }
  return a;
}

// Reassembles fragmented preview JPEGs (latest frame wins).
class PreviewReceiver {
 public:
  explicit PreviewReceiver(int port) : fd_(net::openUdpRecv(port)) {}
  bool ok() const { return fd_ >= 0; }
  std::atomic<uint64_t> decoded{0};

  void start(bool display) {
    display_ = display;
    run_.store(true);
    thread_ = std::thread([this] { loop(); });
  }
  void stop() {
    if (run_.exchange(false) && thread_.joinable()) thread_.join();
    net::closeFd(fd_);
  }
  cv::Mat latest() {
    std::lock_guard<std::mutex> lk(mx_);
    return latest_.clone();
  }

 private:
  void loop() {
    std::vector<uint8_t> buf(2048);
    uint32_t curId = 0;
    std::vector<uint8_t> assembly;
    std::map<uint16_t, bool> got;
    uint16_t need = 0;
    while (run_.load()) {
      const int n = net::recvWithTimeout(fd_, buf.data(), buf.size(), 100);
      if (n <= static_cast<int>(kPreviewHeaderSize)) continue;
      PreviewFragHeader h;
      if (!decodePreviewHeader(buf.data(), n, h)) continue;
      if (h.frameId != curId) {  // new frame -> reset
        curId = h.frameId;
        assembly.assign(h.totalSize, 0);
        got.clear();
        need = h.fragCount;
      }
      const size_t off = static_cast<size_t>(h.fragIndex) * 1400;
      const size_t payload = n - kPreviewHeaderSize;
      if (off + payload <= assembly.size()) {
        std::memcpy(assembly.data() + off, buf.data() + kPreviewHeaderSize, payload);
        got[h.fragIndex] = true;
      }
      if (got.size() == need && need > 0) {
        cv::Mat img = cv::imdecode(assembly, cv::IMREAD_COLOR);
        if (!img.empty()) {
          decoded.fetch_add(1);
          std::lock_guard<std::mutex> lk(mx_);
          latest_ = img;
        }
        need = 0;  // wait for next frame id
      }
    }
  }

  int fd_;
  bool display_ = false;
  std::thread thread_;
  std::atomic<bool> run_{false};
  std::mutex mx_;
  cv::Mat latest_;
};

}  // namespace

int main(int argc, char** argv) {
  Args args = parseArgs(argc, argv);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  net::UdpTarget robot;
  if (!robot.resolve(args.robot, args.ctrlPort)) {
    std::cerr << "cannot resolve robot host: " << args.robot << "\n";
    return 1;
  }
  const int txFd = net::openUdpSend();

  if ((args.input == "mouse" || args.viewOnly) && !args.display) {
    std::cerr << "this mode needs the preview window (drop --no-display)\n";
    return 1;
  }
  std::unique_ptr<GamepadInput> pad;
  if (!args.viewOnly) {
    pad = makeGamepad(args.input);
    if (!pad->open()) {
      std::cerr << "gamepad open failed (try --input mouse or --input scripted)\n";
      return 1;
    }
  }

  PreviewReceiver preview(args.previewPort);
  if (!preview.ok()) {
    std::cerr << "cannot bind preview port " << args.previewPort << "\n";
    return 1;
  }
  preview.start(args.display);

  std::cerr << "control station -> " << args.robot << ":" << args.ctrlPort
            << " (input=" << args.input << ")\n";

  uint32_t seq = 0;
  const auto t0 = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(20);  // 50 Hz

  while (g_run.load()) {
    const auto tick = std::chrono::steady_clock::now();
    GamepadState gs;
    MotorCmd cmd;  // stays neutral in view-only mode
    if (!args.viewOnly) {
      pad->poll(gs);
      if (gs.quit) break;

      cmd = mixArcade(gs.throttle, gs.steer, args.gain);

      ControlPacket c;
      c.seq = seq++;
      c.tsMs = nowMs();
      c.left = static_cast<int16_t>(cmd.left);
      c.right = static_cast<int16_t>(cmd.right);
      c.axes[0] = static_cast<int16_t>(gs.steer * 32767);
      c.axes[1] = static_cast<int16_t>(gs.throttle * 32767);
      if (gs.recording) c.flagBits |= flags::kRecording;
      if (gs.estop) c.flagBits |= flags::kEstop;
      c.flagBits |= flags::kHeartbeat;

      uint8_t buf[kControlPacketSize];
      encodeControl(c, buf);
      net::sendTo(txFd, robot, buf, kControlPacketSize);
    }

#ifdef HAVE_HIGHGUI
    if (args.display) {
      cv::Mat frame = preview.latest();
      if (frame.empty()) {  // keep the window (and mouse input) alive
        frame = cv::Mat(324, 576, CV_8UC3, cv::Scalar(30, 30, 30));
        cv::putText(frame, "waiting for preview from " + args.robot, {8, 162},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {180, 180, 180}, 1);
      }
      if (!args.viewOnly) {  // view-only frames arrive already annotated
        cv::putText(frame, (gs.recording ? "REC" : "idle"), {8, 20},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    gs.recording ? cv::Scalar(0, 0, 255) : cv::Scalar(180, 180, 180), 2);
        cv::putText(frame, "L" + std::to_string(cmd.left) + " R" + std::to_string(cmd.right),
                    {8, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 220, 0}, 1);
        if (gs.estop)
          cv::putText(frame, "ESTOP", {8, 68}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      {0, 0, 255}, 2);
        // Virtual-stick HUD: dot = current (steer, throttle).
        const cv::Point hub{frame.cols - 48, frame.rows - 48};
        cv::circle(frame, hub, 40, {120, 120, 120}, 1);
        cv::circle(frame,
                   hub + cv::Point(static_cast<int>(gs.steer * 36),
                                   static_cast<int>(-gs.throttle * 36)),
                   4, {0, 220, 0}, cv::FILLED);
      }
      cv::imshow("preview", frame);
      const int key = cv::waitKey(1);
      if (key == 'q') break;
      if (pad && key >= 0) pad->onKey(key);
    }
#endif

    std::this_thread::sleep_until(tick + period);
    if (args.runSeconds > 0 && std::chrono::steady_clock::now() - t0 >
                                   std::chrono::seconds(args.runSeconds))
      break;
  }

  // Best-effort neutral + estop on exit.
  if (!args.viewOnly) {
    ControlPacket stop;
    stop.seq = seq++;
    stop.tsMs = nowMs();
    stop.flagBits = flags::kEstop;
    uint8_t buf[kControlPacketSize];
    encodeControl(stop, buf);
    net::sendTo(txFd, robot, buf, kControlPacketSize);
  }

  preview.stop();
  net::closeFd(txFd);
  std::cerr << "station stopped. control packets sent=" << seq
            << " preview frames decoded=" << preview.decoded.load() << "\n";
  return 0;
}
