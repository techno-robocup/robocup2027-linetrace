// Collector: drives the robot from UDP control packets, captures camera frames,
// pairs each frame with the live control command, records the dataset on the Pi
// at a fixed rate, and streams a low-res JPEG preview back to the station.
//
//   collector [options]
//     --camera <spec>       synthetic | libcamera | <index> | <gst pipeline> | <file>
//     --ctrl-port <p>       UDP control listen port (default 9101)
//     --preview-port <p>    UDP preview destination port on the station (default 9102)
//     --dataset-root <dir>  where sessions are written (default ./collected)
//     --esp-port <dev>      ESP32 serial device (default auto-detect)
//     --no-esp              don't open the ESP32 (desktop testing)
//     --record-hz <n>       dataset write rate (default 10)
//     --fps <n>             camera fps (default 30)
//     --run-seconds <n>     auto-exit after n seconds (0 = run forever; for tests)
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <ctime>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera/frame_source.h"
#include "common/proto.h"
#include "dataset_writer.h"
#include "esp32_driver/esp32_driver.h"
#include "netutil/net.h"
#include "shared_state.h"

using namespace rc;

namespace {

std::atomic<bool> g_run{true};
void onSignal(int) { g_run.store(false); }

uint64_t nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
int64_t monoNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
std::string sessionStamp() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
  return buf;
}

struct Args {
  std::string camera = "synthetic";
  int ctrlPort = 9101;
  int previewPort = 9102;
  std::string datasetRoot = "collected";
  std::string espPort;
  bool noEsp = false;
  int recordHz = 10;
  double fps = 30.0;
  int runSeconds = 0;
};

Args parseArgs(int argc, char** argv) {
  Args a;
  auto nx = [&](int& i) { return std::string(argv[++i]); };
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    if (s == "--camera") a.camera = nx(i);
    else if (s == "--ctrl-port") a.ctrlPort = std::stoi(nx(i));
    else if (s == "--preview-port") a.previewPort = std::stoi(nx(i));
    else if (s == "--dataset-root") a.datasetRoot = nx(i);
    else if (s == "--esp-port") a.espPort = nx(i);
    else if (s == "--no-esp") a.noEsp = true;
    else if (s == "--record-hz") a.recordHz = std::stoi(nx(i));
    else if (s == "--fps") a.fps = std::stod(nx(i));
    else if (s == "--run-seconds") a.runSeconds = std::stoi(nx(i));
    else { std::cerr << "unknown arg: " << s << "\n"; std::exit(2); }
  }
  return a;
}

// Latest-frame-wins preview JPEG sender (fragmented UDP to the station).
class PreviewSender {
 public:
  PreviewSender(int fd, int previewPort, int quality, double fps)
      : fd_(fd), port_(previewPort), quality_(quality), fps_(fps) {}

  void setStation(const in_addr& a) {
    std::lock_guard<std::mutex> lk(mx_);
    stationAddr_ = a;
    hasStation_ = true;
  }
  void offer(const cv::Mat& bgr) {
    std::lock_guard<std::mutex> lk(mx_);
    latest_ = bgr.clone();
  }
  void start() {
    run_.store(true);
    thread_ = std::thread([this] { loop(); });
  }
  void stop() {
    if (run_.exchange(false) && thread_.joinable()) thread_.join();
  }

 private:
  void loop() {
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, fps_));
    uint32_t frameId = 0;
    while (run_.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      cv::Mat frame;
      net::UdpTarget tgt;
      bool ready = false;
      {
        std::lock_guard<std::mutex> lk(mx_);
        if (hasStation_ && !latest_.empty()) {
          frame = latest_;
          latest_.release();
          tgt.addr.sin_family = AF_INET;
          tgt.addr.sin_addr = stationAddr_;
          tgt.addr.sin_port = htons(static_cast<uint16_t>(port_));
          ready = true;
        }
      }
      if (ready) sendFrame(frame, tgt, frameId++);
      std::this_thread::sleep_until(
          t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
    }
  }

  void sendFrame(const cv::Mat& bgr, const net::UdpTarget& tgt, uint32_t frameId) {
    cv::Mat small;
    cv::resize(bgr, small, cv::Size(384, 216));
    std::vector<uchar> jpg;
    cv::imencode(".jpg", small, jpg, {cv::IMWRITE_JPEG_QUALITY, quality_});

    const size_t payloadMax = 1400;
    const uint16_t fragCount =
        static_cast<uint16_t>((jpg.size() + payloadMax - 1) / payloadMax);
    std::vector<uint8_t> pkt(kPreviewHeaderSize + payloadMax);
    for (uint16_t i = 0; i < fragCount; ++i) {
      const size_t off = i * payloadMax;
      const size_t n = std::min(payloadMax, jpg.size() - off);
      PreviewFragHeader h{frameId, i, fragCount, static_cast<uint32_t>(jpg.size())};
      encodePreviewHeader(h, pkt.data());
      std::memcpy(pkt.data() + kPreviewHeaderSize, jpg.data() + off, n);
      net::sendTo(fd_, tgt, pkt.data(), kPreviewHeaderSize + n);
    }
  }

  int fd_;
  int port_;
  int quality_;
  double fps_;
  std::mutex mx_;
  cv::Mat latest_;
  in_addr stationAddr_{};
  bool hasStation_ = false;
  std::thread thread_;
  std::atomic<bool> run_{false};
};

}  // namespace

int main(int argc, char** argv) {
  Args args = parseArgs(argc, argv);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  SharedState state;
  DatasetWriter writer(args.datasetRoot);
  writer.start();

  std::unique_ptr<esp32::Esp32Driver> driver;
  if (!args.noEsp) {
    esp32::Config cfg;
    cfg.port = args.espPort;
    driver = std::make_unique<esp32::Esp32Driver>(cfg);
    driver->start();
  }

  const int ctrlFd = net::openUdpRecv(args.ctrlPort);
  if (ctrlFd < 0) {
    std::cerr << "failed to bind control port " << args.ctrlPort << "\n";
    return 1;
  }
  const int previewFd = net::openUdpSend();
  PreviewSender preview(previewFd, args.previewPort, /*quality=*/55, /*fps=*/15.0);
  preview.start();

  // Control-receive + failsafe-watchdog thread.
  constexpr int64_t kLinkTimeoutNs = 150'000'000;  // 150 ms
  std::thread ctrlThread([&] {
    uint8_t buf[64];
    uint32_t lastSeq = 0;
    while (g_run.load()) {
      sockaddr_in src{};
      socklen_t slen = sizeof(src);
      const int n = [&] {
        pollfd pfd{ctrlFd, POLLIN, 0};
        if (::poll(&pfd, 1, 50) <= 0) return 0;
        return static_cast<int>(
            ::recvfrom(ctrlFd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &slen));
      }();

      const int64_t now = monoNs();
      if (n > 0) {
        ControlPacket c;
        if (decodeControl(buf, static_cast<size_t>(n), c) && c.seq >= lastSeq) {
          lastSeq = c.seq;
          state.left.store(c.left);
          state.right.store(c.right);
          state.ctrlSeq.store(c.seq);
          state.ctrlTsMs.store(c.tsMs);
          state.lastRxMonoNs.store(now);
          state.recording.store(c.recording());
          state.estop.store(c.estop());
          state.linkLost.store(false);
          preview.setStation(src.sin_addr);
          if (driver) {
            if (c.estop()) driver->setMotors(1500, 1500);
            else driver->setMotors(c.left, c.right);
          }
        }
      }
      // Failsafe: no fresh control -> neutral, stop recording.
      if (now - state.lastRxMonoNs.load() > kLinkTimeoutNs) {
        if (!state.linkLost.exchange(true)) std::cerr << "control link lost -> failsafe\n";
        state.left.store(1500);
        state.right.store(1500);
        if (driver) driver->setMotors(1500, 1500);
      }
    }
  });

  // Camera frame handler: pair frame with the live control, gate recording at
  // record-hz, and always feed the preview.
  const int64_t recordPeriodNs = args.recordHz > 0 ? 1'000'000'000LL / args.recordHz : 0;
  std::atomic<int64_t> lastRecordNs{0};
  std::string activeSession;
  std::mutex sessionMx;

  FrameSourceConfig fsc;
  fsc.spec = args.camera;
  fsc.fps = args.fps;
  fsc.rotate180 = true;
  auto cam = makeFrameSource(fsc);

  auto onFrame = [&](const cv::Mat& bgr, uint64_t tsNs) {
    preview.offer(bgr);

    const bool rec = state.recording.load() && !state.estop.load() && !state.linkLost.load();
    if (!rec) return;
    if (recordPeriodNs > 0 &&
        static_cast<int64_t>(tsNs) - lastRecordNs.load() < recordPeriodNs)
      return;
    lastRecordNs.store(static_cast<int64_t>(tsNs));

    {  // start a session on the first recorded frame of a run
      std::lock_guard<std::mutex> lk(sessionMx);
      if (activeSession.empty()) {
        const std::string stamp = sessionStamp();
        std::string meta = std::string("{\"camera\":\"") + args.camera +
                           "\",\"record_hz\":" + std::to_string(args.recordHz) +
                           ",\"rotate180_source\":true,\"label\":\"natural (left,right) pwm\"}";
        if (writer.startSession(stamp, meta))
          activeSession = writer.sessionDir();
        std::cerr << "recording session: " << activeSession << "\n";
      }
    }

    FrameJob job;
    job.image = bgr;  // DatasetWriter clones via imwrite; safe to move a shallow copy
    job.unixMs = nowMs();
    job.left = state.left.load();
    job.right = state.right.load();
    if (driver) {
      job.bno = driver->bno();
      job.uson = driver->ultrasonic();
      job.button = driver->button();
    }
    writer.enqueue(std::move(job));
  };

  if (!cam || !cam->start(onFrame)) {
    std::cerr << "failed to start camera source: " << args.camera << "\n";
    g_run.store(false);
  } else {
    std::cerr << "collector running (camera=" << args.camera << ", ctrl :" << args.ctrlPort
              << ")\n";
  }

  const auto start = std::chrono::steady_clock::now();
  while (g_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (args.runSeconds > 0 &&
        std::chrono::steady_clock::now() - start > std::chrono::seconds(args.runSeconds))
      g_run.store(false);
  }

  if (cam) cam->stop();
  if (ctrlThread.joinable()) ctrlThread.join();
  preview.stop();
  writer.stop();
  if (driver) driver->stop();
  net::closeFd(ctrlFd);
  net::closeFd(previewFd);
  std::cerr << "collector stopped. frames written=" << writer.written()
            << " dropped=" << writer.dropped() << "\n";
  return 0;
}
