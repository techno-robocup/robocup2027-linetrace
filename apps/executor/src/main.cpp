// executor: autonomous line-trace. Loads the trained LineNet, grabs camera
// frames (same FrameSource as the collector), runs inference in a worker thread
// (decoupled from capture so the camera pipeline never stalls), and drives the
// ESP32. The physical kill button (GET button == OFF) forces a stop.
//
//   executor --model <model.pt> [options]
//     --camera <spec>     synthetic | libcamera | <index> | <gst> | <file>
//     --info <path>       model_info.json (default: alongside --model)
//     --esp-port <dev>    ESP32 serial (default auto-detect)
//     --no-esp            don't open the ESP32
//     --grayscale --ts --sensors   override model_info
//     --rate <hz>         inference/drive rate (default 30)
//     --print             print motor commands
//     --preview-to <host> stream annotated preview (predicted L/R burned in)
//                         to this host's preview port; view with
//                         control_station --view-only
//     --preview-port <p>  preview destination port (default 9102)
//     --run-seconds <n>   auto-exit (0 = forever)
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <torch/torch.h>

#include <opencv2/imgproc.hpp>

#include "camera/frame_source.h"
#include "common/config.h"
#include "common/label_codec.h"
#include "common_torch/model.h"
#include "common_torch/preprocess.h"
#include "esp32_driver/esp32_driver.h"
#include "netutil/preview_sender.h"

using namespace rc;

namespace {
std::atomic<bool> g_run{true};
void onSignal(int) { g_run.store(false); }

// Tiny single-level JSON field readers (model_info.json is flat).
std::string readFile(const std::string& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
bool jsonBool(const std::string& j, const std::string& key, bool dflt) {
  auto k = j.find("\"" + key + "\"");
  if (k == std::string::npos) return dflt;
  auto c = j.find(':', k);
  if (c == std::string::npos) return dflt;
  return j.find("true", c) < j.find("false", c);
}
std::string jsonStr(const std::string& j, const std::string& key, const std::string& dflt) {
  auto k = j.find("\"" + key + "\"");
  if (k == std::string::npos) return dflt;
  auto c = j.find(':', k);
  auto q1 = j.find('"', c);
  if (q1 == std::string::npos) return dflt;
  auto q2 = j.find('"', q1 + 1);
  if (q2 == std::string::npos) return dflt;
  return j.substr(q1 + 1, q2 - q1 - 1);
}
int jsonInt(const std::string& j, const std::string& key, int dflt) {
  auto k = j.find("\"" + key + "\"");
  if (k == std::string::npos) return dflt;
  auto c = j.find(':', k);
  if (c == std::string::npos) return dflt;
  try {
    return std::stoi(j.substr(c + 1));
  } catch (...) {
    return dflt;
  }
}

struct Args {
  std::string model, info, camera = "synthetic", espPort, previewTo;
  int previewPort = 9102;
  bool noEsp = false, grayscale = false, sensors = false, print = false;
  bool memory = false;       // from model_info.json (use_memory)
  int memoryHidden = 64;     // from model_info.json (memory_hidden)
  bool forceGray = false, forceSensors = false;
  TargetSpace space = TargetSpace::LR;
  bool forceTs = false;
  double rate = 30.0;
  int runSeconds = 0;
};

Args parseArgs(int argc, char** argv) {
  Args a;
  auto nx = [&](int& i) { return std::string(argv[++i]); };
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    if (s == "--model") a.model = nx(i);
    else if (s == "--info") a.info = nx(i);
    else if (s == "--camera") a.camera = nx(i);
    else if (s == "--esp-port") a.espPort = nx(i);
    else if (s == "--no-esp") a.noEsp = true;
    else if (s == "--grayscale") { a.grayscale = true; a.forceGray = true; }
    else if (s == "--sensors") { a.sensors = true; a.forceSensors = true; }
    else if (s == "--ts") { a.space = TargetSpace::ThrottleSteer; a.forceTs = true; }
    else if (s == "--print") a.print = true;
    else if (s == "--preview-to") a.previewTo = nx(i);
    else if (s == "--preview-port") a.previewPort = std::stoi(nx(i));
    else if (s == "--rate") a.rate = std::stod(nx(i));
    else if (s == "--run-seconds") a.runSeconds = std::stoi(nx(i));
    else { std::cerr << "unknown arg: " << s << "\n"; std::exit(2); }
  }
  return a;
}
}  // namespace

int main(int argc, char** argv) {
  Args args = parseArgs(argc, argv);
  if (args.model.empty()) {
    std::cerr << "--model is required\n";
    return 2;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  torch::set_num_threads(std::max(1u, std::thread::hardware_concurrency()));

  // Pull architecture/preprocess settings from model_info.json so they cannot
  // silently mismatch the trained weights. CLI flags still override.
  const std::string infoPath =
      args.info.empty()
          ? (std::filesystem::path(args.model).parent_path() / "model_info.json").string()
          : args.info;
  const std::string info = readFile(infoPath);
  if (!info.empty()) {
    if (!args.forceGray) args.grayscale = (jsonStr(info, "channels", "bgr") == "gray");
    if (!args.forceSensors) args.sensors = jsonBool(info, "use_sensors", false);
    if (!args.forceTs)
      args.space = (jsonStr(info, "target_space", "lr") == "throttle_steer")
                       ? TargetSpace::ThrottleSteer
                       : TargetSpace::LR;
    args.memory = jsonBool(info, "use_memory", false);
    args.memoryHidden = jsonInt(info, "memory_hidden", 64);
    std::cerr << "loaded " << infoPath << " (channels=" << (args.grayscale ? "gray" : "bgr")
              << ", sensors=" << args.sensors << ", memory=" << args.memory << ")\n";
  } else {
    std::cerr << "no model_info.json at " << infoPath << "; using CLI/defaults\n";
  }

  LineNetOptions opt;
  opt.inChannels = args.grayscale ? 1 : 3;
  opt.sensorDim = args.sensors ? kSensorDim : 0;
  opt.useMemory = args.memory;
  opt.memoryHidden = args.memoryHidden;
  LineNet net(opt);
  try {
    torch::load(net, args.model);
  } catch (const std::exception& e) {
    std::cerr << "failed to load model: " << e.what() << "\n";
    return 1;
  }
  net->eval();

  // Self-test: run one dummy frame through the exact inference path so an
  // architecture mismatch (e.g. missing/wrong model_info.json) fails here
  // with a clear message instead of crashing mid-drive.
  try {
    torch::NoGradGuard ng;
    torch::Tensor dummy = torch::zeros({1, opt.inChannels, kModelH, kModelW});
    if (args.memory) {
      torch::Tensor h;
      net->forwardStep(dummy, h);
    } else if (args.sensors) {
      net->forward(dummy, torch::zeros({1L, static_cast<long>(kSensorDim)}));
    } else {
      net->forward(dummy);
    }
  } catch (const std::exception& e) {
    const std::string what = e.what();
    std::cerr << "model self-test failed: " << what.substr(0, what.find('\n')) << "\n"
              << "the loaded weights don't match the constructed architecture "
              << "(channels/sensors/memory).\nMake sure the model_info.json written "
              << "by the trainer sits next to the model (or pass --info <path>).\n";
    return 1;
  }

  std::unique_ptr<esp32::Esp32Driver> driver;
  if (!args.noEsp) {
    esp32::Config cfg;
    cfg.port = args.espPort;
    driver = std::make_unique<esp32::Esp32Driver>(cfg);
    driver->start();
  }

  // Optional live preview of what the model sees / wants to do.
  std::unique_ptr<rc::net::PreviewSender> previewer;
  int previewFd = -1;
  if (!args.previewTo.empty()) {
    previewFd = rc::net::openUdpSend();
    previewer = std::make_unique<rc::net::PreviewSender>(previewFd, args.previewPort,
                                                     /*quality=*/55, /*fps=*/15.0);
    if (previewFd < 0 || !previewer->resolveStation(args.previewTo)) {
      std::cerr << "cannot resolve preview host: " << args.previewTo << "\n";
      return 1;
    }
    previewer->start();
    std::cerr << "streaming preview to " << args.previewTo << ":" << args.previewPort << "\n";
  }

  PreprocConfig pc;
  pc.channels = args.grayscale ? Channels::Gray : Channels::BGR;
  pc.rotate180 = false;  // FrameSource already orients frames to match training

  // Latest-frame hand-off from the capture callback to the inference worker.
  std::mutex frameMx;
  std::condition_variable frameCv;
  cv::Mat latest;
  uint64_t latestSeq = 0, processedSeq = 0;

  auto onFrame = [&](const cv::Mat& bgr, uint64_t) {
    std::lock_guard<std::mutex> lk(frameMx);
    latest = bgr;  // shallow ref; cloned by the worker before use
    ++latestSeq;
    frameCv.notify_one();
  };

  std::atomic<uint64_t> infers{0};
  std::thread worker([&] {
    torch::Tensor hidden;  // GRU state carried across frames (memory models)
    while (g_run.load()) {
      cv::Mat frame;
      {
        std::unique_lock<std::mutex> lk(frameMx);
        if (!frameCv.wait_for(lk, std::chrono::milliseconds(200),
                              [&] { return !g_run.load() || latestSeq != processedSeq; }))
          continue;  // timeout: no new frame -> watchdog tick
        if (!g_run.load()) break;
        frame = latest.clone();
        processedSeq = latestSeq;
      }

      torch::NoGradGuard ng;
      torch::Tensor x = preprocess(frame, pc);
      torch::Tensor y;
      if (args.memory) {
        y = net->forwardStep(x, hidden);  // stateful streaming
      } else if (args.sensors) {
        y = net->forward(x, torch::zeros({1L, static_cast<long>(kSensorDim)}));
      } else {
        y = net->forward(x);
      }
      MotorCmd cmd = decode({y[0][0].item<float>(), y[0][1].item<float>()}, args.space);

      const bool killed = driver && !driver->button();
      if (killed) cmd = {kMotorStop, kMotorStop};
      if (driver) driver->setMotors(cmd.left, cmd.right);
      const uint64_t n = infers.fetch_add(1);
      if (args.print && n % 15 == 0)
        std::cout << "motor L=" << cmd.left << " R=" << cmd.right
                  << (killed ? " [KILLED]" : "") << "\n";

      if (previewer) {
        // Burn the prediction into the frame the model actually saw.
        cv::Mat vis = frame.clone();
        cv::putText(vis, "L" + std::to_string(cmd.left) + " R" + std::to_string(cmd.right),
                    {8, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 220, 0}, 2);
        if (killed)
          cv::putText(vis, "KILLED", {8, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);
        // Steering bar: green tick offset right/left of center by the wheel
        // differential, vertical extent by forward speed.
        const int cx = vis.cols / 2, by = vis.rows - 12;
        const float steer = (cmd.left - cmd.right) / (2.0f * kMotorSpan);    // [-1,1]
        const float thr = (cmd.left + cmd.right - 2 * kMotorStop) / (2.0f * kMotorSpan);
        cv::line(vis, {cx - 120, by}, {cx + 120, by}, {120, 120, 120}, 1);
        const int tx = cx + static_cast<int>(steer * 120);
        cv::line(vis, {tx, by}, {tx, by - 10 - static_cast<int>(std::abs(thr) * 40)},
                 thr >= 0 ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 140, 255), 3);
        previewer->offer(vis);
      }
    }
  });

  FrameSourceConfig fsc;
  fsc.spec = args.camera;
  fsc.fps = args.rate;
  fsc.rotate180 = true;
  auto cam = makeFrameSource(fsc);
  if (!cam || !cam->start(onFrame)) {
    std::cerr << "failed to start camera: " << args.camera << "\n";
    g_run.store(false);
  } else {
    std::cerr << "executor running (model=" << args.model << ", camera=" << args.camera << ")\n";
  }

  const auto t0 = std::chrono::steady_clock::now();
  while (g_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (args.runSeconds > 0 &&
        std::chrono::steady_clock::now() - t0 > std::chrono::seconds(args.runSeconds))
      g_run.store(false);
  }

  if (cam) cam->stop();
  frameCv.notify_all();
  if (worker.joinable()) worker.join();
  if (previewer) previewer->stop();
  if (previewFd >= 0) rc::net::closeFd(previewFd);
  if (driver) driver->stop();
  std::cerr << "executor stopped. inferences=" << infers.load() << "\n";
  return 0;
}
