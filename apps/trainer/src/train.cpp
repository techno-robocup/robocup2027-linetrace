// LibTorch trainer for the line-trace model.
//
//   trainer --data <dataset_root> [options]
//
// Reads the labels.csv sessions, trains LineNet (image -> 2 normalized motor
// outputs), reports validation MAE in PWM units, and saves the trained weights
// (torch::save) plus a model_info.json the executor reads back.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <torch/torch.h>

#include "common/config.h"
#include "common/label_codec.h"
#include "common_torch/model.h"
#include "common_torch/preprocess.h"
#include "dataset.h"

using namespace rc;

namespace {

struct Args {
  std::string data = "../linetrace_data";
  std::string out = "models";
  int epochs = 30;
  int batch = 64;
  int workers = 8;
  double lr = 1e-3;
  double valFrac = 0.2;
  int limit = 0;          // 0 = use all; >0 caps train samples (smoke runs)
  bool grayscale = false;
  bool sensors = false;
  bool keepStopped = false;
  TargetSpace space = TargetSpace::LR;
  int patienceLR = 4;
  int patienceStop = 8;
  uint64_t seed = 1234;
};

Args parseArgs(int argc, char** argv) {
  Args a;
  auto next = [&](int& i) { return std::string(argv[++i]); };
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    if (s == "--data") a.data = next(i);
    else if (s == "--out") a.out = next(i);
    else if (s == "--epochs") a.epochs = std::stoi(next(i));
    else if (s == "--batch") a.batch = std::stoi(next(i));
    else if (s == "--workers") a.workers = std::stoi(next(i));
    else if (s == "--lr") a.lr = std::stod(next(i));
    else if (s == "--val-frac") a.valFrac = std::stod(next(i));
    else if (s == "--limit") a.limit = std::stoi(next(i));
    else if (s == "--grayscale") a.grayscale = true;
    else if (s == "--sensors") a.sensors = true;
    else if (s == "--keep-stopped") a.keepStopped = true;
    else if (s == "--target-space") a.space = (next(i) == "ts") ? TargetSpace::ThrottleSteer
                                                                : TargetSpace::LR;
    else if (s == "--seed") a.seed = std::stoull(next(i));
    else { std::cerr << "unknown arg: " << s << "\n"; std::exit(2); }
  }
  return a;
}

// Split samples into train/val by holding out whole sessions (chronological),
// avoiding near-duplicate leakage between consecutive 10 Hz frames.
void splitBySession(const std::vector<Sample>& all, double valFrac,
                    std::vector<Sample>& train, std::vector<Sample>& val) {
  std::vector<int> sessions;
  for (const auto& s : all)
    if (sessions.empty() || sessions.back() != s.sessionId) {
      if (std::find(sessions.begin(), sessions.end(), s.sessionId) == sessions.end())
        sessions.push_back(s.sessionId);
    }
  std::sort(sessions.begin(), sessions.end());
  const int n = static_cast<int>(sessions.size());

  if (n >= 2) {
    const int nVal = std::max(1, static_cast<int>(std::round(valFrac * n)));
    std::vector<int> valSessions(sessions.end() - nVal, sessions.end());
    auto isVal = [&](int id) {
      return std::find(valSessions.begin(), valSessions.end(), id) != valSessions.end();
    };
    for (const auto& s : all) (isVal(s.sessionId) ? val : train).push_back(s);
  } else {
    // single session: chronological frame split (data is already time-ordered)
    const size_t cut = static_cast<size_t>((1.0 - valFrac) * all.size());
    for (size_t i = 0; i < all.size(); ++i) (i < cut ? train : val).push_back(all[i]);
  }
}

void writeModelInfo(const Args& a, int sensorDim, double valLoss, double valMaePwm,
                    int epochsRun) {
  std::ofstream f(a.out + "/model_info.json");
  f << "{\n"
    << "  \"model_input_w\": " << kModelW << ",\n"
    << "  \"model_input_h\": " << kModelH << ",\n"
    << "  \"channels\": \"" << (a.grayscale ? "gray" : "bgr") << "\",\n"
    << "  \"crop_ratio\": " << kCropRatio << ",\n"
    << "  \"motor_min\": " << kMotorMin << ",\n"
    << "  \"motor_max\": " << kMotorMax << ",\n"
    << "  \"target_space\": \"" << (a.space == TargetSpace::LR ? "lr" : "throttle_steer")
    << "\",\n"
    << "  \"use_sensors\": " << (a.sensors ? "true" : "false") << ",\n"
    << "  \"sensor_dim\": " << sensorDim << ",\n"
    // Frames are oriented by the capture source (collector/executor FrameSource
    // applies rotate180), so preprocessing itself never rotates. Training data
    // must already be in that same orientation.
    << "  \"preprocess_rotate180\": false,\n"
    << "  \"frames_pre_rotated_by_source\": true,\n"
    << "  \"final_val_loss\": " << valLoss << ",\n"
    << "  \"final_val_mae_pwm\": " << valMaePwm << ",\n"
    << "  \"epochs_trained\": " << epochsRun << "\n"
    << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  Args a = parseArgs(argc, argv);
  torch::manual_seed(a.seed);
  std::filesystem::create_directories(a.out);

  torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
  std::cout << "device: " << (device.is_cuda() ? "CUDA" : "CPU") << "\n";

  // --- build dataset ---
  auto sessionDirs = findSessionDirs(a.data);
  if (sessionDirs.empty()) {
    std::cerr << "no session_* dirs under " << a.data << "\n";
    return 1;
  }
  auto all = scanSessions(sessionDirs, /*skipStopped=*/!a.keepStopped);
  std::cout << "sessions: " << sessionDirs.size() << "  samples: " << all.size() << "\n";

  std::vector<Sample> trainS, valS;
  splitBySession(all, a.valFrac, trainS, valS);

  if (a.limit > 0) {
    std::mt19937 g(a.seed);
    std::shuffle(trainS.begin(), trainS.end(), g);
    if (static_cast<int>(trainS.size()) > a.limit) trainS.resize(a.limit);
    const int vlim = std::max(64, a.limit / 5);
    std::shuffle(valS.begin(), valS.end(), g);
    if (static_cast<int>(valS.size()) > vlim) valS.resize(vlim);
  }
  std::cout << "train: " << trainS.size() << "  val: " << valS.size() << "\n";
  if (trainS.empty() || valS.empty()) {
    std::cerr << "empty train or val split\n";
    return 1;
  }

  const int sensorDim = a.sensors ? kSensorDim : 0;
  DatasetConfig dcfg;
  dcfg.preproc.channels = a.grayscale ? Channels::Gray : Channels::BGR;
  dcfg.preproc.rotate180 = false;  // recorded frames already oriented
  dcfg.targetSpace = a.space;
  dcfg.useSensors = a.sensors;
  dcfg.seed = a.seed;

  DatasetConfig trainCfg = dcfg;
  trainCfg.augment = true;

  auto trainLoader = torch::data::make_data_loader<torch::data::samplers::RandomSampler>(
      LineDataset(trainS, trainCfg).map(torch::data::transforms::Stack<>()),
      torch::data::DataLoaderOptions().batch_size(a.batch).workers(a.workers));
  auto valLoader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(
      LineDataset(valS, dcfg).map(torch::data::transforms::Stack<>()),
      torch::data::DataLoaderOptions().batch_size(a.batch).workers(a.workers));

  // --- model + optimizer ---
  LineNetOptions opt;
  opt.inChannels = a.grayscale ? 1 : 3;
  opt.sensorDim = sensorDim;
  LineNet net(opt);
  net->to(device);

  torch::optim::Adam optim(net->parameters(),
                           torch::optim::AdamOptions(a.lr).weight_decay(1e-4));

  namespace F = torch::nn::functional;
  auto smoothL1 = [](torch::Tensor p, torch::Tensor t) {
    return F::smooth_l1_loss(p, t);
  };

  double bestVal = 1e30;
  int sinceImprove = 0, sinceLR = 0, epochsRun = 0;
  double bestMaePwm = 0.0;
  double curLr = a.lr;

  for (int epoch = 1; epoch <= a.epochs; ++epoch) {
    auto t0 = std::chrono::steady_clock::now();
    // ---- train ----
    net->train();
    double trLoss = 0.0;
    int64_t trN = 0;
    for (auto& batch : *trainLoader) {
      auto data = batch.data.to(device);
      auto label = batch.target.narrow(1, 0, 2).to(device);
      optim.zero_grad();
      torch::Tensor pred;
      if (a.sensors) {
        auto sensors = batch.target.narrow(1, 2, kSensorDim).to(device);
        pred = net->forward(data, sensors);
      } else {
        pred = net->forward(data);
      }
      auto loss = smoothL1(pred, label);
      loss.backward();
      optim.step();
      trLoss += loss.item<double>() * data.size(0);
      trN += data.size(0);
    }

    // ---- validate ----
    net->eval();
    double vLoss = 0.0, vMae = 0.0;
    int64_t vN = 0;
    {
      torch::NoGradGuard ng;
      for (auto& batch : *valLoader) {
        auto data = batch.data.to(device);
        auto label = batch.target.narrow(1, 0, 2).to(device);
        torch::Tensor pred;
        if (a.sensors) {
          auto sensors = batch.target.narrow(1, 2, kSensorDim).to(device);
          pred = net->forward(data, sensors);
        } else {
          pred = net->forward(data);
        }
        vLoss += smoothL1(pred, label).item<double>() * data.size(0);
        vMae += (pred - label).abs().mean().item<double>() * data.size(0);
        vN += data.size(0);
      }
    }
    trLoss /= std::max<int64_t>(1, trN);
    vLoss /= std::max<int64_t>(1, vN);
    const double vMaeNorm = vMae / std::max<int64_t>(1, vN);
    const double vMaePwm = vMaeNorm * (kMotorMax - kMotorMin);
    epochsRun = epoch;

    auto t1 = std::chrono::steady_clock::now();
    double secs = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "epoch " << epoch << "/" << a.epochs << "  train " << trLoss << "  val "
              << vLoss << "  valMAE " << vMaeNorm << " (" << vMaePwm << " pwm)  lr " << curLr
              << "  " << secs << "s\n";

    // ---- best / early-stop / plateau LR ----
    if (vLoss < bestVal - 1e-6) {
      bestVal = vLoss;
      bestMaePwm = vMaePwm;
      sinceImprove = 0;
      sinceLR = 0;
      torch::save(net, a.out + "/linetrace_model.pt");
      writeModelInfo(a, sensorDim, bestVal, bestMaePwm, epochsRun);
    } else {
      ++sinceImprove;
      ++sinceLR;
      if (sinceLR >= a.patienceLR && curLr > 1e-6) {
        curLr = std::max(1e-6, curLr * 0.5);
        for (auto& group : optim.param_groups())
          static_cast<torch::optim::AdamOptions&>(group.options()).lr(curLr);
        sinceLR = 0;
        std::cout << "  -> lr reduced to " << curLr << "\n";
      }
      if (sinceImprove >= a.patienceStop) {
        std::cout << "early stop (no val improvement for " << a.patienceStop << ")\n";
        break;
      }
    }
  }

  std::cout << "best val loss " << bestVal << "  (MAE " << bestMaePwm
            << " pwm)\nsaved to " << a.out << "/linetrace_model.pt\n";
  return 0;
}
