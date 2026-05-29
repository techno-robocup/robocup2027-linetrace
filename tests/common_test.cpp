// Smoke test for the shared libraries: label-codec invertibility, mixing
// sanity, and a real preprocess -> LineNet forward pass. Runs on the PC with no
// hardware. Returns non-zero on failure.
#include <cstdio>
#include <cmath>

#include <opencv2/core.hpp>
#include <torch/torch.h>

#include "common/label_codec.h"
#include "common/mixing.h"
#include "common/proto.h"
#include "common_torch/model.h"
#include "common_torch/preprocess.h"

static int g_failures = 0;
#define EXPECT(cond, msg)                                            \
  do {                                                              \
    if (!(cond)) {                                                  \
      std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);  \
      ++g_failures;                                                 \
    }                                                               \
  } while (0)

int main() {
  using namespace rc;

  // --- label codec round-trips for the real recorded motor combos ---
  const MotorCmd combos[] = {{1500, 1500}, {1680, 1680}, {1600, 1400},
                             {1400, 1600}, {1780, 1580}, {1000, 2000}};
  for (const auto& c : combos) {
    MotorCmd lr = decodeLR(encodeLR(c));
    EXPECT(lr.left == c.left && lr.right == c.right, "LR round-trip");
    MotorCmd ts = decodeTS(encodeTS(c));
    // ThrottleSteer round-trips exactly for integer pwm (l+r, r-l both integer).
    EXPECT(std::abs(ts.left - c.left) <= 1 && std::abs(ts.right - c.right) <= 1,
          "TS round-trip");
  }

  // normalized outputs are in [0,1]
  auto n = encodeLR({1680, 1400});
  EXPECT(n[0] >= 0.0f && n[0] <= 1.0f && n[1] >= 0.0f && n[1] <= 1.0f, "norm range");

  // --- mixing sanity ---
  MotorCmd fwd = mixArcade(1.0f, 0.0f);
  EXPECT(fwd.left == kMotorMax && fwd.right == kMotorMax, "full forward");
  MotorCmd stop = mixArcade(0.0f, 0.0f);
  EXPECT(stop.left == kMotorStop && stop.right == kMotorStop, "neutral");
  MotorCmd right = mixArcade(0.5f, 0.5f);
  EXPECT(right.left > right.right, "steer right => left wheel faster");
  EXPECT(applyDeadzone(0.05f) == 0.0f, "deadzone kills small input");

  // --- preprocess + model forward ---
  cv::Mat synthetic(324, 576, CV_8UC3, cv::Scalar(20, 200, 40));  // a green-ish frame
  PreprocConfig pc;  // defaults: 160x90 BGR, crop 0.6
  torch::Tensor x = preprocess(synthetic, pc);
  EXPECT(x.sizes() == torch::IntArrayRef({1, 3, kModelH, kModelW}), "preprocess shape");
  EXPECT(x.min().item<float>() >= 0.0f && x.max().item<float>() <= 1.0f, "preprocess range");

  LineNet net(LineNetOptions{});
  net->eval();
  torch::NoGradGuard ng;
  torch::Tensor y = net->forward(x);
  EXPECT(y.sizes() == torch::IntArrayRef({1, 2}), "model output shape");
  EXPECT(y.min().item<float>() >= 0.0f && y.max().item<float>() <= 1.0f, "sigmoid range");

  // sensor-fusion variant
  LineNet net2(LineNetOptions{/*inChannels=*/3, kModelH, kModelW, /*sensorDim=*/9});
  net2->eval();
  torch::Tensor sensors = torch::zeros({1, 9});
  torch::Tensor y2 = net2->forward(x, sensors);
  EXPECT(y2.sizes() == torch::IntArrayRef({1, 2}), "sensor-fusion output shape");

  // --- UDP control packet round-trip + crc ---
  {
    ControlPacket c;
    c.seq = 0xDEADBEEF;
    c.tsMs = 0x0102030405060708ull;
    c.left = 1750;
    c.right = 1420;
    c.axes[0] = -12345;
    c.axes[5] = 32000;
    c.flagBits = flags::kRecording | flags::kHeartbeat;
    uint8_t buf[kControlPacketSize];
    EXPECT(encodeControl(c, buf) == kControlPacketSize, "encodeControl size");
    ControlPacket d;
    EXPECT(decodeControl(buf, sizeof(buf), d), "decodeControl ok");
    EXPECT(d.seq == c.seq && d.tsMs == c.tsMs && d.left == 1750 && d.right == 1420 &&
               d.axes[0] == -12345 && d.axes[5] == 32000 && d.recording() && !d.estop(),
           "control round-trip");
    buf[10] ^= 0xFF;  // corrupt a byte
    EXPECT(!decodeControl(buf, sizeof(buf), d), "crc rejects corruption");
  }
  // --- preview header round-trip ---
  {
    PreviewFragHeader h;
    h.frameId = 77;
    h.fragIndex = 3;
    h.fragCount = 9;
    h.totalSize = 123456;
    uint8_t buf[kPreviewHeaderSize];
    encodePreviewHeader(h, buf);
    PreviewFragHeader g;
    EXPECT(decodePreviewHeader(buf, sizeof(buf), g), "decodePreviewHeader ok");
    EXPECT(g.frameId == 77 && g.fragIndex == 3 && g.fragCount == 9 && g.totalSize == 123456,
           "preview header round-trip");
  }

  if (g_failures == 0) {
    std::printf("common_test: ALL PASSED\n");
    return 0;
  }
  std::printf("common_test: %d FAILURE(S)\n", g_failures);
  return 1;
}
