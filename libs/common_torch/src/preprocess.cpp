#include "common_torch/preprocess.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace rc {

torch::Tensor preprocess(const cv::Mat& bgr, const PreprocConfig& cfg, bool addBatch) {
  cv::Mat img = bgr;

  if (cfg.rotate180) {
    cv::Mat rotated;
    cv::rotate(img, rotated, cv::ROTATE_180);
    img = rotated;
  }

  // Center-width crop.
  if (cfg.cropRatio < 0.999f && cfg.cropRatio > 0.0f) {
    const int cropW = std::max(1, static_cast<int>(img.cols * cfg.cropRatio));
    const int x0 = (img.cols - cropW) / 2;
    img = img(cv::Rect(x0, 0, cropW, img.rows));
  }

  cv::Mat resized;
  cv::resize(img, resized, cv::Size(cfg.w, cfg.h), 0, 0, cv::INTER_AREA);

  int channels = 3;
  if (cfg.channels == Channels::Gray) {
    cv::Mat gray;
    cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);
    resized = gray;
    channels = 1;
  }

  // To float [0,1], contiguous, then HWC -> CHW.
  cv::Mat f32;
  resized.convertTo(f32, CV_32F, 1.0 / 255.0);
  if (!f32.isContinuous()) f32 = f32.clone();

  torch::Tensor t = torch::from_blob(f32.data, {cfg.h, cfg.w, channels}, torch::kFloat32)
                        .permute({2, 0, 1})    // CHW
                        .contiguous()
                        .clone();              // own the memory (f32 is local)

  return addBatch ? t.unsqueeze(0) : t;
}

}  // namespace rc
