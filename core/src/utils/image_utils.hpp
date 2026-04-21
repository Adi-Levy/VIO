#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

namespace vio {

bool IsGrayscaleImage(const cv::Mat& image);

}  // namespace vio
