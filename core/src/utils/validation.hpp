#pragma once

#include <cstdint>

#include <opencv2/core/mat.hpp>

namespace vio {

bool MeetsMinimumImageSize(const cv::Mat& image,
                           std::uint32_t min_image_width,
                           std::uint32_t min_image_height);

bool IsSupportedTrackingImage(const cv::Mat& image,
                              std::uint32_t min_image_width,
                              std::uint32_t min_image_height);

}  // namespace vio
