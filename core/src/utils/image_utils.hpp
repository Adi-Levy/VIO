#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

#include <vio/types.hpp>

namespace vio {

bool IsGrayscaleImage(const cv::Mat& image);
cv::Mat BuildCameraMatrix(const CameraCalibration& camera_calibration);
cv::Mat ConvertMaskToMat(const std::vector<std::uint8_t>& mask);
std::vector<std::uint8_t> ConvertMaskToVector(const cv::Mat& mask);
std::size_t CountNonZeroMaskEntries(const cv::Mat& mask);

}  // namespace vio
