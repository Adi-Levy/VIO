#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

#include "estimator/prepared_correspondences.hpp"

#include <vio/config.hpp>
#include <vio/types.hpp>

namespace vio {

bool IsGrayscaleImage(const cv::Mat& image);
cv::Mat BuildCameraMatrix(const CameraCalibration& camera_calibration);
cv::Mat BuildDistortionCoefficientsVector(const DistortionCoeffs& distortion);
cv::Mat ConvertMaskToMat(const std::vector<std::uint8_t>& mask);
std::vector<std::uint8_t> ConvertMaskToVector(const cv::Mat& mask);
std::size_t CountNonZeroMaskEntries(const cv::Mat& mask);
PreparedCorrespondences PrepareCorrespondences(
    const std::vector<cv::Point2f>& previous_image_points,
    const std::vector<cv::Point2f>& current_image_points,
    const std::vector<std::uint8_t>& valid_mask,
    const CameraCalibration& camera_calibration,
    const GeometryConfig& geometry_config);

}  // namespace vio
