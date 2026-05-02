#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "estimator/prepared_correspondences.hpp"

#include <vio/config.hpp>

namespace vio {

bool MeetsMinimumImageSize(const cv::Mat& image,
                           std::uint32_t min_image_width,
                           std::uint32_t min_image_height);

bool IsSupportedTrackingImage(const cv::Mat& image,
                              std::uint32_t min_image_width,
                              std::uint32_t min_image_height);

bool HasValidCameraIntrinsics(const CameraCalibration& camera_calibration);

bool HasValidPointCorrespondences(const std::vector<cv::Point2f>& previous_points,
                                  const std::vector<cv::Point2f>& current_points,
                                  std::uint32_t min_correspondence_count);

bool ArePointsWithinImageBounds(
    const std::vector<cv::Point2f>& points,
    const PinholeIntrinsics& intrinsics);

bool HasConsistentPreparedCorrespondences(
    const PreparedCorrespondences& correspondences);

bool AreSupportedEssentialMatrixInputs(
    const PreparedCorrespondences& correspondences,
    const GeometryConfig& geometry_config);

bool HasUsableEssentialMatrix(const cv::Mat& essential_matrix);

bool AreSupportedPoseRecoveryInputs(
    const cv::Mat& essential_matrix,
    const PreparedCorrespondences& correspondences,
    const std::vector<std::uint8_t>& input_inlier_mask,
    const GeometryConfig& geometry_config);

}  // namespace vio
