#include "validation.hpp"

#include <cmath>

#include "image_utils.hpp"

namespace vio {

bool MeetsMinimumImageSize(const cv::Mat& image,
                           std::uint32_t min_image_width,
                           std::uint32_t min_image_height) {
    return image.cols >= static_cast<int>(min_image_width) &&
           image.rows >= static_cast<int>(min_image_height);
}

bool IsSupportedTrackingImage(const cv::Mat& image,
                              std::uint32_t min_image_width,
                              std::uint32_t min_image_height) {
    return IsGrayscaleImage(image) &&
           MeetsMinimumImageSize(image, min_image_width, min_image_height);
}

bool HasValidCameraIntrinsics(const CameraCalibration& camera_calibration) {
    const auto& intrinsics = camera_calibration.intrinsics;
    return intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
}

bool HasValidPointCorrespondences(const std::vector<cv::Point2f>& previous_points,
                                  const std::vector<cv::Point2f>& current_points,
                                  std::uint32_t min_correspondence_count) {
    if (previous_points.size() != current_points.size() ||
        previous_points.size() < min_correspondence_count) {
        return false;
    }

    for (std::size_t i = 0; i < previous_points.size(); ++i) {
        const auto& previous_point = previous_points[i];
        const auto& current_point = current_points[i];

        if (!std::isfinite(previous_point.x) || !std::isfinite(previous_point.y) ||
            !std::isfinite(current_point.x) || !std::isfinite(current_point.y)) {
            return false;
        }
    }

    return true;
}

bool ArePointsWithinImageBounds(
    const std::vector<cv::Point2f>& points,
    const PinholeIntrinsics& intrinsics) {
    if (intrinsics.width <= 0 || intrinsics.height <= 0) {
        return true;
    }

    for (const auto& point : points) {
        if (point.x < 0.0f || point.y < 0.0f ||
            point.x >= static_cast<float>(intrinsics.width) ||
            point.y >= static_cast<float>(intrinsics.height)) {
            return false;
        }
    }

    return true;
}

bool HasConsistentPreparedCorrespondences(
    const PreparedCorrespondences& correspondences) {
    return correspondences.success &&
           correspondences.previous_image_points.size() ==
               correspondences.current_image_points.size() &&
           correspondences.previous_image_points.size() ==
               correspondences.previous_normalized_points.size() &&
           correspondences.previous_image_points.size() ==
               correspondences.current_normalized_points.size();
}

bool AreSupportedEssentialMatrixInputs(
    const PreparedCorrespondences& correspondences,
    const GeometryConfig& geometry_config) {
    if (!HasConsistentPreparedCorrespondences(correspondences) ||
        correspondences.Count() < geometry_config.min_correspondence_count) {
        return false;
    }
    return true;
}

bool HasUsableEssentialMatrix(const cv::Mat& essential_matrix) {
    return !essential_matrix.empty() &&
           essential_matrix.rows == 3 &&
           essential_matrix.cols == 3;
}

bool AreSupportedPoseRecoveryInputs(
    const cv::Mat& essential_matrix,
    const PreparedCorrespondences& correspondences,
    const std::vector<std::uint8_t>& input_inlier_mask,
    const GeometryConfig& geometry_config) {
    return HasUsableEssentialMatrix(essential_matrix) &&
           AreSupportedEssentialMatrixInputs(correspondences, geometry_config) &&
           input_inlier_mask.size() == correspondences.Count();
}

}  // namespace vio
