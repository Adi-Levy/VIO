#include "image_utils.hpp"

#include <opencv2/calib3d.hpp>

namespace vio {

bool IsGrayscaleImage(const cv::Mat& image) {
    return !image.empty() && image.type() == CV_8UC1;
}

cv::Mat BuildCameraMatrix(const CameraCalibration& camera_calibration) {
    const auto& intrinsics = camera_calibration.intrinsics;
    return (cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0,
            intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0);
}

cv::Mat BuildDistortionCoefficientsVector(const DistortionCoeffs& distortion) {
    if (distortion.coeffs.empty()) {
        return {};
    }

    cv::Mat coefficients(static_cast<int>(distortion.coeffs.size()), 1, CV_64F);
    for (std::size_t i = 0; i < distortion.coeffs.size(); ++i) {
        coefficients.at<double>(static_cast<int>(i), 0) = distortion.coeffs[i];
    }
    return coefficients;
}

cv::Mat ConvertMaskToMat(const std::vector<std::uint8_t>& mask) {
    if (mask.empty()) {
        return {};
    }

    cv::Mat mat(static_cast<int>(mask.size()), 1, CV_8U);
    for (std::size_t i = 0; i < mask.size(); ++i) {
        mat.at<std::uint8_t>(static_cast<int>(i), 0) = mask[i];
    }
    return mat;
}

std::vector<std::uint8_t> ConvertMaskToVector(const cv::Mat& mask) {
    if (mask.empty()) {
        return {};
    }

    const auto* begin = mask.ptr<std::uint8_t>();
    return std::vector<std::uint8_t>(begin, begin + mask.total());
}

std::size_t CountNonZeroMaskEntries(const cv::Mat& mask) {
    return static_cast<std::size_t>(cv::countNonZero(mask));
}

PreparedCorrespondences PrepareCorrespondences(
    const std::vector<cv::Point2f>& previous_image_points,
    const std::vector<cv::Point2f>& current_image_points,
    const std::vector<std::uint8_t>& valid_mask,
    const CameraCalibration& camera_calibration,
    const GeometryConfig& geometry_config) {
    PreparedCorrespondences prepared;

    const auto& intrinsics = camera_calibration.intrinsics;
    if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
        return prepared;
    }

    if (previous_image_points.size() != current_image_points.size() ||
        previous_image_points.size() != valid_mask.size()) {
        return prepared;
    }

    for (std::size_t i = 0; i < valid_mask.size(); ++i) {
        if (valid_mask[i] == 0) {
            continue;
        }

        const auto& previous_point = previous_image_points[i];
        const auto& current_point = current_image_points[i];

        if (geometry_config.require_points_within_image_bounds &&
            intrinsics.width > 0 && intrinsics.height > 0 &&
            (previous_point.x < 0.0f || previous_point.y < 0.0f ||
             current_point.x < 0.0f || current_point.y < 0.0f ||
             previous_point.x >= static_cast<float>(intrinsics.width) ||
             previous_point.y >= static_cast<float>(intrinsics.height) ||
             current_point.x >= static_cast<float>(intrinsics.width) ||
             current_point.y >= static_cast<float>(intrinsics.height))) {
            continue;
        }

        prepared.previous_image_points.push_back(previous_point);
        prepared.current_image_points.push_back(current_point);
    }

    if (prepared.previous_image_points.empty()) {
        return prepared;
    }

    cv::undistortPoints(prepared.previous_image_points,
                        prepared.previous_normalized_points,
                        BuildCameraMatrix(camera_calibration),
                        BuildDistortionCoefficientsVector(camera_calibration.distortion));
    cv::undistortPoints(prepared.current_image_points,
                        prepared.current_normalized_points,
                        BuildCameraMatrix(camera_calibration),
                        BuildDistortionCoefficientsVector(camera_calibration.distortion));

    prepared.success =
        prepared.previous_image_points.size() == prepared.current_image_points.size() &&
        prepared.previous_image_points.size() == prepared.previous_normalized_points.size() &&
        prepared.previous_image_points.size() == prepared.current_normalized_points.size();
    return prepared;
}

}  // namespace vio
