#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <vio/config.hpp>

namespace vio {

struct PoseRecoveryResult {
    Eigen::Matrix3d R_1_2{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t_hat_1_2{Eigen::Vector3d::Zero()};
    std::vector<std::uint8_t> refined_inlier_mask{};
    std::size_t inlier_count{0};
    bool success{false};
};

class PoseRecovery {
public:
    explicit PoseRecovery(const Config& config);

    // previous_points[i], current_points[i], and input_inlier_mask[i] must
    // refer to the same matched correspondence entry.
    PoseRecoveryResult RecoverPose(
        const cv::Mat& essential_matrix,
        const std::vector<cv::Point2f>& previous_points,
        const std::vector<cv::Point2f>& current_points,
        const std::vector<std::uint8_t>& input_inlier_mask) const;

private:
    GeometryConfig geometry_config_{};
    CameraCalibration camera_calibration_{};
};

}  // namespace vio
