#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>

#include "estimator/prepared_correspondences.hpp"
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

    // The input correspondences are expected to be pre-filtered and already
    // normalized by the orchestrator measurement-preparation step.
    PoseRecoveryResult RecoverPose(
        const cv::Mat& essential_matrix,
        const PreparedCorrespondences& correspondences,
        const std::vector<std::uint8_t>& input_inlier_mask) const;

private:
    GeometryConfig geometry_config_{};
};

}  // namespace vio
