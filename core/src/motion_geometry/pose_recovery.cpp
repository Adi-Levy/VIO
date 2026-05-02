#include "pose_recovery.hpp"

#include "utils/image_utils.hpp"
#include "utils/validation.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace vio {

PoseRecovery::PoseRecovery(const Config& config)
    : geometry_config_(config.geometry) {}

PoseRecoveryResult PoseRecovery::RecoverPose(
    const cv::Mat& essential_matrix,
    const PreparedCorrespondences& correspondences,
    const std::vector<std::uint8_t>& input_inlier_mask) const {
    PoseRecoveryResult result;

    if (!AreSupportedPoseRecoveryInputs(essential_matrix,
                                        correspondences,
                                        input_inlier_mask,
                                        geometry_config_)) {
        return result;
    }

    cv::Mat rotation;
    cv::Mat translation;
    cv::Mat refined_mask = ConvertMaskToMat(input_inlier_mask);

    const int recovered_inliers =
        cv::recoverPose(essential_matrix,
                        correspondences.previous_normalized_points,
                        correspondences.current_normalized_points,
                        rotation,
                        translation,
                        1.0,
                        cv::Point2d(0.0, 0.0),
                        refined_mask);

    if (recovered_inliers < static_cast<int>(geometry_config_.min_inlier_count)) {
        return result;
    }

    cv::cv2eigen(rotation, result.R_1_2);
    result.t_hat_1_2 = Eigen::Vector3d(translation.at<double>(0, 0),
                                       translation.at<double>(1, 0),
                                       translation.at<double>(2, 0));
    result.refined_inlier_mask = ConvertMaskToVector(refined_mask);
    result.inlier_count = static_cast<std::size_t>(recovered_inliers);
    result.success = true;
    return result;
}

}  // namespace vio
