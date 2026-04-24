#include "pose_recovery.hpp"

#include "utils/image_utils.hpp"
#include "utils/validation.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace vio {

PoseRecovery::PoseRecovery(const Config& config)
    : geometry_config_(config.geometry),
      camera_calibration_(config.camera_calibration) {}

PoseRecoveryResult PoseRecovery::RecoverPose(
    const cv::Mat& essential_matrix,
    const std::vector<cv::Point2f>& previous_points,
    const std::vector<cv::Point2f>& current_points,
    const std::vector<std::uint8_t>& input_inlier_mask) const {
    PoseRecoveryResult result;

    if (!AreSupportedPoseRecoveryInputs(essential_matrix,
                                        previous_points,
                                        current_points,
                                        input_inlier_mask,
                                        camera_calibration_,
                                        geometry_config_)) {
        return result;
    }

    cv::Mat rotation;
    cv::Mat translation;
    cv::Mat refined_mask = ConvertMaskToMat(input_inlier_mask);

    const int recovered_inliers = cv::recoverPose(essential_matrix,
                                                  previous_points,
                                                  current_points,
                                                  BuildCameraMatrix(camera_calibration_),
                                                  rotation,
                                                  translation,
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
