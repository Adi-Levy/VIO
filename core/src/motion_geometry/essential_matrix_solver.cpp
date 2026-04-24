#include "essential_matrix_solver.hpp"

#include "utils/image_utils.hpp"
#include "utils/validation.hpp"

#include <opencv2/calib3d.hpp>

namespace vio {

EssentialMatrixSolver::EssentialMatrixSolver(const Config& config)
    : geometry_config_(config.geometry),
      camera_calibration_(config.camera_calibration) {}

EssentialMatrixResult EssentialMatrixSolver::Solve(
    const std::vector<cv::Point2f>& previous_points,
    const std::vector<cv::Point2f>& current_points) const {
    EssentialMatrixResult result;
    const std::size_t correspondence_count = previous_points.size();

    if (!AreSupportedEssentialMatrixInputs(previous_points,
                                           current_points,
                                           camera_calibration_,
                                           geometry_config_)) {
        return result;
    }

    cv::Mat inlier_mask;
    result.essential_matrix = cv::findEssentialMat(
        previous_points,
        current_points,
        BuildCameraMatrix(camera_calibration_),
        cv::RANSAC,
        geometry_config_.ransac_confidence,
        geometry_config_.ransac_reprojection_threshold_px,
        inlier_mask);

    result.inlier_mask = ConvertMaskToVector(inlier_mask);
    result.inlier_count = CountNonZeroMaskEntries(inlier_mask);
    result.success = !result.essential_matrix.empty() &&
                     result.inlier_mask.size() == correspondence_count &&
                     result.inlier_count >= geometry_config_.min_inlier_count;
    return result;
}

}  // namespace vio
