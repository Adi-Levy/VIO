#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <vio/config.hpp>

namespace vio {

struct EssentialMatrixResult {
    cv::Mat essential_matrix{};
    std::vector<std::uint8_t> inlier_mask{};
    std::size_t inlier_count{0};
    bool success{false};
};

class EssentialMatrixSolver {
public:
    explicit EssentialMatrixSolver(const Config& config);

    // previous_points[i] and current_points[i] must form a matched 2D-2D
    // correspondence pair. This solver does not perform matching or reorder
    // the inputs.
    EssentialMatrixResult Solve(const std::vector<cv::Point2f>& previous_points,
                                const std::vector<cv::Point2f>& current_points) const;

private:
    GeometryConfig geometry_config_{};
    CameraCalibration camera_calibration_{};
};

}  // namespace vio
