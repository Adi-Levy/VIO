#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "estimator/prepared_correspondences.hpp"
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

    // The input correspondences are expected to be pre-filtered and already
    // normalized by the orchestrator measurement-preparation step.
    EssentialMatrixResult Solve(
        const PreparedCorrespondences& correspondences) const;

private:
    GeometryConfig geometry_config_{};
};

}  // namespace vio
