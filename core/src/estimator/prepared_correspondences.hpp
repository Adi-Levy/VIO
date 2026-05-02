#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core/types.hpp>

namespace vio {

struct PreparedCorrespondences {
    std::vector<cv::Point2f> previous_image_points{};
    std::vector<cv::Point2f> current_image_points{};
    std::vector<cv::Point2f> previous_normalized_points{};
    std::vector<cv::Point2f> current_normalized_points{};
    bool success{false};

    std::size_t Count() const {
        return previous_image_points.size();
    }
};

}  // namespace vio
