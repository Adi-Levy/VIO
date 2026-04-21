#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <vio/config.hpp>

namespace vio {

struct FeatureDetectionResult {
    std::vector<cv::Point2f> points{};

    std::size_t Count() const {
        return points.size();
    }
};

class FeatureDetector {
public:
    explicit FeatureDetector(const Config& config);

    FeatureDetectionResult Detect(const cv::Mat& image_gray) const;

private:
    VisualProcessingConfig visual_processing_config_{};
    std::uint32_t min_image_width_{0};
    std::uint32_t min_image_height_{0};
};

}  // namespace vio
