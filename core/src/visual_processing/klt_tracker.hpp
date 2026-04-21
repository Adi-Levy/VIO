#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <vio/config.hpp>

namespace vio {

struct KltTrackResult {
    std::vector<cv::Point2f> previous_points{};
    std::vector<cv::Point2f> current_points{};
    std::vector<std::uint8_t> valid_mask{};
    std::size_t tracked_count{0};
};

class KltTracker {
public:
    explicit KltTracker(const Config& config);

    KltTrackResult Track(const cv::Mat& previous_image_gray,
                         const cv::Mat& current_image_gray,
                         const std::vector<cv::Point2f>& previous_points) const;

private:
    KltTrackingConfig tracking_config_{};
    std::uint32_t min_image_width_{0};
    std::uint32_t min_image_height_{0};
};

}  // namespace vio
