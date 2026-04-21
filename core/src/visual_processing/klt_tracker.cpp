#include "klt_tracker.hpp"

#include "utils/validation.hpp"

#include <opencv2/video/tracking.hpp>

namespace vio {

namespace {

cv::Size BuildWindowSize(const KltTrackingConfig& tracking_config) {
    return cv::Size(tracking_config.window_size_px, tracking_config.window_size_px);
}

cv::TermCriteria BuildTerminationCriteria(const KltTrackingConfig& tracking_config) {
    return cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                            tracking_config.max_iterations,
                            tracking_config.epsilon);
}

}  // namespace

KltTracker::KltTracker(const Config& config)
    : tracking_config_(config.klt_tracking),
      min_image_width_(config.min_image_width),
      min_image_height_(config.min_image_height) {}

KltTrackResult KltTracker::Track(const cv::Mat& previous_image_gray,
                                 const cv::Mat& current_image_gray,
                                 const std::vector<cv::Point2f>& previous_points) const {
    KltTrackResult result;

    if (previous_points.empty() ||
        !IsSupportedTrackingImage(previous_image_gray,
                                  min_image_width_,
                                  min_image_height_) ||
        !IsSupportedTrackingImage(current_image_gray,
                                  min_image_width_,
                                  min_image_height_)) {
        return result;
    }

    std::vector<cv::Point2f> current_points;
    std::vector<std::uint8_t> status;
    std::vector<float> errors;

    cv::calcOpticalFlowPyrLK(previous_image_gray,
                             current_image_gray,
                             previous_points,
                             current_points,
                             status,
                             errors,
                             BuildWindowSize(tracking_config_),
                             tracking_config_.pyramid_levels,
                             BuildTerminationCriteria(tracking_config_));

    result.previous_points = previous_points;
    result.current_points = current_points;
    result.valid_mask = status;

    if (tracking_config_.use_forward_backward_check) {
        std::vector<cv::Point2f> backward_points;
        std::vector<std::uint8_t> backward_status;
        std::vector<float> backward_errors;

        cv::calcOpticalFlowPyrLK(current_image_gray,
                                 previous_image_gray,
                                 current_points,
                                 backward_points,
                                 backward_status,
                                 backward_errors,
                                 BuildWindowSize(tracking_config_),
                                 tracking_config_.pyramid_levels,
                                 BuildTerminationCriteria(tracking_config_));

        constexpr float kMaxForwardBackwardErrorPx = 1.0f;

        for (std::size_t i = 0; i < result.valid_mask.size(); ++i) {
            if (result.valid_mask[i] == 0 || backward_status[i] == 0) {
                result.valid_mask[i] = 0;
                continue;
            }

            const auto delta = backward_points[i] - previous_points[i];
            if (cv::norm(delta) > kMaxForwardBackwardErrorPx) {
                result.valid_mask[i] = 0;
            }
        }
    }

    for (const auto is_valid : result.valid_mask) {
        if (is_valid != 0) {
            ++result.tracked_count;
        }
    }

    return result;
}

}  // namespace vio
