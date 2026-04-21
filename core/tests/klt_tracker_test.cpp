#include "visual_processing/feature_detector.hpp"
#include "visual_processing/klt_tracker.hpp"

#include <cmath>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

namespace {

cv::Mat CreateCornerRichImage() {
    cv::Mat image = cv::Mat::zeros(160, 160, CV_8UC1);
    cv::rectangle(image, cv::Rect(20, 20, 24, 24), cv::Scalar(255), cv::FILLED);
    cv::rectangle(image, cv::Rect(100, 25, 22, 22), cv::Scalar(255), cv::FILLED);
    cv::rectangle(image, cv::Rect(40, 95, 26, 26), cv::Scalar(255), cv::FILLED);
    return image;
}

cv::Mat TranslateImage(const cv::Mat& image, double dx, double dy) {
    cv::Mat translated;
    const cv::Mat transform = (cv::Mat_<double>(2, 3) << 1.0, 0.0, dx, 0.0, 1.0, dy);
    cv::warpAffine(image,
                   translated,
                   transform,
                   image.size(),
                   cv::INTER_NEAREST,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(0));
    return translated;
}

bool TracksTranslatedFeatures() {
    vio::Config config;
    config.visual_processing.max_features = 20;
    config.visual_processing.min_feature_distance_px = 5.0;
    config.visual_processing.quality_level = 0.01;
    config.visual_processing.block_size_px = 3;
    config.klt_tracking.window_size_px = 21;
    config.klt_tracking.pyramid_levels = 3;
    config.klt_tracking.max_iterations = 30;
    config.klt_tracking.epsilon = 0.01;

    const auto previous_image = CreateCornerRichImage();
    const auto current_image = TranslateImage(previous_image, 4.0, 3.0);

    const vio::FeatureDetector detector(config);
    const auto detection_result = detector.Detect(previous_image);
    if (detection_result.Count() < 4) {
        return false;
    }

    const vio::KltTracker tracker(config);
    const auto track_result =
        tracker.Track(previous_image, current_image, detection_result.points);

    if (track_result.tracked_count < 4 ||
        track_result.tracked_count > detection_result.Count()) {
        return false;
    }

    for (std::size_t i = 0; i < track_result.valid_mask.size(); ++i) {
        if (track_result.valid_mask[i] == 0) {
            continue;
        }

        const auto delta =
            track_result.current_points[i] - track_result.previous_points[i];
        if (std::abs(delta.x - 4.0f) > 0.75f || std::abs(delta.y - 3.0f) > 0.75f) {
            return false;
        }
    }

    return true;
}

bool RejectsEmptyInputs() {
    const vio::KltTracker tracker(vio::Config{});
    const auto result = tracker.Track(cv::Mat{}, cv::Mat{}, {});
    return result.tracked_count == 0 &&
           result.previous_points.empty() &&
           result.current_points.empty() &&
           result.valid_mask.empty();
}

bool RejectsTooSmallImages() {
    vio::Config config;
    config.min_image_width = 32;
    config.min_image_height = 32;

    const cv::Mat small_image = cv::Mat::zeros(16, 16, CV_8UC1);
    const std::vector<cv::Point2f> points{cv::Point2f(4.0F, 4.0F)};

    const vio::KltTracker tracker(config);
    const auto result = tracker.Track(small_image, small_image, points);

    return result.tracked_count == 0 &&
           result.previous_points.empty() &&
           result.current_points.empty() &&
           result.valid_mask.empty();
}

}  // namespace

int main() {
    return (TracksTranslatedFeatures() &&
            RejectsEmptyInputs() &&
            RejectsTooSmallImages())
               ? 0
               : 1;
}
