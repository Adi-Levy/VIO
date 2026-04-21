#include "visual_processing/feature_detector.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

namespace {

cv::Mat CreateCornerRichImage() {
    cv::Mat image = cv::Mat::zeros(128, 128, CV_8UC1);
    cv::rectangle(image, cv::Rect(16, 16, 24, 24), cv::Scalar(255), cv::FILLED);
    cv::rectangle(image, cv::Rect(80, 20, 20, 20), cv::Scalar(255), cv::FILLED);
    cv::rectangle(image, cv::Rect(24, 80, 28, 28), cv::Scalar(255), cv::FILLED);
    return image;
}

bool DetectsFeaturesOnStructuredImage() {
    vio::Config config;
    config.visual_processing.max_features = 20;
    config.visual_processing.min_feature_distance_px = 5.0;
    config.visual_processing.quality_level = 0.01;
    config.visual_processing.block_size_px = 3;

    const vio::FeatureDetector detector(config);
    const auto result = detector.Detect(CreateCornerRichImage());

    return result.Count() >= 4 && result.Count() <= config.visual_processing.max_features;
}

bool ReturnsNoFeaturesForEmptyImage() {
    const vio::FeatureDetector detector(vio::Config{});
    const auto result = detector.Detect(cv::Mat{});

    return result.Count() == 0;
}

bool ReturnsNoFeaturesForTooSmallImage() {
    vio::Config config;
    config.min_image_width = 32;
    config.min_image_height = 32;

    const vio::FeatureDetector detector(config);
    const auto result = detector.Detect(cv::Mat::zeros(16, 16, CV_8UC1));

    return result.Count() == 0;
}

}  // namespace

int main() {
    return (DetectsFeaturesOnStructuredImage() &&
            ReturnsNoFeaturesForEmptyImage() &&
            ReturnsNoFeaturesForTooSmallImage())
               ? 0
               : 1;
}
