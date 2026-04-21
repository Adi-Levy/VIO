#include "feature_detector.hpp"

#include <opencv2/imgproc.hpp>

namespace vio {

namespace {

bool IsSupportedGrayscaleImage(const cv::Mat& image_gray,
                               std::uint32_t min_image_width,
                               std::uint32_t min_image_height) {
    return !image_gray.empty() &&
           image_gray.type() == CV_8UC1 &&
           image_gray.cols >= static_cast<int>(min_image_width) &&
           image_gray.rows >= static_cast<int>(min_image_height);
}

}  // namespace

FeatureDetector::FeatureDetector(const Config& config)
    : visual_processing_config_(config.visual_processing),
      min_image_width_(config.min_image_width),
      min_image_height_(config.min_image_height) {}

FeatureDetectionResult FeatureDetector::Detect(const cv::Mat& image_gray) const {
    FeatureDetectionResult result;

    if (!IsSupportedGrayscaleImage(
            image_gray, min_image_width_, min_image_height_)) {
        return result;
    }

    cv::goodFeaturesToTrack(image_gray,
                            result.points,
                            static_cast<int>(visual_processing_config_.max_features),
                            visual_processing_config_.quality_level,
                            visual_processing_config_.min_feature_distance_px,
                            cv::noArray(),
                            visual_processing_config_.block_size_px);

    return result;
}

}  // namespace vio
