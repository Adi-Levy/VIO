#include "feature_detector.hpp"

#include "utils/validation.hpp"

#include <opencv2/imgproc.hpp>

namespace vio {

FeatureDetector::FeatureDetector(const Config& config)
    : visual_processing_config_(config.visual_processing),
      min_image_width_(config.min_image_width),
      min_image_height_(config.min_image_height) {}

FeatureDetectionResult FeatureDetector::Detect(const cv::Mat& image_gray) const {
    FeatureDetectionResult result;

    if (!IsSupportedTrackingImage(
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
