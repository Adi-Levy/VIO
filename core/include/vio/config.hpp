#pragma once

#include <cstdint>

#include <vio/types.hpp>

namespace vio {

enum class VisualProcessingMode {
    kKlt,
    kOrb,
};

struct VisualProcessingConfig {
    VisualProcessingMode mode{VisualProcessingMode::kKlt};
    std::uint32_t max_features{300};
    double min_feature_distance_px{15.0};
    double quality_level{0.01};
    int block_size_px{7};
};

struct KltTrackingConfig {
    int window_size_px{21};
    int pyramid_levels{3};
    int max_iterations{30};
    double epsilon{0.01};
    bool use_forward_backward_check{false};
};

struct GeometryConfig {
    double ransac_reprojection_threshold_px{1.0};
    double ransac_confidence{0.999};
    std::uint32_t min_inlier_count{25};
};

struct Phase1VoConfig {
    CameraCalibration camera_calibration{};
    VisualProcessingConfig visual_processing{};
    KltTrackingConfig klt_tracking{};
    GeometryConfig geometry{};
    std::uint32_t min_image_width{32};
    std::uint32_t min_image_height{32};
};

using Config = Phase1VoConfig;

}  // namespace vio
