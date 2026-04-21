#include "vo_estimator.hpp"

#include <utility>

namespace vio {

namespace {

bool IsImageFrameValid(const ImageFrame& frame, const Config& config) {
    return !frame.image_gray.empty() &&
           frame.image_gray.cols >=
               static_cast<int>(config.min_image_width) &&
           frame.image_gray.rows >=
               static_cast<int>(config.min_image_height);
}

}  // namespace

VioEstimator::VioEstimator(Config config) : config_(std::move(config)) {}

bool VioEstimator::ProcessImageFrame(const ImageFrame& frame) {
    latest_estimate_.timestamp_ns = frame.timestamp_ns;
    latest_estimate_.valid = false;

    if (!IsImageFrameValid(frame, config_)) {
        latest_diagnostics_.runtime_status = RuntimeStatus::kWaitingForData;
        latest_diagnostics_.pose_recovery_success = false;
        latest_diagnostics_.triangulation_success = false;
        return false;
    }

    has_received_frame_ = true;
    latest_diagnostics_.runtime_status = RuntimeStatus::kInitializing;
    latest_diagnostics_.pose_recovery_success = false;
    latest_diagnostics_.triangulation_success = false;
    return true;
}

const StateEstimate& VioEstimator::GetLatestEstimate() const {
    return latest_estimate_;
}

const Diagnostics& VioEstimator::GetLatestDiagnostics() const {
    return latest_diagnostics_;
}

void VioEstimator::Reset() {
    latest_estimate_ = {};
    latest_diagnostics_ = {};
    has_received_frame_ = false;
}

}  // namespace vio
