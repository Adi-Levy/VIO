#include "vo_estimator.hpp"

#include "utils/validation.hpp"

#include <utility>

namespace vio {

VioEstimator::VioEstimator(Config config) : config_(std::move(config)) {}

bool VioEstimator::ProcessImageFrame(const ImageFrame& frame) {
    latest_estimate_.timestamp_ns = frame.timestamp_ns;
    latest_estimate_.valid = false;

    if (!IsSupportedTrackingImage(frame.image_gray,
                                  config_.min_image_width,
                                  config_.min_image_height)) {
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
