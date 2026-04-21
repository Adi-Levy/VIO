#include "vo_state_manager.hpp"

namespace vio {

void VoStateManager::Reset() {
    latest_estimate_ = {};
}

void VoStateManager::SetLatestEstimate(const StateEstimate& estimate) {
    latest_estimate_ = estimate;
}

const StateEstimate& VoStateManager::GetLatestEstimate() const {
    return latest_estimate_;
}

}  // namespace vio
