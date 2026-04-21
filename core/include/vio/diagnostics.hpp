#pragma once

#include <cstdint>

#include <vio/enums.hpp>

namespace vio {

struct Diagnostics {
    RuntimeStatus runtime_status{RuntimeStatus::kWaitingForData};
    std::uint32_t detected_features{0};
    std::uint32_t tracked_features{0};
    std::uint32_t inlier_count{0};
    double inlier_ratio{0.0};
    bool pose_recovery_success{false};
    bool triangulation_success{false};
};

}  // namespace vio
