#pragma once

namespace vio {

enum class RuntimeStatus {
    kWaitingForData,
    kInitializing,
    kTracking,
    kDegraded,
    kLost,
};

}  // namespace vio
