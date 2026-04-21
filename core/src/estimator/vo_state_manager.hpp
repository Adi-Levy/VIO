#pragma once

#include <vio/types.hpp>

namespace vio {

class VoStateManager {
public:
    void Reset();
    void SetLatestEstimate(const StateEstimate& estimate);
    const StateEstimate& GetLatestEstimate() const;

private:
    StateEstimate latest_estimate_{};
};

}  // namespace vio
