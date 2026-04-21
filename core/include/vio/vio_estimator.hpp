#pragma once

#include <vio/config.hpp>
#include <vio/diagnostics.hpp>
#include <vio/types.hpp>

namespace vio {

class VioEstimator {
public:
    explicit VioEstimator(Config config = {});

    bool ProcessImageFrame(const ImageFrame& frame);
    const StateEstimate& GetLatestEstimate() const;
    const Diagnostics& GetLatestDiagnostics() const;
    void Reset();

private:
    Config config_{};
    StateEstimate latest_estimate_{};
    Diagnostics latest_diagnostics_{};
    bool has_received_frame_{false};
};

}  // namespace vio
