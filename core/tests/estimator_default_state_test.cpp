#include <vio/vio_estimator.hpp>

int main() {
    vio::VioEstimator estimator;

    const auto& estimate = estimator.GetLatestEstimate();
    const auto& diagnostics = estimator.GetLatestDiagnostics();

    if (estimate.timestamp_ns != 0) {
        return 1;
    }

    if (estimate.valid) {
        return 1;
    }

    if (diagnostics.runtime_status != vio::RuntimeStatus::kWaitingForData) {
        return 1;
    }

    return 0;
}
