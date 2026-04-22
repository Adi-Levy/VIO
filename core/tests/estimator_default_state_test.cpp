#include <gtest/gtest.h>

#include <vio/vio_estimator.hpp>

TEST(VioEstimatorTest, DefaultStateStartsWaitingForData) {
    vio::VioEstimator estimator;

    const auto& estimate = estimator.GetLatestEstimate();
    const auto& diagnostics = estimator.GetLatestDiagnostics();

    EXPECT_EQ(estimate.timestamp_ns, 0);
    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ(diagnostics.runtime_status, vio::RuntimeStatus::kWaitingForData);
}
