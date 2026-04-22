#include <gtest/gtest.h>

#include <vio/vio_estimator.hpp>

#include <opencv2/core/mat.hpp>

TEST(VioEstimatorTest, ValidFrameTransitionsEstimatorToInitializing) {
    vio::VioEstimator estimator;

    vio::ImageFrame frame;
    frame.timestamp_ns = 1;
    frame.image_gray = cv::Mat::zeros(64, 64, CV_8UC1);

    EXPECT_TRUE(estimator.ProcessImageFrame(frame));

    const auto& estimate = estimator.GetLatestEstimate();
    const auto& diagnostics = estimator.GetLatestDiagnostics();

    EXPECT_EQ(estimate.timestamp_ns, frame.timestamp_ns);
    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ(diagnostics.runtime_status, vio::RuntimeStatus::kInitializing);
}
