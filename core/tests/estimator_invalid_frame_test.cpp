#include <gtest/gtest.h>

#include <vio/vio_estimator.hpp>

#include <opencv2/core/mat.hpp>

TEST(VioEstimatorTest, RejectsEmptyFrame) {
    vio::VioEstimator estimator;
    vio::ImageFrame frame;
    frame.timestamp_ns = 2;

    EXPECT_FALSE(estimator.ProcessImageFrame(frame));

    const auto& diagnostics = estimator.GetLatestDiagnostics();
    EXPECT_EQ(diagnostics.runtime_status, vio::RuntimeStatus::kWaitingForData);
}

TEST(VioEstimatorTest, RejectsTooSmallFrame) {
    vio::VioEstimator estimator;
    vio::ImageFrame frame;
    frame.timestamp_ns = 3;
    frame.image_gray = cv::Mat::zeros(16, 16, CV_8UC1);

    EXPECT_FALSE(estimator.ProcessImageFrame(frame));

    const auto& diagnostics = estimator.GetLatestDiagnostics();
    EXPECT_EQ(diagnostics.runtime_status, vio::RuntimeStatus::kWaitingForData);
}
