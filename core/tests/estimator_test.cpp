#include <gtest/gtest.h>

#include <vio/vio_estimator.hpp>

#include <opencv2/core/mat.hpp>

TEST(VioEstimatorTest, DefaultStateStartsWaitingForData) {
    vio::VioEstimator estimator;

    const auto& estimate = estimator.GetLatestEstimate();
    const auto& diagnostics = estimator.GetLatestDiagnostics();

    EXPECT_EQ(estimate.timestamp_ns, 0);
    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ(diagnostics.runtime_status, vio::RuntimeStatus::kWaitingForData);
}

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

TEST(VioEstimatorTest, ResetRestoresDefaultState) {
    vio::VioEstimator estimator;

    vio::ImageFrame frame;
    frame.timestamp_ns = 4;
    frame.image_gray = cv::Mat::zeros(64, 64, CV_8UC1);

    ASSERT_TRUE(estimator.ProcessImageFrame(frame));

    estimator.Reset();

    const auto& estimate = estimator.GetLatestEstimate();
    const auto& diagnostics = estimator.GetLatestDiagnostics();

    EXPECT_EQ(estimate.timestamp_ns, 0);
    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ(diagnostics.runtime_status, vio::RuntimeStatus::kWaitingForData);
}
