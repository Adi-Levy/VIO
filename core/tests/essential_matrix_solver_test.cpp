#include <gtest/gtest.h>

#include "motion_geometry/essential_matrix_solver.hpp"

#include <opencv2/calib3d.hpp>

namespace {

vio::Config MakeGeometryConfig() {
    vio::Config config;
    config.camera_calibration.intrinsics.fx = 400.0;
    config.camera_calibration.intrinsics.fy = 400.0;
    config.camera_calibration.intrinsics.cx = 320.0;
    config.camera_calibration.intrinsics.cy = 240.0;
    config.camera_calibration.intrinsics.width = 640;
    config.camera_calibration.intrinsics.height = 480;
    config.geometry.min_correspondence_count = 5;
    config.geometry.ransac_reprojection_threshold_px = 1.0;
    config.geometry.ransac_confidence = 0.999;
    config.geometry.min_inlier_count = 5;
    return config;
}

cv::Mat MakeCameraMatrix(const vio::Config& config) {
    const auto& intrinsics = config.camera_calibration.intrinsics;
    return (cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0,
            intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0);
}

void GenerateProjectedCorrespondences(const vio::Config& config,
                                      std::vector<cv::Point2f>* previous_points,
                                      std::vector<cv::Point2f>* current_points) {
    ASSERT_NE(previous_points, nullptr);
    ASSERT_NE(current_points, nullptr);

    const cv::Mat camera_matrix = MakeCameraMatrix(config);
    const cv::Mat zero_distortion = cv::Mat::zeros(4, 1, CV_64F);

    const std::vector<cv::Point3f> world_points{
        {-0.6f, -0.4f, 4.5f},
        {-0.2f, 0.1f, 4.8f},
        {0.3f, -0.1f, 5.1f},
        {0.5f, 0.4f, 5.4f},
        {-0.4f, 0.5f, 4.7f},
        {0.1f, -0.5f, 5.0f},
        {0.6f, 0.2f, 5.6f},
        {-0.1f, 0.3f, 5.2f},
    };

    const cv::Vec3d rvec_1(0.0, 0.0, 0.0);
    const cv::Vec3d tvec_1(0.0, 0.0, 0.0);
    const cv::Vec3d rvec_2(0.0, 0.02, -0.01);
    const cv::Vec3d tvec_2(0.2, 0.0, 0.0);

    cv::projectPoints(world_points,
                      rvec_1,
                      tvec_1,
                      camera_matrix,
                      zero_distortion,
                      *previous_points);
    cv::projectPoints(world_points,
                      rvec_2,
                      tvec_2,
                      camera_matrix,
                      zero_distortion,
                      *current_points);
}

TEST(EssentialMatrixSolverTest, RejectsInsufficientCorrespondences) {
    const vio::EssentialMatrixSolver solver(MakeGeometryConfig());

    const std::vector<cv::Point2f> previous_points{
        {10.0f, 10.0f}, {20.0f, 20.0f}, {30.0f, 20.0f}, {40.0f, 15.0f}};
    const std::vector<cv::Point2f> current_points{
        {12.0f, 11.0f}, {22.0f, 21.0f}, {31.0f, 19.0f}, {41.0f, 16.0f}};

    const auto result = solver.Solve(previous_points, current_points);

    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.essential_matrix.empty());
    EXPECT_TRUE(result.inlier_mask.empty());
    EXPECT_EQ(result.inlier_count, 0U);
}

TEST(EssentialMatrixSolverTest, RejectsOutOfBoundsCorrespondencesWhenEnabled) {
    const vio::EssentialMatrixSolver solver(MakeGeometryConfig());

    const std::vector<cv::Point2f> previous_points{
        {10.0f, 10.0f}, {20.0f, 20.0f}, {30.0f, 20.0f},
        {40.0f, 15.0f}, {50.0f, 25.0f}};
    const std::vector<cv::Point2f> current_points{
        {12.0f, 11.0f}, {22.0f, 21.0f}, {31.0f, 19.0f},
        {641.0f, 16.0f}, {52.0f, 24.0f}};

    const auto result = solver.Solve(previous_points, current_points);

    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.essential_matrix.empty());
    EXPECT_TRUE(result.inlier_mask.empty());
    EXPECT_EQ(result.inlier_count, 0U);
}

TEST(EssentialMatrixSolverTest, EstimatesEssentialMatrixForValidCorrespondences) {
    const vio::Config config = MakeGeometryConfig();
    std::vector<cv::Point2f> previous_points;
    std::vector<cv::Point2f> current_points;
    GenerateProjectedCorrespondences(config, &previous_points, &current_points);

    const vio::EssentialMatrixSolver solver(config);
    const auto result = solver.Solve(previous_points, current_points);

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.inlier_mask.size(), previous_points.size());
    EXPECT_GE(result.inlier_count, config.geometry.min_inlier_count);
    EXPECT_EQ(result.essential_matrix.rows, 3);
    EXPECT_EQ(result.essential_matrix.cols, 3);
}

TEST(EssentialMatrixSolverTest, RejectsMissingCameraIntrinsics) {
    vio::Config config;
    config.geometry.min_inlier_count = 5;
    const vio::EssentialMatrixSolver solver(config);

    const std::vector<cv::Point2f> previous_points{
        {10.0f, 10.0f}, {20.0f, 20.0f}, {30.0f, 20.0f},
        {40.0f, 15.0f}, {50.0f, 25.0f}};
    const std::vector<cv::Point2f> current_points{
        {12.0f, 11.0f}, {22.0f, 21.0f}, {31.0f, 19.0f},
        {41.0f, 16.0f}, {52.0f, 24.0f}};

    const auto result = solver.Solve(previous_points, current_points);

    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.essential_matrix.empty());
    EXPECT_TRUE(result.inlier_mask.empty());
    EXPECT_EQ(result.inlier_count, 0U);
}

}  // namespace
