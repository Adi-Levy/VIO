#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

namespace vio {

// Time is represented in integer nanoseconds throughout the public API.
using TimestampNs = std::int64_t;

// A single grayscale image frame.
// The image is expected to be single-channel. Validation can be enforced
// by the caller or in the estimator implementation.
struct ImageFrame {
    TimestampNs timestamp_ns{0};
    cv::Mat image_gray{};
    std::uint32_t frame_id{0};
};

// A single IMU measurement.
// accel_mps2 is linear acceleration in meters per second squared.
// gyro_radps is angular velocity in radians per second.
struct ImuSample {
    TimestampNs timestamp_ns{0};
    Eigen::Vector3d accel_mps2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_radps{Eigen::Vector3d::Zero()};
};

// Standard pinhole camera intrinsics.
struct PinholeIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int width{0};
    int height{0};
};

// Distortion coefficients for the camera model.
// The exact interpretation depends on the calibration / frontend code.
// For v1 this can store the OpenCV-style distortion vector.
struct DistortionCoeffs {
    std::vector<double> coeffs{};
};

// Full camera calibration bundle.
struct CameraCalibration {
    PinholeIntrinsics intrinsics{};
    DistortionCoeffs distortion{};
};

// R_b_c and t_b_c define the transform from camera frame c to body frame b.
// In other words, for a point p_c in camera coordinates:
// p_b = R_b_c * p_c + t_b_c
struct Extrinsics {
    Eigen::Matrix3d R_b_c{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t_b_c{Eigen::Vector3d::Zero()};
};

// Rigid pose of the body frame b in the world frame w.
// q_w_b rotates vectors from body frame to world frame.
// p_w_b is the position of the body origin expressed in world coordinates.
struct Pose3 {
    Eigen::Quaterniond q_w_b{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d p_w_b{Eigen::Vector3d::Zero()};
};

// Full nominal VIO state for a body-centric estimate.
struct VelocityBiasState {
    Pose3 pose{};
    Eigen::Vector3d v_w_b{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
};

// 15x15 covariance matrix for the standard VIO error-state.
//
// State ordering (error-state):
//
// Index range   Variable                Description
// ---------------------------------------------------------------
// [0 - 2]       δp_w_b                  Position error (world frame)
// [3 - 5]       δθ_w_b                  Orientation error (small-angle, tangent space)
// [6 - 8]       δv_w_b                  Velocity error (world frame)
// [9 - 11]      δb_a                    Accelerometer bias error
// [12 - 14]     δb_g                    Gyroscope bias error
//
// Notes:
// - Orientation error is represented as a 3D minimal perturbation (not quaternion).
// - This follows the standard error-state EKF formulation used in VIO.
// - All blocks are expressed in the world frame unless otherwise specified.
struct StateCovariance {
    Eigen::Matrix<double, 15, 15> P{
        Eigen::Matrix<double, 15, 15>::Zero()
    };
};

// Public estimator output.
// valid indicates whether the estimate is currently trustworthy enough
// to be consumed by downstream systems.
struct StateEstimate {
    TimestampNs timestamp_ns{0};
    VelocityBiasState state{};
    StateCovariance covariance{};
    bool valid{false};
};

}  // namespace vio