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
// The exact interpretation depends on the calibration / visual processing code.
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

// Public estimator output for Phase 1 monocular VO.
// Translation remains up to arbitrary scale in VO-only mode.
struct StateEstimate {
    TimestampNs timestamp_ns{0};
    Pose3 pose{};
    bool valid{false};
};

}  // namespace vio
