#include "image_utils.hpp"

namespace vio {

bool IsGrayscaleImage(const cv::Mat& image) {
    return !image.empty() && image.type() == CV_8UC1;
}

cv::Mat BuildCameraMatrix(const CameraCalibration& camera_calibration) {
    const auto& intrinsics = camera_calibration.intrinsics;
    return (cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0,
            intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0);
}

cv::Mat ConvertMaskToMat(const std::vector<std::uint8_t>& mask) {
    if (mask.empty()) {
        return {};
    }

    cv::Mat mat(static_cast<int>(mask.size()), 1, CV_8U);
    for (std::size_t i = 0; i < mask.size(); ++i) {
        mat.at<std::uint8_t>(static_cast<int>(i), 0) = mask[i];
    }
    return mat;
}

std::vector<std::uint8_t> ConvertMaskToVector(const cv::Mat& mask) {
    if (mask.empty()) {
        return {};
    }

    const auto* begin = mask.ptr<std::uint8_t>();
    return std::vector<std::uint8_t>(begin, begin + mask.total());
}

std::size_t CountNonZeroMaskEntries(const cv::Mat& mask) {
    return static_cast<std::size_t>(cv::countNonZero(mask));
}

}  // namespace vio
