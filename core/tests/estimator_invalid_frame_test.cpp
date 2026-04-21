#include <vio/vio_estimator.hpp>

#include <opencv2/core/mat.hpp>

namespace {

bool RejectsEmptyFrame() {
    vio::VioEstimator estimator;
    vio::ImageFrame frame;
    frame.timestamp_ns = 2;

    if (estimator.ProcessImageFrame(frame)) {
        return false;
    }

    const auto& diagnostics = estimator.GetLatestDiagnostics();
    return diagnostics.runtime_status == vio::RuntimeStatus::kWaitingForData;
}

bool RejectsTooSmallFrame() {
    vio::VioEstimator estimator;
    vio::ImageFrame frame;
    frame.timestamp_ns = 3;
    frame.image_gray = cv::Mat::zeros(16, 16, CV_8UC1);

    if (estimator.ProcessImageFrame(frame)) {
        return false;
    }

    const auto& diagnostics = estimator.GetLatestDiagnostics();
    return diagnostics.runtime_status == vio::RuntimeStatus::kWaitingForData;
}

}  // namespace

int main() {
    return (RejectsEmptyFrame() && RejectsTooSmallFrame()) ? 0 : 1;
}
