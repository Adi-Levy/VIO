#include <vio/vio_estimator.hpp>

#include <opencv2/core/mat.hpp>

int main() {
    vio::VioEstimator estimator;

    vio::ImageFrame frame;
    frame.timestamp_ns = 4;
    frame.image_gray = cv::Mat::zeros(64, 64, CV_8UC1);

    if (!estimator.ProcessImageFrame(frame)) {
        return 1;
    }

    estimator.Reset();

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
