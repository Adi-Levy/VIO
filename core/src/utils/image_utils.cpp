#include "image_utils.hpp"

namespace vio {

bool IsGrayscaleImage(const cv::Mat& image) {
    return !image.empty() && image.type() == CV_8UC1;
}

}  // namespace vio
