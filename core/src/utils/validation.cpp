#include "validation.hpp"

#include "image_utils.hpp"

namespace vio {

bool MeetsMinimumImageSize(const cv::Mat& image,
                           std::uint32_t min_image_width,
                           std::uint32_t min_image_height) {
    return image.cols >= static_cast<int>(min_image_width) &&
           image.rows >= static_cast<int>(min_image_height);
}

bool IsSupportedTrackingImage(const cv::Mat& image,
                              std::uint32_t min_image_width,
                              std::uint32_t min_image_height) {
    return IsGrayscaleImage(image) &&
           MeetsMinimumImageSize(image, min_image_width, min_image_height);
}

}  // namespace vio
