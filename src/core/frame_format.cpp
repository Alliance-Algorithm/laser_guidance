#include "core/frame_format.hpp"

#include <opencv2/imgproc.hpp>

namespace rmcs_laser_guidance {

auto detect_frame_format(const cv::Mat& image) noexcept -> FrameFormat {
    if (image.empty() || image.depth() != CV_8U)
        return FrameFormat::unknown;

    switch (image.channels()) {
    case 1:
        return FrameFormat::gray8;
    case 3:
        return FrameFormat::bgr8;
    case 4:
        return FrameFormat::bgra8;
    default:
        return FrameFormat::unknown;
    }
}

auto is_supported_frame_format(const FrameFormat format) noexcept -> bool {
    return format == FrameFormat::gray8 || format == FrameFormat::bgr8 ||
           format == FrameFormat::bgra8;
}

auto frame_format_name(const FrameFormat format) noexcept -> const char* {
    switch (format) {
    case FrameFormat::gray8:
        return "gray8";
    case FrameFormat::bgr8:
        return "bgr8";
    case FrameFormat::bgra8:
        return "bgra8";
    default:
        return "unknown";
    }
}

auto to_gray_image(const cv::Mat& image) -> cv::Mat {
    cv::Mat gray;
    switch (detect_frame_format(image)) {
    case FrameFormat::gray8:
        gray = image.clone();
        break;
    case FrameFormat::bgr8:
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        break;
    case FrameFormat::bgra8:
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        break;
    default:
        break;
    }

    return gray;
}

} // namespace rmcs_laser_guidance
