#pragma once

#include <opencv2/core/mat.hpp>

namespace rmcs_laser_guidance {

enum class FrameFormat {
    unknown = 0,
    gray8,
    bgr8,
    bgra8,
};

auto detect_frame_format(const cv::Mat& image) noexcept -> FrameFormat;
auto is_supported_frame_format(FrameFormat format) noexcept -> bool;
auto frame_format_name(FrameFormat format) noexcept -> const char*;
auto to_gray_image(const cv::Mat& image) -> cv::Mat;

} // namespace rmcs_laser_guidance
