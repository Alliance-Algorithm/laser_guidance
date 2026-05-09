#include "vision/detector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "core/frame_format.hpp"

namespace rmcs_laser_guidance {
namespace {

constexpr double kMinDetectedBrightness = 250.0;

auto contour_center(const std::vector<cv::Point>& contour) -> cv::Point2f {
    const cv::Moments moments = cv::moments(contour);
    if (std::abs(moments.m00) > 1e-6) {
        return {
            static_cast<float>(moments.m10 / moments.m00),
            static_cast<float>(moments.m01 / moments.m00),
        };
    }

    const cv::Rect bounds = cv::boundingRect(contour);
    return {
        static_cast<float>(bounds.x) + static_cast<float>(bounds.width) / 2.0F,
        static_cast<float>(bounds.y) + static_cast<float>(bounds.height) / 2.0F,
    };
}

} // namespace

Detector::Detector(const Config& config) {
    (void)config;
}

auto Detector::detect(const Frame& frame) const -> TargetObservation {
    if (frame.image.empty())
        return {};

    cv::Mat gray = to_gray_image(frame.image);
    if (gray.empty())
        return {};

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, {5, 5}, 0.0);

    double max_value = 0.0;
    cv::Point max_location;
    cv::minMaxLoc(blurred, nullptr, &max_value, nullptr, &max_location);
    if (max_value < kMinDetectedBrightness)
        return {};

    cv::Mat mask;
    cv::threshold(blurred, mask, std::max(245.0, max_value - 10.0), 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return {};

    const auto largest = std::max_element(
        contours.begin(), contours.end(), [](const auto& lhs, const auto& rhs) {
            return cv::contourArea(lhs) < cv::contourArea(rhs);
        });

    TargetObservation observation;
    observation.detected = true;
    observation.center = contour_center(*largest);
    observation.brightness = static_cast<float>(max_value);
    observation.contour.reserve(largest->size());
    for (const cv::Point& point : *largest)
        observation.contour.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));

    (void)max_location;
    return observation;
}

} // namespace rmcs_laser_guidance
