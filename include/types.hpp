#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs_laser_guidance {

using Clock = std::chrono::steady_clock;

struct Frame {
    cv::Mat image;
    Clock::time_point timestamp{};
};

struct ModelCandidate {
    float score = 0.0F;
    std::int32_t class_id = -1;
    cv::Rect2f bbox;
    cv::Point2f center { -1.0F, -1.0F };
};

struct TargetObservation {
    bool detected = false;
    cv::Point2f center{-1.0F, -1.0F};
    std::vector<cv::Point2f> contour{};
    float brightness = 0.0F;
    std::vector<ModelCandidate> candidates{};
};

} // namespace rmcs_laser_guidance
