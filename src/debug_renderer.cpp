#include "internal/debug_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace rmcs_laser_guidance {
namespace {

    auto class_color(int class_id) -> cv::Scalar {
        switch (class_id) {
        case 0:  return { 255, 0, 255 }; // purple
        case 1:  return { 0, 0, 255 };   // red
        case 2:  return { 255, 0, 0 };   // blue
        default: return { 0, 255, 0 };
        }
    }

    auto class_name(int class_id) -> std::string {
        switch (class_id) {
        case 0:  return "purple";
        case 1:  return "red";
        case 2:  return "blue";
        default: return "?";
        }
    }

    constexpr float kMinScore = 0.25F;

} // namespace

DebugRenderer::DebugRenderer(const DebugConfig& debug_config)
    : debug_(debug_config) {}

auto DebugRenderer::draw(cv::Mat& image, const TargetObservation& observation) const -> void {
    if (image.empty() || !debug_.draw_overlay)
        return;

    if (!observation.detected) {
        cv::putText(
            image, "no target", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
        return;
    }

    if (!observation.candidates.empty()) {
        for (const auto& c : observation.candidates) {
            if (c.score < kMinScore) continue;
            const auto color = class_color(c.class_id);
            const cv::Rect r(static_cast<int>(c.bbox.x), static_cast<int>(c.bbox.y),
                             static_cast<int>(c.bbox.width), static_cast<int>(c.bbox.height));
            cv::rectangle(image, r, color, 2);

            const auto label = std::format("{} {:.0f}%", class_name(c.class_id),
                                           c.score * 100.0F);
            cv::putText(image, label,
                        cv::Point(r.x, std::max(r.y - 6, 16)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }

        const auto& best = observation.candidates.front();
        if (best.score >= kMinScore) {
            const int cx = static_cast<int>(best.center.x);
            const int cy = static_cast<int>(best.center.y);
            const int g = 8;
            cv::line(image, { cx - g, cy }, { cx + g, cy }, { 0, 255, 255 }, 1);
            cv::line(image, { cx, cy - g }, { cx, cy + g }, { 0, 255, 255 }, 1);
        }
    } else {
        if (!observation.contour.empty()) {
            std::vector<std::vector<cv::Point>> contour_set(1);
            contour_set.front().reserve(observation.contour.size());
            for (const cv::Point2f& point : observation.contour) {
                contour_set.front().emplace_back(
                    static_cast<int>(std::lround(point.x)), static_cast<int>(std::lround(point.y)));
            }
            cv::polylines(image, contour_set, true, {0, 255, 0}, 2);
        }

        cv::circle(image, observation.center, 6, {0, 0, 255}, 2);
    }
}

} // namespace rmcs_laser_guidance
