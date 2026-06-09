#include "core/debug_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace rmcs_laser_guidance {
namespace {

constexpr float kMinScore = 0.25F;

} // namespace

auto class_color(int class_id) -> cv::Scalar {
    switch (class_id) {
    case 0: return {255, 0, 255}; // purple
    case 1: return {0, 0, 255};   // red
    case 2: return {255, 0, 0};   // blue
    default: return {0, 255, 0};
    }
}

auto class_name(int class_id) -> std::string {
    switch (class_id) {
    case 0: return "purple";
    case 1: return "red";
    case 2: return "blue";
    default: return "?";
    }
}

auto draw_candidates(cv::Mat& image, const std::vector<ModelCandidate>& candidates) -> void {
    for (const auto& c : candidates) {
        if (c.score < kMinScore)
            continue;
        const auto color = class_color(c.class_id);
        const cv::Rect r(
            static_cast<int>(c.bbox.x), static_cast<int>(c.bbox.y), static_cast<int>(c.bbox.width),
            static_cast<int>(c.bbox.height));
        cv::rectangle(image, r, color, 2);

        const auto label = std::format("{} {:.0f}%", class_name(c.class_id), c.score * 100.0F);
        cv::putText(
            image, label, cv::Point(r.x, std::max(r.y - 6, 16)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            color, 2);
    }

    if (candidates.empty())
        return;

    const auto& best = candidates.front();
    if (best.score < kMinScore)
        return;

    const int cx = static_cast<int>(best.center.x);
    const int cy = static_cast<int>(best.center.y);
    const int g = 8;
    cv::line(image, {cx - g, cy}, {cx + g, cy}, {0, 255, 255}, 1);
    cv::line(image, {cx, cy - g}, {cx, cy + g}, {0, 255, 255}, 1);
}

auto draw_ekf_state(cv::Mat& image, const EkfState& state) -> void {
    if (!state.initialized)
        return;

    const int cx = static_cast<int>(state.position.x);
    const int cy = static_cast<int>(state.position.y);

    if (state.lost) {
        cv::putText(image, "EKF LOST", {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
        return;
    }

    cv::circle(image, {cx, cy}, 5, {0, 255, 0}, -1);

    constexpr float kArrowScale = 0.5F;
    const int vx = static_cast<int>(state.velocity.x * kArrowScale);
    const int vy = static_cast<int>(state.velocity.y * kArrowScale);
    if (vx != 0 || vy != 0)
        cv::arrowedLine(image, {cx, cy}, {cx + vx, cy + vy}, {0, 255, 0}, 2);

    const float speed = std::hypot(state.velocity.x, state.velocity.y);
    const auto label = std::format("EKF {:.0f} px/s", speed);
    cv::putText(image, label, {cx + 10, cy - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
}

auto draw_guidance_status(
    cv::Mat& image, bool guidance_active, bool ekf_ok, bool depth_ok, const std::string& msg)
    -> void {
    if (!guidance_active) {
        cv::putText(
            image, "GUIDANCE: disabled", {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 200, 255}, 2);
        return;
    }
    if (!ekf_ok) {
        cv::putText(
            image, "GUIDANCE: EKF lost/waiting", {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
            {0, 200, 255}, 2);
        return;
    }
    if (!depth_ok) {
        cv::putText(
            image, "GUIDANCE: waiting for depth", {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
            {0, 200, 255}, 2);
        return;
    }
    if (!msg.empty()) {
        cv::putText(
            image, std::format("GUIDANCE: {}", msg), {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
            {0, 200, 255}, 2);
        return;
    }
    cv::putText(image, "GUIDANCE OK", {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
}

auto draw_status_bar(
    cv::Mat& image, bool streaming, bool recording, int enemy_class_id, bool using_trt) -> void {
    std::string line;
    line += using_trt ? " [TRT]" : " [ONNX]";
    if (streaming)
        line += " [RTP]";
    if (recording)
        line += " [REC]";
    switch (enemy_class_id) {
    case 1: line += " [RED]"; break;
    case 2: line += " [BLUE]"; break;
    default: break;
    }
    if (line.empty())
        return;
    cv::putText(image, line, {image.cols - 220, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
}

auto draw_hit_progress(cv::Mat& image, const HitProgress& hp) -> void {
    const int bar_x = 10;
    const int bar_y = image.rows - 40;
    const int bar_w = 200;
    const int bar_h = 20;
    const int bar_border = 2;

    cv::rectangle(
        image, {bar_x - bar_border, bar_y - bar_border},
        {bar_x + bar_w + bar_border, bar_y + bar_h + bar_border}, {80, 80, 80}, cv::FILLED);

    if (!hp.is_locked() && !hp.is_exhausted()) {
        const int fill_w = static_cast<int>(hp.progress_ratio() * static_cast<float>(bar_w));
        cv::rectangle(
            image, {bar_x, bar_y}, {bar_x + fill_w, bar_y + bar_h},
            hp.is_hitting() ? cv::Scalar{0, 0, 255} : cv::Scalar{0, 165, 255}, cv::FILLED);
    }

    cv::rectangle(
        image, {bar_x - bar_border, bar_y - bar_border},
        {bar_x + bar_w + bar_border, bar_y + bar_h + bar_border}, {200, 200, 200}, bar_border);

    std::string status_text;
    if (hp.is_exhausted()) {
        status_text = "LOCK EXHAUSTED (3/3)";
    } else if (hp.is_locked()) {
        status_text = std::format("LOCKED {:.0f}s  [{}/3]", hp.lock_remaining_s(), hp.lock_count());
    } else {
        status_text = std::format(
            "P={:.0f}/{:.0f}  stage={}  locks={}", hp.progress(), hp.p0(), hp.stage(),
            hp.lock_count());
    }
    cv::putText(
        image, status_text, {bar_x, bar_y - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {200, 200, 200}, 1);
}

DebugRenderer::DebugRenderer(const DebugConfig& debug_config)
    : debug_(debug_config) {}

auto DebugRenderer::draw(cv::Mat& image, const TargetObservation& observation) const -> void {
    if (image.empty() || !debug_.draw_overlay)
        return;

    if (!observation.detected) {
        cv::putText(image, "no target", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
        return;
    }

    if (!observation.candidates.empty()) {
        rmcs_laser_guidance::draw_candidates(image, observation.candidates);
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

auto DebugRenderer::draw_ekf_state(cv::Mat& image, const EkfState& state) const -> void {
    if (image.empty() || !debug_.draw_overlay)
        return;

    rmcs_laser_guidance::draw_ekf_state(image, state);
}

} // namespace rmcs_laser_guidance
