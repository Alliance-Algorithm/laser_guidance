#include "runtime/overlay_renderer.hpp"

#include <format>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "core/debug_renderer.hpp"

namespace rmcs_laser_guidance::runtime_internal {
namespace {

auto to_model_candidates(const DetectionBatch& batch) -> std::vector<ModelCandidate> {
    std::vector<ModelCandidate> candidates;
    candidates.reserve(batch.detections.size());
    for (const auto& detection : batch.detections) {
        candidates.push_back(
            ModelCandidate{
                .score = detection.score,
                .class_id = detection.class_id,
                .bbox = detection.bbox,
                .center = detection.center,
            });
    }
    return candidates;
}

auto to_enemy_class_id(const EnemyColor color) -> int { return static_cast<int>(color); }

} // namespace

auto OverlayRenderer::render(ControlLoopFrame& frame, const OverlayRenderContext& context) const -> void {
    // Draw detections shifted onto the current display frame using track.aim_center
    // (age-compensated EKF). Raw async boxes look "stuck" on a smooth video stream.
    if (frame.track.detected && frame.track.selected_detection.has_value()) {
        const auto& src = *frame.track.selected_detection;
        const cv::Point2f shift = frame.track.aim_center - src.center;
        ModelCandidate drawn{
            .score = src.score,
            .class_id = src.class_id,
            .bbox =
                cv::Rect2f{
                    src.bbox.x + shift.x, src.bbox.y + shift.y, src.bbox.width, src.bbox.height},
            .center = frame.track.aim_center,
        };
        draw_candidates(frame.display, {drawn});
    } else if (frame.detection.detected || !frame.detection.detections.empty()) {
        draw_candidates(frame.display, to_model_candidates(frame.detection));
    }
    if (frame.ekf_state.has_value() && frame.track.ekf_enabled) {
        // Show age-compensated aim, not the stale filter position alone.
        EkfState draw_state = *frame.ekf_state;
        if (frame.track.initialized && !frame.track.lost) {
            draw_state.position = frame.track.aim_center;
            draw_state.velocity = frame.track.velocity;
        }
        draw_ekf_state(frame.display, draw_state);
    }

    if (context.calibration_mode && context.guidance_ready && context.calibration_state != nullptr) {
        const auto label =
            context.command_model == GuidanceCommandModelKind::direct_voltage
                ? std::format(
                      "CALIB voltage=({:.3f}V,{:.3f}V) step={:.3f}V [WASD move, </> step]",
                      context.calibration_state->voltage_x, context.calibration_state->voltage_y,
                      context.calibration_state->voltage_step_v)
                : std::format(
                      "CALIB galvo=({:.3f}°,{:.3f}°) step={:.3f}° [WASD move, </> step]",
                      context.calibration_state->angle_x_deg, context.calibration_state->angle_y_deg,
                      context.calibration_state->angle_step_deg);
        cv::putText(
            frame.display, label, {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
    } else {
        const bool ekf_ok =
            frame.track.ekf_enabled ? (frame.track.initialized && !frame.track.lost) : frame.track.detected;
        draw_guidance_status(
            frame.display, context.guidance_enabled && context.guidance_ready, ekf_ok,
            frame.guidance.aim_output.depth_valid,
            frame.track.ekf_enabled ? frame.guidance.aim_output.message : std::string("EKF OFF (raw)"));
    }

    if (context.hit_progress != nullptr) {
        draw_hit_progress(frame.display, *context.hit_progress);
    }
    if (context.referee != nullptr) {
        draw_referee_status(frame.display, *context.referee);
    }
    draw_status_bar(
        frame.display, context.streaming_active, context.recording_active,
        to_enemy_class_id(context.enemy_color), context.using_tensorrt);
}

} // namespace rmcs_laser_guidance::runtime_internal
