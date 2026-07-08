#pragma once

#include <cstddef>
#include <optional>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "laser_guidance/runtime.hpp"
#include "tracking/ekf_tracker.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct GuidanceTelemetry {
    std::optional<float> measured_depth_mm{};
    std::optional<float> active_depth_mm{};
    std::optional<cv::Point3f> selected_target_point{};
    bool used_cached_depth = false;
};

struct GuidanceFrameResult {
    AimOutput aim_output{};
    GuidanceTelemetry telemetry{};
};

struct ControlLoopFrame {
    Frame frame{};
    cv::Mat display{};
    DetectionBatch detection{};
    std::optional<EkfState> ekf_state{};
    TargetTrack track{};
    GuidanceFrameResult guidance{};
    std::size_t dropped_frames = 0;
};

// Shared by ControlLoop and GuidanceOpsApp: turns a raw detection batch plus
// optional EKF state into the TargetTrack guidance consumes. ekf_lookahead_ms
// controls how far the EKF-predicted aim point is projected ahead of the
// filtered position/velocity.
inline auto select_target_track(
    const DetectionBatch& batch, const std::optional<EkfState>& ekf_state, const bool ekf_enabled,
    const double ekf_lookahead_ms) -> TargetTrack {
    TargetTrack track;
    track.detected = batch.detected;
    track.ekf_enabled = ekf_enabled;
    track.raw_center = batch.selected_center;
    track.aim_center = batch.selected_center;
    if (!batch.detections.empty()) {
        track.selected_detection = batch.detections.front();
    }
    if (ekf_state.has_value()) {
        track.initialized = ekf_state->initialized;
        track.lost = ekf_state->lost;
        track.missed_frames = ekf_state->missed_frames;
        track.dt_seconds = ekf_state->dt_seconds;
        track.ekf_position = ekf_state->position;
        track.velocity = ekf_state->velocity;
        track.ekf_acceleration = ekf_state->acceleration;
        if (ekf_enabled && ekf_state->initialized && !ekf_state->lost) {
            const float latency_s = static_cast<float>(ekf_lookahead_ms * 0.001);
            track.aim_center = cv::Point2f{
                ekf_state->position.x + ekf_state->velocity.x * latency_s,
                ekf_state->position.y + ekf_state->velocity.y * latency_s,
            };
        }
    }
    return track;
}

} // namespace rmcs_laser_guidance::runtime_internal
