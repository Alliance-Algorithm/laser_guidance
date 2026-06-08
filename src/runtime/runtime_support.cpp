#include "runtime/runtime_support.hpp"

#include <algorithm>

namespace rmcs_laser_guidance::runtime_internal {

auto to_enemy_class_id(const EnemyColor color) -> int { return static_cast<int>(color); }

auto to_enemy_color(const int class_id) -> EnemyColor {
    switch (class_id) {
    case 1: return EnemyColor::red;
    case 2: return EnemyColor::blue;
    default: return EnemyColor::auto_select;
    }
}

auto to_backend_name(const RuntimeBackend backend) -> std::string {
    return backend == RuntimeBackend::tensorrt ? "TensorRT" : "ONNX";
}

auto make_hit_progress_snapshot(const HitProgress& progress) -> HitProgressSnapshot {
    return HitProgressSnapshot{
        .progress = progress.progress(),
        .progress_ratio = progress.progress_ratio(),
        .is_hitting = progress.is_hitting(),
        .is_locked = progress.is_locked(),
        .lock_remaining_s = progress.lock_remaining_s(),
        .lock_count = progress.lock_count(),
        .stage = progress.stage(),
        .p0 = progress.p0(),
        .is_exhausted = progress.is_exhausted(),
    };
}

auto make_capture_snapshot(const V4l2NegotiatedFormat& format) -> CaptureFormatSnapshot {
    return CaptureFormatSnapshot{
        .device_path = format.device_path,
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .fourcc = format.fourcc,
    };
}

auto make_runtime_status(
    const RuntimeSnapshot& previous_snapshot, const bool running, const bool stop_requested,
    const bool capture_open, const bool inference_enabled, const bool guidance_enabled,
    const bool guidance_ready, const bool ekf_enabled,
    const std::optional<RuntimeBackend> active_backend, const EnemyColor enemy_color,
    std::string last_error, const bool streaming_active, const bool recording_active) -> RuntimeStatus {
    RuntimeStatus status = previous_snapshot.status;
    status.running = running;
    status.stop_requested = stop_requested;
    status.capture_open = capture_open;
    status.inference_enabled = inference_enabled;
    status.streaming_active = streaming_active;
    status.recording_active = recording_active;
    status.guidance_enabled = guidance_enabled;
    status.guidance_ready = guidance_ready;
    status.ekf_enabled = ekf_enabled;
    status.backend_uses_tensorrt =
        active_backend.has_value() && *active_backend == RuntimeBackend::tensorrt;
    status.enemy_color = enemy_color;
    status.last_error = std::move(last_error);
    return status;
}

auto to_detection(const ModelCandidate& candidate) -> Detection {
    return Detection{
        .score = candidate.score,
        .class_id = candidate.class_id,
        .bbox = candidate.bbox,
        .center = candidate.center,
    };
}

auto filter_detections(std::vector<Detection>& detections, const EnemyColor enemy_color) -> void {
    const int enemy_class_id = to_enemy_class_id(enemy_color);
    if (enemy_class_id < 0) {
        return;
    }
    detections.erase(
        std::remove_if(
            detections.begin(), detections.end(),
            [enemy_class_id](const Detection& detection) {
                return detection.class_id != 0 && detection.class_id != enemy_class_id;
            }),
        detections.end());
}

auto to_detection_batch(const ModelInferResult& result) -> DetectionBatch {
    DetectionBatch batch;
    batch.detections.reserve(result.candidates.size());
    for (const auto& candidate : result.candidates) {
        batch.detections.push_back(to_detection(candidate));
    }
    batch.detected = result.observation.detected;
    batch.selected_center = result.observation.center;
    batch.lidar_frame = result.observation.lidar_frame;
    return batch;
}

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

auto to_target_observation(const DetectionBatch& batch) -> TargetObservation {
    return TargetObservation{
        .detected = batch.detected,
        .center = batch.selected_center,
        .candidates = to_model_candidates(batch),
        .lidar_frame = batch.lidar_frame,
    };
}

auto select_target_track(
    const DetectionBatch& batch, const std::optional<EkfState>& ekf_state, const bool ekf_enabled,
    const float lookahead_ms) -> TargetTrack {
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
            const float latency_s = lookahead_ms * 0.001F;
            track.aim_center = cv::Point2f{
                ekf_state->position.x + ekf_state->velocity.x * latency_s,
                ekf_state->position.y + ekf_state->velocity.y * latency_s,
            };
        }
    }
    return track;
}

} // namespace rmcs_laser_guidance::runtime_internal
