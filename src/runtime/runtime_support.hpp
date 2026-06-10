#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "capture/capture_device.hpp"
#include "laser_guidance/runtime.hpp"
#include "tracking/ekf_tracker.hpp"
#include "tracking/hit_progress.hpp"
#include "vision/model_infer.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct QueuedFrame {
    cv::Mat image;
    Clock::time_point capture_time{};
};

auto to_enemy_class_id(EnemyColor color) -> int;
auto to_enemy_color(int class_id) -> EnemyColor;
auto to_backend_name(RuntimeBackend backend) -> std::string;
auto make_hit_progress_snapshot(const HitProgress& progress) -> HitProgressSnapshot;
auto make_capture_snapshot(const CaptureFormat& format) -> CaptureFormatSnapshot;
auto make_runtime_status(
    const RuntimeSnapshot& previous_snapshot, bool running, bool stop_requested, bool capture_open,
    bool inference_enabled, bool guidance_enabled, bool guidance_ready, bool ekf_enabled,
    std::optional<RuntimeBackend> active_backend, EnemyColor enemy_color, std::string last_error,
    bool streaming_active, bool recording_active) -> RuntimeStatus;
auto to_detection(const ModelCandidate& candidate) -> Detection;
auto filter_detections(std::vector<Detection>& detections, EnemyColor enemy_color) -> void;
auto to_detection_batch(const ModelInferResult& result) -> DetectionBatch;
auto to_model_candidates(const DetectionBatch& batch) -> std::vector<ModelCandidate>;
auto to_target_observation(const DetectionBatch& batch) -> TargetObservation;
auto select_target_track(
    const DetectionBatch& batch, const std::optional<EkfState>& ekf_state, bool ekf_enabled,
    float lookahead_ms) -> TargetTrack;

} // namespace rmcs_laser_guidance::runtime_internal
