#include <print>
#include <stdexcept>

#include "laser_guidance/runtime.hpp"
#include "runtime/runtime_support.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::Detection;
        using rmcs_laser_guidance::DetectionBatch;
        using rmcs_laser_guidance::EnemyColor;
        using rmcs_laser_guidance::RuntimeBackend;
        using rmcs_laser_guidance::RuntimeCommandType;
        using rmcs_laser_guidance::RuntimeSnapshot;
        using rmcs_laser_guidance::runtime_internal::make_runtime_status;
        using rmcs_laser_guidance::runtime_internal::select_target_track;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_near;

        {
            const auto command = rmcs_laser_guidance::RuntimeCommand::set_streaming(true);
            require(command.type == RuntimeCommandType::set_streaming, "stream command type mismatch");
            require(command.enabled, "stream command enabled mismatch");
        }

        {
            const auto command = rmcs_laser_guidance::RuntimeCommand::set_enemy_color(EnemyColor::red);
            require(
                command.type == RuntimeCommandType::set_enemy_color,
                "enemy command type mismatch");
            require(command.enemy_color == EnemyColor::red, "enemy command color mismatch");
        }

        {
            const auto command =
                rmcs_laser_guidance::RuntimeCommand::set_backend(RuntimeBackend::onnx);
            require(command.type == RuntimeCommandType::set_backend, "backend command type mismatch");
            require(command.backend == RuntimeBackend::onnx, "backend command value mismatch");
        }

        {
            DetectionBatch batch;
            batch.detected = true;
            batch.selected_center = {100.0F, 120.0F};
            batch.detections.push_back(Detection{
                .score = 0.9F,
                .class_id = 1,
                .bbox = {90.0F, 100.0F, 20.0F, 30.0F},
                .center = {100.0F, 120.0F},
            });
            const rmcs_laser_guidance::EkfState ekf{
                .position = {101.0F, 122.0F},
                .velocity = {10.0F, -5.0F},
                .acceleration = {0.1F, 0.2F},
                .initialized = true,
                .lost = false,
                .missed_frames = 0,
                .dt_seconds = 0.016,
            };

            const auto track = select_target_track(batch, ekf, true, 20.0F);
            require(track.detected, "track detected mismatch");
            require(track.ekf_enabled, "track ekf flag mismatch");
            require(track.initialized, "track initialized mismatch");
            require(!track.lost, "track lost mismatch");
            require(track.selected_detection != nullptr, "track selected detection missing");
            require_near(track.aim_center.x, 101.2F, 1e-3F, "track aim_center.x");
            require_near(track.aim_center.y, 121.9F, 1e-3F, "track aim_center.y");
            require(track.ekf_position.has_value(), "track ekf position missing");
            require(track.ekf_acceleration.has_value(), "track ekf acceleration missing");
        }

        {
            DetectionBatch batch;
            batch.detected = false;
            batch.selected_center = {-1.0F, -1.0F};
            const auto track = select_target_track(batch, std::nullopt, false, 0.0F);
            require(!track.detected, "raw track should stay undetected");
            require(!track.ekf_enabled, "raw track ekf flag mismatch");
            require(track.selected_detection == nullptr, "raw track should not select detection");
            require_near(track.aim_center.x, -1.0F, 1e-3F, "raw track aim_center.x");
            require_near(track.aim_center.y, -1.0F, 1e-3F, "raw track aim_center.y");
        }

        {
            RuntimeSnapshot snapshot;
            snapshot.status.last_guidance_message = "keep";
            const auto status = make_runtime_status(
                snapshot, true, false, true, true, true, true, false, RuntimeBackend::tensorrt,
                EnemyColor::blue, "warn", true, false);
            require(status.running, "running flag mismatch");
            require(!status.stop_requested, "stop flag mismatch");
            require(status.capture_open, "capture flag mismatch");
            require(status.inference_enabled, "inference flag mismatch");
            require(status.guidance_enabled, "guidance enabled mismatch");
            require(status.guidance_ready, "guidance ready mismatch");
            require(!status.ekf_enabled, "ekf flag mismatch");
            require(status.backend_uses_tensorrt, "backend runtime status mismatch");
            require(status.enemy_color == EnemyColor::blue, "enemy color mismatch");
            require(status.streaming_active, "streaming status mismatch");
            require(!status.recording_active, "recording status mismatch");
            require(status.last_error == "warn", "last error mismatch");
            require(
                status.last_guidance_message == "keep",
                "status helper should preserve guidance message");
        }

        {
            RuntimeSnapshot snapshot;
            const auto status = make_runtime_status(
                snapshot, false, true, false, true, false, false, true, std::nullopt,
                EnemyColor::auto_select, {}, false, false);
            require(!status.backend_uses_tensorrt, "inactive backend should clear TRT status");
            require(status.stop_requested, "stop_requested should remain true");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "runtime_api_test failed: {}", e.what());
        return 1;
    }
}
