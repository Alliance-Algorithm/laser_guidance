#pragma once

#include <chrono>
#include <expected>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>

#include "capture/capture_device.hpp"
#include "config.hpp"
#include "runtime/capture_retry_policy.hpp"
#include "runtime/control_loop_types.hpp"
#include "runtime/guidance_calibration.hpp"
#include "runtime/guidance_session.hpp"
#include "runtime/overlay_renderer.hpp"
#include "runtime/perception_runner.hpp"
#include "tracking/hit_state_machine.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct GuidanceRecorderPaths {
    std::filesystem::path geometry_path = "test_data/calib/rotation_calib_records.csv";
    std::filesystem::path hit_path = "test_data/calib/geometry_hit_calib_records.csv";
    std::filesystem::path voltage_path = "test_data/calib/voltage_records.csv";
};

class GuidanceOpsApp {
public:
    explicit GuidanceOpsApp(Config config, GuidanceRecorderPaths paths = {});

    auto run() -> std::expected<void, Error>;

private:
    auto initialize() -> std::expected<void, Error>;
    auto teardown() -> void;
    auto run_loop() -> void;
    auto handle_key(int key, const ControlLoopFrame& frame) -> void;
    auto maybe_record_calibration(const ControlLoopFrame& frame) -> void;
    auto maybe_record_hit_edge(const ControlLoopFrame& frame) -> void;
    [[nodiscard]] auto calibration_state() -> GuidanceCalibrationState&;

    Config config_{};
    GuidanceRecorderPaths paths_{};
    CaptureDevice capture_;
    PerceptionRunner perception_;
    OverlayRenderer overlay_;
    std::shared_ptr<GuidanceCalibrationState> calibration_state_{};
    std::optional<GuidanceSession> guidance_{};
    std::optional<CaptureFormat> negotiated_format_{};
    HitStateMachine hit_state_machine_;
    HitState last_hit_state_ = HitState::None;
    std::ofstream calibration_file_{};
    std::ofstream voltage_file_{};
    std::ofstream hit_file_{};
    bool stop_requested_ = false;
    bool window_open_ = false;
    CaptureRetryPolicy retry_policy_{};
};

} // namespace rmcs_laser_guidance::runtime_internal
