#pragma once

#include <expected>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>

#include "capture/capture_device.hpp"
#include "config.hpp"
#include "laser_guidance/runtime.hpp"
#include "bridges/ros_bridge.hpp"
#include "runtime/guidance_session.hpp"
#include "runtime/overlay_renderer.hpp"
#include "runtime/perception_runner.hpp"
#include "runtime/runtime_outputs.hpp"
#include "tracking/hit_progress.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct ControlLoopState {
    RuntimeSnapshot latest_snapshot{};
    bool running = false;
    bool stop_requested = false;
    EnemyColor enemy_color = EnemyColor::auto_select;
    bool ekf_enabled = false;
    bool streaming_requested = false;
    bool recording_requested = false;
    std::string last_error{};
};

class ControlLoop {
public:
    explicit ControlLoop(Config config, CompetitionRuntimeOptions options = {});
    ~ControlLoop();

    ControlLoop(const ControlLoop&) = delete;
    auto operator=(const ControlLoop&) -> ControlLoop& = delete;

    auto start() -> std::expected<void, std::string>;
    auto run() -> std::expected<void, std::string>;
    auto stop() -> void;
    auto join() -> void;
    auto submit_command(const RuntimeCommand& command) -> std::expected<void, std::string>;
    [[nodiscard]] auto snapshot() const -> RuntimeSnapshot;

private:
    static auto make_output_capabilities(CompetitionProfile profile)
        -> RuntimeOutputCapabilities;

    auto initialize_components() -> std::expected<void, std::string>;
    auto run_loop() -> void;
    auto teardown_components() -> void;
    auto request_stop() -> void;
    [[nodiscard]] auto stop_requested() const -> bool;
    auto update_status_locked() -> void;
    auto sync_last_error(std::string error) -> void;
    auto update_hit_progress(const DetectionBatch& detection) -> void;
    [[nodiscard]] auto show_window() const -> bool;
    [[nodiscard]] auto window_name() const -> const char*;
    [[nodiscard]] auto allows_streaming() const -> bool;
    [[nodiscard]] auto allows_recording() const -> bool;
    [[nodiscard]] auto guidance_enabled_in_profile() const -> bool;
    [[nodiscard]] auto assemble_snapshot(
        const ControlLoopFrame& frame, const RuntimeOutputsStatus& output_status) const
        -> RuntimeSnapshot;

    Config config_{};
    CompetitionRuntimeOptions options_{};
    mutable std::mutex state_mutex_;
    ControlLoopState state_{};
    CaptureDevice capture_;
    PerceptionRunner perception_;
    RuntimeOutputs outputs_;
    std::unique_ptr<RosBridge> ros_bridge_{};
    std::optional<GuidanceSession> guidance_{};
    HitProgress hit_progress_{};
    std::optional<CaptureFormat> negotiated_format_{};
    OverlayRenderer overlay_{};
    cv::Mat previous_output_{};
    bool window_open_ = false;
    std::thread main_thread_{};
};

} // namespace rmcs_laser_guidance::runtime_internal
