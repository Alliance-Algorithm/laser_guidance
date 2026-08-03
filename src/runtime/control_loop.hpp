#pragma once

#include <atomic>
#include <chrono>
#include <expected>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>

#include "capture/capture_device.hpp"
#include "config.hpp"
#include "laser_guidance/error.hpp"
#include "laser_guidance/runtime.hpp"
#include "bridges/ros_bridge.hpp"
#include "runtime/guidance_session.hpp"
#include "runtime/overlay_renderer.hpp"
#include "runtime/perception_runner.hpp"
#include "runtime/referee_link.hpp"
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
    std::optional<std::pair<float, float>> pending_offset{};
    std::string last_error{};
};

class ControlLoop {
public:
    explicit ControlLoop(Config config, CompetitionRuntimeOptions options = {});
    ~ControlLoop();

    ControlLoop(const ControlLoop&) = delete;
    auto operator=(const ControlLoop&) -> ControlLoop& = delete;

    auto start() -> std::expected<void, Error>;
    auto run() -> std::expected<void, Error>;
    auto stop() -> void;
    auto join() -> void;
    auto submit_command(const RuntimeCommand& command) -> std::expected<void, Error>;
    [[nodiscard]] auto snapshot() const -> RuntimeSnapshot;

private:
    static auto make_output_capabilities(CompetitionProfile profile)
        -> RuntimeOutputCapabilities;

    auto initialize_components() -> std::expected<void, Error>;
    auto run_loop() -> void;
    auto teardown_components() -> void;
    auto request_stop() -> void;
    [[nodiscard]] auto stop_requested() const -> bool;
    auto update_status_locked() -> void;
    auto sync_last_error(std::string error) -> void;
    auto update_hit_progress(const DetectionBatch& detection) -> void;
    auto maybe_switch_hik_profile() -> void;
    [[nodiscard]] auto show_window() const -> bool;
    [[nodiscard]] auto window_name() const -> const char*;
    [[nodiscard]] auto allows_streaming() const -> bool;
    [[nodiscard]] auto allows_recording() const -> bool;
    [[nodiscard]] auto guidance_enabled_in_profile() const -> bool;
    auto start_guidance_init_thread() -> void;
    auto take_pending_offset() -> std::optional<std::pair<float, float>>;
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
    std::jthread guidance_init_thread_{};
    HitProgress hit_progress_{};
    RefereeLink referee_link_{RefereeConfig{}};
    // Real elapsed time between update_hit_progress calls; HitProgress must
    // not assume the loop runs at the camera's nominal framerate.
    Clock::time_point last_hit_progress_time_{};
    // Hik lit/unlit profile switching runs off the hot path: the main loop
    // only arms the switch, a background thread applies it (can block on the
    // backend mutex up to the camera read timeout).
    std::atomic<int> active_hik_profile_difficulty_{1};
    std::atomic<bool> profile_switch_pending_{false};
    std::atomic<Clock::time_point> profile_switch_fail_until_{};
    // Debounce deadline: once local/referee lock count reaches stage 3, wait
    // profile_switch_delay_s before switching to unlit so a jittery counter
    // does not flip the camera profile mid-lock.
    Clock::time_point profile_switch_deadline_{};
    std::jthread profile_switch_thread_{};
    std::optional<CaptureFormat> negotiated_format_{};
    OverlayRenderer overlay_{};
    bool window_open_ = false;
    std::jthread main_thread_{};
};

} // namespace rmcs_laser_guidance::runtime_internal
