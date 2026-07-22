#include "runtime/control_loop.hpp"

#include <chrono>
#include <iostream>
#include <print>
#include <utility>

#include <opencv2/highgui.hpp>

#include "laser_guidance/support.hpp"
#include "runtime/capture_retry_policy.hpp"

namespace rmcs_laser_guidance::runtime_internal {
namespace {

constexpr const char* kMainWindowName = "laser_guidance_competition";
constexpr const char* kPreviewWindowName = "laser_guidance_preview";

auto to_enemy_color(const int class_id) -> EnemyColor {
    switch (class_id) {
    case 1: return EnemyColor::red;
    case 2: return EnemyColor::blue;
    default: return EnemyColor::auto_select;
    }
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
        .difficulty = progress.difficulty(),
        .p0 = progress.p0(),
        .is_exhausted = progress.is_exhausted(),
    };
}

auto make_capture_snapshot(const CaptureFormat& format) -> CaptureFormatSnapshot {
    return CaptureFormatSnapshot{
        .device_path = format.device_path,
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .fourcc = format.pixel_encoding,
    };
}

auto make_runtime_status(
    const RuntimeSnapshot& previous_snapshot, const bool running, const bool stop_requested,
    const bool capture_open, const bool inference_enabled, const bool guidance_enabled,
    const bool guidance_ready, const bool ekf_enabled,
    const std::optional<RuntimeBackend> active_backend, const EnemyColor enemy_color,
    std::string last_error, const bool streaming_active, const bool recording_active)
    -> RuntimeStatus {
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

} // namespace

ControlLoop::ControlLoop(Config config, CompetitionRuntimeOptions options)
    : config_(config)
    , options_(options)
    , capture_(config)
    , perception_(config)
    , outputs_(
          config,
          options.profile == CompetitionProfile::main ? options.record_options
                                                      : RecordSessionOptions{},
          make_output_capabilities(options.profile)) {
    state_.enemy_color = to_enemy_color(config_.inference.enemy_class_id);
    state_.ekf_enabled = config_.ekf.enabled;
    state_.streaming_requested = config_.rtp.enabled && allows_streaming();
    state_.recording_requested = config_.runtime.record_enabled && allows_recording();
}

ControlLoop::~ControlLoop() {
    stop();
    join();
}

auto ControlLoop::start() -> std::expected<void, std::string> {
    {
        std::scoped_lock lock(state_mutex_);
        if (state_.running) {
            return {};
        }
    }

    if (auto result = initialize_components(); !result) {
        teardown_components();
        return result;
    }

    {
        std::scoped_lock lock(state_mutex_);
        state_.running = true;
        state_.stop_requested = false;
        update_status_locked();
    }

    main_thread_ = std::thread([this] { run_loop(); });
    return {};
}

auto ControlLoop::run() -> std::expected<void, std::string> {
    {
        std::scoped_lock lock(state_mutex_);
        if (state_.running) {
            return std::unexpected("control loop is already running");
        }
    }

    if (auto result = initialize_components(); !result) {
        teardown_components();
        return result;
    }

    {
        std::scoped_lock lock(state_mutex_);
        state_.running = true;
        state_.stop_requested = false;
        update_status_locked();
    }

    run_loop();
    return {};
}

auto ControlLoop::stop() -> void { request_stop(); }

auto ControlLoop::join() -> void {
    if (main_thread_.joinable()) {
        main_thread_.join();
    }
}

auto ControlLoop::submit_command(const RuntimeCommand& command)
    -> std::expected<void, std::string> {
    if (command.type == RuntimeCommandType::set_backend) {
        if (!perception_.has_backend(command.backend)) {
            return std::unexpected("requested backend is not available");
        }
        if (!perception_.set_active_backend(command.backend)) {
            return std::unexpected("failed to switch active backend");
        }
    }

    EnemyColor enemy_color = EnemyColor::auto_select;
    {
        std::scoped_lock lock(state_mutex_);
        switch (command.type) {
        case RuntimeCommandType::set_streaming:
            state_.streaming_requested = command.enabled && allows_streaming();
            break;
        case RuntimeCommandType::set_recording:
            state_.recording_requested = command.enabled && allows_recording();
            break;
        case RuntimeCommandType::set_enemy_color:
            state_.enemy_color = command.enemy_color;
            break;
        case RuntimeCommandType::set_backend: break;
        case RuntimeCommandType::set_ekf:
            state_.ekf_enabled = command.enabled;
            break;
        case RuntimeCommandType::shutdown:
            state_.stop_requested = true;
            break;
        }
        enemy_color = state_.enemy_color;
        update_status_locked();
    }

    perception_.set_enemy_color(enemy_color);
    if (command.type == RuntimeCommandType::shutdown) {
        request_stop();
    }
    return {};
}

auto ControlLoop::snapshot() const -> RuntimeSnapshot {
    std::scoped_lock lock(state_mutex_);
    return state_.latest_snapshot;
}

auto ControlLoop::make_output_capabilities(const CompetitionProfile profile)
    -> RuntimeOutputCapabilities {
    switch (profile) {
    case CompetitionProfile::main:
        return RuntimeOutputCapabilities{
            .allow_rtp = true,
            .allow_shm = true,
            .allow_telemetry = true,
            .allow_recording = true,
        };
    case CompetitionProfile::preview:
        return RuntimeOutputCapabilities{
            .allow_rtp = true,
            .allow_shm = true,
            .allow_telemetry = true,
            .allow_recording = false,
        };
    }
    return {};
}

auto ControlLoop::initialize_components() -> std::expected<void, std::string> {
    previous_output_ = cv::Mat{};
    negotiated_format_.reset();
    guidance_.reset();

    const auto open_result = capture_.open();
    if (!open_result) {
        std::println(
            stderr, "camera init failed: {}, will retry...", open_result.error());
        sync_last_error("Camera open failed: " + open_result.error());
    } else {
        negotiated_format_ = *open_result;
    }

    if (auto result = perception_.start(); !result) {
        std::println(stderr, "perception init failed: {}, degraded runtime", result.error());
        sync_last_error("Perception init failed: " + result.error());
    }
    perception_.set_enemy_color(state_.enemy_color);

    if (guidance_enabled_in_profile() && negotiated_format_.has_value()) {
                auto guidance = try_create_guidance_session(config_, *negotiated_format_, nullptr);
        if (!guidance) {
            std::println(
                stderr, "guidance init failed: {}, guidance disabled", guidance.error());
            sync_last_error("Guidance init failed: " + guidance.error());
        } else {
            guidance_ = std::move(*guidance);
        }
    }

    if (negotiated_format_.has_value()) {
        outputs_.start(
            *negotiated_format_, state_.streaming_requested, state_.recording_requested);
    }

    ros_bridge_ = std::make_unique<RosBridge>();

    if (show_window()) {
        cv::namedWindow(window_name(), cv::WINDOW_NORMAL);
        window_open_ = true;
    }
    return {};
}

auto ControlLoop::run_loop() -> void {
    using Clock = std::chrono::steady_clock;
    constexpr auto kReadErrorDelay = std::chrono::milliseconds(100);

    CaptureRetryPolicy retry_policy;

    while (!stop_requested()) {
        if (retry_policy.reconnect_pending()) {
            const auto now = Clock::now();
            if (!retry_policy.reconnect_due(now)) {
                std::this_thread::sleep_for(kReadErrorDelay);
                continue;
            }

            sync_last_error("Attempting to reconnect camera...");
            if (auto reconnect_result = capture_.reconnect(); reconnect_result) {
                std::println("camera reconnected");
                sync_last_error("Camera reconnected");

                auto new_format = capture_.negotiated_format();

                {
                    std::scoped_lock lock(state_mutex_);
                    negotiated_format_ = std::move(new_format);
                }

                if (guidance_enabled_in_profile()) {
                    {
                        std::scoped_lock lock(state_mutex_);
                        guidance_.reset();
                    }
                    retry_policy.arm_guidance_retry(Clock::now());
                }

                retry_policy.on_reconnect_succeeded();
            } else {
                std::println(
                    stderr, "reconnect failed: {}", reconnect_result.error());
                sync_last_error("Reconnect failed: " + reconnect_result.error());
                retry_policy.on_reconnect_failed(Clock::now());
            }
            continue;
        }

        if (guidance_enabled_in_profile() && !guidance_ && negotiated_format_.has_value()) {
            const auto now = Clock::now();
            if (retry_policy.guidance_retry_due(now)) {
        auto guidance = try_create_guidance_session(config_, *negotiated_format_, nullptr);
                if (guidance) {
                    {
                        std::scoped_lock lock(state_mutex_);
                        guidance_ = std::move(*guidance);
                    }
                    retry_policy.clear_guidance_retry();
                    std::println("guidance initialized");
                } else {
                    std::println(stderr, "guidance retry failed: {}", guidance.error());
                    retry_policy.defer_guidance_retry(now);
                }
            }
        }

        auto frame_result = capture_.read_frame();
        if (!frame_result) {
            sync_last_error(frame_result.error());

            if (retry_policy.on_read_error(Clock::now())) {
                decltype(guidance_) stale_guidance;
                {
                    std::scoped_lock lock(state_mutex_);
                    stale_guidance = std::move(guidance_);
                }
                if (stale_guidance) {
                    stale_guidance->shutdown();
                }
                std::println(
                    stderr, "camera read failed repeatedly: {}, entering reconnect state",
                    frame_result.error());
                sync_last_error("Camera read failed repeatedly; entering reconnect state");
            } else {
                std::this_thread::sleep_for(kReadErrorDelay);
            }
            continue;
        }

        retry_policy.on_read_success();

        ControlLoopFrame frame;
        frame.frame = std::move(*frame_result);
        frame.display = frame.frame.image.clone();

        outputs_.publish_previous(previous_output_);

        const auto perception_result = perception_.poll();
        frame.detection = perception_result.detection;
        frame.ekf_state = perception_result.ekf_state;
        frame.dropped_frames = perception_.overwrite_count();

        // Periodic detection log (~1 Hz)
        {
            static auto last_log = Clock::now();
            const auto now = Clock::now();
            if (now - last_log >= std::chrono::milliseconds(500)) {
                last_log = now;
                if (frame.detection.detected && !frame.detection.detections.empty()) {
                    const auto& top = frame.detection.detections.front();
                    std::println(
                        std::clog,
                        "[DETECT] score={:.3f} class_id={} bbox=[{:.0f}, {:.0f}, {:.0f}, {:.0f}]",
                        top.score, top.class_id, top.bbox.x, top.bbox.y, top.bbox.width,
                        top.bbox.height);
                } else if (!frame.detection.detections.empty()) {
                    const auto& top = frame.detection.detections.front();
                    std::println(
                        std::clog,
                        "[DETECT] below_threshold top_score={:.4f} candidates={}",
                        top.score, frame.detection.detections.size());
                } else {
                    std::println(std::clog, "[DETECT] no_candidates");
                }
            }
        }
        if (const auto error = perception_.last_error(); !error.empty()) {
            sync_last_error(error);
        }

        if (perception_.degraded()) {
            frame.detection = {};
            frame.ekf_state.reset();
        } else if (!perception_.submit(frame.frame)) {
            sync_last_error(perception_.last_error());
            request_stop();
            break;
        }

        bool ekf_enabled = false;
        EnemyColor enemy_color = EnemyColor::auto_select;
        bool streaming_requested = false;
        bool recording_requested = false;
        {
            std::scoped_lock lock(state_mutex_);
            ekf_enabled = state_.ekf_enabled;
            enemy_color = state_.enemy_color;
            streaming_requested = state_.streaming_requested;
            recording_requested = state_.recording_requested;
        }

        frame.track = select_target_track(
            frame.detection, frame.ekf_state, ekf_enabled, config_.ekf.lookahead_ms);
        if (guidance_) {
            frame.guidance = guidance_->execute(frame.track);
        }

        update_hit_progress(frame.detection);
        maybe_switch_hik_profile();
        const auto pre_output_status = outputs_.status();
        overlay_.render(
            frame,
            OverlayRenderContext{
                .guidance_enabled = guidance_.has_value(),
                .guidance_ready = guidance_.has_value(),
                .calibration_mode = false,
                .command_model = config_.guidance.command_model,
                .calibration_state = nullptr,
                .hit_progress =
                    options_.profile == CompetitionProfile::main ? &hit_progress_ : nullptr,
                .streaming_active = pre_output_status.streaming_active,
                .recording_active = pre_output_status.recording_active,
                .enemy_color = enemy_color,
                .using_tensorrt =
                    perception_.active_backend() == RuntimeBackend::tensorrt,
            });

        outputs_.apply_requests(
            streaming_requested, recording_requested, negotiated_format_);
        outputs_.record_current(frame.display);
        const auto output_status = outputs_.status();

        RuntimeSnapshot latest_snapshot;
        {
            std::scoped_lock lock(state_mutex_);
            latest_snapshot = assemble_snapshot(frame, output_status);
            state_.latest_snapshot = latest_snapshot;
        }
        outputs_.publish_snapshot(latest_snapshot);

        if (ros_bridge_ && ros_bridge_->ready()) {
            ros_bridge_->publish_snapshot(latest_snapshot);
            ros_bridge_->spin();
        }

        if (show_window()) {
            cv::imshow(window_name(), frame.display);
            const int key = cv::waitKey(1);
            const bool window_visible =
                cv::getWindowProperty(window_name(), cv::WND_PROP_VISIBLE) >= 1.0;
            if (should_exit_from_key(key) || !window_visible) {
                request_stop();
            }
        }

        previous_output_ = frame.display;
    }

    teardown_components();
}

auto ControlLoop::teardown_components() -> void {
    perception_.stop();
    if (guidance_) {
        guidance_->shutdown();
    }
    outputs_.stop();
    capture_.close();
    if (window_open_) {
        cv::destroyWindow(window_name());
        window_open_ = false;
    }

    {
        std::scoped_lock lock(state_mutex_);
        state_.running = false;
        state_.stop_requested = false;
        update_status_locked();
    }

    negotiated_format_.reset();
    guidance_.reset();
    previous_output_ = cv::Mat{};
}

auto ControlLoop::request_stop() -> void {
    {
        std::scoped_lock lock(state_mutex_);
        if (!state_.running) {
            return;
        }
        state_.stop_requested = true;
        update_status_locked();
    }
    perception_.shutdown();
}

auto ControlLoop::stop_requested() const -> bool {
    std::scoped_lock lock(state_mutex_);
    return state_.stop_requested;
}

auto ControlLoop::update_status_locked() -> void {
    const auto output_status = outputs_.status();
    const auto active_backend = perception_.active_backend();
    state_.latest_snapshot.status = make_runtime_status(
        state_.latest_snapshot, state_.running, state_.stop_requested, capture_.is_open(),
        perception_.enabled(), guidance_.has_value(), guidance_.has_value(), state_.ekf_enabled,
        active_backend, state_.enemy_color, state_.last_error, output_status.streaming_active,
        output_status.recording_active);
    state_.latest_snapshot.active_backend_name = perception_.active_backend_name();
    state_.latest_snapshot.current_recording_root = output_status.recording_root;
    if (negotiated_format_) {
        state_.latest_snapshot.negotiated_format = make_capture_snapshot(*negotiated_format_);
    }
}

auto ControlLoop::sync_last_error(std::string error) -> void {
    std::scoped_lock lock(state_mutex_);
    state_.last_error = std::move(error);
    update_status_locked();
}

auto ControlLoop::update_hit_progress(const DetectionBatch& detection) -> void {
    if (options_.profile != CompetitionProfile::main || !negotiated_format_) {
        return;
    }
    const auto* top_detection = detection.detections.empty() ? nullptr : &detection.detections.front();
    const bool is_purple = detection.detected && top_detection != nullptr && top_detection->class_id == 0
                        && top_detection->score >= 0.25F;
    const float frame_dt_s = negotiated_format_->framerate > 0.0
                               ? 1.0F / static_cast<float>(negotiated_format_->framerate)
                               : 1.0F / 60.0F;
    hit_progress_.update(is_purple, frame_dt_s);
}

auto ControlLoop::maybe_switch_hik_profile() -> void {
    if (config_.capture_backend != CaptureBackendKind::hikcamera) {
        return;
    }
    if (!config_.hik.has_unlit_profile) {
        return;
    }
    if (!capture_.is_open()) {
        return;
    }

    const int difficulty = hit_progress_.difficulty();
    const bool want_unlit = difficulty >= 3;
    const int target = want_unlit ? 3 : 1;
    if (target == active_hik_profile_difficulty_) {
        return;
    }

    const HikRuntimeProfile profile =
        want_unlit ? config_.hik.unlit : config_.hik.lit_profile();
    if (auto applied = capture_.apply_runtime_profile(profile); !applied) {
        std::println(
            stderr, "Hik profile switch to {} failed: {}", want_unlit ? "unlit" : "lit",
            applied.error());
        return;
    }

    active_hik_profile_difficulty_ = target;
    if (negotiated_format_ && profile.framerate > 0.0F) {
        negotiated_format_->framerate = static_cast<double>(profile.framerate);
    }
    std::println(
        stderr,
        "Hik profile -> {} (difficulty={}) exposure_us={} gain={} fps={} wb={}",
        want_unlit ? "unlit" : "lit", difficulty, profile.exposure_us, profile.gain,
        profile.framerate, profile.set_white_balance);
}

auto ControlLoop::show_window() const -> bool { return config_.debug.show_window; }

auto ControlLoop::window_name() const -> const char* {
    return options_.profile == CompetitionProfile::preview ? kPreviewWindowName : kMainWindowName;
}

auto ControlLoop::allows_streaming() const -> bool { return outputs_.capabilities().allow_rtp; }

auto ControlLoop::allows_recording() const -> bool {
    return outputs_.capabilities().allow_recording;
}

auto ControlLoop::guidance_enabled_in_profile() const -> bool {
    return options_.profile == CompetitionProfile::main && config_.guidance.enabled
        && !config_.guidance.calib_mode;
}

auto ControlLoop::assemble_snapshot(
    const ControlLoopFrame& frame, const RuntimeOutputsStatus& output_status) const
    -> RuntimeSnapshot {
    RuntimeSnapshot snapshot = state_.latest_snapshot;
    snapshot.detection = frame.detection;
    snapshot.track = frame.track;
    snapshot.aim = frame.guidance.aim_output;
    snapshot.dropped_frames = frame.dropped_frames;
    snapshot.hit_progress =
        options_.profile == CompetitionProfile::main ? make_hit_progress_snapshot(hit_progress_)
                                                     : HitProgressSnapshot{};
    snapshot.current_recording_root = output_status.recording_root;
    snapshot.active_backend_name = perception_.active_backend_name();
    snapshot.status = make_runtime_status(
        snapshot, state_.running, state_.stop_requested, capture_.is_open(),
        perception_.enabled(), guidance_.has_value(), guidance_.has_value(), state_.ekf_enabled,
        perception_.active_backend(), state_.enemy_color, state_.last_error,
        output_status.streaming_active, output_status.recording_active);
    snapshot.status.last_guidance_message = frame.guidance.aim_output.message;
    if (negotiated_format_.has_value()) {
        snapshot.negotiated_format = make_capture_snapshot(*negotiated_format_);
    }
    return snapshot;
}

} // namespace rmcs_laser_guidance::runtime_internal
