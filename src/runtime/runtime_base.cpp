#include "runtime/runtime_base.hpp"

#include <print>
#include <utility>

#include <opencv2/highgui.hpp>

#include "core/debug_renderer.hpp"
#include "laser_guidance/support.hpp"
#include "runtime/inference_facade.hpp"
#include "runtime/runtime_support.hpp"

namespace rmcs_laser_guidance::runtime_internal {

RuntimeOutputController::RuntimeOutputController(
    RtpFramePublisher& rtp_publisher, ShmFramePublisher& shm_publisher,
    std::unique_ptr<VideoSessionRecorder>& recorder,
    std::chrono::steady_clock::time_point& recording_start)
    : rtp_publisher_(rtp_publisher)
    , shm_publisher_(shm_publisher)
    , recorder_(recorder)
    , recording_start_(recording_start) {}

auto RuntimeOutputController::start_streaming(const V4l2NegotiatedFormat& format) -> bool {
    return rtp_publisher_.start(
        format.width, format.height, static_cast<float>(format.framerate));
}

auto RuntimeOutputController::apply_requests(
    const bool streaming_requested, const bool recording_requested,
    const RecordSessionOptions& record_options,
    const std::optional<V4l2NegotiatedFormat>& negotiated_format) -> void {
    if (streaming_requested && negotiated_format.has_value() && !rtp_publisher_.is_active()) {
        start_streaming(*negotiated_format);
    } else if (!streaming_requested && rtp_publisher_.is_active()) {
        rtp_publisher_.stop();
    }

    if (recording_requested && negotiated_format.has_value()) {
        begin_recording(record_options, *negotiated_format);
    } else if (recorder_) {
        flush_recording();
    }
}

auto RuntimeOutputController::publish(cv::Mat& previous_frame) -> void {
    if (previous_frame.empty()) {
        return;
    }
    shm_publisher_.publish(previous_frame);
    rtp_publisher_.publish(std::move(previous_frame));
}

auto RuntimeOutputController::record_frame(const cv::Mat& frame) -> void {
    if (recorder_) {
        recorder_->record_frame(frame);
    }
}

auto RuntimeOutputController::stop() -> void {
    flush_recording();
    rtp_publisher_.stop();
    shm_publisher_.stop();
}

auto RuntimeOutputController::streaming_active() const -> bool { return rtp_publisher_.is_active(); }

auto RuntimeOutputController::recording_active() const -> bool {
    return static_cast<bool>(recorder_);
}

auto RuntimeOutputController::recording_root() const -> std::filesystem::path {
    return recorder_ ? recorder_->session_root() : std::filesystem::path{};
}

auto RuntimeOutputController::begin_recording(
    const RecordSessionOptions& record_options, const V4l2NegotiatedFormat& format) -> void {
    if (recorder_ || record_options.output_root.empty()) {
        return;
    }
    const auto capture_start = std::chrono::system_clock::now();
    const auto session_id = format_session_id(capture_start);
    recorder_ = std::make_unique<VideoSessionRecorder>(
        record_options.output_root,
        VideoSessionMetadata{
            .session_id = session_id,
            .relative_video_path = "raw.mp4",
            .device_path = format.device_path,
            .width = format.width,
            .height = format.height,
            .framerate = format.framerate > 0.0 ? format.framerate : 60.0,
            .fourcc = format.fourcc,
            .capture_start_unix_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(capture_start.time_since_epoch())
                    .count(),
            .duration_ms = 0,
            .lighting_tag = record_options.lighting_tag,
            .background_tag = record_options.background_tag,
            .distance_tag = record_options.distance_tag,
            .target_color = record_options.target_color,
            .operator_note_present = false,
        });
    recording_start_ = std::chrono::steady_clock::now();
}

auto RuntimeOutputController::flush_recording() -> void {
    if (!recorder_) {
        return;
    }
    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - recording_start_)
                                 .count();
    recorder_->flush(duration_ms);
    recorder_.reset();
}

RuntimeBase::RuntimeBase(
    Config config, std::string window_name, const bool enable_guidance,
    RecordSessionOptions record_options)
    : config_(std::move(config))
    , window_name_(std::move(window_name))
    , enable_guidance_(enable_guidance)
    , record_options_(std::move(record_options))
    , capture_(config_.v4l2)
    , telemetry_(config_.udp)
    , shm_publisher_()
    , rtp_publisher_(config_.rtp)
    , inference_(config_)
    , tracker_(config_.ekf)
    , stale_policy_{
          .max_input_age_ms = std::chrono::milliseconds(config_.runtime.max_input_age_ms),
          .max_observation_age_ms =
              std::chrono::milliseconds(config_.runtime.max_observation_age_ms),
      }
    , output_controller_(rtp_publisher_, shm_publisher_, recorder_, recording_start_) {
    state_.enemy_color = to_enemy_color(config_.inference.enemy_class_id);
    state_.ekf_enabled = config_.ekf.enabled;
    state_.streaming_requested = config_.rtp.enabled;
    state_.recording_requested = config_.runtime.record_enabled;
}

RuntimeBase::~RuntimeBase() {
    stop();
    join();
}

auto RuntimeBase::start() -> std::expected<void, std::string> {
    std::scoped_lock lock(state_.mutex);
    if (state_.running) {
        return {};
    }
    const auto open_result = capture_.open();
    if (!open_result) {
        return std::unexpected(open_result.error());
    }
    negotiated_format_ = *open_result;

    if (auto result = inference_.start(); !result) {
        state_.last_error = result.error();
        update_status_locked();
        return result;
    }

    if (enable_guidance_ && config_.guidance.enabled) {
        initialize_guidance();
    }

    if (state_.streaming_requested) {
        state_.snapshot.status.streaming_active = output_controller_.start_streaming(*negotiated_format_);
    }

    shm_publisher_.start(negotiated_format_->width, negotiated_format_->height);
    state_.running = true;
    state_.stop_requested = false;
    update_status_locked();
    main_thread_ = std::thread([this] { run(); });
    return {};
}

auto RuntimeBase::stop() -> void {
    {
        std::scoped_lock lock(state_.mutex);
        if (!state_.running) {
            return;
        }
        state_.stop_requested = true;
        update_status_locked();
    }
    frame_queue_.shutdown();
}

auto RuntimeBase::join() -> void {
    if (main_thread_.joinable()) {
        main_thread_.join();
    }
}

auto RuntimeBase::submit_command(const RuntimeCommand& command) -> std::expected<void, std::string> {
    if (command.type == RuntimeCommandType::set_backend) {
        if (!inference_.has_backend(command.backend)) {
            return std::unexpected("requested backend is not available");
        }
        if (!inference_.set_active_backend(command.backend)) {
            return std::unexpected("failed to switch active backend");
        }
    }

    bool shutdown_queue = false;
    std::scoped_lock lock(state_.mutex);
    switch (command.type) {
    case RuntimeCommandType::set_streaming:
        state_.streaming_requested = command.enabled;
        break;
    case RuntimeCommandType::set_recording:
        state_.recording_requested = command.enabled;
        break;
    case RuntimeCommandType::set_enemy_color:
        state_.enemy_color = command.enemy_color;
        break;
    case RuntimeCommandType::set_backend:
        break;
    case RuntimeCommandType::set_ekf:
        state_.ekf_enabled = command.enabled;
        break;
    case RuntimeCommandType::shutdown:
        state_.stop_requested = true;
        shutdown_queue = true;
        break;
    }
    update_status_locked();
    if (shutdown_queue) {
        frame_queue_.shutdown();
    }
    return {};
}

auto RuntimeBase::snapshot() const -> RuntimeSnapshot {
    std::scoped_lock lock(state_.mutex);
    return state_.snapshot;
}

auto RuntimeBase::initialize_guidance() -> void {
    auto spi_result = Ft4222Spi::open(
        Ft4222Config{
            .sys_clock = Ft4222SysClock::k60MHz,
            .clock_div = Ft4222SpiDiv::kDiv2,
            .cpol = Ft4222Cpol::kIdleLow,
            .cpha = Ft4222Cpha::kTrailing,
            .cs_active = Ft4222CsActive::kLow,
            .cs_channel = 0,
        });
    if (!spi_result) {
        state_.last_error = "FT4222 open failed: " + spi_result.error();
        return;
    }
    spi_ = std::make_unique<Ft4222Spi>(std::move(*spi_result));
    solver_ = std::make_unique<AimSolver>(config_, negotiated_format_->width, negotiated_format_->height);
    executor_ = std::make_unique<GalvoExecutor>(config_, *spi_);
    if (executor_->is_initialized()) {
        scanner_ = std::make_unique<ScanController>(config_.guidance, *executor_);
    }
}

auto RuntimeBase::make_snapshot_locked(
    const DetectionBatch& batch, const std::optional<TargetTrack>& track, const AimOutput& aim,
    const HitProgress& hit_progress, const std::size_t dropped_frames,
    const std::filesystem::path& recording_root) -> RuntimeSnapshot {
    state_.snapshot.detection = batch;
    state_.snapshot.track = track;
    state_.snapshot.aim = aim;
    state_.snapshot.hit_progress = make_hit_progress_snapshot(hit_progress);
    state_.snapshot.dropped_frames = dropped_frames;
    state_.snapshot.current_recording_root = recording_root;
    state_.snapshot.status.last_guidance_message = aim.message;
    update_status_locked();
    return state_.snapshot;
}

auto RuntimeBase::update_status_locked() -> void {
    const auto active_backend = inference_.active_backend();
    state_.snapshot.status = make_runtime_status(
        state_.snapshot, state_.running, state_.stop_requested, capture_.is_open(),
        inference_.enabled(), config_.guidance.enabled && enable_guidance_,
        executor_ && executor_->is_initialized() && solver_ && solver_->is_initialized(),
        state_.ekf_enabled, active_backend, state_.enemy_color, state_.last_error,
        output_controller_.streaming_active(), output_controller_.recording_active());
    state_.snapshot.active_backend_name = inference_.active_backend_name();
    if (negotiated_format_) {
        state_.snapshot.negotiated_format = make_capture_snapshot(*negotiated_format_);
    }
}

auto RuntimeBase::read_results() const -> RuntimeResults {
    std::scoped_lock lock(result_mutex_);
    return RuntimeResults{
        .detection = latest_detection_,
        .ekf_state = latest_ekf_,
    };
}

auto RuntimeBase::select_track(
    const DetectionBatch& detection, const std::optional<EkfState>& ekf_state) const -> TargetTrack {
    const bool ekf_enabled = [&] {
        std::scoped_lock lock(state_.mutex);
        return state_.ekf_enabled;
    }();
    return select_target_track(
        detection, ekf_state, ekf_enabled, static_cast<float>(config_.ekf.lookahead_ms));
}

auto RuntimeBase::draw_results_overlay(
    cv::Mat& display, const DetectionBatch& detection, const std::optional<EkfState>& ekf_state) const
    -> void {
    if (detection.detected || !detection.detections.empty()) {
        draw_candidates(display, to_model_candidates(detection));
    }
    if (ekf_state.has_value()) {
        draw_ekf_state(display, *ekf_state);
    }
}

auto RuntimeBase::queue_frame_for_inference(Frame& frame) -> bool {
    if (!inference_.enabled()) {
        return true;
    }
    try {
        frame_queue_.push(QueuedFrame{
            .image = std::move(frame.image),
            .capture_time = frame.timestamp,
        });
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

auto RuntimeBase::process_guidance_step(
    const DetectionBatch& detection, const TargetTrack& track, const float last_valid_depth_mm,
    const bool ekf_was_lost) -> GuidanceStepResult {
    GuidanceStepResult result{
        .last_valid_depth_mm = last_valid_depth_mm,
        .ekf_was_lost = ekf_was_lost,
    };

    // State transition rules:
    //   ekf=OFF + detected              -> solve -> update depth
    //   ekf=OFF + undetected + depth    -> recenter + clear depth + stop scan
    //   ekf=OFF + undetected + no depth -> idle
    //   ekf=ON  + initialized + !lost   -> solve -> update depth
    //   ekf=ON  + lost(first frame)     -> recenter + clear depth + stop scan
    //   ekf=ON  + lost(steady)          -> idle
    //
    // Guidance-unready frames intentionally keep ekf_was_lost synchronized to track.lost,
    // matching the pre-refactor main-loop behavior.
    if (!(solver_ && executor_ && solver_->is_initialized() && executor_->is_initialized())) {
        return GuidanceStepResult{
            .aim_output = std::move(result.aim_output),
            .last_valid_depth_mm = result.last_valid_depth_mm,
            .ekf_was_lost = track.lost,
        };
    }

    AimInput aim_input{
        .ekf_enabled = track.ekf_enabled,
        .track = track,
        .lidar_frame = detection.lidar_frame,
        .last_valid_depth_mm = result.last_valid_depth_mm,
    };

    if (!track.ekf_enabled) {
        if (track.detected) {
            result.aim_output = solver_->solve(aim_input);
            result.last_valid_depth_mm = aim_input.last_valid_depth_mm;
        } else if (result.last_valid_depth_mm > 0.0F) {
            result.aim_output.recentered = true;
            result.aim_output.message = executor_->set_center();
            result.last_valid_depth_mm = 0.0F;
            if (scanner_) {
                scanner_->deactivate();
            }
        }
    } else if (track.initialized && !track.lost) {
        result.aim_output = solver_->solve(aim_input);
        result.last_valid_depth_mm = aim_input.last_valid_depth_mm;
    } else if (track.lost && !result.ekf_was_lost) {
        result.aim_output.recentered = true;
        result.aim_output.message = executor_->set_center();
        result.last_valid_depth_mm = 0.0F;
        if (scanner_) {
            scanner_->deactivate();
        }
    }

    if (result.aim_output.command_issued) {
        if (scanner_ && scanner_->enabled()) {
            if (result.aim_output.output_angles.has_value()) {
                scanner_->update_angles_center(*result.aim_output.output_angles);
            } else if (result.aim_output.output_voltages.has_value()) {
                scanner_->update_voltage_center(*result.aim_output.output_voltages);
            }
        } else {
            const auto apply_result = executor_->apply(result.aim_output);
            if (!apply_result.empty()) {
                result.aim_output.message = apply_result;
            }
        }
    }

    result.aim_output.output_angles = executor_->latest_output_angles();
    result.aim_output.output_voltages = executor_->latest_output_voltages();
    result.ekf_was_lost = track.lost;
    return result;
}

auto RuntimeBase::update_hit_state(const DetectionBatch& detection) -> void {
    const auto top_detection = detection.detections.empty() ? nullptr : &detection.detections.front();
    const bool is_purple = detection.detected && top_detection != nullptr && top_detection->class_id == 0
                        && top_detection->score >= 0.25F;
    const float frame_dt_s = negotiated_format_->framerate > 0.0
                               ? 1.0F / static_cast<float>(negotiated_format_->framerate)
                               : 1.0F / 60.0F;
    hit_progress_.update(is_purple, frame_dt_s);
}

auto RuntimeBase::draw_runtime_overlay(
    cv::Mat& display, const TargetTrack& track, const AimOutput& aim_output) -> void {
    const bool guidance_ok =
        solver_ && executor_ && solver_->is_initialized() && executor_->is_initialized();
    const bool ekf_ok = track.ekf_enabled ? (track.initialized && !track.lost) : track.detected;
    draw_guidance_status(
        display, guidance_ok, ekf_ok, aim_output.depth_valid,
        track.ekf_enabled ? aim_output.message : std::string("EKF OFF (raw)"));
    draw_hit_progress(display, hit_progress_);
    {
        std::scoped_lock lock(state_.mutex);
        draw_status_bar(
            display, output_controller_.streaming_active(), output_controller_.recording_active(),
            to_enemy_class_id(state_.enemy_color),
            inference_.active_backend() == RuntimeBackend::tensorrt);
    }
}

auto RuntimeBase::apply_output_requests() -> void {
    std::scoped_lock lock(state_.mutex);
    output_controller_.apply_requests(
        state_.streaming_requested, state_.recording_requested, record_options_, negotiated_format_);
}

auto RuntimeBase::publish_snapshot(
    cv::Mat& display, const DetectionBatch& detection, const TargetTrack& track,
    const AimOutput& aim_output) -> void {
    output_controller_.record_frame(display);

    RuntimeSnapshot snapshot;
    {
        std::scoped_lock lock(state_.mutex);
        snapshot = make_snapshot_locked(
            detection, track, aim_output, hit_progress_, frame_queue_.overwrite_count(),
            output_controller_.recording_root());
    }
    telemetry_.publish(snapshot);
    after_frame_processed(display, snapshot);
}

auto RuntimeBase::run_inference_worker() -> void {
    try {
        while (true) {
            QueuedFrame queued_frame;
            try {
                queued_frame = frame_queue_.pop();
            } catch (const std::exception&) {
                break;
            }
            const auto worker_start = Clock::now();
            auto before_infer = stale_policy_.make_before_inference_sample(
                queued_frame.capture_time, worker_start, Clock::now());
            if (before_infer.stale_reason != StaleReason::none) {
                continue;
            }

            const Frame infer_frame{
                .image = queued_frame.image,
                .timestamp = queued_frame.capture_time,
            };
            const auto infer_start = Clock::now();
            auto infer_result = inference_.infer(infer_frame);
            if (!infer_result.has_value()) {
                continue;
            }
            auto batch = to_detection_batch(*infer_result);

            EnemyColor enemy_color = EnemyColor::auto_select;
            {
                std::scoped_lock lock(state_.mutex);
                enemy_color = state_.enemy_color;
            }
            filter_detections(batch.detections, enemy_color);
            if (!batch.detections.empty() && batch.detections.front().score >= 0.25F) {
                batch.detected = true;
                batch.selected_center = batch.detections.front().center;
            } else {
                batch.detected = false;
                batch.selected_center = {-1.0F, -1.0F};
            }

            if (batch.detected) {
                tracker_.process(batch.selected_center, infer_frame.timestamp);
            } else {
                tracker_.predict(infer_frame.timestamp);
            }

            const auto publish_time = Clock::now();
            auto after_publish = stale_policy_.make_after_publish_sample(
                queued_frame.capture_time, worker_start, infer_start, publish_time);
            if (after_publish.stale_reason != StaleReason::none) {
                continue;
            }

            std::scoped_lock lock(result_mutex_);
            latest_detection_ = std::move(batch);
            latest_ekf_ = tracker_.state();
        }
    } catch (const std::exception& e) {
        std::scoped_lock lock(state_.mutex);
        state_.last_error = std::string("inference worker error: ") + e.what();
    }
}

auto RuntimeBase::run() -> void {
    if (inference_.enabled()) {
        inference_thread_ = std::thread([this] { run_inference_worker(); });
    }

    cv::Mat output;
    bool ekf_was_lost = false;
    float last_valid_depth_mm = 0.0F;

    while (true) {
        {
            std::scoped_lock lock(state_.mutex);
            if (state_.stop_requested) {
                break;
            }
        }
        auto frame = capture_.read_frame();
        if (!frame) {
            std::scoped_lock lock(state_.mutex);
            state_.last_error = frame.error();
            continue;
        }

        output_controller_.publish(output);

        cv::Mat display = frame->image.clone();

        const auto results = read_results();
        auto detection = results.detection;
        auto ekf_state = results.ekf_state;
        draw_results_overlay(display, detection, ekf_state);

        if (!queue_frame_for_inference(*frame)) {
            break;
        }

        TargetTrack track = select_track(detection, ekf_state);
        const auto guidance = process_guidance_step(detection, track, last_valid_depth_mm, ekf_was_lost);
        auto aim_output = guidance.aim_output;
        last_valid_depth_mm = guidance.last_valid_depth_mm;
        ekf_was_lost = guidance.ekf_was_lost;

        update_hit_state(detection);
        draw_runtime_overlay(display, track, aim_output);
        apply_output_requests();
        publish_snapshot(display, detection, track, aim_output);

        output = display;

        if (config_.debug.show_window) {
            cv::imshow(window_name_, display);
            const int key = cv::waitKey(1);
            if (should_exit_from_key(key)) {
                stop();
            }
            if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1) {
                stop();
            }
        }
    }

    if (scanner_) {
        scanner_->stop();
    }
    if (inference_thread_.joinable()) {
        inference_thread_.join();
    }
    inference_.stop();
    if (executor_ && executor_->is_initialized()) {
        if (const auto center_error = executor_->set_center(); !center_error.empty()) {
            std::println(stderr, "guidance: {}", center_error);
        }
    }
    output_controller_.stop();
    capture_.close();
    {
        std::scoped_lock lock(state_.mutex);
        state_.running = false;
        state_.stop_requested = false;
        update_status_locked();
    }
}

CompetitionRuntimeAdapter::CompetitionRuntimeAdapter(Config config, RecordSessionOptions record_options)
    : RuntimeBase(std::move(config), kCompetitionWindowName, true, std::move(record_options)) {}

auto CompetitionRuntimeAdapter::after_frame_processed(cv::Mat&, const RuntimeSnapshot&) -> void {}

PreviewRuntimeAdapter::PreviewRuntimeAdapter(Config config)
    : RuntimeBase(std::move(config), kPreviewWindowName, false) {}

auto PreviewRuntimeAdapter::after_frame_processed(cv::Mat&, const RuntimeSnapshot&) -> void {}

} // namespace rmcs_laser_guidance::runtime_internal
