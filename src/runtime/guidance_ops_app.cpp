#include "runtime/guidance_ops_app.hpp"

#include <algorithm>
#include <format>
#include <print>

#include <opencv2/highgui.hpp>

#include "laser_guidance/support.hpp"
#include "runtime/capture_retry_policy.hpp"

namespace rmcs_laser_guidance::runtime_internal {
namespace {

constexpr char kWindowName[] = "laser_guidance";

auto apply_calibration_key(GuidanceCalibrationState& state, const GuidanceConfig& config, int key)
    -> std::optional<std::string> {
    const bool voltage_mode = config.command_model == GuidanceCommandModelKind::direct_voltage;
    switch (key) {
    case 'w':
    case 'W':
        if (voltage_mode) {
            state.voltage_y -= state.voltage_step_v;
        } else {
            state.angle_y_deg -= state.angle_step_deg;
        }
        break;
    case 's':
    case 'S':
        if (voltage_mode) {
            state.voltage_y += state.voltage_step_v;
        } else {
            state.angle_y_deg += state.angle_step_deg;
        }
        break;
    case 'a':
    case 'A':
        if (voltage_mode) {
            state.voltage_x -= state.voltage_step_v;
        } else {
            state.angle_x_deg -= state.angle_step_deg;
        }
        break;
    case 'd':
    case 'D':
        if (voltage_mode) {
            state.voltage_x += state.voltage_step_v;
        } else {
            state.angle_x_deg += state.angle_step_deg;
        }
        break;
    case ',':
    case '<':
        if (voltage_mode) {
            state.voltage_step_v =
                std::max(GuidanceCalibrationState::kMinVoltageStepV, state.voltage_step_v * 0.5F);
            return std::format("CALIB: voltage step -> {:.3f}V", state.voltage_step_v);
        }
        state.angle_step_deg =
            std::max(GuidanceCalibrationState::kMinAngleStepDeg, state.angle_step_deg * 0.5F);
        return std::format("CALIB: angle step -> {:.3f}°", state.angle_step_deg);
    case '.':
    case '>':
        if (voltage_mode) {
            state.voltage_step_v =
                std::min(GuidanceCalibrationState::kMaxVoltageStepV, state.voltage_step_v * 2.0F);
            return std::format("CALIB: voltage step -> {:.3f}V", state.voltage_step_v);
        }
        state.angle_step_deg =
            std::min(GuidanceCalibrationState::kMaxAngleStepDeg, state.angle_step_deg * 2.0F);
        return std::format("CALIB: angle step -> {:.3f}°", state.angle_step_deg);
    default: break;
    }

    state.voltage_x = std::clamp(state.voltage_x, -config.voltage_limit_v, config.voltage_limit_v);
    state.voltage_y = std::clamp(state.voltage_y, -config.voltage_limit_v, config.voltage_limit_v);
    return std::nullopt;
}

auto format_voltage_record(
    const Clock::time_point timestamp, const Detection& top, const float manual_vx,
    const float manual_vy) -> std::string {
    const float area = top.bbox.width * top.bbox.height;
    return std::format(
        "{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.5f},{},{:.5f},{:.5f}\n",
        std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count(),
        top.center.x, top.center.y, top.bbox.x, top.bbox.y, top.bbox.width, top.bbox.height, area,
        top.score, top.class_id, manual_vx, manual_vy);
}

auto format_geometry_record(float angle_x_deg, float angle_y_deg, const cv::Point3f& point)
    -> std::string {
    return std::format(
        "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n", angle_x_deg, angle_y_deg, point.x, point.y, point.z);
}

auto should_record_hit_edge(
    const HitState previous_state, const HitState current_state, const Detection& top,
    const bool detected) -> bool {
    return current_state == HitState::Confirmed && previous_state != HitState::Confirmed && detected
        && top.class_id == 0 && top.score >= 0.25F;
}

} // namespace

GuidanceOpsApp::GuidanceOpsApp(Config config, GuidanceRecorderPaths paths)
    : config_(std::move(config))
    , paths_(std::move(paths))
    , capture_(config_)
    , perception_(config_)
    , calibration_state_(std::make_shared<GuidanceCalibrationState>())
    , hit_state_machine_(
          config_.runtime.hit_confirm_frames, config_.runtime.hit_release_frames) {
    calibration_state_->angle_x_deg = config_.guidance.calib_angle_x_deg;
    calibration_state_->angle_y_deg = config_.guidance.calib_angle_y_deg;
}

auto GuidanceOpsApp::run() -> std::expected<void, std::string> {
    if (auto result = initialize(); !result) {
        teardown();
        return result;
    }
    run_loop();
    teardown();
    return {};
}

auto GuidanceOpsApp::initialize() -> std::expected<void, std::string> {
    stop_requested_ = false;
    retry_policy_ = CaptureRetryPolicy{};

    auto format = capture_.open();
    if (!format) {
        std::println(
            stderr, "camera init failed: {}, will retry...", format.error());
        negotiated_format_.reset();
    } else {
        negotiated_format_ = *format;
    }

    if (auto result = perception_.start(); !result) {
        std::println(stderr, "perception init failed: {}, degraded guidance app", result.error());
    }

    if (config_.guidance.enabled && negotiated_format_.has_value()) {
        auto guidance = try_create_guidance_session(
            config_, *negotiated_format_,
            config_.guidance.calib_mode ? calibration_state_ : nullptr);
        if (!guidance) {
            std::println(
                stderr, "guidance init failed: {}, guidance disabled",
                guidance.error());
            retry_policy_.arm_guidance_retry(std::chrono::steady_clock::now());
        } else {
            guidance_ = std::move(*guidance);
        }
    }

    if (config_.guidance.calib_mode
        && config_.guidance.command_model == GuidanceCommandModelKind::geometry) {
        calibration_file_.open(paths_.geometry_path, std::ios::app);
        std::println("CALIB: saving records to {}", paths_.geometry_path.string());
    }

    if (config_.guidance.calib_mode
        && config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage) {
        voltage_file_.open(paths_.voltage_path, std::ios::app);
        std::println("CALIB: saving records to {}", paths_.voltage_path.string());
        if (voltage_file_.tellp() == 0) {
            voltage_file_
                << "timestamp_ns,center_x,center_y,bbox_x,bbox_y,bbox_w,bbox_h,bbox_area,score,"
                   "class_id,manual_vx,manual_vy\n";
        }
    }

    if (config_.guidance.enabled && !config_.guidance.calib_mode) {
        hit_file_.open(paths_.hit_path, std::ios::app);
        std::println("HIT-CALIB: saving confirmed purple hits to {}", paths_.hit_path.string());
    }

    if (config_.debug.show_window) {
        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        window_open_ = true;
    }
    return {};
}

auto GuidanceOpsApp::teardown() -> void {
    perception_.stop();
    if (guidance_) {
        guidance_->shutdown();
    }
    capture_.close();
    if (window_open_ && config_.debug.show_window) {
        cv::destroyWindow(kWindowName);
        window_open_ = false;
    }
    guidance_.reset();
    negotiated_format_.reset();
    calibration_file_.close();
    voltage_file_.close();
    hit_file_.close();
}

auto GuidanceOpsApp::run_loop() -> void {
    using Clock = std::chrono::steady_clock;
    constexpr auto kReadErrorDelay = std::chrono::milliseconds(100);

    while (!stop_requested_) {
        if (retry_policy_.reconnect_pending()) {
            const auto now = Clock::now();
            if (!retry_policy_.reconnect_due(now)) {
                std::this_thread::sleep_for(kReadErrorDelay);
                continue;
            }

            if (auto reconnect_result = capture_.reconnect(); reconnect_result) {
                std::println("guidance app camera reconnected");
                negotiated_format_ = capture_.negotiated_format();
                guidance_.reset();
                retry_policy_.arm_guidance_retry(Clock::now());
                retry_policy_.on_reconnect_succeeded();
            } else {
                std::println(stderr, "guidance app reconnect failed: {}", reconnect_result.error());
                retry_policy_.on_reconnect_failed(Clock::now());
            }
            continue;
        }

        if (config_.guidance.enabled && !guidance_ && negotiated_format_.has_value()) {
            const auto now = Clock::now();
            if (retry_policy_.guidance_retry_due(now)) {
                auto guidance = try_create_guidance_session(
                    config_, *negotiated_format_,
                    config_.guidance.calib_mode ? calibration_state_ : nullptr);
                if (guidance) {
                    guidance_ = std::move(*guidance);
                    retry_policy_.clear_guidance_retry();
                    std::println("guidance app guidance initialized");
                } else {
                    std::println(stderr, "guidance app retry failed: {}", guidance.error());
                    retry_policy_.defer_guidance_retry(now);
                }
            }
        }

        auto frame_result = capture_.read_frame();
        if (!frame_result) {
            std::println(stderr, "capture read failed: {}", frame_result.error());
            if (retry_policy_.on_read_error(Clock::now())) {
                if (guidance_) {
                    guidance_->shutdown();
                    guidance_.reset();
                }
            } else {
                std::this_thread::sleep_for(kReadErrorDelay);
            }
            continue;
        }

        retry_policy_.on_read_success();

        ControlLoopFrame frame;
        frame.frame = std::move(*frame_result);
        frame.display = frame.frame.image.clone();

        const auto perception_result = perception_.poll();
        frame.detection = perception_result.detection;
        frame.ekf_state = perception_result.ekf_state;
        frame.dropped_frames = perception_.overwrite_count();

        if (perception_.degraded()) {
            frame.detection = {};
            frame.ekf_state.reset();
        } else if (!perception_.submit(frame.frame)) {
            std::println(stderr, "perception submit failed: {}", perception_.last_error());
            break;
        }

        frame.track = select_target_track(
            frame.detection, frame.ekf_state, config_.ekf.enabled, config_.ekf.lookahead_ms);
        if (guidance_) {
            frame.guidance = guidance_->execute(frame.track);
        }

        overlay_.render(
            frame,
            OverlayRenderContext{
                .guidance_enabled = guidance_.has_value(),
                .guidance_ready = guidance_.has_value(),
                .calibration_mode = config_.guidance.calib_mode,
                .command_model = config_.guidance.command_model,
                .calibration_state =
                    config_.guidance.calib_mode ? &calibration_state() : nullptr,
                .streaming_active = false,
                .recording_active = false,
                .enemy_color = EnemyColor::auto_select,
                .using_tensorrt =
                    perception_.active_backend() == RuntimeBackend::tensorrt,
            });

        int key = -1;
        if (config_.debug.show_window) {
            cv::imshow(kWindowName, frame.display);
            key = cv::waitKey(1);
            const bool window_visible =
                cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) >= 1.0;
            if (should_exit_from_key(key) || !window_visible) {
                stop_requested_ = true;
            }
        }

        handle_key(key, frame);
        maybe_record_hit_edge(frame);
    }
}

auto GuidanceOpsApp::handle_key(const int key, const ControlLoopFrame& frame) -> void {
    if (!config_.guidance.calib_mode) {
        return;
    }

    if (auto message = apply_calibration_key(calibration_state(), config_.guidance, key);
        message.has_value()) {
        std::println("{}", *message);
    }

    if (key != ' ') {
        return;
    }

    maybe_record_calibration(frame);
    if (config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage
        && frame.detection.detected && !frame.detection.detections.empty()) {
        const auto& top = frame.detection.detections.front();
        const auto area = top.bbox.width * top.bbox.height;
        std::println(
            ">>> VOLTAGE RECORD: V=({:.3f}V,{:.3f}V) center=({:.1f},{:.1f}) area={:.1f}",
            calibration_state_->voltage_x, calibration_state_->voltage_y, top.center.x,
            top.center.y, area);
    } else if (frame.guidance.telemetry.selected_target_point.has_value()) {
        const auto& point = *frame.guidance.telemetry.selected_target_point;
        std::println(
            ">>> RECORD: θ=({:.2f}°,{:.2f}°) P_c=({:.1f},{:.1f},{:.1f})mm",
            calibration_state_->angle_x_deg, calibration_state_->angle_y_deg, point.x, point.y,
            point.z);
    }
}

auto GuidanceOpsApp::maybe_record_calibration(const ControlLoopFrame& frame) -> void {
    if (config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage) {
        if (frame.detection.detected && !frame.detection.detections.empty() && voltage_file_) {
            voltage_file_ << format_voltage_record(
                frame.frame.timestamp, frame.detection.detections.front(), calibration_state_->voltage_x,
                calibration_state_->voltage_y);
            voltage_file_.flush();
        }
        return;
    }

    if (frame.guidance.telemetry.selected_target_point.has_value() && calibration_file_) {
        calibration_file_ << format_geometry_record(
            calibration_state_->angle_x_deg, calibration_state_->angle_y_deg,
            *frame.guidance.telemetry.selected_target_point);
        calibration_file_.flush();
    }
}

auto GuidanceOpsApp::maybe_record_hit_edge(const ControlLoopFrame& frame) -> void {
    const auto* top = frame.detection.detections.empty() ? nullptr : &frame.detection.detections.front();
    const bool is_purple = frame.detection.detected && top != nullptr && top->class_id == 0
                        && top->score >= 0.25F;
    const auto hit_state = hit_state_machine_.update(is_purple);
    const auto previous = last_hit_state_;
    last_hit_state_ = hit_state;

    if (config_.guidance.calib_mode || top == nullptr
        || !should_record_hit_edge(previous, hit_state, *top, frame.detection.detected)
        || !frame.guidance.aim_output.output_angles.has_value()
        || !frame.guidance.telemetry.selected_target_point.has_value()) {
        return;
    }

    const auto& hit_angles = *frame.guidance.aim_output.output_angles;
    const auto& point = *frame.guidance.telemetry.selected_target_point;
    std::println(
        ">>> HIT-CALIB RECORD: θ=({:.2f}°,{:.2f}°) P_c=({:.1f},{:.1f},{:.1f})mm class=purple",
        hit_angles.x, hit_angles.y, point.x, point.y, point.z);
    if (hit_file_) {
        hit_file_ << format_geometry_record(hit_angles.x, hit_angles.y, point);
        hit_file_.flush();
    }
}

auto GuidanceOpsApp::calibration_state() -> GuidanceCalibrationState& {
    return *calibration_state_;
}

} // namespace rmcs_laser_guidance::runtime_internal
