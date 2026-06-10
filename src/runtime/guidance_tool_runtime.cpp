#include "runtime/guidance_tool_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <format>
#include <print>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "core/debug_renderer.hpp"
#include "laser_guidance/support.hpp"

namespace rmcs_laser_guidance::runtime_internal {
namespace {

constexpr std::string_view kGeometryCalibPath = "test_data/calib/geometry_calib_records.csv";
constexpr std::string_view kGeometryHitPath = "test_data/calib/geometry_hit_calib_records.csv";
constexpr std::string_view kVoltageCalibPath = "test_data/calib/voltage_records.csv";
constexpr const char* kWindowName = "laser_guidance";

auto open_ft4222() -> std::expected<Ft4222Spi, std::string> {
    return Ft4222Spi::open(
        Ft4222Config{
            .sys_clock = Ft4222SysClock::k60MHz,
            .clock_div = Ft4222SpiDiv::kDiv2,
            .cpol = Ft4222Cpol::kIdleLow,
            .cpha = Ft4222Cpha::kTrailing,
            .cs_active = Ft4222CsActive::kLow,
            .cs_channel = 0,
        });
}

} // namespace

GuidanceToolRuntime::GuidanceToolRuntime(Config config)
    : config_(std::move(config))
    , capture_(create_capture_device(config_))
    , tracker_(config_.ekf)
    , calibration_state_{
          .angle_x_deg = config_.guidance.calib_angle_x_deg,
          .angle_y_deg = config_.guidance.calib_angle_y_deg,
      }
    , hit_state_machine_(
          config_.runtime.hit_confirm_frames, config_.runtime.hit_release_frames) {}

GuidanceToolRuntime::~GuidanceToolRuntime() {
    stop_inference_thread();
    shutdown_guidance();
    capture_->close();
    cv::destroyWindow(kWindowName);
}

auto GuidanceToolRuntime::run() -> int {
    const auto open_result = capture_->open();
    if (!open_result) {
        std::println(stderr, "Failed to open camera: {}", open_result.error());
        return 1;
    }
    std::println(
        "Camera: {} {}x{} @ {:.0f}fps", open_result->device_path, open_result->width,
        open_result->height, open_result->framerate);

    if (config_.inference.backend != InferenceBackendKind::bright_spot) {
        model_ready_ = std::async(std::launch::async, [this] {
            infer_ = std::make_unique<ModelInfer>(config_.inference);
        });
    }

    if (config_.guidance.enabled) {
        auto spi_result = open_ft4222();
        if (!spi_result) {
            std::println(stderr, "Failed to open FT4222: {}", spi_result.error());
            std::println(stderr, "Continuing without galvo control");
        } else {
            spi_ = std::make_unique<Ft4222Spi>(std::move(*spi_result));
            std::println("FT4222: opened, SCLK ~{} Hz", spi_->negotiated_clock_hz());
            guidance_ = std::make_unique<GuidancePipeline>(config_, *spi_);
            if (!guidance_->is_initialized()) {
                std::println(
                    stderr,
                    "Guidance pipeline failed to initialize; check camera_calib_path and FT4222");
            }
        }
    }

    open_outputs();
    start_inference_thread();

    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    auto loop_t0 = std::chrono::steady_clock::now();
    unsigned loop_frames = 0;

    while (running_) {
        auto frame = capture_->read_frame();
        if (!frame) {
            std::println(stderr, "Frame read error: {}", frame.error());
            continue;
        }

        cv::Mat display = frame->image.clone();

        TargetObservation observation;
        EkfState ekf_state;
        {
            std::scoped_lock lock(infer_mutex_);
            observation = latest_observation_;
            ekf_state = latest_ekf_state_;
            if (infer_) {
                pending_frame_ = std::move(frame->image);
                has_pending_frame_ = true;
            }
        }
        if (infer_) {
            infer_cv_.notify_one();
        }

        std::string guidance_msg;
        apply_guidance(observation, ekf_state, guidance_msg, last_valid_depth_mm_, depth_valid_, ekf_was_lost_);
        maybe_record_hit_edge(observation, hit_state_machine_.update(
            observation.detected && !observation.candidates.empty()
            && observation.candidates.front().class_id == 0
            && observation.candidates.front().score >= 0.25F), last_hit_state_);

        const bool guidance_active = guidance_ && guidance_->is_initialized();
        draw_overlay(display, observation, ekf_state, guidance_msg, guidance_active, depth_valid_);

        cv::imshow(kWindowName, display);
        const int key = cv::waitKey(1);
        handle_input(key, observation, guidance_msg);

        if (should_exit_from_key(key)
            || cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) < 1) {
            running_ = false;
        }

        if (++loop_frames % 30 == 0) {
            const auto elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_t0).count();
            std::println(
                stderr, "[main] {} frames {:.1f}s ({:.0f}fps)", loop_frames, elapsed,
                loop_frames / elapsed);
            loop_t0 = std::chrono::steady_clock::now();
            loop_frames = 0;
        }
    }

    return 0;
}

auto GuidanceToolRuntime::apply_calibration_key(
    GuidanceCalibrationState& state, const GuidanceConfig& config, const int key)
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

auto GuidanceToolRuntime::format_voltage_record(
    const Clock::time_point timestamp, const ModelCandidate& top, const float manual_vx,
    const float manual_vy) -> std::string {
    const float area = top.bbox.width * top.bbox.height;
    return std::format(
        "{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.5f},{},{:.5f},{:.5f}\n",
        std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count(),
        top.center.x, top.center.y, top.bbox.x, top.bbox.y, top.bbox.width, top.bbox.height, area,
        top.score, top.class_id, manual_vx, manual_vy);
}

auto GuidanceToolRuntime::format_geometry_record(
    const float angle_x_deg, const float angle_y_deg, const cv::Point3f& point) -> std::string {
    return std::format(
        "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n", angle_x_deg, angle_y_deg, point.x, point.y, point.z);
}

auto GuidanceToolRuntime::format_hit_record(
    const float angle_x_deg, const float angle_y_deg, const cv::Point3f& point) -> std::string {
    return format_geometry_record(angle_x_deg, angle_y_deg, point);
}

auto GuidanceToolRuntime::should_record_hit_edge(
    const HitState previous_state, const HitState current_state, const TargetObservation& observation)
    -> bool {
    return current_state == HitState::Confirmed && previous_state != HitState::Confirmed
        && observation.detected && !observation.candidates.empty()
        && observation.candidates.front().class_id == 0
        && observation.candidates.front().score >= 0.25F;
}

auto GuidanceToolRuntime::open_outputs() -> void {
    if (config_.guidance.calib_mode
        && config_.guidance.command_model == GuidanceCommandModelKind::geometry) {
        calibration_file_.open(kGeometryCalibPath.data(), std::ios::app);
        std::println("CALIB: saving records to {}", kGeometryCalibPath);
    }

    if (config_.guidance.calib_mode
        && config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage) {
        voltage_file_.open(kVoltageCalibPath.data(), std::ios::app);
        std::println("CALIB: saving records to {}", kVoltageCalibPath);
        if (voltage_file_.tellp() == 0) {
            voltage_file_
                << "timestamp_ns,center_x,center_y,bbox_x,bbox_y,bbox_w,bbox_h,bbox_area,score,"
                   "class_id,manual_vx,manual_vy\n";
        }
    }

    if (config_.guidance.enabled && !config_.guidance.calib_mode) {
        hit_file_.open(kGeometryHitPath.data(), std::ios::app);
        std::println("HIT-CALIB: saving confirmed purple hits to {}", kGeometryHitPath);
    }
}

auto GuidanceToolRuntime::start_inference_thread() -> void {
    if (config_.inference.backend == InferenceBackendKind::bright_spot) {
        return;
    }

    infer_thread_ = std::thread([this] {
        model_ready_.wait();
        try {
            while (true) {
                cv::Mat frame_to_process;
                {
                    std::unique_lock lock(infer_mutex_);
                    infer_cv_.wait(lock, [this] { return has_pending_frame_ || !running_; });
                    if (!running_) {
                        break;
                    }
                    frame_to_process = std::move(pending_frame_);
                    has_pending_frame_ = false;
                }
                const Frame infer_frame{
                    .image = frame_to_process,
                    .timestamp = Clock::now(),
                };
                const auto result = infer_->infer(infer_frame);
                if (result.observation.detected) {
                    tracker_.process(result.observation.center, infer_frame.timestamp);
                } else {
                    tracker_.predict(infer_frame.timestamp);
                }
                std::scoped_lock lock(infer_mutex_);
                latest_observation_ = result.observation;
                latest_observation_.candidates = result.candidates;
                latest_ekf_state_ = tracker_.state();
            }
        } catch (const std::exception& e) {
            std::println(stderr, "[infer] error: {}", e.what());
        }
    });
}

auto GuidanceToolRuntime::stop_inference_thread() -> void {
    {
        std::scoped_lock lock(infer_mutex_);
        running_ = false;
    }
    infer_cv_.notify_all();
    if (infer_thread_.joinable()) {
        infer_thread_.join();
    }
}

auto GuidanceToolRuntime::apply_guidance(
    const TargetObservation& observation, const EkfState& ekf_state, std::string& guidance_msg,
    float& last_valid_depth_mm, bool& depth_valid, bool& ekf_was_lost) -> void {
    if (!guidance_ || !guidance_->is_initialized()) {
        return;
    }

    if (config_.guidance.calib_mode) {
        if (config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage) {
            guidance_msg = guidance_->process_calib_voltage(
                calibration_state_.voltage_x, calibration_state_.voltage_y);
        } else {
            guidance_msg = guidance_->process_calib_angle(
                calibration_state_.angle_x_deg, calibration_state_.angle_y_deg);
        }
        maybe_update_calibration_projection(observation, depth_valid, last_valid_depth_mm);
        return;
    }

    if (ekf_state.initialized && !ekf_state.lost) {
        const ModelCandidate* candidate =
            observation.detected && !observation.candidates.empty() ? &observation.candidates.front()
                                                                    : nullptr;
        if (candidate != nullptr) {
            depth_valid = true;
        }
        if (depth_valid) {
            const float latency_s = static_cast<float>(config_.ekf.lookahead_ms) * 0.001F;
            const cv::Point2f aim_pos(
                ekf_state.position.x + ekf_state.velocity.x * latency_s,
                ekf_state.position.y + ekf_state.velocity.y * latency_s);
            guidance_msg =
                guidance_->process_ekf_guided(aim_pos, candidate, nullptr, last_valid_depth_mm);
        }
    } else if (ekf_state.lost && !ekf_was_lost) {
        guidance_msg = guidance_->set_center();
        depth_valid = false;
        last_valid_depth_mm = 0.0F;
    }

    ekf_was_lost = ekf_state.lost;
}

auto GuidanceToolRuntime::maybe_update_calibration_projection(
    const TargetObservation& observation, const bool depth_valid, const float last_valid_depth_mm) -> void {
    if (!(observation.detected && !observation.candidates.empty())
        || config_.guidance.command_model != GuidanceCommandModelKind::geometry) {
        return;
    }
    const auto& top = observation.candidates.front();
    if (top.bbox.width <= 0.0F) {
        return;
    }

    float depth_mm = last_valid_depth_mm;
    bool has_depth = depth_valid;
    if (const auto depth = guidance_->estimate_depth(top)) {
        depth_mm = *depth;
        has_depth = true;
        last_valid_depth_mm_ = *depth;
        depth_valid_ = true;
    }

    if (has_depth && depth_mm > 0.0F) {
        calibration_state_.last_projected_point = guidance_->project_to_camera(top.center, depth_mm);
        calibration_state_.has_projected_point = true;
    }
}

auto GuidanceToolRuntime::maybe_record_calibration(const TargetObservation& observation) -> void {
    if (!config_.guidance.calib_mode) {
        return;
    }
    if (config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage) {
        if (observation.detected && !observation.candidates.empty() && voltage_file_) {
            voltage_file_ << format_voltage_record(
                Clock::now(), observation.candidates.front(), calibration_state_.voltage_x,
                calibration_state_.voltage_y);
            voltage_file_.flush();
        }
    } else if (calibration_state_.has_projected_point && calibration_file_) {
        calibration_file_ << format_geometry_record(
            calibration_state_.angle_x_deg, calibration_state_.angle_y_deg,
            calibration_state_.last_projected_point);
        calibration_file_.flush();
    }
}

auto GuidanceToolRuntime::maybe_record_hit_edge(
    const TargetObservation& observation, const HitState hit_state, HitState& last_hit_state) -> void {
    const auto previous = last_hit_state;
    last_hit_state = hit_state;
    if (!should_record_hit_edge(previous, hit_state, observation)
        || !guidance_ || !guidance_->is_initialized() || config_.guidance.calib_mode) {
        return;
    }

    const auto* top_candidate = observation.candidates.empty() ? nullptr : &observation.candidates.front();
    if (top_candidate == nullptr) {
        return;
    }
    const auto hit_depth = guidance_->estimate_depth(*top_candidate);
    const auto hit_angles = guidance_->latest_output_angles();
    if (!hit_depth || !hit_angles) {
        return;
    }
    const auto point = guidance_->project_to_camera(top_candidate->center, *hit_depth);
    std::println(
        ">>> HIT-CALIB RECORD: θ=({:.2f}°,{:.2f}°) P_c=({:.1f},{:.1f},{:.1f})mm class=purple",
        hit_angles->x, hit_angles->y, point.x, point.y, point.z);
    if (hit_file_) {
        hit_file_ << format_hit_record(hit_angles->x, hit_angles->y, point);
        hit_file_.flush();
    }
}

auto GuidanceToolRuntime::draw_overlay(
    cv::Mat& display, const TargetObservation& observation, const EkfState& ekf_state,
    const std::string_view guidance_msg, const bool guidance_active, const bool depth_valid) const
    -> void {
    if (observation.detected || !observation.candidates.empty()) {
        draw_candidates(display, observation.candidates);
    }
    draw_ekf_state(display, ekf_state);

    const bool calib = guidance_active && config_.guidance.calib_mode;
    const bool ekf_ok = ekf_state.initialized && !ekf_state.lost;
    if (calib) {
        const auto label =
            config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage
                ? std::format(
                      "CALIB voltage=({:.3f}V,{:.3f}V) step={:.3f}V [WASD move, </> step]",
                      calibration_state_.voltage_x, calibration_state_.voltage_y,
                      calibration_state_.voltage_step_v)
                : std::format(
                      "CALIB galvo=({:.3f}°,{:.3f}°) step={:.3f}° [WASD move, </> step]",
                      calibration_state_.angle_x_deg, calibration_state_.angle_y_deg,
                      calibration_state_.angle_step_deg);
        cv::putText(
            display, label, {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
    } else {
        draw_guidance_status(display, guidance_active, ekf_ok, depth_valid, std::string(guidance_msg));
    }
}

auto GuidanceToolRuntime::handle_input(
    const int key, const TargetObservation& observation, std::string& guidance_msg) -> void {
    if (!config_.guidance.calib_mode) {
        return;
    }

    if (auto message = apply_calibration_key(calibration_state_, config_.guidance, key);
        message.has_value()) {
        std::println("{}", *message);
    }

    if (key == ' ') {
        maybe_record_calibration(observation);
        if (config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage
            && observation.detected && !observation.candidates.empty()) {
            const auto area = observation.candidates.front().bbox.width
                            * observation.candidates.front().bbox.height;
            std::println(
                ">>> VOLTAGE RECORD: V=({:.3f}V,{:.3f}V) center=({:.1f},{:.1f}) area={:.1f}",
                calibration_state_.voltage_x, calibration_state_.voltage_y,
                observation.candidates.front().center.x, observation.candidates.front().center.y, area);
        } else if (calibration_state_.has_projected_point) {
            std::println(
                ">>> RECORD: θ=({:.2f}°,{:.2f}°) P_c=({:.1f},{:.1f},{:.1f})mm",
                calibration_state_.angle_x_deg, calibration_state_.angle_y_deg,
                calibration_state_.last_projected_point.x, calibration_state_.last_projected_point.y,
                calibration_state_.last_projected_point.z);
        }
    }

    if (guidance_ && guidance_->is_initialized()) {
        if (config_.guidance.command_model == GuidanceCommandModelKind::direct_voltage) {
            guidance_msg = guidance_->process_calib_voltage(
                calibration_state_.voltage_x, calibration_state_.voltage_y);
        } else {
            guidance_msg = guidance_->process_calib_angle(
                calibration_state_.angle_x_deg, calibration_state_.angle_y_deg);
        }
    }
}

auto GuidanceToolRuntime::shutdown_guidance() -> void {
    if (guidance_) {
        if (const auto error = guidance_->set_center(); !error.empty()) {
            std::println("guidance shutdown: {}", error);
        }
    }
}

} // namespace rmcs_laser_guidance::runtime_internal
