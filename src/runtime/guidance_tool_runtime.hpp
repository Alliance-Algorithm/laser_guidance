#pragma once

#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "capture/v4l2_capture.hpp"
#include "config.hpp"
#include "guidance/guidance_pipeline.hpp"
#include "io/ft4222_spi.hpp"
#include "tracking/ekf_tracker.hpp"
#include "tracking/hit_state_machine.hpp"
#include "types.hpp"
#include "vision/model_infer.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct GuidanceCalibrationState {
    float angle_x_deg = 0.0F;
    float angle_y_deg = 0.0F;
    float angle_step_deg = 0.01F;
    float voltage_x = 0.0F;
    float voltage_y = 0.0F;
    float voltage_step_v = 0.005F;
    cv::Point3f last_projected_point{-1.0F, -1.0F, -1.0F};
    bool has_projected_point = false;

    static constexpr float kMinAngleStepDeg = 0.001F;
    static constexpr float kMaxAngleStepDeg = 0.5F;
    static constexpr float kMinVoltageStepV = 0.001F;
    static constexpr float kMaxVoltageStepV = 0.5F;
};

class GuidanceToolRuntime {
public:
    explicit GuidanceToolRuntime(Config config);
    ~GuidanceToolRuntime();

    GuidanceToolRuntime(const GuidanceToolRuntime&) = delete;
    auto operator=(const GuidanceToolRuntime&) -> GuidanceToolRuntime& = delete;

    auto run() -> int;

    static auto apply_calibration_key(GuidanceCalibrationState& state, const GuidanceConfig& config, int key)
        -> std::optional<std::string>;
    static auto format_voltage_record(
        Clock::time_point timestamp, const ModelCandidate& top, float manual_vx, float manual_vy)
        -> std::string;
    static auto format_geometry_record(float angle_x_deg, float angle_y_deg, const cv::Point3f& point)
        -> std::string;
    static auto format_hit_record(float angle_x_deg, float angle_y_deg, const cv::Point3f& point)
        -> std::string;
    static auto should_record_hit_edge(
        HitState previous_state, HitState current_state, const TargetObservation& observation) -> bool;

private:
    auto open_outputs() -> void;
    auto start_inference_thread() -> void;
    auto stop_inference_thread() -> void;
    auto apply_guidance(
        const TargetObservation& observation, const EkfState& ekf_state, std::string& guidance_msg,
        float& last_valid_depth_mm, bool& depth_valid, bool& ekf_was_lost) -> void;
    auto maybe_update_calibration_projection(
        const TargetObservation& observation, bool depth_valid, float last_valid_depth_mm) -> void;
    auto maybe_record_calibration(const TargetObservation& observation) -> void;
    auto maybe_record_hit_edge(
        const TargetObservation& observation, HitState hit_state, HitState& last_hit_state) -> void;
    auto draw_overlay(
        cv::Mat& display, const TargetObservation& observation, const EkfState& ekf_state,
        std::string_view guidance_msg, bool guidance_active, bool depth_valid) const -> void;
    auto handle_input(
        int key, const TargetObservation& observation, std::string& guidance_msg) -> void;
    auto shutdown_guidance() -> void;

    Config config_;
    V4l2Capture capture_;
    std::unique_ptr<ModelInfer> infer_;
    std::future<void> model_ready_{};
    std::thread infer_thread_{};
    std::unique_ptr<Ft4222Spi> spi_;
    std::unique_ptr<GuidancePipeline> guidance_;
    std::mutex infer_mutex_;
    std::condition_variable infer_cv_;
    cv::Mat pending_frame_{};
    bool has_pending_frame_ = false;
    bool running_ = true;
    TargetObservation latest_observation_{};
    EkfTracker tracker_;
    EkfState latest_ekf_state_{};
    GuidanceCalibrationState calibration_state_{};
    float last_valid_depth_mm_ = 0.0F;
    bool depth_valid_ = false;
    bool ekf_was_lost_ = false;
    HitStateMachine hit_state_machine_;
    HitState last_hit_state_ = HitState::None;
    std::ofstream calibration_file_{};
    std::ofstream voltage_file_{};
    std::ofstream hit_file_{};
};

} // namespace rmcs_laser_guidance::runtime_internal
