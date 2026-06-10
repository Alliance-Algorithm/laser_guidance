#pragma once

#include <chrono>
#include <expected>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>

#include "capture/capture_device.hpp"
#include "capture/capture_device_factory.hpp"
#include "config.hpp"
#include "guidance/aim_solver.hpp"
#include "guidance/galvo_executor.hpp"
#include "guidance/scan_controller.hpp"
#include "io/ft4222_spi.hpp"
#include "laser_guidance/bridges.hpp"
#include "laser_guidance/runtime.hpp"
#include "runtime/inference_facade.hpp"
#include "runtime/runtime_support.hpp"
#include "tracking/ekf_tracker.hpp"
#include "tracking/freshness_queue.hpp"
#include "tracking/hit_progress.hpp"
#include "tracking/runtime_metrics.hpp"
#include "vision/training_data.hpp"

namespace rmcs_laser_guidance::runtime_internal {

constexpr const char* kCompetitionWindowName = "laser_guidance_competition";
constexpr const char* kPreviewWindowName = "laser_guidance_preview";

struct RuntimeState {
    mutable std::mutex mutex;
    RuntimeSnapshot snapshot{};
    bool running = false;
    bool stop_requested = false;
    EnemyColor enemy_color = EnemyColor::auto_select;
    bool ekf_enabled = false;
    bool streaming_requested = false;
    bool recording_requested = false;
    std::string last_error{};
};

struct RuntimeResults {
    DetectionBatch detection{};
    std::optional<EkfState> ekf_state{};
};

class RuntimeOutputController {
public:
    RuntimeOutputController(
        RtpFramePublisher& rtp_publisher, ShmFramePublisher& shm_publisher,
        std::unique_ptr<VideoSessionRecorder>& recorder, std::chrono::steady_clock::time_point& recording_start);

    auto start_streaming(const CaptureFormat& format) -> bool;
    auto apply_requests(
        bool streaming_requested, bool recording_requested, const RecordSessionOptions& record_options,
        const std::optional<CaptureFormat>& negotiated_format) -> void;
    auto publish(cv::Mat& previous_frame) -> void;
    auto record_frame(const cv::Mat& frame) -> void;
    auto stop() -> void;
    [[nodiscard]] auto streaming_active() const -> bool;
    [[nodiscard]] auto recording_active() const -> bool;
    [[nodiscard]] auto recording_root() const -> std::filesystem::path;

private:
    auto begin_recording(const RecordSessionOptions& record_options, const CaptureFormat& format)
        -> void;
    auto flush_recording() -> void;

    RtpFramePublisher& rtp_publisher_;
    ShmFramePublisher& shm_publisher_;
    std::unique_ptr<VideoSessionRecorder>& recorder_;
    std::chrono::steady_clock::time_point& recording_start_;
};

class RuntimeBase {
public:
    RuntimeBase(
        Config config, std::string window_name, bool enable_guidance,
        RecordSessionOptions record_options = {});
    virtual ~RuntimeBase();

    RuntimeBase(const RuntimeBase&) = delete;
    auto operator=(const RuntimeBase&) -> RuntimeBase& = delete;

    auto start() -> std::expected<void, std::string>;
    auto stop() -> void;
    auto join() -> void;
    auto submit_command(const RuntimeCommand& command) -> std::expected<void, std::string>;
    [[nodiscard]] auto snapshot() const -> RuntimeSnapshot;

protected:
    virtual auto after_frame_processed(cv::Mat& frame, const RuntimeSnapshot& snapshot) -> void = 0;

private:
    struct GuidanceStepResult {
        AimOutput aim_output{};
        float last_valid_depth_mm = 0.0F;
        bool ekf_was_lost = false;
    };

    auto initialize_guidance() -> void;
    auto make_snapshot_locked(
        const DetectionBatch& batch, const std::optional<TargetTrack>& track, const AimOutput& aim,
        const HitProgress& hit_progress, std::size_t dropped_frames,
        const std::filesystem::path& recording_root) -> RuntimeSnapshot;
    auto update_status_locked() -> void;
    auto read_results() const -> RuntimeResults;
    auto select_track(const DetectionBatch& detection, const std::optional<EkfState>& ekf_state) const
        -> TargetTrack;
    auto draw_results_overlay(
        cv::Mat& display, const DetectionBatch& detection, const std::optional<EkfState>& ekf_state) const
        -> void;
    auto queue_frame_for_inference(Frame& frame) -> bool;
    auto process_guidance_step(
        const DetectionBatch& detection, const TargetTrack& track, float last_valid_depth_mm,
        bool ekf_was_lost) -> GuidanceStepResult;
    auto update_hit_state(const DetectionBatch& detection) -> void;
    auto draw_runtime_overlay(cv::Mat& display, const TargetTrack& track, const AimOutput& aim_output)
        -> void;
    auto apply_output_requests() -> void;
    auto publish_snapshot(
        cv::Mat& display, const DetectionBatch& detection, const TargetTrack& track,
        const AimOutput& aim_output) -> void;
    auto run_inference_worker() -> void;
    auto run() -> void;

    Config config_;
    std::string window_name_;
    bool enable_guidance_ = false;
    RecordSessionOptions record_options_{};
    std::unique_ptr<ICaptureDevice> capture_;
    UdpTelemetryPublisher telemetry_;
    ShmFramePublisher shm_publisher_;
    RtpFramePublisher rtp_publisher_;
    InferenceFacade inference_;
    EkfTracker tracker_;
    HitProgress hit_progress_{};
    StaleFramePolicy stale_policy_{};
    RuntimeOutputController output_controller_;
    RuntimeState state_{};
    mutable std::mutex result_mutex_;
    DetectionBatch latest_detection_{};
    std::optional<EkfState> latest_ekf_{};
    std::optional<CaptureFormat> negotiated_format_{};
    LatestValue<QueuedFrame> frame_queue_{};
    std::unique_ptr<Ft4222Spi> spi_;
    std::unique_ptr<AimSolver> solver_;
    std::unique_ptr<GalvoExecutor> executor_;
    std::unique_ptr<ScanController> scanner_;
    std::unique_ptr<VideoSessionRecorder> recorder_;
    std::chrono::steady_clock::time_point recording_start_{};
    std::thread main_thread_;
    std::thread inference_thread_;
};

class CompetitionRuntimeImpl final : public RuntimeBase {
public:
    explicit CompetitionRuntimeImpl(Config config, RecordSessionOptions record_options);

private:
    auto after_frame_processed(cv::Mat& frame, const RuntimeSnapshot& snapshot) -> void override;
};

class PreviewRuntimeImpl final : public RuntimeBase {
public:
    explicit PreviewRuntimeImpl(Config config);

private:
    auto after_frame_processed(cv::Mat& frame, const RuntimeSnapshot& snapshot) -> void override;
};

} // namespace rmcs_laser_guidance::runtime_internal
