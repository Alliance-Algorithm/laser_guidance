#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <opencv2/core/mat.hpp>

#include "capture/capture_device.hpp"
#include "config.hpp"
#include "laser_guidance/bridges.hpp"
#include "laser_guidance/runtime.hpp"
#include "laser_guidance/support.hpp"
#include "vision/training_data.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct RuntimeOutputsStatus {
    bool streaming_active = false;
    bool recording_active = false;
    std::filesystem::path recording_root{};
};

struct RuntimeOutputCapabilities {
    bool allow_rtp = false;
    bool allow_shm = false;
    bool allow_telemetry = false;
    bool allow_recording = false;
};

class RuntimeOutputs {
public:
    RuntimeOutputs(
        Config config, RecordSessionOptions record_options, RuntimeOutputCapabilities capabilities);

    auto start(const CaptureFormat& format, bool streaming_requested, bool recording_requested) -> void;
    auto apply_requests(
        bool streaming_requested, bool recording_requested,
        const std::optional<CaptureFormat>& negotiated_format) -> void;
    auto publish_previous(cv::Mat& previous_output) -> void;
    auto record_current(const cv::Mat& frame) -> void;
    auto publish_snapshot(const RuntimeSnapshot& snapshot) -> void;
    auto stop() -> void;

    [[nodiscard]] auto status() const -> RuntimeOutputsStatus;
    [[nodiscard]] auto capabilities() const -> const RuntimeOutputCapabilities& {
        return capabilities_;
    }

private:
    auto start_streaming(const CaptureFormat& format) -> bool;
    auto stop_streaming() -> void;
    auto start_shm(const CaptureFormat& format) -> void;
    auto stop_shm() -> void;
    auto begin_recording(const CaptureFormat& format) -> void;
    auto stop_recording() -> void;

    Config config_{};
    RecordSessionOptions record_options_{};
    RuntimeOutputCapabilities capabilities_{};
    UdpTelemetryPublisher telemetry_;
    ShmFramePublisher shm_publisher_;
    RtpFramePublisher rtp_publisher_;
    std::unique_ptr<VideoSessionRecorder> recorder_{};
    std::chrono::steady_clock::time_point recording_start_{};
    bool shm_active_ = false;
};

} // namespace rmcs_laser_guidance::runtime_internal
