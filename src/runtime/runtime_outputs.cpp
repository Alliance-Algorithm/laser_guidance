#include "runtime/runtime_outputs.hpp"

namespace rmcs_laser_guidance::runtime_internal {

RuntimeOutputs::RuntimeOutputs(
    Config config, RecordSessionOptions record_options, RuntimeOutputCapabilities capabilities)
    : config_(std::move(config))
    , record_options_(std::move(record_options))
    , capabilities_(capabilities)
    , telemetry_(config_.udp)
    , zmq_telemetry_(config_.zmq)
    , shm_publisher_()
    , rtp_publisher_(config_.rtp) {}

auto RuntimeOutputs::start(
    const CaptureFormat& format, const bool streaming_requested, const bool recording_requested)
    -> void {
    start_shm(format);
    apply_requests(streaming_requested, recording_requested, format);
}

auto RuntimeOutputs::apply_requests(
    const bool streaming_requested, const bool recording_requested,
    const std::optional<CaptureFormat>& negotiated_format) -> void {
    if (capabilities_.allow_rtp && streaming_requested && negotiated_format.has_value()) {
        if (!status().streaming_active) {
            (void)start_streaming(*negotiated_format);
        }
    } else {
        stop_streaming();
    }

    if (capabilities_.allow_recording && recording_requested && negotiated_format.has_value()) {
        if (!status().recording_active) {
            begin_recording(*negotiated_format);
        }
    } else {
        stop_recording();
    }
}

auto RuntimeOutputs::publish_previous(cv::Mat& previous_output) -> void {
    if (previous_output.empty()) {
        return;
    }

    if (capabilities_.allow_shm && shm_active_) {
        shm_publisher_.publish(previous_output);
    }

    if (capabilities_.allow_rtp && rtp_publisher_.is_active()) {
        rtp_publisher_.publish(std::move(previous_output));
        // cv::Mat move leaves source empty — no explicit reset needed
    }
}

auto RuntimeOutputs::record_current(const cv::Mat& frame) -> void {
    if (recorder_) {
        recorder_->record_frame(frame);
    }
}

auto RuntimeOutputs::publish_snapshot(const RuntimeSnapshot& snapshot) -> void {
    if (capabilities_.allow_telemetry) {
        telemetry_.publish(snapshot);
        zmq_telemetry_.publish(snapshot);
    }
}

auto RuntimeOutputs::stop() -> void {
    stop_recording();
    stop_streaming();
    stop_shm();
}

auto RuntimeOutputs::status() const -> RuntimeOutputsStatus {
    return RuntimeOutputsStatus{
        .streaming_active = rtp_publisher_.is_active(),
        .recording_active = static_cast<bool>(recorder_),
        .recording_root = recorder_ ? recorder_->session_root() : std::filesystem::path{},
    };
}

auto RuntimeOutputs::start_streaming(const CaptureFormat& format) -> bool {
    if (!capabilities_.allow_rtp) {
        return false;
    }
    return rtp_publisher_.start(format.width, format.height, static_cast<float>(format.framerate));
}

auto RuntimeOutputs::stop_streaming() -> void {
    if (!capabilities_.allow_rtp) {
        return;
    }
    rtp_publisher_.stop();
}

auto RuntimeOutputs::start_shm(const CaptureFormat& format) -> void {
    if (!capabilities_.allow_shm) {
        return;
    }
    shm_active_ = shm_publisher_.start(format.width, format.height);
}

auto RuntimeOutputs::stop_shm() -> void {
    if (!capabilities_.allow_shm) {
        return;
    }
    shm_publisher_.stop();
    shm_active_ = false;
}

auto RuntimeOutputs::begin_recording(const CaptureFormat& format) -> void {
    if (!capabilities_.allow_recording || record_options_.output_root.empty() || recorder_) {
        return;
    }

    const auto capture_start = std::chrono::system_clock::now();
    const auto session_id = format_session_id(capture_start);
    recorder_ = std::make_unique<VideoSessionRecorder>(
        record_options_.output_root,
        VideoSessionMetadata{
            .session_id = session_id,
            .relative_video_path = "raw.mp4",
            .device_path = format.device_path,
            .width = format.width,
            .height = format.height,
            .framerate = format.framerate > 0.0 ? format.framerate : 60.0,
            .fourcc = format.pixel_encoding,
            .capture_start_unix_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(capture_start.time_since_epoch())
                    .count(),
            .duration_ms = 0,
            .lighting_tag = record_options_.lighting_tag,
            .background_tag = record_options_.background_tag,
            .distance_tag = record_options_.distance_tag,
            .target_color = record_options_.target_color,
            .operator_note_present = false,
        });
    recording_start_ = std::chrono::steady_clock::now();
}

auto RuntimeOutputs::stop_recording() -> void {
    if (!recorder_) {
        return;
    }
    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - recording_start_)
                                 .count();
    recorder_->flush(duration_ms);
    recorder_.reset();
}

} // namespace rmcs_laser_guidance::runtime_internal
