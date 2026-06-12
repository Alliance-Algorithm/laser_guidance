#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <print>

#include <opencv2/highgui.hpp>

#include "capture/capture_device.hpp"
#include "config.hpp"
#include "laser_guidance/support.hpp"
#include "vision/training_data.hpp"

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
    return rmcs_laser_guidance::default_config_path();
}

auto resolve_output_root(int argc, char** argv) -> std::filesystem::path {
    if (argc > 2)
        return argv[2];
    return {};
}

auto resolve_duration_seconds(int argc, char** argv) -> double {
    if (argc > 3)
        return std::stod(argv[3]);
    return 0.0;
}

auto resolve_lighting_tag(int argc, char** argv) -> std::string {
    if (argc > 4)
        return argv[4];
    return {};
}

auto resolve_background_tag(int argc, char** argv) -> std::string {
    if (argc > 5)
        return argv[5];
    return {};
}

auto resolve_distance_tag(int argc, char** argv) -> std::string {
    if (argc > 6)
        return argv[6];
    return {};
}

auto resolve_target_color(int argc, char** argv) -> std::string {
    if (argc > 7)
        return argv[7];
    return {};
}

auto print_mode(
    const rmcs_laser_guidance::Config& requested,
    const rmcs_laser_guidance::CaptureFormat& actual) -> void {
    if (requested.capture_backend == rmcs_laser_guidance::CaptureBackendKind::v4l2) {
        std::println(
            "requested device={} mode={}x{}@{} format={}", requested.v4l2.device_path.string(),
            requested.v4l2.width, requested.v4l2.height, requested.v4l2.framerate,
            rmcs_laser_guidance::pixel_format_name(requested.v4l2.pixel_format));
    } else {
        std::println(
            "requested device={} mode={}x{}@{} backend=hikcamera", requested.hik.device_id,
            requested.v4l2.width, requested.v4l2.height, requested.hik.framerate);
    }
    std::println(
        "actual    device={} mode={}x{}@{} format={}", actual.device_path, actual.width,
        actual.height, actual.framerate, actual.pixel_encoding);
}

auto unix_time_milliseconds(const std::chrono::system_clock::time_point value) -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(value.time_since_epoch()).count();
}

auto handle_stop_signal(int) -> void { g_stop_requested = 1; }

auto install_signal_handlers() -> void {
    (void)std::signal(SIGINT, handle_stop_signal);
    (void)std::signal(SIGTERM, handle_stop_signal);
}

auto stop_requested() -> bool { return g_stop_requested != 0; }

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path = resolve_config_path(argc, argv);
        auto record_options =
            rmcs_laser_guidance::load_record_session_options(config_path);
        if (argc > 2)
            record_options.output_root = resolve_output_root(argc, argv);
        if (argc > 3)
            record_options.duration_seconds = resolve_duration_seconds(argc, argv);
        if (argc > 4)
            record_options.lighting_tag = resolve_lighting_tag(argc, argv);
        if (argc > 5)
            record_options.background_tag = resolve_background_tag(argc, argv);
        if (argc > 6)
            record_options.distance_tag = resolve_distance_tag(argc, argv);
        if (argc > 7)
            record_options.target_color = resolve_target_color(argc, argv);

        if (record_options.output_root.empty()) {
            std::println(stderr, "output_root must not be empty");
            return 1;
        }
        if (record_options.duration_seconds <= 0.0) {
            std::println(stderr, "duration_seconds must be positive");
            return 1;
        }

        g_stop_requested = 0;
        install_signal_handlers();

        const auto config = rmcs_laser_guidance::load_config(config_path);
        auto record_config = config;
        record_config.v4l2.pixel_format = rmcs_laser_guidance::V4l2PixelFormat::yuyv;
        std::println(
            "record_session override: forcing v4l2.pixel_format={} for capture",
            rmcs_laser_guidance::pixel_format_name(record_config.v4l2.pixel_format));

        rmcs_laser_guidance::CaptureDevice capture(record_config);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open capture: {}", open_result.error());
            return 1;
        }

        print_mode(record_config, *open_result);

        const auto capture_start = std::chrono::system_clock::now();
        const std::string session_id = rmcs_laser_guidance::format_session_id(capture_start);
        const double fps =
            open_result->framerate > 0.0 ? open_result->framerate : config.v4l2.framerate;
        rmcs_laser_guidance::VideoSessionRecorder recorder(
            record_options.output_root,
            rmcs_laser_guidance::VideoSessionMetadata{
                .session_id = session_id,
                .relative_video_path = "raw.mp4",
                .device_path = open_result->device_path,
                .width = open_result->width,
                .height = open_result->height,
                .framerate = fps,
                .fourcc = open_result->pixel_encoding,
                .capture_start_unix_ms = unix_time_milliseconds(capture_start),
                .duration_ms = 0,
                .lighting_tag = record_options.lighting_tag,
                .background_tag = record_options.background_tag,
                .distance_tag = record_options.distance_tag,
                .target_color = record_options.target_color,
                .operator_note_present = false,
            });

        const auto monotonic_start = std::chrono::steady_clock::now();
        const auto deadline =
            monotonic_start + std::chrono::duration<double>(record_options.duration_seconds);
        if (!config.debug.show_window) {
            std::println(
                "recording without preview window; wait {:.1f}s or press Ctrl+C to "
                "finalize",
                record_options.duration_seconds);
        }

        while (!stop_requested() && std::chrono::steady_clock::now() < deadline) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Failed to read frame: {}", frame.error());
                continue;
            }

            recorder.record_frame(frame->image);

            if (config.debug.show_window) {
                cv::imshow("rmcs_laser_guidance_record_session", frame->image);
                if (rmcs_laser_guidance::should_exit_from_key(cv::waitKey(1)))
                    break;
            }
        }

        capture.close();
        if (stop_requested())
            std::println("stop requested, finalizing recorded session");

        const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - monotonic_start)
                                     .count();
        recorder.flush(duration_ms);

        std::println(
            "session_id={} recorded_frames={} session_root={}", session_id,
            recorder.recorded_frames(), recorder.session_root().string());
        return recorder.recorded_frames() > 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::println(stderr, "example_v4l2_record_session failed: {}", e.what());
        return 1;
    }
}
