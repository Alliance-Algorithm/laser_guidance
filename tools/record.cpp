#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <print>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "laser_guidance/error.hpp"

#include "capture/capture_device.hpp"
#include "config.hpp"
#include "laser_guidance/support.hpp"
#include "vision/training_data.hpp"

namespace {
volatile std::sig_atomic_t g_stop_requested = 0;
FILE* g_ffplay_pipe = nullptr;

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

auto launch_ffplay_viewer(int width, int height, double framerate, const std::string& pixel_fmt) -> FILE* {
    std::string cmd = std::format(
        "ffplay -hide_banner -loglevel warning -nostats -autoexit "
        "-f rawvideo -pixel_format {} -video_size {}x{} "
        "-framerate {:.3f} -i pipe:0",
        pixel_fmt, width, height, framerate);
    FILE* pipe = popen(cmd.c_str(), "w");
    if (!pipe)
        std::println(stderr, "warning: ffplay not found, recording without preview");
    return pipe;
}

auto close_ffplay_viewer(FILE* pipe) -> void {
    if (pipe) pclose(pipe);
}

auto record_preview_enabled(const rmcs_laser_guidance::Config& config) -> bool {
    const char* env = std::getenv("LASER_RECORD_PREVIEW");
    if (env && (std::string_view(env) == "0" || std::string_view(env) == "false"))
        return false;
    return config.debug.show_window;
}

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
            std::println(stderr, "Failed to open capture: {}", format_error(open_result.error()));
            return 1;
        }

        print_mode(record_config, *open_result);

        const auto capture_start = std::chrono::system_clock::now();
        const std::string session_id = rmcs_laser_guidance::format_session_id(capture_start);
        const double fps =
            open_result->framerate > 0.0 ? open_result->framerate : config.v4l2.framerate;
        const auto session_root = record_options.output_root / session_id;
        std::filesystem::create_directories(session_root);

        const auto monotonic_start = std::chrono::steady_clock::now();
        const auto deadline =
            monotonic_start + std::chrono::duration<double>(record_options.duration_seconds);
        const bool preview_enabled = record_preview_enabled(config);
        if (!preview_enabled) {
            const char* preview_env = std::getenv("LASER_RECORD_PREVIEW");
            const bool disabled_by_env =
                preview_env != nullptr
                && (std::string_view(preview_env) == "0"
                    || std::string_view(preview_env) == "false");
            if (disabled_by_env) {
                std::println(
                    "recording without preview window (LASER_RECORD_PREVIEW={}); wait {:.1f}s "
                    "or Ctrl+C to finalize",
                    preview_env, record_options.duration_seconds);
            } else {
                std::println(
                    "recording without preview window (debug.show_window=false); wait {:.1f}s "
                    "or Ctrl+C to finalize",
                    record_options.duration_seconds);
            }
        }

        const auto pixel_fmt = (open_result->pixel_encoding == "BGR8" ||
                                open_result->pixel_encoding.find("BGR") != std::string::npos)
                                   ? "bgr24"
                                   : "rgb24";
        g_ffplay_pipe = preview_enabled
                            ? launch_ffplay_viewer(open_result->width, open_result->height, fps, pixel_fmt)
                            : nullptr;

        const bool use_h264 = record_options.frame_format == "h264";
        std::unique_ptr<rmcs_laser_guidance::VideoSessionRecorder> h264_recorder;
        std::filesystem::path frames_dir;
        std::string ext;
        std::vector<int> encode_params;
        std::string encode_label;

        if (use_h264) {
            h264_recorder = std::make_unique<rmcs_laser_guidance::VideoSessionRecorder>(
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
                    .h264_qp = record_options.h264_qp,
                },
                record_options.h264_qp);
            encode_label = std::format("H264 qp={}", record_options.h264_qp);
        } else {
            const bool use_png = record_options.frame_format == "png";
            ext = use_png ? ".png" : ".jpg";
            frames_dir = session_root / "frames";
            std::filesystem::create_directories(frames_dir);
            if (use_png) {
                encode_params = {cv::IMWRITE_PNG_COMPRESSION, 1};
                encode_label = "PNG (lossless)";
            } else {
                encode_params = {cv::IMWRITE_JPEG_QUALITY, record_options.jpeg_quality};
                encode_label = std::format("JPEG q{}", record_options.jpeg_quality);
            }
            encode_label += std::format(" | sample_rate=1/{}", record_options.sample_rate);
        }

        std::println("recording: {} → {}", encode_label, session_root.string());

        std::vector<std::tuple<int, std::string, double>> manifest_entries;

        int frame_index = 0;
        int saved_frames = 0;

        while (!stop_requested() && std::chrono::steady_clock::now() < deadline) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Failed to read frame: {}", format_error(frame.error()));
                continue;
            }

            if (use_h264) {
                h264_recorder->record_frame(frame->image);
                saved_frames++;
            } else if (frame_index % record_options.sample_rate == 0) {
                const auto fname = std::format("{:06d}{}", frame_index, ext);
                const auto fpath = frames_dir / fname;
                if (cv::imwrite(fpath.string(), frame->image, encode_params)) {
                    const double blur = rmcs_laser_guidance::blur_score_for_frame(frame->image);
                    manifest_entries.emplace_back(frame_index, fname, blur);
                    saved_frames++;
                } else {
                    std::println(stderr, "failed to write frame: {}", fpath.string());
                }
            }

            if (g_ffplay_pipe) {
                std::fwrite(frame->image.data, 1, frame->image.total() * frame->image.elemSize(),
                            g_ffplay_pipe);
            }
            frame_index++;
        }

        close_ffplay_viewer(g_ffplay_pipe);
        capture.close();
        if (stop_requested())
            std::println("stop requested, finalizing recorded session");

        const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - monotonic_start)
                                     .count();

        if (use_h264) {
            h264_recorder->flush(duration_ms);
        } else {
            rmcs_laser_guidance::write_video_session_metadata(
                session_root / "session.yaml",
                rmcs_laser_guidance::VideoSessionMetadata{
                    .session_id = session_id,
                    .relative_video_path = "frames/",
                    .device_path = open_result->device_path,
                    .width = open_result->width,
                    .height = open_result->height,
                    .framerate = fps,
                    .fourcc = open_result->pixel_encoding,
                    .capture_start_unix_ms = unix_time_milliseconds(capture_start),
                    .duration_ms = duration_ms,
                    .lighting_tag = record_options.lighting_tag,
                    .background_tag = record_options.background_tag,
                    .distance_tag = record_options.distance_tag,
                    .target_color = record_options.target_color,
                    .operator_note_present = false,
                });
            const auto manifest_path = session_root / "manifest.csv";
            std::ofstream manifest(manifest_path);
            manifest << "frame_index,filename,blur_score\n";
            for (const auto& [idx, name, blur] : manifest_entries) {
                manifest << idx << "," << name << "," << std::fixed << std::setprecision(2) << blur << "\n";
            }
        }

        std::println(
            "session_id={} captured_frames={} saved={} session_root={}",
            session_id, frame_index, saved_frames, session_root.string());
        return (frame_index > 0 && saved_frames > 0) ? 0 : 1;
    } catch (const std::exception& e) {
        std::println(stderr, "example_v4l2_record_session failed: {}", e.what());
        return 1;
    }
}
