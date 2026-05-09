#include <filesystem>
#include <cstdio>
#include <print>
#include <string>

#include <opencv2/highgui.hpp>

#include "example_support.hpp"
#include "config.hpp"
#include "pipeline.hpp"
#include "core/replay.hpp"
#include "capture/v4l2_capture.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
    return rmcs_laser_guidance::examples::default_config_path();
}

auto resolve_output_dir(int argc, char** argv) -> std::filesystem::path {
    if (argc > 2)
        return argv[2];
    return "/tmp/rmcs_laser_guidance_capture";
}

auto resolve_max_frames(int argc, char** argv) -> std::size_t {
    if (argc > 3)
        return static_cast<std::size_t>(std::stoull(argv[3]));
    return 300;
}

auto print_mode(const rmcs_laser_guidance::V4l2Config& requested,
                const rmcs_laser_guidance::V4l2NegotiatedFormat& actual) -> void {
    std::println(
        "requested device={} mode={}x{}@{} format={}",
        requested.device_path.string(),
        requested.width,
        requested.height,
        requested.framerate,
        rmcs_laser_guidance::examples::pixel_format_name(requested.pixel_format));
    std::println(
        "actual    device={} mode={}x{}@{} format={}",
        actual.device_path.string(),
        actual.width,
        actual.height,
        actual.framerate,
        actual.fourcc);
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path config_path = resolve_config_path(argc, argv);
        const std::filesystem::path output_dir = resolve_output_dir(argc, argv);
        const std::size_t max_frames = resolve_max_frames(argc, argv);

        const auto config = rmcs_laser_guidance::load_config(config_path);
        rmcs_laser_guidance::Pipeline pipeline(config);
        rmcs_laser_guidance::ReplayRecorder recorder(output_dir);

        rmcs_laser_guidance::V4l2Capture capture(config.v4l2);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open V4L2 capture: {}", open_result.error());
            return 1;
        }

        print_mode(config.v4l2, *open_result);

        std::size_t captured = 0;
        while (captured < max_frames) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Failed to read frame: {}", frame.error());
                continue;
            }

            recorder.record_frame(*frame);
            ++captured;

            if (config.debug.show_window) {
                const auto observation = pipeline.process(*frame);
                cv::Mat display = frame->image.clone();
                pipeline.draw_debug_overlay(display, observation);
                cv::imshow("rmcs_laser_guidance_v4l2_capture", display);
                if (rmcs_laser_guidance::examples::should_exit_from_key(cv::waitKey(1)))
                    break;
            }
        }

        recorder.flush_manifest();
        capture.close();

        std::println("captured={} output_dir={}", captured, output_dir.string());
        return captured > 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::println(stderr, "example_v4l2_capture failed: {}", e.what());
        return 1;
    }
}
