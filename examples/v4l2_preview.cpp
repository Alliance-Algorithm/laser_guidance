#include <filesystem>
#include <cstdio>
#include <print>

#include <opencv2/highgui.hpp>

#include "example_support.hpp"
#include "config.hpp"
#include "internal/v4l2_capture.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
    return rmcs_laser_guidance::examples::default_config_path();
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
        const auto config = rmcs_laser_guidance::load_config(config_path);

        rmcs_laser_guidance::V4l2Capture capture(config.v4l2);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open V4L2 capture: {}", open_result.error());
            return 1;
        }

        print_mode(config.v4l2, *open_result);

        while (true) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Failed to read frame: {}", frame.error());
                continue;
            }

            if (config.debug.show_window) {
                cv::imshow("rmcs_laser_guidance_v4l2", frame->image);
                if (rmcs_laser_guidance::examples::should_exit_from_key(cv::waitKey(1)))
                    break;
            }
        }

        capture.close();
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "example_v4l2_preview failed: {}", e.what());
        return 1;
    }
}
