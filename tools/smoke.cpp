#include <filesystem>
#include <cstdio>
#include <print>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "example_support.hpp"
#include "config.hpp"
#include "pipeline.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
    return rmcs_laser_guidance::examples::default_config_path();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path config_path = resolve_config_path(argc, argv);
        const auto config = rmcs_laser_guidance::load_config(config_path);
        rmcs_laser_guidance::Pipeline pipeline(config);

        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::circle(image, {320, 240}, 8, {255, 255, 255}, -1);

        rmcs_laser_guidance::Frame frame{
            .image = image,
            .timestamp = rmcs_laser_guidance::Clock::now(),
        };
        const auto observation = pipeline.process(frame);

        std::println(
            "detected={} center=({}, {})",
            observation.detected ? 1 : 0,
            observation.center.x,
            observation.center.y);

        if (config.debug.show_window) {
            cv::Mat display = image.clone();
            pipeline.draw_debug_overlay(display, observation);
            cv::imshow("rmcs_laser_guidance_offline", display);
            cv::waitKey(0);
        }

        return observation.detected ? 0 : 1;
    } catch (const std::exception& e) {
        std::println(stderr, "example_offline_smoke failed: {}", e.what());
        return 1;
    }
}
