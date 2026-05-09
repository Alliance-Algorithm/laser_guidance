#include <cstdio>
#include <print>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "test_utils.hpp"
#include "config.hpp"
#include "core/debug_renderer.hpp"
#include "vision/detector.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        cv::Mat untouched = cv::Mat::zeros(240, 320, CV_8UC3);
        const cv::Mat untouched_before = untouched.clone();
        rmcs_laser_guidance::DebugRenderer disabled_renderer(
            rmcs_laser_guidance::DebugConfig{.show_window = false, .draw_overlay = false});
        disabled_renderer.draw(untouched, {});
        require(cv::norm(untouched, untouched_before, cv::NORM_INF) == 0.0, "disabled renderer changed image");

        cv::Mat empty_image;
        rmcs_laser_guidance::DebugRenderer enabled_renderer(
            rmcs_laser_guidance::DebugConfig{.show_window = false, .draw_overlay = true});
        enabled_renderer.draw(empty_image, {});

        const rmcs_laser_guidance::Config config;
        rmcs_laser_guidance::Detector detector(config);
        cv::Mat target_image = cv::Mat::zeros(240, 320, CV_8UC3);
        cv::circle(target_image, {160, 100}, 8, {255, 255, 255}, -1);
        rmcs_laser_guidance::Frame target_frame{
            .image = target_image,
            .timestamp = rmcs_laser_guidance::Clock::now(),
        };
        const auto positive_observation = detector.detect(target_frame);
        require(positive_observation.detected, "positive observation setup failed");

        cv::Mat rendered_target = target_image.clone();
        enabled_renderer.draw(rendered_target, positive_observation);
        require(
            cv::norm(rendered_target, target_image, cv::NORM_INF) > 0.0,
            "positive renderer should change image");

        cv::Mat negative_image = cv::Mat::zeros(240, 320, CV_8UC3);
        const cv::Mat negative_before = negative_image.clone();
        enabled_renderer.draw(negative_image, {});
        require(
            cv::norm(negative_image, negative_before, cv::NORM_INF) > 0.0,
            "negative renderer should draw no-target overlay");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "debug_renderer_test failed: {}", e.what());
        return 1;
    }
}
