#include <cstdio>
#include <print>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "config.hpp"
#include "core/debug_renderer.hpp"
#include "test_utils.hpp"
#include "types.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        cv::Mat untouched = cv::Mat::zeros(240, 320, CV_8UC3);
        const cv::Mat untouched_before = untouched.clone();
        rmcs_laser_guidance::DebugRenderer disabled_renderer(
            rmcs_laser_guidance::DebugConfig{.show_window = false, .draw_overlay = false});
        disabled_renderer.draw(untouched, {});
        require(
            cv::norm(untouched, untouched_before, cv::NORM_INF) == 0.0,
            "disabled renderer changed image");

        cv::Mat empty_image;
        rmcs_laser_guidance::DebugRenderer enabled_renderer(
            rmcs_laser_guidance::DebugConfig{.show_window = false, .draw_overlay = true});
        enabled_renderer.draw(empty_image, {});

        // 手动构造 positive observation，不依赖 Detector
        rmcs_laser_guidance::TargetObservation positive_observation;
        positive_observation.detected = true;
        positive_observation.center = {160, 100};
        rmcs_laser_guidance::ModelCandidate candidate;
        candidate.score = 0.9F;
        candidate.class_id = 1;
        candidate.bbox = cv::Rect2f(150, 90, 20, 20);
        candidate.center = {160, 100};
        positive_observation.candidates.push_back(candidate);

        cv::Mat rendered_target = cv::Mat::zeros(240, 320, CV_8UC3);
        const cv::Mat target_before = rendered_target.clone();
        enabled_renderer.draw(rendered_target, positive_observation);
        require(
            cv::norm(rendered_target, target_before, cv::NORM_INF) > 0.0,
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