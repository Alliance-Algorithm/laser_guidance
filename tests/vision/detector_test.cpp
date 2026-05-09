#include <cstdio>
#include <print>

#include <opencv2/imgproc.hpp>

#include "test_utils.hpp"
#include "config.hpp"
#include "vision/detector.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        const rmcs_laser_guidance::Config config;
        rmcs_laser_guidance::Detector detector(config);

        cv::Mat image = cv::Mat::zeros(300, 400, CV_8UC3);
        cv::circle(image, {150, 120}, 10, {255, 255, 255}, -1);
        rmcs_laser_guidance::Frame frame{
            .image = image,
            .timestamp = rmcs_laser_guidance::Clock::now(),
        };

        const auto observation = detector.detect(frame);
        require(observation.detected, "detector should detect synthetic target");
        require(!observation.contour.empty(), "detector contour should not be empty");
        require(observation.brightness >= 250.0F, "detector brightness should be populated");
        require_near(observation.center.x, 150.0F, 3.0F, "detector center.x");
        require_near(observation.center.y, 120.0F, 3.0F, "detector center.y");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "detector_test failed: {}", e.what());
        return 1;
    }
}
