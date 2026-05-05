#include <print>

#include <opencv2/imgproc.hpp>

#include "config.hpp"
#include "pipeline.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        auto config = rmcs_laser_guidance::load_config(default_config_path());
        config.inference.backend = rmcs_laser_guidance::InferenceBackendKind::bright_spot;
        rmcs_laser_guidance::Pipeline pipeline(config);

        rmcs_laser_guidance::Frame empty_frame;
        require(!pipeline.process(empty_frame).detected, "empty frame should not detect");

        rmcs_laser_guidance::Frame blank_frame {
            .image     = cv::Mat::zeros(240, 320, CV_8UC3),
            .timestamp = rmcs_laser_guidance::Clock::now(),
        };
        require(!pipeline.process(blank_frame).detected, "blank frame should not detect");

        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::circle(image, { 320, 240 }, 8, { 255, 255, 255 }, -1);
        rmcs_laser_guidance::Frame positive_frame {
            .image     = image,
            .timestamp = rmcs_laser_guidance::Clock::now(),
        };
        const auto observation = pipeline.process(positive_frame);

        require(observation.detected, "positive frame should detect");
        require_near(observation.center.x, 320.0F, 3.0F, "pipeline center.x");
        require_near(observation.center.y, 240.0F, 3.0F, "pipeline center.y");

        auto model_config                 = config;
        model_config.inference.backend    = rmcs_laser_guidance::InferenceBackendKind::model;
        model_config.inference.model_path = "models/mock_detector.onnx";
        rmcs_laser_guidance::Pipeline model_pipeline(model_config);
        bool model_backend_threw = false;
        try {
            (void)model_pipeline.process(positive_frame);
        } catch (const std::exception&) {
            model_backend_threw = true;
        }
        require(model_backend_threw, "model backend should report an explicit error");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "pipeline_test failed: {}", e.what());
        return 1;
    }
}
