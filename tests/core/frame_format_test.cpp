#include <cstdio>
#include <print>

#include <opencv2/core.hpp>

#include "test_utils.hpp"
#include "core/frame_format.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        const cv::Mat gray = cv::Mat::zeros(32, 32, CV_8UC1);
        require(
            rmcs_laser_guidance::detect_frame_format(gray) ==
                rmcs_laser_guidance::FrameFormat::gray8,
            "gray frame format mismatch");
        require(
            rmcs_laser_guidance::is_supported_frame_format(
                rmcs_laser_guidance::detect_frame_format(gray)),
            "gray format should be supported");
        require(!rmcs_laser_guidance::to_gray_image(gray).empty(), "gray conversion failed");

        const cv::Mat bgr = cv::Mat::zeros(32, 32, CV_8UC3);
        require(
            rmcs_laser_guidance::detect_frame_format(bgr) ==
                rmcs_laser_guidance::FrameFormat::bgr8,
            "bgr frame format mismatch");
        require(!rmcs_laser_guidance::to_gray_image(bgr).empty(), "bgr conversion failed");

        const cv::Mat bgra = cv::Mat::zeros(32, 32, CV_8UC4);
        require(
            rmcs_laser_guidance::detect_frame_format(bgra) ==
                rmcs_laser_guidance::FrameFormat::bgra8,
            "bgra frame format mismatch");
        require(!rmcs_laser_guidance::to_gray_image(bgra).empty(), "bgra conversion failed");

        const cv::Mat invalid_channels = cv::Mat::zeros(32, 32, CV_8UC2);
        require(
            rmcs_laser_guidance::detect_frame_format(invalid_channels) ==
                rmcs_laser_guidance::FrameFormat::unknown,
            "invalid channel count should be unknown");
        require(
            rmcs_laser_guidance::to_gray_image(invalid_channels).empty(),
            "invalid channel count should not convert");

        const cv::Mat invalid_depth = cv::Mat::zeros(32, 32, CV_16UC1);
        require(
            rmcs_laser_guidance::detect_frame_format(invalid_depth) ==
                rmcs_laser_guidance::FrameFormat::unknown,
            "invalid depth should be unknown");
        require(
            rmcs_laser_guidance::to_gray_image(invalid_depth).empty(),
            "invalid depth should not convert");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "frame_format_test failed: {}", e.what());
        return 1;
    }
}
