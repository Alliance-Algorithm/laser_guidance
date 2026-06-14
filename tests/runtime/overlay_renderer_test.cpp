#include <print>
#include <stdexcept>

#include <opencv2/core.hpp>

#include "runtime/overlay_renderer.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::HitProgress;
        using rmcs_laser_guidance::runtime_internal::ControlLoopFrame;
        using rmcs_laser_guidance::runtime_internal::GuidanceCalibrationState;
        using rmcs_laser_guidance::runtime_internal::OverlayRenderContext;
        using rmcs_laser_guidance::runtime_internal::OverlayRenderer;
        using rmcs_laser_guidance::tests::require;

        OverlayRenderer renderer;
        ControlLoopFrame frame;
        frame.display = cv::Mat::zeros(240, 320, CV_8UC3);
        frame.detection.detected = true;
        frame.detection.detections.push_back(
            rmcs_laser_guidance::Detection{
                .score = 0.9F,
                .class_id = 1,
                .bbox = {100.0F, 80.0F, 20.0F, 20.0F},
                .center = {110.0F, 90.0F},
            });
        frame.track.detected = true;
        frame.track.ekf_enabled = false;
        frame.guidance.aim_output.depth_valid = true;
        const auto before = frame.display.clone();

        HitProgress hit_progress;
        hit_progress.update(true, 1.0F / 60.0F);
        renderer.render(
            frame,
            OverlayRenderContext{
                .guidance_enabled = true,
                .guidance_ready = true,
                .hit_progress = &hit_progress,
                .streaming_active = true,
                .recording_active = false,
                .enemy_color = rmcs_laser_guidance::EnemyColor::red,
                .using_tensorrt = false,
            });
        require(
            cv::norm(frame.display, before, cv::NORM_INF) > 0.0,
            "normal overlay should change image");

        ControlLoopFrame calibration_frame;
        calibration_frame.display = cv::Mat::zeros(240, 320, CV_8UC3);
        const auto calibration_before = calibration_frame.display.clone();
        const GuidanceCalibrationState calibration_state{
            .angle_x_deg = 1.0F,
            .angle_y_deg = -1.0F,
            .angle_step_deg = 0.01F,
        };
        renderer.render(
            calibration_frame,
            OverlayRenderContext{
                .guidance_enabled = true,
                .guidance_ready = true,
                .calibration_mode = true,
                .calibration_state = &calibration_state,
            });
        require(
            cv::norm(calibration_frame.display, calibration_before, cv::NORM_INF) > 0.0,
            "calibration overlay should change image");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "overlay_renderer_test failed: {}", e.what());
        return 1;
    }
}
