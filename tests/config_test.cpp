#include <print>

#include "../examples/example_support.hpp"
#include "config.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        const auto default_config = rmcs_laser_guidance::load_config(default_config_path());
        require(default_config.v4l2.device_path == std::filesystem::path("/dev/video2"),
            "default device path mismatch");
        require(default_config.v4l2.width == 1920, "default width mismatch");
        require(default_config.v4l2.height == 1080, "default height mismatch");
        require_near(default_config.v4l2.framerate, 60.0F, 1e-3F, "default framerate");
        require(default_config.v4l2.pixel_format == rmcs_laser_guidance::V4l2PixelFormat::mjpeg,
            "default pixel format mismatch");
        require(!default_config.v4l2.invert_image, "default invert_image mismatch");
        require(default_config.debug.show_window, "default show_window mismatch");
        require(default_config.debug.draw_overlay, "default draw_overlay mismatch");
        require(default_config.runtime.max_input_age_ms == 25, "default max_input_age_ms mismatch");
        require(default_config.runtime.max_observation_age_ms == 35,
            "default max_observation_age_ms mismatch");
        require(default_config.runtime.max_infer_fps == 60, "default max_infer_fps mismatch");
        require(default_config.runtime.warmup_frames == 30, "default warmup_frames mismatch");
        require(default_config.runtime.engine_path.empty(), "default engine_path mismatch");
        require(default_config.runtime.hit_confirm_frames == 3,
            "default hit_confirm_frames mismatch");
        require(default_config.runtime.hit_release_frames == 5,
            "default hit_release_frames mismatch");
        require(!default_config.runtime.debug_enabled, "default debug_enabled mismatch");
        require(default_config.runtime.debug_max_fps == 30, "default debug_max_fps mismatch");
        require(!default_config.runtime.record_enabled, "default record_enabled mismatch");
        require(default_config.runtime.record_queue_size == 16,
            "default record_queue_size mismatch");
        require(default_config.inference.backend
                == rmcs_laser_guidance::InferenceBackendKind::model,
            "default inference backend mismatch");
        require(default_config.inference.model_path == std::filesystem::path("models/exp.onnx"),
            "default model path mismatch");
        require(default_video_session_root()
                == (default_config_path().parent_path().parent_path() / "videos"),
            "default video session root mismatch");
        const auto default_record_options =
            rmcs_laser_guidance::examples::load_record_session_options(default_config_path());
        require(default_record_options.output_root == default_video_session_root(),
            "default record output root mismatch");
        require_near(static_cast<float>(default_record_options.duration_seconds), 60.0F, 1e-3F,
            "default record duration mismatch");
        require(default_record_options.lighting_tag == "unspecified",
            "default record lighting tag mismatch");
        require(default_record_options.background_tag == "unspecified",
            "default record background tag mismatch");
        require(default_record_options.distance_tag == "unspecified",
            "default record distance tag mismatch");
        require(
            default_record_options.target_color == "red", "default record target color mismatch");
        const auto forced_record_v4l2_config =
            rmcs_laser_guidance::examples::record_session_v4l2_config({
                .device_path  = "/dev/video7",
                .width        = 1280,
                .height       = 720,
                .framerate    = 59.94F,
                .pixel_format = rmcs_laser_guidance::V4l2PixelFormat::mjpeg,
                .invert_image = true,
            });
        require(forced_record_v4l2_config.device_path == std::filesystem::path("/dev/video7"),
            "record override device path mismatch");
        require(forced_record_v4l2_config.width == 1280, "record override width mismatch");
        require(forced_record_v4l2_config.height == 720, "record override height mismatch");
        require_near(
            forced_record_v4l2_config.framerate, 59.94F, 1e-3F, "record override framerate");
        require(
            forced_record_v4l2_config.pixel_format == rmcs_laser_guidance::V4l2PixelFormat::yuyv,
            "record override pixel format mismatch");
        require(forced_record_v4l2_config.invert_image, "record override invert_image mismatch");

        const auto capture_profile_path =
            default_config_path().parent_path() / "capture_red_20m.yaml";
        const auto capture_profile = rmcs_laser_guidance::load_config(capture_profile_path);
        require(capture_profile.v4l2.device_path == std::filesystem::path("/dev/video2"),
            "capture profile device path mismatch");
        require(capture_profile.v4l2.width == 1920, "capture profile width mismatch");
        require(capture_profile.v4l2.height == 1080, "capture profile height mismatch");
        require_near(
            capture_profile.v4l2.framerate, 60.0F, 1e-3F, "capture profile framerate mismatch");
        require(capture_profile.v4l2.pixel_format == rmcs_laser_guidance::V4l2PixelFormat::mjpeg,
            "capture profile pixel format mismatch");
        require(capture_profile.debug.show_window, "capture profile show_window mismatch");
        require(capture_profile.debug.draw_overlay, "capture profile draw_overlay mismatch");
        const auto capture_record_options =
            rmcs_laser_guidance::examples::load_record_session_options(capture_profile_path);
        require(capture_record_options.output_root == std::filesystem::path("./videos"),
            "capture record output root mismatch");
        require_near(static_cast<float>(capture_record_options.duration_seconds), 240.0F, 1e-3F,
            "capture record duration mismatch");
        require(capture_record_options.lighting_tag == "indoor_lab",
            "capture record lighting tag mismatch");
        require(capture_record_options.background_tag == "plain_wall",
            "capture record background tag mismatch");
        require(
            capture_record_options.distance_tag == "20m", "capture record distance tag mismatch");
        require(
            capture_record_options.target_color == "red", "capture record target color mismatch");

        const auto override_path = make_temp_path("rmcs_laser_guidance_config_override");
        write_text_file(override_path,
            "v4l2:\n"
            "  device_path: /dev/video7\n"
            "  width: 1280\n"
            "  height: 720\n"
            "  framerate: 59.94\n"
            "  pixel_format: yuyv\n"
            "  invert_image: true\n"
            "debug:\n"
            "  show_window: false\n"
            "  draw_overlay: false\n"
            "runtime:\n"
            "  max_input_age_ms: 10\n"
            "  max_observation_age_ms: 20\n"
            "  max_infer_fps: 120\n"
            "  warmup_frames: 5\n"
            "  engine_path: /tmp/model.engine\n"
            "  hit_confirm_frames: 7\n"
            "  hit_release_frames: 9\n"
            "  debug_enabled: true\n"
            "  debug_max_fps: 15\n"
            "  record_enabled: true\n"
            "  record_queue_size: 8\n"
            "inference:\n"
            "  backend: model\n"
            "  model_path: models/mock_detector.onnx\n");
        const auto override_config = rmcs_laser_guidance::load_config(override_path);
        require(override_config.v4l2.device_path == std::filesystem::path("/dev/video7"),
            "override device path mismatch");
        require(override_config.v4l2.width == 1280, "override width mismatch");
        require(override_config.v4l2.height == 720, "override height mismatch");
        require_near(override_config.v4l2.framerate, 59.94F, 1e-3F, "override framerate");
        require(override_config.v4l2.pixel_format == rmcs_laser_guidance::V4l2PixelFormat::yuyv,
            "override pixel format mismatch");
        require(override_config.v4l2.invert_image, "override invert_image mismatch");
        require(!override_config.debug.show_window, "override show_window mismatch");
        require(!override_config.debug.draw_overlay, "override draw_overlay mismatch");
        require(override_config.runtime.max_input_age_ms == 10,
            "override max_input_age_ms mismatch");
        require(override_config.runtime.max_observation_age_ms == 20,
            "override max_observation_age_ms mismatch");
        require(override_config.runtime.max_infer_fps == 120,
            "override max_infer_fps mismatch");
        require(override_config.runtime.warmup_frames == 5, "override warmup_frames mismatch");
        require(override_config.runtime.engine_path == std::filesystem::path("/tmp/model.engine"),
            "override engine_path mismatch");
        require(override_config.runtime.hit_confirm_frames == 7,
            "override hit_confirm_frames mismatch");
        require(override_config.runtime.hit_release_frames == 9,
            "override hit_release_frames mismatch");
        require(override_config.runtime.debug_enabled, "override debug_enabled mismatch");
        require(override_config.runtime.debug_max_fps == 15, "override debug_max_fps mismatch");
        require(override_config.runtime.record_enabled, "override record_enabled mismatch");
        require(override_config.runtime.record_queue_size == 8,
            "override record_queue_size mismatch");
        require(
            override_config.inference.backend == rmcs_laser_guidance::InferenceBackendKind::model,
            "override inference backend mismatch");
        require(override_config.inference.model_path
                == std::filesystem::path("models/mock_detector.onnx"),
            "override model path mismatch");

        const auto bad_framerate_path = make_temp_path("rmcs_laser_guidance_bad_framerate");
        write_text_file(bad_framerate_path,
            "v4l2:\n"
            "  framerate: 0.0\n"
            "  width: 1920\n"
            "  height: 1080\n"
            "  device_path: /dev/video0\n");
        bool bad_framerate_threw = false;
        try {
            (void)rmcs_laser_guidance::load_config(bad_framerate_path);
        } catch (const std::exception&) {
            bad_framerate_threw = true;
        }
        require(bad_framerate_threw, "bad framerate should throw");

        const auto bad_width_path = make_temp_path("rmcs_laser_guidance_bad_width");
        write_text_file(bad_width_path,
            "v4l2:\n"
            "  device_path: /dev/video0\n"
            "  width: 0\n"
            "  height: 1080\n"
            "  framerate: 10.0\n");
        bool bad_width_threw = false;
        try {
            (void)rmcs_laser_guidance::load_config(bad_width_path);
        } catch (const std::exception&) {
            bad_width_threw = true;
        }
        require(bad_width_threw, "bad width should throw");

        const auto bad_device_path = make_temp_path("rmcs_laser_guidance_bad_device_path");
        write_text_file(bad_device_path,
            "v4l2:\n"
            "  device_path: ''\n"
            "  width: 1920\n"
            "  height: 1080\n"
            "  framerate: 10.0\n");
        bool bad_device_path_threw = false;
        try {
            (void)rmcs_laser_guidance::load_config(bad_device_path);
        } catch (const std::exception&) {
            bad_device_path_threw = true;
        }
        require(bad_device_path_threw, "empty device path should throw");

        const auto bad_pixel_format = make_temp_path("rmcs_laser_guidance_bad_pixel_format");
        write_text_file(bad_pixel_format,
            "v4l2:\n"
            "  device_path: /dev/video0\n"
            "  width: 1920\n"
            "  height: 1080\n"
            "  framerate: 10.0\n"
            "  pixel_format: rgb24\n");
        bool bad_pixel_format_threw = false;
        try {
            (void)rmcs_laser_guidance::load_config(bad_pixel_format);
        } catch (const std::exception&) {
            bad_pixel_format_threw = true;
        }
        require(bad_pixel_format_threw, "bad pixel format should throw");

        const auto bad_runtime_path = make_temp_path("rmcs_laser_guidance_bad_runtime");
        write_text_file(bad_runtime_path,
            "runtime:\n"
            "  max_input_age_ms: 0\n"
            "  max_observation_age_ms: 35\n"
            "  max_infer_fps: 60\n"
            "  warmup_frames: 30\n"
            "  hit_confirm_frames: 3\n"
            "  hit_release_frames: 5\n"
            "  debug_max_fps: 30\n"
            "  record_queue_size: 16\n");
        bool bad_runtime_threw = false;
        try {
            (void)rmcs_laser_guidance::load_config(bad_runtime_path);
        } catch (const std::exception&) {
            bad_runtime_threw = true;
        }
        require(bad_runtime_threw, "bad runtime config should throw");

        const auto bad_backend = make_temp_path("rmcs_laser_guidance_bad_backend");
        write_text_file(bad_backend,
            "inference:\n"
            "  backend: tensor_rt\n");
        bool bad_backend_threw = false;
        try {
            (void)rmcs_laser_guidance::load_config(bad_backend);
        } catch (const std::exception&) {
            bad_backend_threw = true;
        }
        require(bad_backend_threw, "bad inference backend should throw");

        bool missing_path_threw = false;
        try {
            (void)rmcs_laser_guidance::load_config("/tmp/does_not_exist_rmcs_laser_guidance.yaml");
        } catch (const std::exception&) {
            missing_path_threw = true;
        }
        require(missing_path_threw, "missing config path should throw");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "config_test failed: {}", e.what());
        return 1;
    }
}
