#pragma once

#include <filesystem>

namespace rmcs_laser_guidance {

enum class V4l2PixelFormat {
    mjpeg = 0,
    yuyv,
    bgr24,
};

enum class InferenceBackendKind {
    bright_spot = 0,
    model,
};

struct V4l2Config {
    std::filesystem::path device_path = "/dev/video0";
    int width                         = 1920;
    int height                        = 1080;
    float framerate                   = 60.0F;
    V4l2PixelFormat pixel_format      = V4l2PixelFormat::mjpeg;
    bool invert_image                 = false;
};

struct DebugConfig {
    bool show_window  = true;
    bool draw_overlay = true;
};

struct RuntimeConfig {
    int max_input_age_ms       = 25;
    int max_observation_age_ms = 35;
    int max_infer_fps          = 60;
    int warmup_frames          = 30;
    std::filesystem::path engine_path { };
    int hit_confirm_frames     = 3;
    int hit_release_frames     = 5;
    bool debug_enabled         = false;
    int debug_max_fps          = 30;
    bool record_enabled        = false;
    int record_queue_size      = 16;
};

struct InferenceConfig {
    InferenceBackendKind backend = InferenceBackendKind::bright_spot;
    std::filesystem::path model_path { };
};

struct Config {
    V4l2Config v4l2 { };
    DebugConfig debug { };
    RuntimeConfig runtime { };
    InferenceConfig inference { };
};

auto load_config(const std::filesystem::path& config_path) -> Config;

} // namespace rmcs_laser_guidance
