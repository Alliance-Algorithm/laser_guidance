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

struct InferenceConfig {
    InferenceBackendKind backend = InferenceBackendKind::bright_spot;
    std::filesystem::path model_path { };
};

struct Config {
    V4l2Config v4l2 { };
    DebugConfig debug { };
    InferenceConfig inference { };
};

auto load_config(const std::filesystem::path& config_path) -> Config;

} // namespace rmcs_laser_guidance
