#include "laser_guidance/support.hpp"

#include <cstdlib>
#include <string>

namespace rmcs_laser_guidance {

auto default_config_path() -> std::filesystem::path {
#ifdef RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH
    return RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH;
#else
    return "config/default.yaml";
#endif
}

auto resolve_config_path(std::filesystem::path explicit_path) -> std::filesystem::path {
    if (!explicit_path.empty()) {
        return explicit_path;
    }
    if (const char* env = std::getenv("LASER_GUIDANCE_CONFIG"); env != nullptr
        && std::string(env).empty() == false) {
        return env;
    }
    return default_config_path();
}

auto default_sample_replay_path() -> std::filesystem::path {
#ifdef RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH
    return RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH;
#else
    return "test_data/sample_images";
#endif
}

auto default_video_session_root() -> std::filesystem::path {
#ifdef RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT
    return RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT;
#else
    return "videos";
#endif
}

auto default_record_session_options() -> RecordSessionOptions {
    return RecordSessionOptions{
        .output_root = default_video_session_root(),
        .duration_seconds = 60.0,
        .lighting_tag = "unspecified",
        .background_tag = "unspecified",
        .distance_tag = "unspecified",
        .target_color = "red",
    };
}

auto load_record_session_options(const std::filesystem::path& config_path) -> RecordSessionOptions {
    auto options = default_record_session_options();

    const YAML::Node yaml = YAML::LoadFile(config_path.string());
    const YAML::Node record = yaml["record"];
    if (!record) {
        return options;
    }

    if (record["output_root"]) {
        options.output_root = record["output_root"].as<std::string>();
    }
    if (record["duration_seconds"]) {
        options.duration_seconds = record["duration_seconds"].as<double>();
    }
    if (record["lighting_tag"]) {
        options.lighting_tag = record["lighting_tag"].as<std::string>();
    }
    if (record["background_tag"]) {
        options.background_tag = record["background_tag"].as<std::string>();
    }
    if (record["distance_tag"]) {
        options.distance_tag = record["distance_tag"].as<std::string>();
    }
    if (record["target_color"]) {
        options.target_color = record["target_color"].as<std::string>();
    }

    if (options.output_root.empty()) {
        throw std::runtime_error("record.output_root must not be empty");
    }
    if (options.duration_seconds <= 0.0) {
        throw std::runtime_error("record.duration_seconds must be positive");
    }

    return options;
}

auto record_session_v4l2_config(V4l2Config config) -> V4l2Config {
    config.pixel_format = V4l2PixelFormat::yuyv;
    return config;
}

auto should_exit_from_key(const int key) -> bool {
    return key == 27 || key == 'q' || key == 'Q';
}

auto pixel_format_name(const V4l2PixelFormat pixel_format) noexcept -> const char* {
    switch (pixel_format) {
    case V4l2PixelFormat::mjpeg: return "mjpeg";
    case V4l2PixelFormat::yuyv: return "yuyv";
    case V4l2PixelFormat::bgr24: return "bgr24";
    default: return "unknown";
    }
}

auto inference_backend_name(const InferenceBackendKind backend) noexcept -> const char* {
    switch (backend) {
    case InferenceBackendKind::bright_spot: return "bright_spot";
    case InferenceBackendKind::model: return "model";
    case InferenceBackendKind::tensorrt: return "tensorrt";
    default: return "unknown";
    }
}

} // namespace rmcs_laser_guidance
