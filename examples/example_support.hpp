#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

#include "config.hpp"
#include <yaml-cpp/yaml.h>

namespace rmcs_laser_guidance::examples {

struct RecordSessionOptions {
    std::filesystem::path output_root { };
    double duration_seconds = 30.0;
    std::string lighting_tag { "unspecified" };
    std::string background_tag { "unspecified" };
    std::string distance_tag { "unspecified" };
    std::string target_color { "red" };
};

inline auto default_config_path() -> std::filesystem::path {
#ifdef RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH
    return RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH;
#else
    return "config/default.yaml";
#endif
}

inline auto default_sample_replay_path() -> std::filesystem::path {
#ifdef RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH
    return RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH;
#else
    return "test_data/sample_images";
#endif
}

inline auto default_video_session_root() -> std::filesystem::path {
#ifdef RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT
    return RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT;
#else
    return "videos";
#endif
}

inline auto default_record_session_options() -> RecordSessionOptions {
    return RecordSessionOptions {
        .output_root      = default_video_session_root(),
        .duration_seconds = 60.0,
        .lighting_tag     = "unspecified",
        .background_tag   = "unspecified",
        .distance_tag     = "unspecified",
        .target_color     = "red",
    };
}

inline auto load_record_session_options(const std::filesystem::path& config_path)
    -> RecordSessionOptions {
    auto options = default_record_session_options();

    const YAML::Node yaml   = YAML::LoadFile(config_path.string());
    const YAML::Node record = yaml["record"];
    if (!record) return options;

    if (record["output_root"]) options.output_root = record["output_root"].as<std::string>();
    if (record["duration_seconds"])
        options.duration_seconds = record["duration_seconds"].as<double>();
    if (record["lighting_tag"]) options.lighting_tag = record["lighting_tag"].as<std::string>();
    if (record["background_tag"])
        options.background_tag = record["background_tag"].as<std::string>();
    if (record["distance_tag"]) options.distance_tag = record["distance_tag"].as<std::string>();
    if (record["target_color"]) options.target_color = record["target_color"].as<std::string>();

    if (options.output_root.empty())
        throw std::runtime_error("record.output_root must not be empty");
    if (options.duration_seconds <= 0.0)
        throw std::runtime_error("record.duration_seconds must be positive");

    return options;
}

inline auto record_session_v4l2_config(rmcs_laser_guidance::V4l2Config config)
    -> rmcs_laser_guidance::V4l2Config {
    config.pixel_format = rmcs_laser_guidance::V4l2PixelFormat::yuyv;
    return config;
}

inline auto should_exit_from_key(const int key) -> bool {
    return key == 27 || key == 'q' || key == 'Q';
}

inline auto pixel_format_name(const rmcs_laser_guidance::V4l2PixelFormat pixel_format) noexcept
    -> const char* {
    switch (pixel_format) {
    case rmcs_laser_guidance::V4l2PixelFormat::mjpeg:
        return "mjpeg";
    case rmcs_laser_guidance::V4l2PixelFormat::yuyv:
        return "yuyv";
    case rmcs_laser_guidance::V4l2PixelFormat::bgr24:
        return "bgr24";
    default:
        return "unknown";
    }
}

inline auto inference_backend_name(const rmcs_laser_guidance::InferenceBackendKind backend) noexcept
    -> const char* {
    switch (backend) {
    case rmcs_laser_guidance::InferenceBackendKind::bright_spot:
        return "bright_spot";
    case rmcs_laser_guidance::InferenceBackendKind::model:
        return "model";
    default:
        return "unknown";
    }
}

} // namespace rmcs_laser_guidance::examples
