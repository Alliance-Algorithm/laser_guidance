#include "laser_guidance/support.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace rmcs_laser_guidance {

auto to_lower(std::string s) -> std::string {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

auto default_config_path() -> std::filesystem::path {
    return "config/default.yaml";
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
    return "test_data/sample_images";
}

auto default_video_session_root() -> std::filesystem::path {
    return "videos";
}

auto default_record_session_options() -> RecordSessionOptions {
    return RecordSessionOptions{
        .output_root = default_video_session_root(),
        .duration_seconds = 60.0,
        .lighting_tag = "unspecified",
        .background_tag = "unspecified",
        .distance_tag = "unspecified",
        .target_color = "red",
        .frame_format = "h264",
        .jpeg_quality = 85,
        .sample_rate = 10,
        .h264_qp = 23,
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
    if (record["frame_format"]) {
        options.frame_format = to_lower(record["frame_format"].as<std::string>());
    }
    if (record["jpeg_quality"]) {
        options.jpeg_quality = record["jpeg_quality"].as<int>();
    }
    if (record["sample_rate"]) {
        options.sample_rate = record["sample_rate"].as<int>();
    }
    if (record["h264_qp"]) {
        options.h264_qp = record["h264_qp"].as<int>();
    }

    if (options.output_root.empty()) {
        throw std::runtime_error("record.output_root must not be empty");
    }
    if (options.duration_seconds <= 0.0) {
        throw std::runtime_error("record.duration_seconds must be positive");
    }
    if (options.frame_format != "h264" && options.frame_format != "jpeg" && options.frame_format != "png") {
        throw std::runtime_error("record.frame_format must be 'h264', 'jpeg', or 'png'");
    }
    if (options.jpeg_quality < 1 || options.jpeg_quality > 100) {
        throw std::runtime_error("record.jpeg_quality must be 1-100");
    }
    if (options.sample_rate < 1) {
        throw std::runtime_error("record.sample_rate must be >= 1");
    }
    if (options.h264_qp < 0 || options.h264_qp > 51) {
        throw std::runtime_error("record.h264_qp must be 0-51 (0=lossless)");
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
