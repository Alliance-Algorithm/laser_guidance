#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

#include "config.hpp"

namespace rmcs_laser_guidance {

struct RecordSessionOptions {
    std::filesystem::path output_root{};
    double duration_seconds = 30.0;
    std::string lighting_tag{"unspecified"};
    std::string background_tag{"unspecified"};
    std::string distance_tag{"unspecified"};
    std::string target_color{"red"};
    std::string frame_format{"h264"};
    int jpeg_quality = 85;
    int sample_rate = 10;
    int h264_qp = 23;
};

auto default_config_path() -> std::filesystem::path;
auto resolve_config_path(std::filesystem::path explicit_path = {}) -> std::filesystem::path;
auto default_sample_replay_path() -> std::filesystem::path;
auto default_video_session_root() -> std::filesystem::path;
auto default_record_session_options() -> RecordSessionOptions;
auto load_record_session_options(const std::filesystem::path& config_path) -> RecordSessionOptions;
auto record_session_v4l2_config(V4l2Config config) -> V4l2Config;
auto should_exit_from_key(int key) -> bool;
auto pixel_format_name(V4l2PixelFormat pixel_format) noexcept -> const char*;
auto inference_backend_name(InferenceBackendKind backend) noexcept -> const char*;

auto to_lower(std::string s) -> std::string;
inline auto squared(double v) -> double { return v * v; }

inline auto shell_quote(std::string_view value) -> std::string {
    std::string quoted;
    quoted.reserve(value.size() + 2);
    quoted.push_back('\'');
    for (const char ch : value) {
        if (ch == '\'')
            quoted += "'\"'\"'";
        else
            quoted.push_back(ch);
    }
    quoted.push_back('\'');
    return quoted;
}

} // namespace rmcs_laser_guidance
