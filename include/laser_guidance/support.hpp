#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

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

} // namespace rmcs_laser_guidance
