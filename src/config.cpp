#include "config.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace rmcs_laser_guidance {
namespace {

    auto to_lower_copy(std::string value) -> std::string {
        std::transform(value.begin(), value.end(), value.begin(),
            [](const unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        return value;
    }

    auto parse_v4l2_pixel_format(const std::string_view value) -> V4l2PixelFormat {
        const std::string lower = to_lower_copy(std::string(value));
        if (lower == "mjpeg") return V4l2PixelFormat::mjpeg;
        if (lower == "yuyv") return V4l2PixelFormat::yuyv;
        if (lower == "bgr24") return V4l2PixelFormat::bgr24;

        throw std::runtime_error("v4l2.pixel_format must be one of: mjpeg, yuyv, bgr24");
    }

    auto parse_inference_backend(const std::string_view value) -> InferenceBackendKind {
        const std::string lower = to_lower_copy(std::string(value));
        if (lower == "bright_spot") return InferenceBackendKind::bright_spot;
        if (lower == "model") return InferenceBackendKind::model;

        throw std::runtime_error("inference.backend must be one of: bright_spot, model");
    }

} // namespace

auto load_config(const std::filesystem::path& config_path) -> Config {
    YAML::Node yaml = YAML::LoadFile(config_path.string());

    Config config;

    if (const YAML::Node v4l2 = yaml["v4l2"]) {
        if (v4l2["device_path"]) config.v4l2.device_path = v4l2["device_path"].as<std::string>();
        if (v4l2["width"]) config.v4l2.width = v4l2["width"].as<int>();
        if (v4l2["height"]) config.v4l2.height = v4l2["height"].as<int>();
        if (v4l2["framerate"]) config.v4l2.framerate = v4l2["framerate"].as<float>();
        if (v4l2["pixel_format"])
            config.v4l2.pixel_format =
                parse_v4l2_pixel_format(v4l2["pixel_format"].as<std::string>());
        if (v4l2["invert_image"]) config.v4l2.invert_image = v4l2["invert_image"].as<bool>();
    }

    if (const YAML::Node debug = yaml["debug"]) {
        if (debug["show_window"]) config.debug.show_window = debug["show_window"].as<bool>();
        if (debug["draw_overlay"]) config.debug.draw_overlay = debug["draw_overlay"].as<bool>();
    }

    if (const YAML::Node inference = yaml["inference"]) {
        if (inference["backend"]) {
            config.inference.backend =
                parse_inference_backend(inference["backend"].as<std::string>());
        }
        if (inference["model_path"])
            config.inference.model_path = inference["model_path"].as<std::string>();
    }

    if (config.v4l2.device_path.empty())
        throw std::runtime_error("v4l2.device_path must not be empty");
    if (config.v4l2.width <= 0) throw std::runtime_error("v4l2.width must be positive");
    if (config.v4l2.height <= 0) throw std::runtime_error("v4l2.height must be positive");
    if (config.v4l2.framerate <= 0.0F) throw std::runtime_error("v4l2.framerate must be positive");

    return config;
}

} // namespace rmcs_laser_guidance
