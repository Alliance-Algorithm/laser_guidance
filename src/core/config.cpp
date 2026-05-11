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
        if (lower == "tensorrt") return InferenceBackendKind::tensorrt;

        throw std::runtime_error("inference.backend must be one of: bright_spot, model, tensorrt");
    }

    auto parse_guidance_command_model(const std::string_view value)
        -> GuidanceCommandModelKind {
        const std::string lower = to_lower_copy(std::string(value));
        if (lower == "geometry") return GuidanceCommandModelKind::geometry;
        if (lower == "direct_voltage") return GuidanceCommandModelKind::direct_voltage;

        throw std::runtime_error(
            "guidance.command_model must be one of: geometry, direct_voltage");
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

    if (const YAML::Node runtime = yaml["runtime"]) {
        if (runtime["max_input_age_ms"])
            config.runtime.max_input_age_ms = runtime["max_input_age_ms"].as<int>();
        if (runtime["max_observation_age_ms"])
            config.runtime.max_observation_age_ms = runtime["max_observation_age_ms"].as<int>();
        if (runtime["max_infer_fps"])
            config.runtime.max_infer_fps = runtime["max_infer_fps"].as<int>();
        if (runtime["warmup_frames"])
            config.runtime.warmup_frames = runtime["warmup_frames"].as<int>();
        if (runtime["engine_path"])
            config.runtime.engine_path = runtime["engine_path"].as<std::string>();
        if (runtime["hit_confirm_frames"])
            config.runtime.hit_confirm_frames = runtime["hit_confirm_frames"].as<int>();
        if (runtime["hit_release_frames"])
            config.runtime.hit_release_frames = runtime["hit_release_frames"].as<int>();
        if (runtime["debug_enabled"]) config.runtime.debug_enabled = runtime["debug_enabled"].as<bool>();
        if (runtime["debug_max_fps"]) config.runtime.debug_max_fps = runtime["debug_max_fps"].as<int>();
        if (runtime["record_enabled"]) config.runtime.record_enabled = runtime["record_enabled"].as<bool>();
        if (runtime["record_queue_size"]) config.runtime.record_queue_size = runtime["record_queue_size"].as<int>();
    }

    if (const YAML::Node inference = yaml["inference"]) {
        if (inference["backend"]) {
            config.inference.backend =
                parse_inference_backend(inference["backend"].as<std::string>());
        }
        if (inference["model_path"])
            config.inference.model_path = inference["model_path"].as<std::string>();
    }

    if (const YAML::Node streaming = yaml["streaming"]) {
        if (streaming["enabled"]) config.rtp.enabled = streaming["enabled"].as<bool>();
        if (streaming["host"]) config.rtp.host = streaming["host"].as<std::string>();
        if (streaming["port"]) config.rtp.port = streaming["port"].as<int>();
        if (streaming["sdp_path"]) config.rtp.sdp_path = streaming["sdp_path"].as<std::string>();
        if (streaming["encoder"]) config.rtp.encoder = streaming["encoder"].as<std::string>();
    }

    if (const YAML::Node udp_cfg = yaml["udp"]) {
        if (udp_cfg["enabled"]) config.udp.enabled = udp_cfg["enabled"].as<bool>();
        if (udp_cfg["host"]) config.udp.host = udp_cfg["host"].as<std::string>();
        if (udp_cfg["port"]) config.udp.port = udp_cfg["port"].as<int>();
    }

    if (const YAML::Node ekf = yaml["ekf"]) {
        if (ekf["process_noise_q"])
            config.ekf.process_noise_q = ekf["process_noise_q"].as<double>();
        if (ekf["measurement_noise_r"])
            config.ekf.measurement_noise_r = ekf["measurement_noise_r"].as<double>();
        if (ekf["initial_pos_std"])
            config.ekf.initial_pos_std = ekf["initial_pos_std"].as<double>();
        if (ekf["initial_vel_std"])
            config.ekf.initial_vel_std = ekf["initial_vel_std"].as<double>();
        if (ekf["initial_acc_std"])
            config.ekf.initial_acc_std = ekf["initial_acc_std"].as<double>();
        if (ekf["max_missed_frames"])
            config.ekf.max_missed_frames = ekf["max_missed_frames"].as<int>();
    }

    if (const YAML::Node guidance = yaml["guidance"]) {
        if (guidance["enabled"])
            config.guidance.enabled = guidance["enabled"].as<bool>();
        if (guidance["command_model"])
            config.guidance.command_model =
                parse_guidance_command_model(guidance["command_model"].as<std::string>());
        if (guidance["camera_calib_path"])
            config.guidance.camera_calib_path =
                guidance["camera_calib_path"].as<std::string>();
        if (guidance["voltage_model_path"])
            config.guidance.voltage_model_path =
                guidance["voltage_model_path"].as<std::string>();
        if (guidance["t_x_mm"])
            config.guidance.t_x_mm = guidance["t_x_mm"].as<float>();
        if (guidance["t_y_mm"])
            config.guidance.t_y_mm = guidance["t_y_mm"].as<float>();
        if (guidance["t_z_mm"])
            config.guidance.t_z_mm = guidance["t_z_mm"].as<float>();
        if (guidance["r_x_deg"])
            config.guidance.r_x_deg = guidance["r_x_deg"].as<float>();
        if (guidance["r_y_deg"])
            config.guidance.r_y_deg = guidance["r_y_deg"].as<float>();
        if (guidance["r_z_deg"])
            config.guidance.r_z_deg = guidance["r_z_deg"].as<float>();
        if (guidance["mirror_separation_mm"])
            config.guidance.mirror_separation_mm =
                guidance["mirror_separation_mm"].as<float>();
        if (guidance["max_optical_angle_deg"])
            config.guidance.max_optical_angle_deg =
                guidance["max_optical_angle_deg"].as<float>();
        if (guidance["input_voltage_range_v"])
            config.guidance.input_voltage_range_v =
                guidance["input_voltage_range_v"].as<float>();
        if (guidance["dac_voltage_range_v"])
            config.guidance.dac_voltage_range_v =
                guidance["dac_voltage_range_v"].as<float>();
        if (guidance["voltage_use_ekf_center"])
            config.guidance.voltage_use_ekf_center =
                guidance["voltage_use_ekf_center"].as<bool>();
        if (guidance["voltage_limit_v"])
            config.guidance.voltage_limit_v = guidance["voltage_limit_v"].as<float>();

        if (const YAML::Node geom_list = guidance["target_geometry"]) {
            config.guidance.target_geometry.clear();
            for (const auto& item : geom_list) {
                TargetGeometry geom;
                if (item["class_id"]) geom.class_id = item["class_id"].as<int>();
                if (item["width_mm"]) geom.width_mm = item["width_mm"].as<float>();
                if (item["height_mm"]) geom.height_mm = item["height_mm"].as<float>();
                config.guidance.target_geometry.push_back(geom);
            }
        }

        if (const YAML::Node wiring = guidance["wiring"]) {
            if (wiring["mode"]) {
                const auto mode_str = to_lower_copy(wiring["mode"].as<std::string>());
                if (mode_str == "differential")
                    config.guidance.wiring.mode = GalvoWiringMode::differential;
                else if (mode_str == "single_ended")
                    config.guidance.wiring.mode = GalvoWiringMode::single_ended;
            }
            if (wiring["x_plus_channel"])
                config.guidance.wiring.x_plus_channel =
                    wiring["x_plus_channel"].as<int>();
            if (wiring["x_minus_channel"])
                config.guidance.wiring.x_minus_channel =
                    wiring["x_minus_channel"].as<int>();
            if (wiring["y_plus_channel"])
                config.guidance.wiring.y_plus_channel =
                    wiring["y_plus_channel"].as<int>();
            if (wiring["y_minus_channel"])
                config.guidance.wiring.y_minus_channel =
                    wiring["y_minus_channel"].as<int>();
        }

        if (guidance["scan_mode"]) {
            const auto mode = to_lower_copy(guidance["scan_mode"].as<std::string>());
            if (mode == "rectangle")
                config.guidance.scan_mode = ScanMode::rectangle;
        }
        if (guidance["scan_width_deg"])
            config.guidance.scan_width_deg = guidance["scan_width_deg"].as<float>();
        if (guidance["scan_height_deg"])
            config.guidance.scan_height_deg = guidance["scan_height_deg"].as<float>();
        if (guidance["scan_grid_n"])
            config.guidance.scan_grid_n = guidance["scan_grid_n"].as<int>();
        if (guidance["calib_mode"])
            config.guidance.calib_mode = guidance["calib_mode"].as<bool>();
        if (guidance["calib_angle_x_deg"])
            config.guidance.calib_angle_x_deg = guidance["calib_angle_x_deg"].as<float>();
        if (guidance["calib_angle_y_deg"])
            config.guidance.calib_angle_y_deg = guidance["calib_angle_y_deg"].as<float>();
    }

    if (config.v4l2.device_path.empty())
        throw std::runtime_error("v4l2.device_path must not be empty");
    if (config.v4l2.width <= 0) throw std::runtime_error("v4l2.width must be positive");
    if (config.v4l2.height <= 0) throw std::runtime_error("v4l2.height must be positive");
    if (config.v4l2.framerate <= 0.0F) throw std::runtime_error("v4l2.framerate must be positive");
    if (config.runtime.max_input_age_ms <= 0)
        throw std::runtime_error("runtime.max_input_age_ms must be positive");
    if (config.runtime.max_observation_age_ms <= 0)
        throw std::runtime_error("runtime.max_observation_age_ms must be positive");
    if (config.runtime.max_infer_fps <= 0)
        throw std::runtime_error("runtime.max_infer_fps must be positive");
    if (config.runtime.warmup_frames < 0)
        throw std::runtime_error("runtime.warmup_frames must be non-negative");
    if (config.runtime.hit_confirm_frames <= 0)
        throw std::runtime_error("runtime.hit_confirm_frames must be positive");
    if (config.runtime.hit_release_frames <= 0)
        throw std::runtime_error("runtime.hit_release_frames must be positive");
    if (config.runtime.debug_max_fps <= 0)
        throw std::runtime_error("runtime.debug_max_fps must be positive");
    if (config.runtime.record_queue_size <= 0)
        throw std::runtime_error("runtime.record_queue_size must be positive");

    return config;
}

} // namespace rmcs_laser_guidance
