#pragma once

#include <array>
#include <filesystem>
#include <vector>

namespace rmcs_laser_guidance {

enum class V4l2PixelFormat {
    mjpeg = 0,
    yuyv,
    bgr24,
};

enum class InferenceBackendKind {
    bright_spot = 0,
    model,
    tensorrt,
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
    int enemy_class_id = -1;
};

struct RtpConfig {
    bool enabled = false;
    std::string host = "127.0.0.1";
    int port = 5002;
    std::filesystem::path sdp_path = "/tmp/laser_guidance.sdp";
    std::string encoder = "h264_nvenc";
    std::string bitrate = "8M";
};

struct UdpConfig {
    bool enabled = false;
    std::string host = "127.0.0.1";
    int port = 5001;
};

struct EkfConfig {
    bool enabled = true;
    double process_noise_q     = 0.05;
    double measurement_noise_r = 0.5;
    double initial_pos_std     = 100.0;
    double initial_vel_std     = 100.0;
    double initial_acc_std     = 50.0;
    int max_missed_frames      = 5;
    double lookahead_ms        = 12.0;
};

enum class GalvoWiringMode : int {
    differential = 0,
    single_ended,
};

enum class ScanMode : int {
    single = 0,
    rectangle,
};

enum class GuidanceCommandModelKind : int {
    geometry = 0,
    direct_voltage,
};

struct TargetGeometry {
    int class_id = 0;
    float width_mm = 150.0F;
    float height_mm = 150.0F;
};

struct GalvoWiringConfig {
    GalvoWiringMode mode = GalvoWiringMode::differential;
    int x_plus_channel = 0;
    int x_minus_channel = 2;
    int y_plus_channel = 1;
    int y_minus_channel = 3;
};

struct GuidanceConfig {
    bool enabled = false;
    GuidanceCommandModelKind command_model = GuidanceCommandModelKind::geometry;
    std::vector<TargetGeometry> target_geometry {};
    std::filesystem::path camera_calib_path {};
    std::filesystem::path voltage_model_path {};
    float t_x_mm = 0.0F;
    float t_y_mm = 0.0F;
    float t_z_mm = 0.0F;
    float r_x_deg = 0.0F;
    float r_y_deg = 0.0F;
    float r_z_deg = 0.0F;
    float mirror_separation_mm = 15.0F;
    float max_optical_angle_deg = 30.0F;
    float input_voltage_range_v = 5.0F;
    float dac_voltage_range_v = 10.0F;
    bool voltage_use_ekf_center = true;
    float voltage_limit_v = 5.0F;
    GalvoWiringConfig wiring {};
    ScanMode scan_mode = ScanMode::single;
    float scan_width_deg = 1.0F;
    float scan_height_deg = 0.8F;
    int scan_grid_n = 10;
    bool calib_mode = false;
    float calib_angle_x_deg = 0.0F;
    float calib_angle_y_deg = 0.0F;
};

struct Config {
    V4l2Config v4l2 { };
    DebugConfig debug { };
    RuntimeConfig runtime { };
    InferenceConfig inference { };
    RtpConfig rtp { };
    UdpConfig udp { };
    EkfConfig ekf { };
    GuidanceConfig guidance { };
};

auto load_config(const std::filesystem::path& config_path) -> Config;

} // namespace rmcs_laser_guidance
