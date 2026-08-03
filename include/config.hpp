#pragma once

#include <filesystem>
#include <vector>

namespace rmcs_laser_guidance {

enum class CaptureBackendKind : int {
    v4l2 = 0,
    hikcamera = 1,
};

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
    int width = 1920;
    int height = 1080;
    float framerate = 60.0F;
    V4l2PixelFormat pixel_format = V4l2PixelFormat::mjpeg;
    bool invert_image = false;
};

struct DebugConfig {
    bool show_window = true;
    bool draw_overlay = true;
};

struct RuntimeConfig {
    int max_input_age_ms = 25;
    // Observation budget: inference time plus queue wait before a result is
    // considered too old to drive aiming. Must leave headroom for the real
    // inference latency (5MP frames), or every result gets dropped and the
    // runtime silently stops tracking.
    int max_observation_age_ms = 50;
    int max_infer_fps = 60;
    int warmup_frames = 30;
    std::filesystem::path engine_path{};
    int hit_confirm_frames = 3;
    int hit_release_frames = 5;
    bool debug_enabled = false;
    int debug_max_fps = 30;
    bool record_enabled = false;
    int record_queue_size = 16;
};

struct InferenceConfig {
    InferenceBackendKind backend = InferenceBackendKind::bright_spot;
    std::filesystem::path model_path{};
    int enemy_class_id = -1;
};

struct RtpConfig {
    bool enabled = false;
    std::string host = "127.0.0.1";
    int port = 5002;
    std::filesystem::path sdp_path = "/tmp/laser_guidance.sdp";
    std::string encoder = "h264_nvenc";
    std::string bitrate = "16M";
    // 0 = full source resolution / camera rate (no downscale or fps cap).
    int max_width = 0;
    int max_fps = 0;
};

struct UdpConfig {
    bool enabled = false;
    std::string host = "127.0.0.1";
    int port = 5002;
};

struct ZmqConfig {
    bool enabled = false;
    std::string host = "127.0.0.1";
    int port = 5555;
};

struct RefereeConfig {
    bool enabled = false;
    std::string zmq_address = "tcp://127.0.0.1:5558";
    int match_duration_s = 420;
    int signal_timeout_s = 5;   // 看门狗：超过该秒数未收到合法消息 → signal_stale
};

struct EkfConfig {
    bool enabled = true;
    double process_noise_q = 0.05;
    double measurement_noise_r = 0.5;
    double initial_pos_std = 100.0;
    double initial_vel_std = 100.0;
    double initial_acc_std = 50.0;
    int max_missed_frames = 5;
    double lookahead_ms = 10.0;
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
    std::vector<TargetGeometry> target_geometry{};
    std::filesystem::path camera_calib_path{};
    std::filesystem::path voltage_model_path{};
    // t_*_mm: galvo origin in camera frame (mm). r_*_deg: Euler for R=Rz(-rz)Ry(-rx)Rx(-ry).
    float t_x_mm = 0.0F;
    float t_y_mm = 0.0F;
    float t_z_mm = 0.0F;
    float r_x_deg = 0.0F;
    float r_y_deg = 0.0F;
    float r_z_deg = 0.0F;
    float mirror_separation_mm = 15.0F;  // dual-mirror axial gap (mm)
    float max_optical_angle_deg = 30.0F;
    float input_voltage_range_v = 5.0F;
    float dac_voltage_range_v = 10.0F;
    bool voltage_use_ekf_center = true;
    float voltage_limit_v = 5.0F;
    float voltage_offset_vx = 0.0F;
    float voltage_offset_vy = 0.0F;
    float depth_scale = 1.0F;
    bool depth_filter_enabled = true;
    double depth_process_noise_q = 400.0;
    double depth_measurement_noise_r = 4000000.0;
    double depth_initial_pos_std = 5000.0;
    double depth_initial_vel_std = 2000.0;
    int depth_max_missed_frames = 5;
    float voltage_gain_x = 1.0F;
    float voltage_gain_y = 1.0F;
    float angle_offset_x_deg = 0.0F;
    float angle_offset_y_deg = 0.0F;
    GalvoWiringConfig wiring{};
    ScanMode scan_mode = ScanMode::single;
    float scan_width_deg = 1.0F;
    float scan_height_deg = 0.8F;
    int scan_grid_n = 10;
    bool calib_mode = false;
    float calib_angle_x_deg = 0.0F;
    float calib_angle_y_deg = 0.0F;
};

struct HikRuntimeProfile {
    float exposure_us = 2000.0F;
    float gain = 16.9807F;
    float framerate = 80.0F;
    bool set_white_balance = false;
    bool white_balance_off = false;
    int white_balance_ratio_red = 1024;
    int white_balance_ratio_green = 1024;
    int white_balance_ratio_blue = 1024;
};

enum class HikProfileKind { lit, unlit };

struct HikCameraConfig {
    std::string device_id{};
    unsigned int timeout_ms = 2000;
    float exposure_us = 2000.0F;
    float framerate = 80.0F;
    float gain = 16.9807F;
    bool invert_image = false;
    bool software_sync = false;
    bool trigger_mode = false;
    bool fixed_framerate = true;
    bool has_unlit_profile = false;
    HikRuntimeProfile unlit{};
    bool set_white_balance = false;
    bool white_balance_off = false;
    int white_balance_ratio_red = 1024;
    int white_balance_ratio_green = 1024;
    int white_balance_ratio_blue = 1024;
    HikProfileKind startup_profile_kind = HikProfileKind::lit;

    [[nodiscard]] auto lit_profile() const -> HikRuntimeProfile {
        return HikRuntimeProfile{
            .exposure_us = exposure_us,
            .gain = gain,
            .framerate = framerate,
            .set_white_balance = set_white_balance,
            .white_balance_off = white_balance_off,
            .white_balance_ratio_red = white_balance_ratio_red,
            .white_balance_ratio_green = white_balance_ratio_green,
            .white_balance_ratio_blue = white_balance_ratio_blue,
        };
    }
};

struct Config {
    CaptureBackendKind capture_backend = CaptureBackendKind::v4l2;
    V4l2Config v4l2{};
    HikCameraConfig hik{};
    DebugConfig debug{};
    RuntimeConfig runtime{};
    InferenceConfig inference{};
    RtpConfig rtp{};
    UdpConfig udp{};
    ZmqConfig zmq{};
    EkfConfig ekf{};
    RefereeConfig referee{};
    GuidanceConfig guidance{};
};

auto load_config(const std::filesystem::path& config_path) -> Config;

} // namespace rmcs_laser_guidance
