#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace rmcs_laser_guidance::calibration {

inline constexpr std::string_view kCalibrationCsvHeader =
    "theta_x_deg,theta_y_deg,pixel_x,pixel_y,depth_mm,depth_source,depth_sigma_mm";

enum class DepthSource { rangefinder, bbox };

struct CalibrationRecord {
    std::size_t line_number = 0;
    double theta_x_deg = 0.0;
    double theta_y_deg = 0.0;
    cv::Point2d pixel{};
    double depth_mm = 0.0;
    DepthSource depth_source = DepthSource::bbox;
    double depth_sigma_mm = 0.0;
};

struct RecordLoadResult {
    std::vector<CalibrationRecord> records{};
    std::vector<std::string> errors{};
};

struct CameraIntrinsics {
    int image_width = 0;
    int image_height = 0;
    cv::Mat camera_matrix{};
    cv::Mat dist_coeffs{};
};

struct PredictedAngles {
    double x_rad = 0.0;
    double y_rad = 0.0;
    bool valid = false;
};

struct WahbaResult {
    Eigen::Quaterniond rotation_galvo_from_camera = Eigen::Quaterniond::Identity();
    Eigen::Vector3d singular_values = Eigen::Vector3d::Zero();
    double coverage_ratio = 0.0;
    bool observable = false;
};

enum class InitialRotation { config, wahba };

struct CalibrationOptions {
    Eigen::Quaterniond configured_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d fixed_translation_camera_mm = Eigen::Vector3d::Zero();
    double fixed_mirror_separation_mm = 15.0;
    InitialRotation initial_rotation = InitialRotation::config;
    double rangefinder_weight = 1.0;
    double bbox_weight = 0.25;
    double angle_sigma_floor_rad = 0.02 * 3.14159265358979323846 / 180.0;
    double huber_delta = 2.5;
};

struct ResidualStats {
    std::size_t count = 0;
    double rms_deg = 0.0;
    double p50_deg = 0.0;
    double p95_deg = 0.0;
    double max_deg = 0.0;
    std::size_t worst_line = 0;
};

struct CalibrationResult {
    bool usable = false;
    std::string message{};
    Eigen::Quaterniond initial_rotation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond optimized_rotation = Eigen::Quaterniond::Identity();
    WahbaResult wahba{};
    ResidualStats rangefinder{};
    ResidualStats bbox{};
    std::string solver_report{};
};

auto load_calibration_records(const std::filesystem::path& path) -> RecordLoadResult;
auto load_camera_intrinsics(const std::filesystem::path& path) -> CameraIntrinsics;
auto camera_direction(const cv::Point2d& pixel, const CameraIntrinsics& intrinsics)
    -> Eigen::Vector3d;
auto camera_point(const CalibrationRecord& record, const CameraIntrinsics& intrinsics)
    -> Eigen::Vector3d;
auto predict_angles(
    const Eigen::Quaterniond& rotation_galvo_from_camera,
    const Eigen::Vector3d& translation_camera_mm, double mirror_separation_mm,
    const Eigen::Vector3d& point_camera_mm) -> PredictedAngles;
auto quaternion_from_config_euler(double r_x_deg, double r_y_deg, double r_z_deg)
    -> Eigen::Quaterniond;
auto config_euler_from_quaternion(const Eigen::Quaterniond& rotation) -> Eigen::Vector3d;
auto solve_wahba(
    const std::vector<Eigen::Vector3d>& camera_directions,
    const std::vector<Eigen::Vector3d>& galvo_directions, const std::vector<double>& weights)
    -> WahbaResult;
auto calibrate_rotation(
    const std::vector<CalibrationRecord>& records, const CameraIntrinsics& intrinsics,
    const CalibrationOptions& options) -> CalibrationResult;

} // namespace rmcs_laser_guidance::calibration
