#pragma once

#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <string>
#include <string_view>

namespace rmcs_laser_guidance::runtime_internal {

struct GuidanceCalibrationState {
    float angle_x_deg = 0.0F;
    float angle_y_deg = 0.0F;
    float angle_step_deg = 0.01F;
    float voltage_x = 0.0F;
    float voltage_y = 0.0F;
    float voltage_step_v = 0.005F;

    static constexpr float kMinAngleStepDeg = 0.001F;
    static constexpr float kMaxAngleStepDeg = 0.5F;
    static constexpr float kMinVoltageStepV = 0.001F;
    static constexpr float kMaxVoltageStepV = 0.5F;
};

inline constexpr std::string_view kGeometryCalibrationCsvHeader =
    "theta_x_deg,theta_y_deg,pixel_x,pixel_y,depth_mm,depth_source,depth_sigma_mm";

inline auto geometry_calibration_csv_header() -> std::string {
    return std::string(kGeometryCalibrationCsvHeader) + '\n';
}

inline auto geometry_calibration_file_is_compatible(const std::filesystem::path& path) -> bool {
    std::error_code error;
    if (!std::filesystem::exists(path, error) || error)
        return !error;
    if (std::filesystem::file_size(path, error) == 0 || error)
        return !error;
    std::ifstream input(path);
    std::string header;
    return input && std::getline(input, header) && header == kGeometryCalibrationCsvHeader;
}

inline auto format_bbox_geometry_calibration_record(
    double angle_x_deg, double angle_y_deg, double pixel_x, double pixel_y, double depth_mm)
    -> std::string {
    const double depth_sigma_mm = std::max(500.0, depth_mm * 0.08);
    return std::format(
        "{:.6f},{:.6f},{:.6f},{:.6f},{:.3f},bbox,{:.3f}\n", angle_x_deg, angle_y_deg, pixel_x,
        pixel_y, depth_mm, depth_sigma_mm);
}

} // namespace rmcs_laser_guidance::runtime_internal
