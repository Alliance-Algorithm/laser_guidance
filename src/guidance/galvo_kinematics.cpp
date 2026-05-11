#include "guidance/galvo_kinematics.hpp"

#include <cmath>

namespace rmcs_laser_guidance {

GalvoKinematics::GalvoKinematics(const GuidanceConfig& config)
    : config_(config) {}

auto GalvoKinematics::compute(const cv::Point3f& P_camera_mm) const -> GalvoAngles {
    const float z_c = P_camera_mm.z;
    if (z_c <= 0.0F) return {};

    const float dx = P_camera_mm.x - config_.t_x_mm;
    const float dy = P_camera_mm.y - config_.t_y_mm;
    const float dz = P_camera_mm.z - config_.t_z_mm;

    constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;
    const float rx = config_.r_x_deg * kDegToRad;
    const float ry = config_.r_y_deg * kDegToRad;
    const float rz = config_.r_z_deg * kDegToRad;

    const float cx = std::cos(rx); const float sx = std::sin(rx);
    const float cy = std::cos(ry); const float sy = std::sin(ry);
    const float cz = std::cos(rz); const float sz = std::sin(rz);

    const float x1 = dx;
    const float y1 = cy * dy + sy * dz;
    const float z1 = -sy * dy + cy * dz;

    const float x2 = cx * x1 - sx * z1;
    const float y2 = y1;
    const float z2 = sx * x1 + cx * z1;

    const float x_g = cz * x2 + sz * y2;
    const float y_g = -sz * x2 + cz * y2;
    const float z_g = z2;

    if (z_g <= 0.0F) return {};

    const float d = config_.mirror_separation_mm;

    // Dual-axis galvo with mirror separation d:
    // θy = 0.5 * atan2(Y_g, Z_g + d)
    // θx = 0.5 * atan2(X_g, sqrt(Y_g² + (Z_g + d)²))
    const float z_eff = z_g + d;
    const float r_yz = std::sqrt(y_g * y_g + z_eff * z_eff);

    const float theta_x_mech_rad = 0.5F * std::atan2(x_g, r_yz);
    const float theta_y_mech_rad = 0.5F * std::atan2(y_g, z_eff);

    // Mechanical angle → optical angle (×2)
    constexpr float kRadToDeg = 180.0F / 3.14159265358979323846F;
    const float theta_x_opt_deg = 2.0F * theta_x_mech_rad * kRadToDeg;
    const float theta_y_opt_deg = 2.0F * theta_y_mech_rad * kRadToDeg;

    return {
        .theta_x_optical_deg = theta_x_opt_deg,
        .theta_y_optical_deg = theta_y_opt_deg,
        .valid = true,
    };
}

} // namespace rmcs_laser_guidance
