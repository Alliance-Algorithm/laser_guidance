#include "guidance/camera_galvo_geometry.hpp"

#include <cmath>

namespace rmcs_laser_guidance {

CameraGalvoGeometry::CameraGalvoGeometry(
    float t_x_mm, float t_y_mm, float t_z_mm,
    float r_x_deg, float r_y_deg, float r_z_deg,
    float mirror_separation_mm)
    : mirror_sep_mm_(mirror_separation_mm) {

    constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;

    rot_ = Eigen::AngleAxisf(-r_z_deg * kDegToRad, Eigen::Vector3f::UnitZ())
         * Eigen::AngleAxisf(-r_x_deg * kDegToRad, Eigen::Vector3f::UnitY())
         * Eigen::AngleAxisf(-r_y_deg * kDegToRad, Eigen::Vector3f::UnitX());

    trans_ = {t_x_mm, t_y_mm, t_z_mm};
}

auto CameraGalvoGeometry::camera_to_galvo(Eigen::Vector3f P_camera) const -> Eigen::Vector3f {
    return rot_ * (P_camera - trans_);
}

auto CameraGalvoGeometry::solve_angles(Eigen::Vector3f P_camera) const -> GalvoAngles {
    if (P_camera.z() <= 0.0F)
        return {};

    Eigen::Vector3f P_g = camera_to_galvo(P_camera);

    if (P_g.z() <= 0.0F)
        return {};

    const float d = mirror_sep_mm_;
    const float z_eff = P_g.z() + d;
    const float r_yz = std::sqrt(P_g.y() * P_g.y() + z_eff * z_eff);

    const float theta_x_mech_rad = 0.5F * std::atan2(P_g.x(), r_yz);
    const float theta_y_mech_rad = 0.5F * std::atan2(P_g.y(), z_eff);

    constexpr float kRadToDeg = 180.0F / 3.14159265358979323846F;
    return {
        .theta_x_optical_deg = 2.0F * theta_x_mech_rad * kRadToDeg,
        .theta_y_optical_deg = 2.0F * theta_y_mech_rad * kRadToDeg,
        .valid = true,
    };
}

auto CameraGalvoGeometry::solve_angles_from_direction(Eigen::Vector3f d_cam) const -> GalvoAngles {
    if (d_cam.z() <= 0.0F)
        return {};

    // d_gal = direction in galvo frame (far-field: translation negligible)
    Eigen::Vector3f d_gal = rot_ * d_cam.normalized();

    if (d_gal.z() <= 0.0F)
        return {};

    // Direction → optical angles.
    // Mirror model: θ_opt = atan2(Y, Z+d) = atan2(d_y, d_z) for d/λ→0.
    constexpr float kRadToDeg = 180.0F / 3.14159265358979323846F;
    return {
        .theta_x_optical_deg = kRadToDeg * std::atan2(d_gal.x(), std::sqrt(d_gal.y() * d_gal.y() + d_gal.z() * d_gal.z())),
        .theta_y_optical_deg = kRadToDeg * std::atan2(d_gal.y(), d_gal.z()),
        .valid = true,
    };
}

} // namespace rmcs_laser_guidance
