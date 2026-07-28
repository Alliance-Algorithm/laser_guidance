#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace rmcs_laser_guidance {

struct GalvoAngles {
    float theta_x_optical_deg = 0.0F;
    float theta_y_optical_deg = 0.0F;
    bool valid = false;
};

class CameraGalvoGeometry {
public:
    CameraGalvoGeometry(float t_x_mm, float t_y_mm, float t_z_mm,
                        float r_x_deg, float r_y_deg, float r_z_deg,
                        float mirror_separation_mm);

    auto camera_to_galvo(Eigen::Vector3f P_camera) const -> Eigen::Vector3f;

    auto solve_angles(Eigen::Vector3f P_camera) const -> GalvoAngles;

    [[nodiscard]] auto rotation() const -> const Eigen::Quaternionf& { return rot_; }
    [[nodiscard]] auto translation() const -> const Eigen::Vector3f& { return trans_; }
    [[nodiscard]] auto mirror_sep() const -> float { return mirror_sep_mm_; }

private:
    Eigen::Quaternionf rot_;
    Eigen::Vector3f trans_;
    float mirror_sep_mm_;
};

} // namespace rmcs_laser_guidance
