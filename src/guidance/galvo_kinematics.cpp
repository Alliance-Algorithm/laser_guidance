#include "guidance/galvo_kinematics.hpp"

namespace rmcs_laser_guidance {

GalvoKinematics::GalvoKinematics(const GuidanceConfig& config)
    : geometry_(config.t_x_mm, config.t_y_mm, config.t_z_mm,
                config.r_x_deg, config.r_y_deg, config.r_z_deg,
                config.mirror_separation_mm) {}

auto GalvoKinematics::compute(const cv::Point3f& P_camera_mm) const -> GalvoAngles {
    Eigen::Vector3f P_c(P_camera_mm.x, P_camera_mm.y, P_camera_mm.z);
    return geometry_.solve_angles(P_c);
}

} // namespace rmcs_laser_guidance
