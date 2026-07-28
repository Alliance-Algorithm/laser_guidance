#include "guidance/galvo_kinematics.hpp"

#include <opencv2/core/mat.hpp>

namespace rmcs_laser_guidance {

GalvoKinematics::GalvoKinematics(const GuidanceConfig& config)
    : geometry_(config.t_x_mm, config.t_y_mm, config.t_z_mm,
                config.r_x_deg, config.r_y_deg, config.r_z_deg,
                config.mirror_separation_mm) {}

auto GalvoKinematics::compute(const cv::Point3f& P_camera_mm) const -> GalvoAngles {
    Eigen::Vector3f P_c(P_camera_mm.x, P_camera_mm.y, P_camera_mm.z);
    return geometry_.solve_angles(P_c);
}

auto GalvoKinematics::compute_from_direction(const cv::Point2f& pixel, const cv::Mat& K) const -> GalvoAngles {
    const float fx = static_cast<float>(K.at<double>(0, 0));
    const float fy = static_cast<float>(K.at<double>(1, 1));
    const float cx = static_cast<float>(K.at<double>(0, 2));
    const float cy = static_cast<float>(K.at<double>(1, 2));
    Eigen::Vector3f d;
    d.x() = (pixel.x - cx) / fx;
    d.y() = (pixel.y - cy) / fy;
    d.z() = 1.0F;
    return geometry_.solve_angles_from_direction(d);
}

} // namespace rmcs_laser_guidance
