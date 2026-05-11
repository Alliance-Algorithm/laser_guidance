#pragma once

#include <opencv2/core/types.hpp>

#include "config.hpp"

namespace rmcs_laser_guidance {

struct GalvoAngles {
    float theta_x_optical_deg = 0.0F;
    float theta_y_optical_deg = 0.0F;
    bool valid = false;
};

class GalvoKinematics {
public:
    explicit GalvoKinematics(const GuidanceConfig& config);

    auto compute(const cv::Point3f& P_camera_mm) const -> GalvoAngles;

private:
    GuidanceConfig config_;
};

} // namespace rmcs_laser_guidance
