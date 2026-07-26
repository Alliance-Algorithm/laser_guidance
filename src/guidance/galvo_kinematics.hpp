#pragma once

#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "guidance/camera_galvo_geometry.hpp"

namespace rmcs_laser_guidance {

class GalvoKinematics {
public:
    explicit GalvoKinematics(const GuidanceConfig& config);

    auto compute(const cv::Point3f& P_camera_mm) const -> GalvoAngles;

private:
    CameraGalvoGeometry geometry_;
};

} // namespace rmcs_laser_guidance
