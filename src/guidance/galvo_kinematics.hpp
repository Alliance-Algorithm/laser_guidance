#pragma once

#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "guidance/camera_galvo_geometry.hpp"

namespace rmcs_laser_guidance {

class GalvoKinematics {
public:
    explicit GalvoKinematics(const GuidanceConfig& config);

    // Depth-based (current): P_camera in mm
    auto compute(const cv::Point3f& P_camera_mm) const -> GalvoAngles;

    // Direction-based (no depth): pixel in image coordinates
    auto compute_from_direction(const cv::Point2f& pixel, const cv::Mat& K) const -> GalvoAngles;

private:
    CameraGalvoGeometry geometry_;
};

} // namespace rmcs_laser_guidance
