#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs_laser_guidance {

class CameraProjection {
public:
    CameraProjection(cv::Mat camera_matrix, cv::Mat dist_coeffs);

    auto project(const cv::Point2f& pixel, float depth_mm) const -> cv::Point3f;

    [[nodiscard]] auto camera_matrix() const -> const cv::Mat& { return camera_matrix_; }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

} // namespace rmcs_laser_guidance
