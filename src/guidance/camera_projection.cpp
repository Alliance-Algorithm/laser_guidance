#include "guidance/camera_projection.hpp"

#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace rmcs_laser_guidance {

CameraProjection::CameraProjection(cv::Mat camera_matrix, cv::Mat dist_coeffs)
    : camera_matrix_(std::move(camera_matrix))
    , dist_coeffs_(std::move(dist_coeffs)) {}

auto CameraProjection::project(const cv::Point2f& pixel, float depth_mm) const -> cv::Point3f {
    if (depth_mm <= 0.0F) return { -1.0F, -1.0F, -1.0F };

    std::vector<cv::Point2f> src{ pixel };
    std::vector<cv::Point2f> dst;

    if (!dist_coeffs_.empty()) {
        cv::undistortPoints(src, dst, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
    } else {
        dst = src;
    }

    const float fx = static_cast<float>(camera_matrix_.at<double>(0, 0));
    const float fy = static_cast<float>(camera_matrix_.at<double>(1, 1));
    const float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
    const float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));

    const float u = dst[0].x;
    const float v = dst[0].y;

    return {
        (u - cx) * depth_mm / fx,
        (v - cy) * depth_mm / fy,
        depth_mm,
    };
}

} // namespace rmcs_laser_guidance
