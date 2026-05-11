#include "guidance/depth_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace rmcs_laser_guidance {

DepthEstimator::DepthEstimator(const GuidanceConfig& config, const cv::Mat& camera_matrix)
    : config_(config), camera_matrix_(camera_matrix) {}

auto DepthEstimator::estimate(const ModelCandidate& candidate) const -> std::optional<float> {
    const float fx = static_cast<float>(camera_matrix_.at<double>(0, 0));
    if (fx <= 0.0F) return std::nullopt;

    const int class_id = candidate.class_id;
    float physical_width_mm = 150.0F;

    for (const auto& geom : config_.target_geometry) {
        if (geom.class_id == class_id) {
            physical_width_mm = geom.width_mm;
            break;
        }
    }

    const float pixel_size = std::max(candidate.bbox.width, candidate.bbox.height);
    if (pixel_size <= 0.0F) return std::nullopt;

    const float depth_mm = fx * physical_width_mm / pixel_size;
    if (depth_mm <= 0.0F) return std::nullopt;

    return depth_mm;
}

} // namespace rmcs_laser_guidance
