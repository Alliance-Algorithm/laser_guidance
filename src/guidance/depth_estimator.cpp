#include "guidance/depth_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace rmcs_laser_guidance {

DepthEstimator::DepthEstimator(const GuidanceConfig& config, const cv::Mat& camera_matrix)
    : config_(config)
    , camera_matrix_(camera_matrix) {}

auto DepthEstimator::estimate(const ModelCandidate& candidate) const -> std::optional<float> {
    const float fx = static_cast<float>(camera_matrix_.at<double>(0, 0));
    const float fy = static_cast<float>(camera_matrix_.at<double>(1, 1));
    if (fx <= 0.0F || fy <= 0.0F)
        return std::nullopt;

    const int class_id = candidate.class_id;
    float physical_width_mm = 150.0F;
    float physical_height_mm = 150.0F;

    for (const auto& geom : config_.target_geometry) {
        if (geom.class_id == class_id) {
            physical_width_mm = geom.width_mm;
            physical_height_mm = geom.height_mm;
            break;
        }
    }

    // Area-based depth: depth² = fx·fy·W·H / (w·h)
    // Uses both bbox dimensions as a product → ~30% lower relative noise
    // than width-only. Pitch-induced bias at 30° is ~7% (vs 0% for width-only
    // but 15% for height-only); at typical long-range pitch this is negligible
    // compared to the detection noise reduction.
    const float bbox_area = candidate.bbox.width * candidate.bbox.height;
    if (bbox_area <= 0.0F)
        return std::nullopt;

    const float depth_sq = fx * fy * physical_width_mm * physical_height_mm / bbox_area;
    if (depth_sq <= 0.0F)
        return std::nullopt;

    return std::sqrt(depth_sq) * config_.depth_scale;
}

} // namespace rmcs_laser_guidance
