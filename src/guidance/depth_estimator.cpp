#include "guidance/depth_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace rmcs_laser_guidance {

DepthEstimator::DepthEstimator(const GuidanceConfig& config, const cv::Mat& camera_matrix)
    : config_(config)
    , camera_matrix_(camera_matrix) {}

auto DepthEstimator::estimate(const ModelCandidate& candidate) const -> std::optional<float> {
    const float fx = static_cast<float>(camera_matrix_.at<double>(0, 0));
    if (fx <= 0.0F)
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

    // The laser detection module is an octagonal prism mounted coaxially on a
    // vertical rigid pole (RM2026 rulebook S189/S191/S192: cannot move relative
    // to the airframe, cannot be inverted). Its axis stays vertical in world
    // space, so the horizontal pixel extent (bbox.width) tracks the module's
    // ~50mm outer diameter and is only affected by yaw/azimuth (±~19% across
    // the 8 faces). The vertical extent (bbox.height, module's 72mm height)
    // shortens with camera pitch/elevation via standard foreshortening, so
    // using it (or max(w,h), which picks height at low pitch) as the depth
    // divisor overestimates depth as pitch increases. Always use bbox.width.
    const float pixel_size = candidate.bbox.width;
    if (pixel_size <= 0.0F)
        return std::nullopt;

    const float depth_mm = fx * physical_width_mm / pixel_size;
    if (depth_mm <= 0.0F)
        return std::nullopt;

    // Aspect-ratio sanity gate: if w/h deviates strongly from expected
    // physical ratio, the target may be at a steep pitch angle where width
    // also begins to foreshorten. In that regime, inflate the variance
    // annotation so the depth filter can assign lower confidence.
    const float bw = candidate.bbox.width;
    const float bh = candidate.bbox.height;
    if (bh > 0.0F && physical_height_mm > 0.0F) {
        const float expected_ratio = physical_width_mm / physical_height_mm;
        const float observed_ratio = bw / bh;
        const float ratio_err = std::abs(observed_ratio - expected_ratio) / expected_ratio;
        if (ratio_err > 0.3F) {
            // Severe aspect mismatch: halve confidence via depth_scale
            return depth_mm * config_.depth_scale * 0.7F;
        }
    }

    return depth_mm * config_.depth_scale;
}

} // namespace rmcs_laser_guidance
