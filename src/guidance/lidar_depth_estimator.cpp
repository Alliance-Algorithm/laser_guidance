#include "guidance/lidar_depth_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <queue>

namespace rmcs_laser_guidance {
namespace {

auto bbox_contains(const cv::Rect2f& bbox, const cv::Point2f& p) -> bool {
    return p.x >= bbox.x && p.x <= bbox.x + bbox.width
        && p.y >= bbox.y && p.y <= bbox.y + bbox.height;
}

auto sqr(float x) -> float { return x * x; }

auto clamp01(float x) -> float {
    return std::clamp(x, 0.0F, 1.0F);
}

} // namespace

LidarDepthEstimator::LidarDepthEstimator(const GuidanceConfig& config)
    : config_(config) {}

auto LidarDepthEstimator::estimate(const ModelCandidate& candidate, const LidarFrame& frame) const
    -> std::optional<float> {
    if (frame.points.empty()) {
        update_debug_info(0, {}, std::nullopt);
        return std::nullopt;
    }

    const auto roi_indices = collect_roi_points(candidate, frame);
    if (roi_indices.size() < static_cast<std::size_t>(config_.lidar_min_cluster_points)) {
        update_debug_info(roi_indices.size(), {}, std::nullopt);
        return std::nullopt;
    }

    const auto clusters = cluster_points(frame, roi_indices);
    if (clusters.empty()) {
        update_debug_info(roi_indices.size(), {}, std::nullopt);
        return std::nullopt;
    }

    const auto target = choose_target_cluster(candidate, clusters);
    update_debug_info(roi_indices.size(), clusters, target);
    if (!target) return std::nullopt;
    if (target->mean_depth_mm <= 0.0F || target->mean_depth_mm > config_.lidar_max_depth_mm) {
        return std::nullopt;
    }
    return target->median_depth_mm > 0.0F ? target->median_depth_mm : target->mean_depth_mm;
}

auto LidarDepthEstimator::last_debug_info() const -> DebugInfo {
    std::scoped_lock lock(debug_mutex_);
    return last_debug_;
}

auto LidarDepthEstimator::collect_roi_points(const ModelCandidate& candidate,
                                             const LidarFrame& frame) const
    -> std::vector<std::size_t> {
    std::vector<std::size_t> indices;
    indices.reserve(frame.points.size());

    cv::Rect2f roi = candidate.bbox;
    roi.x -= config_.lidar_bbox_margin_px;
    roi.y -= config_.lidar_bbox_margin_px;
    roi.width += config_.lidar_bbox_margin_px * 2.0F;
    roi.height += config_.lidar_bbox_margin_px * 2.0F;

    for (std::size_t i = 0; i < frame.points.size(); ++i) {
        const auto& point = frame.points[i];
        if (!std::isfinite(point.z_mm) || point.z_mm <= 0.0F || point.z_mm > config_.lidar_max_depth_mm) {
            continue;
        }
        if (point.col < 0 || point.row < 0) continue;

        const cv::Point2f image_point {
            static_cast<float>(point.col),
            static_cast<float>(point.row),
        };
        if (!bbox_contains(roi, image_point)) continue;
        indices.push_back(i);
    }

    return indices;
}

auto LidarDepthEstimator::cluster_points(const LidarFrame& frame,
                                         const std::vector<std::size_t>& roi_indices) const
    -> std::vector<ClusterStats> {
    std::vector<ClusterStats> clusters;
    std::vector<bool> visited(roi_indices.size(), false);
    const float tolerance_sq = sqr(config_.lidar_cluster_tolerance_mm);

    for (std::size_t seed = 0; seed < roi_indices.size(); ++seed) {
        if (visited[seed]) continue;
        visited[seed] = true;

        std::queue<std::size_t> q;
        q.push(seed);

        ClusterStats stats;
        stats.min_x_mm = std::numeric_limits<float>::max();
        stats.min_y_mm = std::numeric_limits<float>::max();
        stats.min_z_mm = std::numeric_limits<float>::max();
        stats.max_x_mm = std::numeric_limits<float>::lowest();
        stats.max_y_mm = std::numeric_limits<float>::lowest();
        stats.max_z_mm = std::numeric_limits<float>::lowest();
        std::vector<float> depth_values;

        while (!q.empty()) {
            const auto local_idx = q.front();
            q.pop();
            const auto point_idx = roi_indices[local_idx];
            const auto& point = frame.points[point_idx];
            stats.indices.push_back(point_idx);
            stats.mean_depth_mm += point.z_mm;
            stats.mean_x_mm += point.x_mm;
            stats.mean_y_mm += point.y_mm;
            stats.mean_col += static_cast<float>(point.col);
            stats.mean_row += static_cast<float>(point.row);
            stats.mean_intensity += point.intensity;
            stats.min_x_mm = std::min(stats.min_x_mm, point.x_mm);
            stats.max_x_mm = std::max(stats.max_x_mm, point.x_mm);
            stats.min_y_mm = std::min(stats.min_y_mm, point.y_mm);
            stats.max_y_mm = std::max(stats.max_y_mm, point.y_mm);
            stats.min_z_mm = std::min(stats.min_z_mm, point.z_mm);
            stats.max_z_mm = std::max(stats.max_z_mm, point.z_mm);
            depth_values.push_back(point.z_mm);

            for (std::size_t other = 0; other < roi_indices.size(); ++other) {
                if (visited[other]) continue;
                const auto& neighbor = frame.points[roi_indices[other]];
                const float dist_sq = sqr(point.x_mm - neighbor.x_mm)
                                    + sqr(point.y_mm - neighbor.y_mm)
                                    + sqr(point.z_mm - neighbor.z_mm);
                if (dist_sq > tolerance_sq) continue;
                visited[other] = true;
                q.push(other);
            }
        }

        if (stats.indices.size() < static_cast<std::size_t>(config_.lidar_min_cluster_points)) {
            continue;
        }
        const auto denom = static_cast<float>(stats.indices.size());
        stats.point_count = stats.indices.size();
        stats.mean_depth_mm /= denom;
        stats.mean_x_mm /= denom;
        stats.mean_y_mm /= denom;
        stats.mean_col /= denom;
        stats.mean_row /= denom;
        stats.mean_intensity /= denom;

        std::sort(depth_values.begin(), depth_values.end());
        stats.median_depth_mm = depth_values[depth_values.size() / 2];
        float sq_error_sum = 0.0F;
        for (const auto depth : depth_values) {
            sq_error_sum += sqr(depth - stats.mean_depth_mm);
        }
        stats.depth_std_mm = std::sqrt(sq_error_sum / denom);
        clusters.push_back(stats);
    }

    return clusters;
}

auto LidarDepthEstimator::choose_target_cluster(const ModelCandidate& candidate,
                                                const std::vector<ClusterStats>& clusters) const
    -> std::optional<ClusterStats> {
    if (clusters.empty()) return std::nullopt;

    const float bbox_center_col = candidate.bbox.x + candidate.bbox.width * 0.5F;
    const float bbox_center_row = candidate.bbox.y + candidate.bbox.height * 0.5F;
    const float bbox_diag = std::hypot(candidate.bbox.width, candidate.bbox.height);
    const float max_center_dist_px = std::max(8.0F, bbox_diag * 0.6F);

    float best_score = std::numeric_limits<float>::lowest();
    float second_score = std::numeric_limits<float>::lowest();
    std::optional<ClusterStats> best;

    for (const auto& cluster : clusters) {
        ClusterStats scored = cluster;
        scored.center_dist_px = std::hypot(scored.mean_col - bbox_center_col,
                                           scored.mean_row - bbox_center_row);
        const float size_x = scored.max_x_mm - scored.min_x_mm;
        const float size_y = scored.max_y_mm - scored.min_y_mm;
        const float size_z = scored.max_z_mm - scored.min_z_mm;
        const float plane_extent = std::max(size_x, size_y);
        const float target_span_mm = [&] {
            for (const auto& geom : config_.target_geometry) {
                if (geom.class_id == candidate.class_id) {
                    return std::max(geom.width_mm, geom.height_mm);
                }
            }
            return 72.0F;
        }();

        const bool passes_center = scored.center_dist_px <= max_center_dist_px;
        const bool passes_thickness = size_z <= std::max(150.0F, target_span_mm * 2.5F);
        const bool passes_extent = plane_extent <= std::max(500.0F, target_span_mm * 4.0F);
        const bool passes_depth_std = scored.depth_std_mm <= 150.0F;
        scored.passes_gate = passes_center && passes_thickness && passes_extent && passes_depth_std;
        if (!scored.passes_gate) {
            continue;
        }

        const float center_score = 1.0F - clamp01(scored.center_dist_px / max_center_dist_px);
        const float thin_score = 1.0F - clamp01(size_z / std::max(150.0F, target_span_mm * 2.5F));
        const float size_score = 1.0F - clamp01(std::fabs(plane_extent - target_span_mm)
                                                / std::max(1.0F, target_span_mm));
        const float depth_score = 1.0F - clamp01(scored.depth_std_mm / 150.0F);
        const float density = static_cast<float>(scored.point_count)
                            / std::max(1.0F, size_x * size_y);
        const float density_score = clamp01(density * 4000.0F);
        const float intensity_score = clamp01(scored.mean_intensity / 255.0F);
        scored.score = center_score * 0.35F
                     + thin_score * 0.20F
                     + size_score * 0.20F
                     + depth_score * 0.15F
                     + density_score * 0.07F
                     + intensity_score * 0.03F;

        if (scored.score > best_score) {
            second_score = best_score;
            best_score = scored.score;
            best = scored;
        } else if (scored.score > second_score) {
            second_score = scored.score;
        }
    }

    if (!best) return std::nullopt;
    if (best->score < 0.60F) return std::nullopt;
    if (second_score > std::numeric_limits<float>::lowest()
        && (best->score - second_score) < 0.10F) {
        return std::nullopt;
    }

    return best;
}

auto LidarDepthEstimator::update_debug_info(std::size_t roi_points,
                                            const std::vector<ClusterStats>& clusters,
                                            const std::optional<ClusterStats>& target) const -> void {
    DebugInfo debug;
    debug.roi_points = roi_points;
    debug.cluster_count = clusters.size();
    if (target) {
        debug.has_target = true;
        debug.target_depth_mm = target->median_depth_mm > 0.0F
            ? target->median_depth_mm : target->mean_depth_mm;
        debug.target_score = target->score;
        debug.target_center_dist_px = target->center_dist_px;
        debug.target_size_x_mm = target->max_x_mm - target->min_x_mm;
        debug.target_size_y_mm = target->max_y_mm - target->min_y_mm;
        debug.target_size_z_mm = target->max_z_mm - target->min_z_mm;
        debug.target_depth_std_mm = target->depth_std_mm;
        debug.summary = std::format(
            "ROI {} pts, clusters={}, target score={:.2f} depth={:.0f}mm center={:.1f}px size=({:.0f},{:.0f},{:.0f}) std={:.0f}",
            roi_points,
            clusters.size(),
            target->score,
            debug.target_depth_mm,
            target->center_dist_px,
            debug.target_size_x_mm,
            debug.target_size_y_mm,
            debug.target_size_z_mm,
            debug.target_depth_std_mm);
    } else {
        debug.summary = std::format("ROI {} pts, clusters={}, no target cluster", roi_points, clusters.size());
    }
    std::scoped_lock lock(debug_mutex_);
    last_debug_ = std::move(debug);
}

} // namespace rmcs_laser_guidance
