#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class LidarDepthEstimator {
public:
    struct DebugInfo {
        std::size_t roi_points = 0;
        std::size_t cluster_count = 0;
        bool has_target = false;
        float target_depth_mm = 0.0F;
        float target_score = 0.0F;
        float target_center_dist_px = 0.0F;
        float target_size_x_mm = 0.0F;
        float target_size_y_mm = 0.0F;
        float target_size_z_mm = 0.0F;
        float target_depth_std_mm = 0.0F;
        std::string summary {};
    };

    explicit LidarDepthEstimator(const GuidanceConfig& config);

    auto estimate(const ModelCandidate& candidate, const LidarFrame& frame) const
        -> std::optional<float>;
    [[nodiscard]] auto last_debug_info() const -> DebugInfo;

private:
    struct ClusterStats {
        std::vector<std::size_t> indices{};
        std::size_t point_count = 0;
        float mean_depth_mm = 0.0F;
        float median_depth_mm = 0.0F;
        float depth_std_mm = 0.0F;
        float mean_x_mm = 0.0F;
        float mean_y_mm = 0.0F;
        float mean_col = 0.0F;
        float mean_row = 0.0F;
        float center_dist_px = 0.0F;
        float mean_intensity = 0.0F;
        float min_x_mm = 0.0F;
        float max_x_mm = 0.0F;
        float min_y_mm = 0.0F;
        float max_y_mm = 0.0F;
        float min_z_mm = 0.0F;
        float max_z_mm = 0.0F;
        float score = -1.0F;
        bool passes_gate = false;
    };

    auto collect_roi_points(const ModelCandidate& candidate, const LidarFrame& frame) const
        -> std::vector<std::size_t>;
    auto cluster_points(const LidarFrame& frame, const std::vector<std::size_t>& roi_indices) const
        -> std::vector<ClusterStats>;
    auto choose_target_cluster(const ModelCandidate& candidate,
                               const std::vector<ClusterStats>& clusters) const
        -> std::optional<ClusterStats>;
    auto update_debug_info(std::size_t roi_points,
                           const std::vector<ClusterStats>& clusters,
                           const std::optional<ClusterStats>& target) const -> void;

    GuidanceConfig config_;
    mutable std::mutex debug_mutex_;
    mutable DebugInfo last_debug_{};
};

} // namespace rmcs_laser_guidance
