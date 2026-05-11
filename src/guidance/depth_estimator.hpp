#pragma once

#include <optional>

#include <opencv2/core/mat.hpp>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class DepthEstimator {
public:
    DepthEstimator(const GuidanceConfig& config, const cv::Mat& camera_matrix);

    auto estimate(const ModelCandidate& candidate) const -> std::optional<float>;

private:
    GuidanceConfig config_;
    cv::Mat camera_matrix_;
};

} // namespace rmcs_laser_guidance
