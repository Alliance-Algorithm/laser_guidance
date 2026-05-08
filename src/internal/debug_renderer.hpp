#pragma once

#include <opencv2/core/mat.hpp>

#include "config.hpp"
#include "internal/ekf_tracker.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class DebugRenderer {
public:
    explicit DebugRenderer(const DebugConfig& debug_config);

    auto draw(cv::Mat& image, const TargetObservation& observation) const -> void;
    auto draw_ekf_state(cv::Mat& image, const EkfState& state) const -> void;

private:
    DebugConfig debug_;
};

} // namespace rmcs_laser_guidance
