#pragma once

#include <cstddef>
#include <optional>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "laser_guidance/runtime.hpp"
#include "tracking/ekf_tracker.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct GuidanceTelemetry {
    std::optional<float> measured_depth_mm{};
    std::optional<float> active_depth_mm{};
    std::optional<cv::Point3f> selected_target_point{};
    bool used_cached_depth = false;
};

struct GuidanceFrameResult {
    AimOutput aim_output{};
    GuidanceTelemetry telemetry{};
};

struct ControlLoopFrame {
    Frame frame{};
    cv::Mat display{};
    DetectionBatch detection{};
    std::optional<EkfState> ekf_state{};
    TargetTrack track{};
    GuidanceFrameResult guidance{};
    std::size_t dropped_frames = 0;
};

} // namespace rmcs_laser_guidance::runtime_internal
