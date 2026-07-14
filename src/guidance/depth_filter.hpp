#pragma once

#include <memory>

#include "config.hpp"

namespace rmcs_laser_guidance {

struct DepthFilterState {
    float depth_mm = 0.0F;
    float velocity_mm_s = 0.0F;
    bool initialized = false;
};

// 1D constant-velocity Kalman filter over target depth (mm). Depth
// measurements derived from bbox width carry large, mostly-unmodeled
// error (yaw/pitch-induced bbox distortion, verified ~8-15% relative on
// field samples) that single-frame geometric correction could not
// resolve. This filter does not attempt to correct that error per-frame;
// it absorbs it via a deliberately large measurement noise R and lets
// multi-frame temporal averaging settle the estimate.
class DepthFilter {
public:
    explicit DepthFilter(GuidanceConfig config = {});
    ~DepthFilter();

    DepthFilter(const DepthFilter&) = delete;
    auto operator=(const DepthFilter&) -> DepthFilter& = delete;
    DepthFilter(DepthFilter&&) noexcept;
    auto operator=(DepthFilter&&) noexcept -> DepthFilter&;

    auto predict(double dt_seconds) -> void;
    auto update(float measured_depth_mm) -> void;
    auto process(float measured_depth_mm, double dt_seconds) -> void;

    [[nodiscard]] auto state() const -> DepthFilterState;
    [[nodiscard]] auto is_initialized() const -> bool;
    auto reset() -> void;

private:
    struct Details;
    std::unique_ptr<Details> details_;
};

} // namespace rmcs_laser_guidance
