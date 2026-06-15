#pragma once

#include <cstdint>
#include <memory>

#include <opencv2/core/mat.hpp>

namespace rmcs_laser_guidance {

struct DetectionBatch;
struct TargetTrack;
struct AimOutput;
struct RuntimeSnapshot;

class RosBridge {
public:
    RosBridge();
    ~RosBridge();

    RosBridge(const RosBridge&) = delete;
    auto operator=(const RosBridge&) -> RosBridge& = delete;

    /// Returns true if the bridge initialized successfully (ROS2 available).
    [[nodiscard]] auto ready() const noexcept -> bool;

    /// Publish all relevant data from the runtime snapshot to ROS2 topics.
    auto publish_snapshot(const RuntimeSnapshot& snapshot) -> void;

    /// Spin ROS2 internal state (call once per frame).
    auto spin() -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rmcs_laser_guidance
