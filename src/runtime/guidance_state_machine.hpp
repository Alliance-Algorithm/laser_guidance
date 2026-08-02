#pragma once

#include <cstdint>

#include "laser_guidance/runtime.hpp"

namespace rmcs_laser_guidance::runtime_internal {

enum class GuidanceAction : std::uint8_t {
    idle,
    solve,
    recenter,
};

struct GuidanceDecision {
    GuidanceAction action = GuidanceAction::idle;
};

class GuidanceStateMachine {
public:
    [[nodiscard]] auto decide(
        const TargetTrack& track, bool ekf_was_lost, float last_valid_depth_mm) const
        -> GuidanceDecision {
        // Continuous illumination accumulates P (RM2026 §5.6.3); a brief
        // detection loss must not drop the beam to center. Without an EKF,
        // keep solving on the last cached aim when depth is still available.
        if (!track.ekf_enabled) {
            if (track.detected) {
                return GuidanceDecision{.action = GuidanceAction::solve};
            }
            if (last_valid_depth_mm > 0.0F) {
                return GuidanceDecision{.action = GuidanceAction::solve};
            }
            return {};
        }

        if (track.initialized) {
            if (track.detected) {
                return GuidanceDecision{.action = GuidanceAction::solve};
            }
            // Detection loss with EKF: hold the last commanded angle instead
            // of solving on the EKF-predicted aim point plus stale depth.
            // Solving during loss produced large angle jumps (e.g. -3.78° at
            // 41 m) because the depth filter state drifted while no
            // measurement arrived, so the beam snapped off-target.
            return GuidanceDecision{.action = GuidanceAction::idle};
        }
        return {};
    }
};

} // namespace rmcs_laser_guidance::runtime_internal
