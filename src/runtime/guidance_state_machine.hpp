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
        // detection loss must not drop the beam to center. Keep solving on
        // the EKF-predicted aim point (or the last cached aim when the EKF is
        // disabled) so the next detected frame resumes irradiation instantly.
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
            // Healthy or lost: keep solving on the EKF-predicted aim point so
            // the beam stays on the target during brief detection loss.
            return GuidanceDecision{.action = GuidanceAction::solve};
        }
        return {};
    }
};

} // namespace rmcs_laser_guidance::runtime_internal
