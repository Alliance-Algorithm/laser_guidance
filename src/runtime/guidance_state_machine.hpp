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
        if (!track.ekf_enabled) {
            if (track.detected) {
                return GuidanceDecision{.action = GuidanceAction::solve};
            }
            if (last_valid_depth_mm > 0.0F) {
                return GuidanceDecision{.action = GuidanceAction::recenter};
            }
            return {};
        }

        if (track.initialized && !track.lost) {
            return GuidanceDecision{.action = GuidanceAction::solve};
        }
        if (track.lost && !ekf_was_lost) {
            return GuidanceDecision{.action = GuidanceAction::recenter};
        }
        return {};
    }
};

} // namespace rmcs_laser_guidance::runtime_internal
