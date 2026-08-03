#pragma once

#include <chrono>

namespace rmcs_laser_guidance {

enum class ProfileSwitchGate {
    already_active,  // target == active profile, nothing to do
    debouncing,      // stage-3 edge seen, waiting out the delay window
    proceed,         // delay elapsed (or no delay needed), switch now
};

// Decide whether the Hik lit/unlit profile switch should proceed, given the
// local HitProgress difficulty (>=3 wants unlit) and the currently active
// profile target. The stage-3 edge is debounced: on first sighting a deadline
// is armed, and the switch only proceeds once want_unlit has persisted for
// delay_s (guards against a jittery counter flipping the camera profile
// mid-lock). Downswing (difficulty <3, e.g. next-match reset) proceeds with no
// delay.
inline auto decide_profile_switch(
    const bool want_unlit, const int active_target, const std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point& deadline, const float delay_s)
    -> ProfileSwitchGate {
    const int target = want_unlit ? 3 : 1;
    if (target == active_target) {
        deadline = {};
        return ProfileSwitchGate::already_active;
    }
    if (want_unlit) {
        if (deadline == std::chrono::steady_clock::time_point{}) {
            deadline = now + std::chrono::milliseconds(
                                 static_cast<std::int64_t>(delay_s * 1000.0F));
        }
        if (now < deadline)
            return ProfileSwitchGate::debouncing;
        return ProfileSwitchGate::proceed;
    }
    deadline = {};
    return ProfileSwitchGate::proceed;
}

} // namespace rmcs_laser_guidance
