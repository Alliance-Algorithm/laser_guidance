#include <chrono>

#include "runtime/profile_switch_gate.hpp"
#include "test_utils.hpp"

using namespace rmcs_laser_guidance;
using Clock = std::chrono::steady_clock;

int main() {
    using rmcs_laser_guidance::tests::require;

    {
        // Already active: nothing to do, deadline disarmed.
        Clock::time_point deadline = Clock::now() + std::chrono::seconds(10);
        const auto gate = decide_profile_switch(
            /*want_unlit=*/false, /*active_target=*/1, Clock::now(), deadline, 5.0F);
        require(gate == ProfileSwitchGate::already_active, "already active is idle");
        require(deadline == Clock::time_point{}, "deadline cleared when active");
    }

    {
        // Stage-3 edge debounced for the delay window.
        const auto t0 = Clock::now();
        Clock::time_point deadline{};
        const auto gate1 = decide_profile_switch(
            /*want_unlit=*/true, /*active_target=*/1, t0, deadline, 5.0F);
        require(gate1 == ProfileSwitchGate::debouncing, "first sighting arms deadline");
        require(deadline > t0, "deadline armed in the future");

        const auto gate2 = decide_profile_switch(
            /*want_unlit=*/true, /*active_target=*/1, t0 + std::chrono::seconds(4), deadline,
            5.0F);
        require(gate2 == ProfileSwitchGate::debouncing, "still debouncing before deadline");

        const auto gate3 = decide_profile_switch(
            /*want_unlit=*/true, /*active_target=*/1,
            t0 + std::chrono::milliseconds(5001), deadline, 5.0F);
        require(gate3 == ProfileSwitchGate::proceed, "proceeds after delay elapses");
    }

    {
        // Zero delay proceeds immediately.
        Clock::time_point deadline{};
        const auto gate = decide_profile_switch(
            /*want_unlit=*/true, /*active_target=*/1, Clock::now(), deadline, 0.0F);
        require(gate == ProfileSwitchGate::proceed, "zero delay proceeds immediately");
    }

    {
        // Downswing to lit (next-match reset) is immediate and clears deadline.
        const auto t0 = Clock::now();
        Clock::time_point deadline = t0 + std::chrono::seconds(10);
        const auto gate = decide_profile_switch(
            /*want_unlit=*/false, /*active_target=*/3, t0, deadline, 5.0F);
        require(gate == ProfileSwitchGate::proceed, "downswing proceeds immediately");
        require(deadline == Clock::time_point{}, "downswing clears deadline");
    }

    {
        // Deadline disarmed once target becomes active.
        const auto t0 = Clock::now();
        Clock::time_point deadline = t0 + std::chrono::seconds(10);
        const auto gate = decide_profile_switch(
            /*want_unlit=*/true, /*active_target=*/3, t0, deadline, 5.0F);
        require(gate == ProfileSwitchGate::already_active, "active unlit is idle");
        require(deadline == Clock::time_point{}, "deadline cleared when unlit active");
    }

    return 0;
}
