#include <print>
#include <stdexcept>

#include "runtime/guidance_state_machine.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::TargetTrack;
        using rmcs_laser_guidance::runtime_internal::GuidanceAction;
        using rmcs_laser_guidance::runtime_internal::GuidanceStateMachine;
        using rmcs_laser_guidance::tests::require;

        const GuidanceStateMachine state_machine;

        {
            TargetTrack track;
            track.detected = true;
            track.ekf_enabled = false;
            require(
                state_machine.decide(track, false, 0.0F).action == GuidanceAction::solve,
                "raw detected track should solve");
        }

        {
            TargetTrack track;
            track.detected = false;
            track.ekf_enabled = false;
            // Continuous illumination is required to accumulate P; a brief loss
            // must not drop the beam to center. Without EKF the solver keeps
            // steering the last cached aim point when depth is available.
            require(
                state_machine.decide(track, false, 1200.0F).action == GuidanceAction::solve,
                "raw loss with valid depth should keep solving");
            require(
                state_machine.decide(track, false, 0.0F).action == GuidanceAction::idle,
                "raw loss without depth should idle");
        }

        {
            TargetTrack track;
            track.ekf_enabled = true;
            track.initialized = true;
            track.lost = false;
            require(
                state_machine.decide(track, false, 0.0F).action == GuidanceAction::solve,
                "healthy EKF track should solve");
        }

        {
            TargetTrack track;
            track.ekf_enabled = true;
            track.initialized = true;
            track.lost = true;
            // Keep the beam on the EKF-predicted aim point during loss instead
            // of recentering: hitting continuity is what drives P accumulation.
            require(
                state_machine.decide(track, false, 0.0F).action == GuidanceAction::solve,
                "first lost EKF frame should keep solving on prediction");
            require(
                state_machine.decide(track, true, 0.0F).action == GuidanceAction::solve,
                "steady lost EKF state should keep solving");
        }

        {
            TargetTrack recovered;
            recovered.ekf_enabled = true;
            recovered.initialized = true;
            recovered.lost = false;
            require(
                state_machine.decide(recovered, true, 0.0F).action == GuidanceAction::solve,
                "recovered EKF track should resume solving");

            TargetTrack lost_again = recovered;
            lost_again.lost = true;
            require(
                state_machine.decide(lost_again, false, 0.0F).action == GuidanceAction::solve,
                "new EKF loss edge should keep solving");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "guidance_state_machine_test failed: {}", e.what());
        return 1;
    }
}
