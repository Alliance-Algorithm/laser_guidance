#include <exception>
#include <print>

#include "internal/hit_state.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::HitState;
        using rmcs_laser_guidance::HitStateMachine;
        using rmcs_laser_guidance::tests::require;

        {
            HitStateMachine machine;
            require(machine.state() == HitState::None, "initial state should be None");
            require(machine.consecutive() == 0, "initial consecutive should be zero");

            require(machine.update(true) == HitState::Candidate, "first purple should enter Candidate");
            require(machine.state() == HitState::Candidate, "state should remain Candidate after first purple");
            require(machine.consecutive() == 1, "consecutive should count first purple");

            require(machine.update(true) == HitState::Candidate, "second purple should stay Candidate");
            require(machine.consecutive() == 2, "consecutive should count second purple");

            require(machine.update(true) == HitState::Confirmed, "third purple should confirm");
            require(machine.state() == HitState::Confirmed, "state should be Confirmed after three purple frames");
            require(machine.consecutive() == 3, "consecutive should count confirmed purple streak");
        }

        {
            HitStateMachine machine;
            for (int i = 0; i < 3; ++i) {
                (void)machine.update(true);
            }

            require(machine.state() == HitState::Confirmed, "setup should reach Confirmed");

            for (int i = 0; i < 4; ++i) {
                require(machine.update(false) == HitState::Confirmed, "release should not happen before five non-purple frames");
            }

            require(machine.consecutive() == 4, "consecutive should count non-purple streak during hysteresis");
            require(machine.update(false) == HitState::None, "fifth non-purple should release to None");
            require(machine.state() == HitState::None, "state should be None after release");
            require(machine.consecutive() == 5, "consecutive should count release threshold");
        }

        {
            HitStateMachine machine;
            (void)machine.update(true);
            (void)machine.update(true);
            require(machine.state() == HitState::Candidate, "setup should be Candidate before flicker test");

            require(machine.update(false) == HitState::Candidate, "single non-purple flicker should not reset immediately");
            require(machine.state() == HitState::Candidate, "candidate should survive a short flicker");
            require(machine.consecutive() == 1, "consecutive should track the non-purple flicker streak");

            require(machine.update(true) == HitState::Candidate, "purple returning after a short flicker should still be tracked");
            require(machine.consecutive() == 1, "purple streak should restart after flicker");
            require(machine.update(true) == HitState::Candidate, "second purple after flicker should still be Candidate");
            require(machine.update(true) == HitState::Confirmed, "third purple after flicker should confirm");
        }

        {
            HitStateMachine machine;
            (void)machine.update(true);
            (void)machine.update(true);
            machine.reset();

            require(machine.state() == HitState::None, "reset should clear state");
            require(machine.consecutive() == 0, "reset should clear consecutive counter");
            require(machine.update(true) == HitState::Candidate, "machine should work after reset");
            require(machine.consecutive() == 1, "counter should restart after reset");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "hit_state_test failed: {}", e.what());
        return 1;
    }
}
