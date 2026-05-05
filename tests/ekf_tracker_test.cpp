#include <chrono>
#include <exception>
#include <print>

#include "internal/ekf_tracker.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::Clock;
        using rmcs_laser_guidance::EkfConfig;
        using rmcs_laser_guidance::EkfTracker;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_near;

        {
            EkfTracker tracker;
            const auto state = tracker.state();
            require(!state.initialized, "tracker should start uninitialized");
            require(!state.lost, "tracker should not start lost");
            require(state.missed_frames == 0, "tracker should start with zero missed frames");
        }

        {
            EkfTracker tracker;
            tracker.update({100.0F, 120.0F});

            const auto state = tracker.state();
            require(state.initialized, "tracker should initialize on first update");
            require_near(state.position.x, 100.0F, 1e-3F, "initial x position");
            require_near(state.position.y, 120.0F, 1e-3F, "initial y position");
            require_near(state.velocity.x, 0.0F, 1e-3F, "initial vx");
            require_near(state.velocity.y, 0.0F, 1e-3F, "initial vy");
            require(state.missed_frames == 0, "update should reset missed frames");
            require(!state.lost, "update should clear lost state");
        }

        {
            EkfConfig config;
            config.max_missed_frames = 2;
            EkfTracker tracker(config);

            const Clock::time_point t0 { };
            tracker.update({30.0F, 40.0F});
            tracker.predict(t0 + std::chrono::milliseconds(16));
            auto state = tracker.state();
            require(state.missed_frames == 0, "first predict should initialize timing baseline");
            require(!state.lost, "should not be lost when only timing baseline is initialized");

            tracker.predict(t0 + std::chrono::milliseconds(32));
            state = tracker.state();
            require(state.missed_frames == 1, "second predict should be first missed-frame increment");
            require(!state.lost, "should not be lost after first missed frame");

            tracker.predict(t0 + std::chrono::milliseconds(48));
            state = tracker.state();
            require(state.missed_frames == 2, "third predict increments missed frames");
            require(!state.lost, "should not be lost at threshold boundary");

            tracker.predict(t0 + std::chrono::milliseconds(64));
            state = tracker.state();
            require(state.missed_frames == 3, "fourth predict increments missed frames");
            require(state.lost, "should be marked lost after exceeding threshold");

            tracker.update({35.0F, 42.0F});
            state = tracker.state();
            require(state.missed_frames == 0, "reacquisition should clear missed frames");
            require(!state.lost, "reacquisition should clear lost state");
        }

        {
            EkfTracker tracker;
            const Clock::time_point t0 { };
            tracker.update({0.0F, 0.0F});

            tracker.predict(t0 + std::chrono::milliseconds(20));
            tracker.update({2.0F, 0.0F});
            tracker.predict(t0 + std::chrono::milliseconds(40));
            tracker.update({4.0F, 0.0F});
            tracker.predict(t0 + std::chrono::milliseconds(60));
            tracker.update({6.0F, 0.0F});

            const auto state = tracker.state();
            require_near(state.position.x, 6.0F, 1.0F, "position should follow measurements");
            require(state.velocity.x > 0.0F, "tracker should estimate positive x velocity");
            require_near(state.position.y, 0.0F, 1.0F, "y position should stay near measurement line");
        }

        {
            EkfTracker tracker;
            tracker.update({10.0F, 20.0F});
            tracker.reset();
            const auto state = tracker.state();
            require(!state.initialized, "reset should clear initialization");
            require(!state.lost, "reset should clear lost state");
            require(state.missed_frames == 0, "reset should clear missed frames");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "ekf_tracker_test failed: {}", e.what());
        return 1;
    }
}
