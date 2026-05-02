#include <chrono>
#include <exception>
#include <print>
#include <thread>

#include "internal/hit_state.hpp"
#include "internal/mock_runtime.hpp"
#include "internal/runtime_metrics.hpp"
#include "test_utils.hpp"
#include "types.hpp"

namespace {

auto wait_until(auto predicate, const std::chrono::milliseconds timeout, const char* message) -> void {
    const auto deadline = rmcs_laser_guidance::Clock::now() + timeout;
    while (rmcs_laser_guidance::Clock::now() < deadline) {
        if (predicate()) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    rmcs_laser_guidance::tests::require(predicate(), message);
}

} // namespace

int main() {
    try {
        using rmcs_laser_guidance::HitState;
        using rmcs_laser_guidance::MockLatestFrameRuntime;
        using rmcs_laser_guidance::MockRuntimeConfig;
        using rmcs_laser_guidance::MockRuntimeObservation;
        using rmcs_laser_guidance::StaleFramePolicy;
        using rmcs_laser_guidance::tests::require;

        {
            MockRuntimeConfig config;
            config.capture_period = std::chrono::milliseconds(1);
            config.worker_start_delay = std::chrono::milliseconds(0);
            config.inference_latency = std::chrono::milliseconds(12);
            config.max_capture_frames = 80;
            config.stale_policy = StaleFramePolicy {
                .max_input_age_ms = std::chrono::milliseconds(25),
                .max_observation_age_ms = std::chrono::milliseconds(50),
            };
            config.purple_sequence = { true, true, true };

            MockLatestFrameRuntime runtime(config);
            runtime.start();
            wait_until([&] { return runtime.snapshot().capture_thread_exited; },
                std::chrono::milliseconds(500), "capture thread should finish bounded synthetic run");
            wait_until([&] { return runtime.snapshot().capture_overwrites > 0; },
                std::chrono::milliseconds(500), "fast capture should overwrite the latest-frame slot");
            runtime.stop();

            const auto snapshot = runtime.snapshot();
            require(snapshot.capture_thread_exited, "capture thread should exit after max frames");
            require(snapshot.worker_thread_exited, "worker thread should exit after stop");
            require(snapshot.captured_frames == 80, "capture should produce the configured synthetic frame count");
            require(snapshot.max_capture_queue_depth <= 1, "latest-frame handoff should stay capacity one");
            require(snapshot.capture_overwrites > 0, "producer outrunning worker should drop old frames by overwrite");
            require(snapshot.published_observations > 0, "worker should publish some fresh observations");
            require(snapshot.published_observations < snapshot.captured_frames,
                "worker should consume latest frames, not build a backlog");
            require(snapshot.hit_state == HitState::Confirmed, "purple sequence should drive HIT confirmation");
        }

        {
            MockRuntimeConfig config;
            config.capture_period = std::chrono::milliseconds(1);
            config.worker_start_delay = std::chrono::milliseconds(10);
            config.inference_latency = std::chrono::milliseconds(0);
            config.max_capture_frames = 20;
            config.stale_policy = StaleFramePolicy {
                .max_input_age_ms = std::chrono::milliseconds(2),
                .max_observation_age_ms = std::chrono::milliseconds(100),
            };

            MockLatestFrameRuntime runtime(config);
            runtime.start();
            wait_until([&] { return runtime.snapshot().stale_before_inference_drops > 0; },
                std::chrono::milliseconds(500), "worker should drop inputs that are stale before inference");
            runtime.stop();

            const auto snapshot = runtime.snapshot();
            require(snapshot.worker_thread_exited, "worker should exit after stale-drop scenario stop");
            require(snapshot.stale_before_inference_drops > 0, "stale frames should be dropped before inference");
            require(snapshot.published_observations == 0, "stale-before policy should prevent publishing old inputs");
            require(snapshot.capture_overwrites > 0, "overload should be represented as overwrites, not backlog");
            require(snapshot.max_capture_queue_depth <= 1, "stale overload should still keep queue capacity one");
        }

        {
            MockRuntimeConfig config;
            config.capture_period = std::chrono::milliseconds(1);
            config.worker_start_delay = std::chrono::milliseconds(0);
            config.inference_latency = std::chrono::milliseconds(0);
            config.max_capture_frames = 5;
            config.synthetic_capture_age = std::chrono::milliseconds(5);
            config.stale_policy = StaleFramePolicy {
                .max_input_age_ms = std::chrono::milliseconds(1),
                .max_observation_age_ms = std::chrono::milliseconds(100),
            };

            MockLatestFrameRuntime runtime(config);
            runtime.start();
            wait_until([&] { return runtime.snapshot().stale_before_worker_drops > 0; },
                std::chrono::milliseconds(500), "worker should drop frames already stale at worker start");
            runtime.stop();

            const auto snapshot = runtime.snapshot();
            require(snapshot.stale_before_worker_drops > 0,
                "frames already stale when worker receives them should be dropped distinctly");
            require(snapshot.max_capture_queue_depth <= 1,
                "worker-start stale overload should still keep queue capacity one");
        }

        {
            MockRuntimeConfig config;
            config.capture_period = std::chrono::milliseconds(1);
            config.worker_start_delay = std::chrono::milliseconds(0);
            config.inference_latency = std::chrono::milliseconds(0);
            config.max_capture_frames = 60;
            config.stale_policy = StaleFramePolicy {
                .max_input_age_ms = std::chrono::milliseconds(100),
                .max_observation_age_ms = std::chrono::milliseconds(100),
            };
            config.purple_sequence = { true, true, true, false, false, false, false, false };

            MockLatestFrameRuntime runtime(config);
            runtime.start();
            wait_until([&] { return runtime.snapshot().published_observations >= 20; },
                std::chrono::milliseconds(500), "worker should keep publishing while debug output is overloaded");
            runtime.stop();

            const auto snapshot = runtime.snapshot();
            require(snapshot.debug_recording_outputs == snapshot.published_observations,
                "every published observation should attempt debug/recording output");
            require(snapshot.debug_recording_overwrites > 0,
                "undrained debug/recording output should overwrite instead of blocking worker");
            require(snapshot.published_observations >= 20,
                "debug/recording overload should not prevent worker progress");

            MockRuntimeObservation newest_observation;
            require(runtime.try_pop_debug_recording(newest_observation),
                "debug/recording sink should retain the latest observation");
            require(newest_observation.sequence == snapshot.published_observations,
                "debug/recording sink should drop backlog and keep newest observation");
        }

        {
            MockRuntimeConfig config;
            config.capture_period = std::chrono::milliseconds(50);
            config.worker_start_delay = std::chrono::milliseconds(0);
            config.inference_latency = std::chrono::milliseconds(100);

            MockLatestFrameRuntime runtime(config);
            runtime.start();
            wait_until([&] { return runtime.snapshot().worker_popped_frames > 0; },
                std::chrono::milliseconds(500), "worker should start processing before shutdown");
            runtime.stop();

            const auto snapshot = runtime.snapshot();
            require(snapshot.capture_thread_exited, "shutdown should stop capture thread");
            require(snapshot.worker_thread_exited, "shutdown should stop worker thread cleanly");
        }

        {
            MockRuntimeConfig config;
            config.capture_period = std::chrono::milliseconds(100);
            config.inference_latency = std::chrono::milliseconds(0);

            MockLatestFrameRuntime runtime(config);
            runtime.start();
            runtime.stop();

            const auto snapshot = runtime.snapshot();
            require(snapshot.worker_thread_exited, "shutdown should unblock worker waiting on empty queue");
            require(snapshot.capture_thread_exited, "shutdown should join capture thread cleanly");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "mock_runtime_test failed: {}", e.what());
        return 1;
    }
}
