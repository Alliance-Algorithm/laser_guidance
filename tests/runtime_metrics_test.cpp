#include <chrono>
#include <exception>
#include <cstdint>
#include <print>

#include "types.hpp"
#include "internal/runtime_metrics.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::Clock;
        using rmcs_laser_guidance::RuntimeMetricLogSample;
        using rmcs_laser_guidance::RuntimeMetricSample;
        using rmcs_laser_guidance::StaleFramePolicy;
        using rmcs_laser_guidance::StaleReason;
        using rmcs_laser_guidance::age_ms;
        using rmcs_laser_guidance::tests::require;

        const Clock::time_point base { };
        const auto ms = [](const std::int64_t value) {
            return std::chrono::milliseconds(value);
        };

        {
            const Clock::time_point capture_time = base;
            const Clock::time_point worker_start_time = base + ms(4);
            const Clock::time_point infer_start_time = base + ms(12);
            const Clock::time_point publish_time = base + ms(19);

            const RuntimeMetricSample metrics {
                .observation_age_ms = age_ms(capture_time, publish_time),
                .input_age_at_worker_start_ms = age_ms(capture_time, worker_start_time),
                .input_age_at_infer_start_ms = age_ms(capture_time, infer_start_time),
            };

            require(metrics.observation_age_ms == 19,
                "observation_age_ms should measure publish minus capture");
            require(metrics.input_age_at_worker_start_ms == 4,
                "input age at worker start should measure worker minus capture");
            require(metrics.input_age_at_infer_start_ms == 12,
                "input age at infer start should measure inference minus capture");
        }

        {
            StaleFramePolicy policy;

            const RuntimeMetricSample fresh_metrics {
                .observation_age_ms = 35,
                .input_age_at_worker_start_ms = 7,
                .input_age_at_infer_start_ms = 25,
            };

            require(policy.classify_before_inference(fresh_metrics) == StaleReason::none,
                "input age at the boundary should stay fresh");
            require(policy.classify_before_worker(fresh_metrics) == StaleReason::none,
                "worker-start input age at the boundary should stay fresh");
            require(policy.classify_after_publish(fresh_metrics) == StaleReason::none,
                "observation age at the boundary should stay fresh");

            const RuntimeMetricSample stale_worker_metrics {
                .observation_age_ms = 30,
                .input_age_at_worker_start_ms = 26,
                .input_age_at_infer_start_ms = 26,
            };

            require(policy.classify_before_worker(stale_worker_metrics)
                    == StaleReason::too_old_before_worker,
                "input age beyond the boundary at worker start should be dropped before worker");

            const RuntimeMetricSample stale_input_metrics {
                .observation_age_ms = 30,
                .input_age_at_worker_start_ms = 3,
                .input_age_at_infer_start_ms = 26,
            };

            require(policy.classify_before_inference(stale_input_metrics)
                    == StaleReason::too_old_before_inference,
                "input age beyond the boundary should be dropped before inference");

            const RuntimeMetricSample stale_observation_metrics {
                .observation_age_ms = 36,
                .input_age_at_worker_start_ms = 5,
                .input_age_at_infer_start_ms = 24,
            };

            require(policy.classify_after_publish(stale_observation_metrics)
                    == StaleReason::too_old_after_publish,
                "observation age beyond the boundary should be dropped after publish");

            const RuntimeMetricLogSample before_sample = policy.make_before_inference_sample(base, base + ms(2),
                base + ms(27));
            require(before_sample.metrics.input_age_at_worker_start_ms == 2,
                "before-inference sample should preserve worker-start age calculation");
            require(before_sample.metrics.input_age_at_infer_start_ms == 27,
                "before-inference sample should preserve infer-start age calculation");
            require(before_sample.stale_reason == StaleReason::too_old_before_inference,
                "before-inference stale reason should be specific to input age");

            const RuntimeMetricLogSample before_worker_sample = policy.make_before_inference_sample(base,
                base + ms(26), base + ms(27));
            require(before_worker_sample.stale_reason == StaleReason::too_old_before_worker,
                "before-worker stale reason should take priority when worker-start age is already stale");

            const RuntimeMetricLogSample after_sample = policy.make_after_publish_sample(base, base + ms(3),
                base + ms(14), base + ms(36));
            require(after_sample.metrics.observation_age_ms == 36,
                "after-publish sample should preserve observation age calculation");
            require(after_sample.metrics.input_age_at_worker_start_ms == 3,
                "after-publish sample should preserve worker-start age calculation");
            require(after_sample.metrics.input_age_at_infer_start_ms == 14,
                "after-publish sample should preserve infer-start age calculation");
            require(after_sample.stale_reason == StaleReason::too_old_after_publish,
                "after-publish stale reason should be specific to observation age");
        }

        {
            StaleFramePolicy policy;
            const auto sample = policy.make_after_publish_sample(base, base, base + ms(10), base + ms(15));
            require(sample.stale_reason == StaleReason::none, "fresh frames should not be marked stale");
            require(sample.metrics.input_age_at_worker_start_ms == 0,
                "fresh sample worker-start age mismatch");
            require(sample.metrics.input_age_at_infer_start_ms == 10,
                "fresh sample infer-start age mismatch");
            require(sample.metrics.observation_age_ms == 15,
                "fresh sample observation age mismatch");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "runtime_metrics_test failed: {}", e.what());
        return 1;
    }
}
