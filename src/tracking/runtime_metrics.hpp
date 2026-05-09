#pragma once

#include <chrono>
#include <cstdint>

#include "types.hpp"

namespace rmcs_laser_guidance {

[[nodiscard]] inline auto age_ms(
    const Clock::time_point& earlier, const Clock::time_point& later) -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(later - earlier).count();
}

// NOLINTBEGIN(readability-identifier-naming)
enum class StaleReason : std::uint8_t {
    none,
    too_old_before_worker,
    too_old_before_inference,
    too_old_after_publish,
    queue_overwrite,
};
// NOLINTEND(readability-identifier-naming)

struct RuntimeMetricSample {
    std::int64_t observation_age_ms = 0;
    std::int64_t input_age_at_worker_start_ms = 0;
    std::int64_t input_age_at_infer_start_ms = 0;
};

struct RuntimeMetricLogSample {
    RuntimeMetricSample metrics { };
    StaleReason stale_reason = StaleReason::none;
};

struct StaleFramePolicy {
    std::chrono::milliseconds max_input_age_ms { 25 };
    std::chrono::milliseconds max_observation_age_ms { 35 };

    [[nodiscard]] auto classify_before_inference(const RuntimeMetricSample& metrics) const -> StaleReason {
        return metrics.input_age_at_infer_start_ms > max_input_age_ms.count()
            ? StaleReason::too_old_before_inference
            : StaleReason::none;
    }

    [[nodiscard]] auto classify_before_worker(const RuntimeMetricSample& metrics) const -> StaleReason {
        return metrics.input_age_at_worker_start_ms > max_input_age_ms.count()
            ? StaleReason::too_old_before_worker
            : StaleReason::none;
    }

    [[nodiscard]] auto classify_after_publish(const RuntimeMetricSample& metrics) const -> StaleReason {
        return metrics.observation_age_ms > max_observation_age_ms.count()
            ? StaleReason::too_old_after_publish
            : StaleReason::none;
    }

    [[nodiscard]] auto make_before_inference_sample(const Clock::time_point& capture_time,
        const Clock::time_point& worker_start_time, const Clock::time_point& infer_start_time) const
        -> RuntimeMetricLogSample {
        const RuntimeMetricSample metrics {
            .observation_age_ms = age_ms(capture_time, infer_start_time),
            .input_age_at_worker_start_ms = age_ms(capture_time, worker_start_time),
            .input_age_at_infer_start_ms = age_ms(capture_time, infer_start_time),
        };
        return RuntimeMetricLogSample {
            .metrics = metrics,
            .stale_reason = classify_before_worker(metrics) == StaleReason::none
                ? classify_before_inference(metrics)
                : StaleReason::too_old_before_worker,
        };
    }

    [[nodiscard]] auto make_after_publish_sample(const Clock::time_point& capture_time,
        const Clock::time_point& worker_start_time, const Clock::time_point& infer_start_time,
        const Clock::time_point& publish_time) const -> RuntimeMetricLogSample {
        const RuntimeMetricSample metrics {
            .observation_age_ms = age_ms(capture_time, publish_time),
            .input_age_at_worker_start_ms = age_ms(capture_time, worker_start_time),
            .input_age_at_infer_start_ms = age_ms(capture_time, infer_start_time),
        };
        return RuntimeMetricLogSample {
            .metrics = metrics,
            .stale_reason = classify_after_publish(metrics),
        };
    }
};

} // namespace rmcs_laser_guidance
