#pragma once

#include <chrono>

#include "internal/runtime_metrics.hpp"

namespace rmcs_laser_guidance {

enum class StaleDropReason {
    None,
    InputTooOld,
    ObservationTooOld,
};

struct RuntimeMetricLogSample {
    RuntimeMetricSample metrics { };
    StaleDropReason stale_drop_reason = StaleDropReason::None;
};

struct StaleFramePolicy {
    std::chrono::milliseconds max_input_age_ms { 25 };
    std::chrono::milliseconds max_observation_age_ms { 35 };

    [[nodiscard]] auto classify_before_inference(const RuntimeMetricSample& metrics) const
        -> StaleDropReason {
        return metrics.input_age_ms > max_input_age_ms.count() ? StaleDropReason::InputTooOld
                                                                : StaleDropReason::None;
    }

    [[nodiscard]] auto classify_after_publish(const RuntimeMetricSample& metrics) const
        -> StaleDropReason {
        return metrics.observation_age_ms > max_observation_age_ms.count()
            ? StaleDropReason::ObservationTooOld
            : StaleDropReason::None;
    }

    [[nodiscard]] auto make_before_inference_sample(const Clock::time_point& capture_time,
        const Clock::time_point& queue_enter_time, const Clock::time_point& inference_start_time) const
        -> RuntimeMetricLogSample {
        const RuntimeMetricSample metrics = make_runtime_metric_sample(capture_time, queue_enter_time,
            inference_start_time, inference_start_time);
        return RuntimeMetricLogSample {
            .metrics = metrics,
            .stale_drop_reason = classify_before_inference(metrics),
        };
    }

    [[nodiscard]] auto make_after_publish_sample(const Clock::time_point& capture_time,
        const Clock::time_point& queue_enter_time, const Clock::time_point& inference_start_time,
        const Clock::time_point& publish_time) const -> RuntimeMetricLogSample {
        const RuntimeMetricSample metrics =
            make_runtime_metric_sample(capture_time, queue_enter_time, inference_start_time, publish_time);
        return RuntimeMetricLogSample {
            .metrics = metrics,
            .stale_drop_reason = classify_after_publish(metrics),
        };
    }
};

} // namespace rmcs_laser_guidance
