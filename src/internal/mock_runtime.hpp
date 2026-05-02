#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

#include "internal/freshness_queue.hpp"
#include "internal/hit_state.hpp"
#include "internal/runtime_metrics.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

struct MockRuntimeConfig {
    std::chrono::milliseconds capture_period { 1 };
    std::chrono::milliseconds worker_start_delay { 0 };
    std::chrono::milliseconds inference_latency { 5 };
    StaleFramePolicy stale_policy { };
    std::size_t max_capture_frames = 0;
    std::chrono::milliseconds synthetic_capture_age { 0 };
    std::vector<bool> purple_sequence { true, true, true, false, false, false, false, false };
};

struct MockRuntimeObservation {
    std::uint64_t sequence = 0;
    RuntimeMetricLogSample metrics { };
    HitState hit_state = HitState::None;
    bool purple = false;
};

struct MockRuntimeSnapshot {
    std::uint64_t captured_frames = 0;
    std::uint64_t worker_popped_frames = 0;
    std::uint64_t published_observations = 0;
    std::uint64_t stale_before_worker_drops = 0;
    std::uint64_t stale_before_inference_drops = 0;
    std::uint64_t stale_after_publish_drops = 0;
    std::uint64_t debug_recording_outputs = 0;
    std::size_t capture_overwrites = 0;
    std::size_t debug_recording_overwrites = 0;
    std::size_t max_capture_queue_depth = 0;
    HitState hit_state = HitState::None;
    bool capture_thread_exited = false;
    bool worker_thread_exited = false;
};

class MockLatestFrameRuntime {
public:
    explicit MockLatestFrameRuntime(MockRuntimeConfig config)
        : config_(std::move(config)) {}

    MockLatestFrameRuntime(const MockLatestFrameRuntime&) = delete;
    auto operator=(const MockLatestFrameRuntime&) -> MockLatestFrameRuntime& = delete;
    MockLatestFrameRuntime(MockLatestFrameRuntime&&) = delete;
    auto operator=(MockLatestFrameRuntime&&) -> MockLatestFrameRuntime& = delete;

    ~MockLatestFrameRuntime() {
        stop();
    }

    auto start() -> void {
        if (started_.exchange(true))
            throw std::runtime_error("MockLatestFrameRuntime already started");

        capture_thread_ = std::jthread([this](const std::stop_token& stop_token) {
            capture_loop(stop_token);
        });
        worker_thread_ = std::jthread([this](const std::stop_token& stop_token) {
            worker_loop(stop_token);
        });
    }

    auto stop() -> void {
        if (!started_.exchange(false))
            return;

        if (capture_thread_.joinable()) capture_thread_.request_stop();
        if (worker_thread_.joinable()) worker_thread_.request_stop();

        capture_queue_.shutdown();
        debug_recording_queue_.shutdown();

        capture_thread_ = std::jthread{};
        worker_thread_ = std::jthread{};
    }

    [[nodiscard]] auto snapshot() const -> MockRuntimeSnapshot {
        const std::scoped_lock lock(mutex_);
        return snapshot_;
    }

    [[nodiscard]] auto observations() const -> std::vector<MockRuntimeObservation> {
        const std::scoped_lock lock(mutex_);
        return observations_;
    }

    auto try_pop_debug_recording(MockRuntimeObservation& observation) -> bool {
        return debug_recording_queue_.try_pop(observation);
    }

private:
    auto capture_loop(const std::stop_token& stop_token) -> void {
        while (!stop_token.stop_requested()) {
            const std::uint64_t next_sequence = next_capture_sequence_.fetch_add(1) + 1;
            Frame frame {
                .image = {},
                .timestamp = Clock::now() - config_.synthetic_capture_age,
            };

            try {
                const std::size_t overwrites = capture_queue_.push(std::move(frame));
                {
                    const std::scoped_lock lock(mutex_);
                    snapshot_.captured_frames = next_sequence;
                    snapshot_.capture_overwrites = overwrites;
                    snapshot_.max_capture_queue_depth = 1;
                }
            } catch (const std::exception&) {
                break;
            }

            if (config_.max_capture_frames != 0 && next_sequence >= config_.max_capture_frames)
                break;

            std::this_thread::sleep_for(config_.capture_period);
        }

        const std::scoped_lock lock(mutex_);
        snapshot_.capture_thread_exited = true;
    }

    auto worker_loop(const std::stop_token& stop_token) -> void {
        while (!stop_token.stop_requested()) {
            Frame frame;
            try {
                frame = capture_queue_.pop();
            } catch (const std::exception&) {
                break;
            }

            const auto worker_start_time = Clock::now();
            record_worker_pop();

            const RuntimeMetricSample worker_start_metrics {
                .observation_age_ms = age_ms(frame.timestamp, worker_start_time),
                .input_age_at_worker_start_ms = age_ms(frame.timestamp, worker_start_time),
                .input_age_at_infer_start_ms = age_ms(frame.timestamp, worker_start_time),
            };
            if (config_.stale_policy.classify_before_worker(worker_start_metrics)
                == StaleReason::too_old_before_worker) {
                record_stale_before_worker();
                continue;
            }

            std::this_thread::sleep_for(config_.worker_start_delay);
            const auto infer_start_time = Clock::now();
            const RuntimeMetricLogSample before_inference = config_.stale_policy.make_before_inference_sample(
                frame.timestamp, worker_start_time, infer_start_time);
            if (before_inference.stale_reason != StaleReason::none) {
                record_stale_before();
                continue;
            }

            std::this_thread::sleep_for(config_.inference_latency);
            const auto publish_time = Clock::now();
            const RuntimeMetricLogSample after_publish = config_.stale_policy.make_after_publish_sample(
                frame.timestamp, worker_start_time, infer_start_time, publish_time);
            if (after_publish.stale_reason != StaleReason::none) {
                record_stale_after();
                continue;
            }

            publish_observation(after_publish);
        }

        const std::scoped_lock lock(mutex_);
        snapshot_.worker_thread_exited = true;
    }

    auto record_worker_pop() -> void {
        const std::scoped_lock lock(mutex_);
        ++snapshot_.worker_popped_frames;
    }

    auto record_stale_before() -> void {
        const std::scoped_lock lock(mutex_);
        ++snapshot_.stale_before_inference_drops;
    }

    auto record_stale_before_worker() -> void {
        const std::scoped_lock lock(mutex_);
        ++snapshot_.stale_before_worker_drops;
    }

    auto record_stale_after() -> void {
        const std::scoped_lock lock(mutex_);
        ++snapshot_.stale_after_publish_drops;
    }

    [[nodiscard]] auto next_purple_value() -> bool {
        if (config_.purple_sequence.empty())
            return false;

        const std::size_t index = next_detection_sequence_.fetch_add(1);
        return config_.purple_sequence[index % config_.purple_sequence.size()];
    }

    auto publish_observation(const RuntimeMetricLogSample& metrics) -> void {
        const bool purple = next_purple_value();
        const HitState hit_state = hit_state_machine_.update(purple);

        const MockRuntimeObservation observation {
            .sequence = next_observation_sequence_.fetch_add(1) + 1,
            .metrics = metrics,
            .hit_state = hit_state,
            .purple = purple,
        };

        std::size_t debug_overwrites = 0;
        try {
            debug_overwrites = debug_recording_queue_.push(observation);
        } catch (const std::exception&) {
            return;
        }

        const std::scoped_lock lock(mutex_);
        ++snapshot_.published_observations;
        ++snapshot_.debug_recording_outputs;
        snapshot_.debug_recording_overwrites = debug_overwrites;
        snapshot_.hit_state = hit_state;
        observations_.push_back(observation);
    }

    MockRuntimeConfig config_;
    LatestValue<Frame> capture_queue_;
    LatestValue<MockRuntimeObservation> debug_recording_queue_;
    HitStateMachine hit_state_machine_;
    std::jthread capture_thread_;
    std::jthread worker_thread_;
    std::atomic<bool> started_ = false;
    std::atomic<std::uint64_t> next_capture_sequence_ = 0;
    std::atomic<std::uint64_t> next_detection_sequence_ = 0;
    std::atomic<std::uint64_t> next_observation_sequence_ = 0;
    mutable std::mutex mutex_;
    MockRuntimeSnapshot snapshot_ { };
    std::vector<MockRuntimeObservation> observations_;
};

} // namespace rmcs_laser_guidance
