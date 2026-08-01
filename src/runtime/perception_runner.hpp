#pragma once

#include <atomic>
#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>

#include "config.hpp"
#include "laser_guidance/runtime.hpp"
#include "tracking/ekf_tracker.hpp"
#include "tracking/freshness_queue.hpp"
#include "tracking/runtime_metrics.hpp"
#include "vision/model_infer.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct QueuedFrame {
    cv::Mat image;
    Clock::time_point capture_time{};
};

struct PerceptionPollResult {
    DetectionBatch detection{};
    std::optional<EkfState> ekf_state{};
};

class PerceptionRunner {
public:
    explicit PerceptionRunner(Config config);
    ~PerceptionRunner();

    PerceptionRunner(const PerceptionRunner&) = delete;
    auto operator=(const PerceptionRunner&) -> PerceptionRunner& = delete;

    auto start() -> std::expected<void, std::string>;
    auto submit(Frame frame) -> bool;
    [[nodiscard]] auto poll() const -> PerceptionPollResult;
    [[nodiscard]] auto overwrite_count() const -> std::size_t;
    auto shutdown() -> void;
    auto stop() -> void;

    auto set_enemy_color(EnemyColor color) -> void;
    auto set_active_backend(RuntimeBackend backend) -> bool;

    [[nodiscard]] auto has_backend(RuntimeBackend backend) const -> bool;
    [[nodiscard]] auto active_backend() const -> std::optional<RuntimeBackend>;
    [[nodiscard]] auto active_backend_name() const -> std::string;
    [[nodiscard]] auto enabled() const -> bool;
    [[nodiscard]] auto last_error() const -> std::string;
    [[nodiscard]] auto degraded() const -> bool;

private:
    auto run() -> void;
    auto initialize_backends() -> std::expected<void, std::string>;
    [[nodiscard]] auto infer_active(const Frame& frame) const -> std::optional<ModelInferResult>;

    Config config_{};
    EkfTracker tracker_;
    StaleFramePolicy stale_policy_{};
    std::unique_ptr<LatestValue<QueuedFrame>> frame_queue_{};
    mutable std::mutex result_mutex_;
    DetectionBatch latest_detection_{};
    std::optional<EkfState> latest_ekf_{};
    mutable std::mutex state_mutex_;
    EnemyColor enemy_color_ = EnemyColor::auto_select;
    std::string last_error_{};
    std::unique_ptr<ModelInfer> infer_onnx_{};
    std::unique_ptr<ModelInfer> infer_trt_{};
    std::optional<RuntimeBackend> active_backend_{};
    bool started_ = false;
    // Set when the worker thread dies on an exception. degraded() reports it
    // so the control loop stops using frozen detection results.
    std::atomic<bool> worker_failed_{false};
    std::jthread worker_{};
};

} // namespace rmcs_laser_guidance::runtime_internal
