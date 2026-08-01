#include "runtime/perception_runner.hpp"

#include <algorithm>
#include <utility>

namespace rmcs_laser_guidance::runtime_internal {
namespace {

auto to_enemy_color(const int class_id) -> EnemyColor {
    switch (class_id) {
    case 0: return EnemyColor::red;
    case 1: return EnemyColor::blue;
    default: return EnemyColor::auto_select;
    }
}

auto to_backend_name(const RuntimeBackend backend) -> std::string {
    return backend == RuntimeBackend::tensorrt ? "TensorRT" : "ONNX";
}

auto to_detection(const ModelCandidate& candidate) -> Detection {
    return Detection{
        .score = candidate.score,
        .class_id = candidate.class_id,
        .bbox = candidate.bbox,
        .center = candidate.center,
    };
}

// Order detections so the physical guidance path prefers the RM2026
// countermeasure targets (purple=2, colorless=3) over any other class the
// model emits (red/blue enemy aircraft), then falls back to score order.
// Every class stays available to guidance — this only changes selection
// priority, never discards detections.
auto filter_detections(std::vector<Detection>& detections, const EnemyColor /*enemy_color*/)
    -> void {
    const auto priority = [](const Detection& detection) -> int {
        if (detection.class_id == 2) {
            return 0;
        }
        if (detection.class_id == 3) {
            return 1;
        }
        return 2;
    };
    std::stable_sort(
        detections.begin(), detections.end(),
        [&](const Detection& lhs, const Detection& rhs) {
            const int lhs_priority = priority(lhs);
            const int rhs_priority = priority(rhs);
            if (lhs_priority != rhs_priority) {
                return lhs_priority < rhs_priority;
            }
            return lhs.score > rhs.score;
        });
}

auto to_detection_batch(const ModelInferResult& result) -> DetectionBatch {
    DetectionBatch batch;
    batch.detections.reserve(result.candidates.size());
    for (const auto& candidate : result.candidates) {
        batch.detections.push_back(to_detection(candidate));
    }
    batch.detected = result.observation.detected;
    batch.selected_center = result.observation.center;
    return batch;
}

auto make_backend_config(const Config& config, const InferenceBackendKind backend)
    -> InferenceConfig {
    InferenceConfig backend_config = config.inference;
    backend_config.backend = backend;
    if (backend == InferenceBackendKind::tensorrt && !config.runtime.engine_path.empty()) {
        backend_config.model_path = config.runtime.engine_path;
    } else if (backend == InferenceBackendKind::model
               && config.inference.model_path.extension() == ".engine") {
        backend_config.model_path.clear();
    }
    return backend_config;
}

} // namespace

PerceptionRunner::PerceptionRunner(Config config)
    : config_(std::move(config))
    , tracker_(config_.ekf)
    , stale_policy_{
          .max_input_age_ms = std::chrono::milliseconds(config_.runtime.max_input_age_ms),
          .max_observation_age_ms =
              std::chrono::milliseconds(config_.runtime.max_observation_age_ms),
      }
    , enemy_color_(to_enemy_color(config_.inference.enemy_class_id)) {}

PerceptionRunner::~PerceptionRunner() { stop(); }

auto PerceptionRunner::start() -> std::expected<void, std::string> {
    stop();
    frame_queue_ = std::make_unique<LatestValue<QueuedFrame>>();
    tracker_ = EkfTracker(config_.ekf);

    if (auto result = initialize_backends(); !result) {
        std::scoped_lock lock(state_mutex_);
        last_error_ = result.error();
        return result;
    }

    worker_failed_.store(false);

    {
        std::scoped_lock lock(state_mutex_);
        last_error_.clear();
        started_ = true;
    }

    if (enabled()) {
        worker_ = std::jthread([this](std::stop_token) { run(); });
    }
    return {};
}

auto PerceptionRunner::submit(Frame frame) -> bool {
    if (!enabled()) {
        return true;
    }

    if (!frame_queue_ || frame_queue_->is_shutdown()) {
        return false;
    }
    frame_queue_->push(QueuedFrame{
        .image = std::move(frame.image),
        .capture_time = frame.timestamp,
    });
    return true;
}

auto PerceptionRunner::poll() const -> PerceptionPollResult {
    std::scoped_lock lock(result_mutex_);
    return PerceptionPollResult{
        .detection = latest_detection_,
        .ekf_state = latest_ekf_,
    };
}

auto PerceptionRunner::overwrite_count() const -> std::size_t {
    return frame_queue_ ? frame_queue_->overwrite_count() : 0;
}

auto PerceptionRunner::shutdown() -> void {
    if (enabled() && frame_queue_) {
        frame_queue_->shutdown();
    }
}

auto PerceptionRunner::stop() -> void {
    shutdown();
    worker_ = std::jthread{};
    worker_failed_.store(false);

    {
        std::scoped_lock lock(state_mutex_);
        started_ = false;
        active_backend_.reset();
        infer_onnx_.reset();
        infer_trt_.reset();
    }
    frame_queue_.reset();

    {
        std::scoped_lock lock(result_mutex_);
        latest_detection_ = {};
        latest_ekf_.reset();
    }
}

auto PerceptionRunner::set_enemy_color(const EnemyColor color) -> void {
    std::scoped_lock lock(state_mutex_);
    enemy_color_ = color;
}

auto PerceptionRunner::set_active_backend(const RuntimeBackend backend) -> bool {
    std::scoped_lock lock(state_mutex_);
    if (backend == RuntimeBackend::tensorrt) {
        if (infer_trt_ == nullptr) {
            return false;
        }
    } else if (infer_onnx_ == nullptr) {
        return false;
    }
    active_backend_ = backend;
    return true;
}

auto PerceptionRunner::has_backend(const RuntimeBackend backend) const -> bool {
    if (backend == RuntimeBackend::tensorrt) {
        return infer_trt_ != nullptr;
    }
    return infer_onnx_ != nullptr;
}

auto PerceptionRunner::active_backend() const -> std::optional<RuntimeBackend> {
    std::scoped_lock lock(state_mutex_);
    return active_backend_;
}

auto PerceptionRunner::active_backend_name() const -> std::string {
    const auto backend = active_backend();
    return backend.has_value() ? to_backend_name(*backend) : std::string{};
}

auto PerceptionRunner::enabled() const -> bool {
    return config_.inference.backend != InferenceBackendKind::bright_spot;
}

auto PerceptionRunner::last_error() const -> std::string {
    std::scoped_lock lock(state_mutex_);
    return last_error_;
}

auto PerceptionRunner::degraded() const -> bool {
    std::scoped_lock lock(state_mutex_);
    // worker_failed_ is set without the lock by the dying worker; treat it as
    // degraded so callers stop trusting frozen detection results.
    return !enabled() || !active_backend_.has_value() || worker_failed_.load();
}

auto PerceptionRunner::run() -> void {
    try {
        while (true) {
            auto queued = frame_queue_->pop();
            if (!queued.has_value()) {
                break;
            }
            QueuedFrame queued_frame = std::move(*queued);

            const auto worker_start = Clock::now();
            const auto before_infer = stale_policy_.make_before_inference_sample(
                queued_frame.capture_time, worker_start, Clock::now());
            if (before_infer.stale_reason != StaleReason::none) {
                continue;
            }

            const Frame infer_frame{
                .image = queued_frame.image,
                .timestamp = queued_frame.capture_time,
            };
            const auto infer_start = Clock::now();
            auto infer_result = infer_active(infer_frame);
            if (!infer_result.has_value()) {
                continue;
            }
            if (!infer_result->success) {
                std::scoped_lock lock(state_mutex_);
                last_error_ = "perception inference failed: " + infer_result->message;
                continue;
            }
            {
                std::scoped_lock lock(state_mutex_);
                if (last_error_.starts_with("perception inference failed: ")) {
                    last_error_.clear();
                }
            }

            auto batch = to_detection_batch(*infer_result);
            batch.capture_time = queued_frame.capture_time;
            EnemyColor enemy_color = EnemyColor::auto_select;
            {
                std::scoped_lock lock(state_mutex_);
                enemy_color = enemy_color_;
            }

            filter_detections(batch.detections, enemy_color);
            if (!batch.detections.empty() && batch.detections.front().score >= 0.25F) {
                batch.detected = true;
                batch.selected_center = batch.detections.front().center;
            } else {
                batch.detected = false;
                batch.selected_center = {-1.0F, -1.0F};
            }

            if (batch.detected) {
                tracker_.process(batch.selected_center, infer_frame.timestamp);
            } else {
                tracker_.predict(infer_frame.timestamp);
            }

            const auto publish_time = Clock::now();
            const auto after_publish = stale_policy_.make_after_publish_sample(
                queued_frame.capture_time, worker_start, infer_start, publish_time);
            if (after_publish.stale_reason != StaleReason::none) {
                // The observation is too old to drive aiming, but the tracker
                // has already advanced on this measurement — publish the
                // filter state so the aim point keeps extrapolating instead of
                // freezing on the last non-stale frame.
                std::scoped_lock lock(result_mutex_);
                latest_ekf_ = tracker_.state();
                continue;
            }

            std::scoped_lock lock(result_mutex_);
            latest_detection_ = std::move(batch);
            latest_ekf_ = tracker_.state();
        }
    } catch (const std::exception& e) {
        std::scoped_lock lock(state_mutex_);
        last_error_ = std::string("perception worker error: ") + e.what();
        worker_failed_.store(true);
    }
}

auto PerceptionRunner::initialize_backends() -> std::expected<void, std::string> {
    {
        std::scoped_lock lock(state_mutex_);
        infer_onnx_.reset();
        infer_trt_.reset();
        active_backend_.reset();
    }

    if (!enabled()) {
        return {};
    }

    const bool prefer_trt = config_.inference.backend == InferenceBackendKind::tensorrt;
    auto primary = std::make_unique<ModelInfer>(make_backend_config(
        config_, prefer_trt ? InferenceBackendKind::tensorrt : InferenceBackendKind::model));
    const std::string primary_error =
        primary->is_ready() ? std::string{} : primary->startup_message();

    std::unique_ptr<ModelInfer> fallback;
    std::string fallback_error;
    if (!primary->is_ready()) {
        fallback = std::make_unique<ModelInfer>(make_backend_config(
            config_, prefer_trt ? InferenceBackendKind::model : InferenceBackendKind::tensorrt));
        fallback_error = fallback->is_ready() ? std::string{} : fallback->startup_message();
    }

    {
        std::scoped_lock lock(state_mutex_);
        if (primary->is_ready()) {
            if (prefer_trt) {
                infer_trt_ = std::move(primary);
            } else {
                infer_onnx_ = std::move(primary);
            }
        } else if (fallback && fallback->is_ready()) {
            if (prefer_trt) {
                infer_onnx_ = std::move(fallback);
            } else {
                infer_trt_ = std::move(fallback);
            }
        }

        const RuntimeBackend preferred =
            config_.inference.backend == InferenceBackendKind::tensorrt
                ? RuntimeBackend::tensorrt
                : RuntimeBackend::onnx;
        if (has_backend(preferred)) {
            active_backend_ = preferred;
        } else if (has_backend(RuntimeBackend::onnx)) {
            active_backend_ = RuntimeBackend::onnx;
        } else if (has_backend(RuntimeBackend::tensorrt)) {
            active_backend_ = RuntimeBackend::tensorrt;
        }

        if (!active_backend_.has_value()) {
            if (!primary_error.empty())
                return std::unexpected(primary_error);
            if (!fallback_error.empty())
                return std::unexpected(fallback_error);
            return std::unexpected("no requested inference backend available");
        }
    }

    return {};
}

auto PerceptionRunner::infer_active(const Frame& frame) const -> std::optional<ModelInferResult> {
    if (!enabled()) {
        return std::nullopt;
    }

    const ModelInfer* active = nullptr;
    {
        std::scoped_lock lock(state_mutex_);
        if (active_backend_ == RuntimeBackend::tensorrt) {
            active = infer_trt_.get();
        } else if (active_backend_ == RuntimeBackend::onnx) {
            active = infer_onnx_.get();
        }
    }
    if (active == nullptr) {
        return std::nullopt;
    }
    return active->infer(frame);
}

} // namespace rmcs_laser_guidance::runtime_internal
