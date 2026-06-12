#include "runtime/perception_runner.hpp"

#include <algorithm>
#include <utility>

namespace rmcs_laser_guidance::runtime_internal {
namespace {

auto to_enemy_class_id(const EnemyColor color) -> int { return static_cast<int>(color); }

auto to_enemy_color(const int class_id) -> EnemyColor {
    switch (class_id) {
    case 1: return EnemyColor::red;
    case 2: return EnemyColor::blue;
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

auto filter_detections(std::vector<Detection>& detections, const EnemyColor enemy_color) -> void {
    const int enemy_class_id = to_enemy_class_id(enemy_color);
    if (enemy_class_id < 0) {
        return;
    }
    detections.erase(
        std::remove_if(
            detections.begin(), detections.end(),
            [enemy_class_id](const Detection& detection) {
                return detection.class_id != 0 && detection.class_id != enemy_class_id;
            }),
        detections.end());
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

    {
        std::scoped_lock lock(state_mutex_);
        last_error_.clear();
        started_ = true;
    }

    if (enabled()) {
        worker_ = std::thread([this] { run(); });
    }
    return {};
}

auto PerceptionRunner::submit(Frame frame) -> bool {
    if (!enabled()) {
        return true;
    }

    try {
        frame_queue_->push(QueuedFrame{
            .image = std::move(frame.image),
            .capture_time = frame.timestamp,
        });
        return true;
    } catch (const std::exception& e) {
        std::scoped_lock lock(state_mutex_);
        last_error_ = e.what();
        return false;
    }
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
    if (worker_.joinable()) {
        worker_.join();
    }

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
    if (!has_backend(backend)) {
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

auto PerceptionRunner::run() -> void {
    try {
        while (true) {
            QueuedFrame queued_frame;
            try {
                queued_frame = frame_queue_->pop();
            } catch (const std::exception&) {
                break;
            }

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

            auto batch = to_detection_batch(*infer_result);
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
                continue;
            }

            std::scoped_lock lock(result_mutex_);
            latest_detection_ = std::move(batch);
            latest_ekf_ = tracker_.state();
        }
    } catch (const std::exception& e) {
        std::scoped_lock lock(state_mutex_);
        last_error_ = std::string("perception worker error: ") + e.what();
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

    auto onnx = std::make_unique<ModelInfer>(make_backend_config(config_, InferenceBackendKind::model));
    auto trt = std::make_unique<ModelInfer>(make_backend_config(config_, InferenceBackendKind::tensorrt));

    const std::string onnx_error = onnx->is_ready() ? std::string{} : onnx->startup_message();
    const std::string trt_error = trt->is_ready() ? std::string{} : trt->startup_message();

    {
        std::scoped_lock lock(state_mutex_);
        if (onnx->is_ready()) {
            infer_onnx_ = std::move(onnx);
        }
        if (trt->is_ready()) {
            infer_trt_ = std::move(trt);
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
            if (!onnx_error.empty()) {
                return std::unexpected(onnx_error);
            }
            if (!trt_error.empty()) {
                return std::unexpected(trt_error);
            }
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
