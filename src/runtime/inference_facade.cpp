#include "runtime/inference_facade.hpp"

#include <future>
#include <utility>

#include "runtime/runtime_support.hpp"

namespace rmcs_laser_guidance::runtime_internal {
namespace {

class ModelInferRunner final : public IInferRunner {
public:
    explicit ModelInferRunner(InferenceConfig config)
        : infer_(std::move(config)) {}

    [[nodiscard]] auto is_ready() const -> bool override { return infer_.is_ready(); }

    [[nodiscard]] auto startup_message() const -> const std::string& override {
        return infer_.startup_message();
    }

    auto infer(const Frame& frame) const -> ModelInferResult override { return infer_.infer(frame); }

private:
    ModelInfer infer_;
};

} // namespace

InferenceFacade::InferenceFacade(Config config, InferRunnerFactory runner_factory)
    : config_(std::move(config))
    , runner_factory_(std::move(runner_factory)) {}

InferenceFacade::~InferenceFacade() { stop(); }

auto InferenceFacade::start() -> std::expected<void, std::string> {
    stop();
    if (config_.inference.backend == InferenceBackendKind::bright_spot) {
        return {};
    }

    std::promise<std::string> ready_promise;
    auto ready_future = ready_promise.get_future();
    loader_ = std::thread([this, promise = std::move(ready_promise)]() mutable {
        auto build_runner =
            [this](InferenceConfig backend_config) -> std::pair<std::unique_ptr<IInferRunner>, std::string> {
            auto runner = make_runner(backend_config);
            if (!runner) {
                return {nullptr, "runner factory returned null"};
            }
            if (!runner->is_ready()) {
                auto error = runner->startup_message();
                if (error.empty()) {
                    error = "backend unavailable";
                }
                return {nullptr, std::move(error)};
            }
            return {std::move(runner), {}};
        };

        std::string error;
        try {
            auto onnx_cfg = config_.inference;
            onnx_cfg.backend = InferenceBackendKind::model;
            onnx_cfg.model_path = config_.inference.model_path;
            auto [onnx_runner, onnx_error] = build_runner(std::move(onnx_cfg));

            auto trt_cfg = config_.inference;
            trt_cfg.backend = InferenceBackendKind::tensorrt;
            trt_cfg.model_path = config_.runtime.engine_path.empty() ? config_.inference.model_path
                                                                     : config_.runtime.engine_path;
            auto [trt_runner, trt_error] = build_runner(std::move(trt_cfg));

            {
                std::scoped_lock lock(runners_mutex_);
                infer_onnx_ = std::move(onnx_runner);
                infer_trt_ = std::move(trt_runner);

                const RuntimeBackend preferred =
                    config_.inference.backend == InferenceBackendKind::tensorrt
                        ? RuntimeBackend::tensorrt
                        : RuntimeBackend::onnx;
                if (!set_active_backend_locked(preferred)
                    && !set_active_backend_locked(RuntimeBackend::onnx)
                    && !set_active_backend_locked(RuntimeBackend::tensorrt)) {
                    has_active_backend_ = false;
                    if (!onnx_error.empty()) {
                        error = std::move(onnx_error);
                    } else if (!trt_error.empty()) {
                        error = std::move(trt_error);
                    } else {
                        error = "no requested inference backend available";
                    }
                }
            }
        } catch (const std::exception& e) {
            error = e.what();
        }
        promise.set_value(error);
    });

    const auto error = ready_future.get();
    if (!error.empty()) {
        stop();
        return std::unexpected(error);
    }
    return {};
}

auto InferenceFacade::stop() -> void {
    if (loader_.joinable()) {
        loader_.join();
    }
    std::scoped_lock lock(runners_mutex_);
    has_active_backend_ = false;
    infer_onnx_.reset();
    infer_trt_.reset();
}

auto InferenceFacade::infer(const Frame& frame) const -> std::optional<ModelInferResult> {
    if (config_.inference.backend == InferenceBackendKind::bright_spot) {
        return std::nullopt;
    }
    std::scoped_lock lock(runners_mutex_);
    const auto* active = active_runner_locked();
    if (active == nullptr) {
        return std::nullopt;
    }
    return active->infer(frame);
}

auto InferenceFacade::set_active_backend(const RuntimeBackend backend) -> bool {
    std::scoped_lock lock(runners_mutex_);
    return set_active_backend_locked(backend);
}

auto InferenceFacade::active_backend() const -> std::optional<RuntimeBackend> {
    std::scoped_lock lock(runners_mutex_);
    if (!has_active_backend_) {
        return std::nullopt;
    }
    return active_backend_;
}

auto InferenceFacade::active_backend_name() const -> std::string {
    const auto backend = active_backend();
    return backend.has_value() ? to_backend_name(*backend) : std::string{};
}

auto InferenceFacade::has_backend(const RuntimeBackend backend) const -> bool {
    std::scoped_lock lock(runners_mutex_);
    return backend == RuntimeBackend::tensorrt ? infer_trt_ != nullptr : infer_onnx_ != nullptr;
}

auto InferenceFacade::enabled() const -> bool {
    return config_.inference.backend != InferenceBackendKind::bright_spot;
}

auto InferenceFacade::make_runner(const InferenceConfig& config) const -> std::unique_ptr<IInferRunner> {
    if (runner_factory_) {
        return runner_factory_(config);
    }
    return std::make_unique<ModelInferRunner>(config);
}

auto InferenceFacade::set_active_backend_locked(const RuntimeBackend backend) -> bool {
    IInferRunner* preferred = backend == RuntimeBackend::tensorrt ? infer_trt_.get() : infer_onnx_.get();
    if (preferred == nullptr) {
        return false;
    }
    active_backend_ = backend;
    has_active_backend_ = true;
    return true;
}

auto InferenceFacade::active_runner_locked() const -> const IInferRunner* {
    if (!has_active_backend_) {
        return nullptr;
    }
    return active_backend_ == RuntimeBackend::tensorrt ? infer_trt_.get() : infer_onnx_.get();
}

} // namespace rmcs_laser_guidance::runtime_internal
