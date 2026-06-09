#pragma once

#include <expected>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "config.hpp"
#include "laser_guidance/runtime.hpp"
#include "types.hpp"
#include "vision/model_infer.hpp"

namespace rmcs_laser_guidance::runtime_internal {

class IInferRunner {
public:
    virtual ~IInferRunner() = default;

    [[nodiscard]] virtual auto is_ready() const -> bool = 0;
    [[nodiscard]] virtual auto startup_message() const -> const std::string& = 0;
    virtual auto infer(const Frame& frame) const -> ModelInferResult = 0;
};

using InferRunnerFactory = std::function<std::unique_ptr<IInferRunner>(const InferenceConfig&)>;

class InferenceFacade {
public:
    explicit InferenceFacade(Config config, InferRunnerFactory runner_factory = {});
    ~InferenceFacade();

    InferenceFacade(const InferenceFacade&) = delete;
    auto operator=(const InferenceFacade&) -> InferenceFacade& = delete;

    auto start() -> std::expected<void, std::string>;
    auto stop() -> void;
    auto infer(const Frame& frame) const -> std::optional<ModelInferResult>;
    auto set_active_backend(RuntimeBackend backend) -> bool;

    [[nodiscard]] auto active_backend() const -> std::optional<RuntimeBackend>;
    [[nodiscard]] auto active_backend_name() const -> std::string;
    [[nodiscard]] auto has_backend(RuntimeBackend backend) const -> bool;
    [[nodiscard]] auto enabled() const -> bool;

private:
    auto make_runner(const InferenceConfig& config) const -> std::unique_ptr<IInferRunner>;
    auto set_active_backend_locked(RuntimeBackend backend) -> bool;
    auto active_runner_locked() const -> const IInferRunner*;

    Config config_;
    InferRunnerFactory runner_factory_{};
    mutable std::mutex runners_mutex_;
    std::unique_ptr<IInferRunner> infer_onnx_;
    std::unique_ptr<IInferRunner> infer_trt_;
    RuntimeBackend active_backend_ = RuntimeBackend::onnx;
    bool has_active_backend_ = false;
    std::thread loader_;
};

} // namespace rmcs_laser_guidance::runtime_internal
