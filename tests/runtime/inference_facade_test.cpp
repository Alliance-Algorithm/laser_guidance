#include <print>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <opencv2/core/mat.hpp>

#include "runtime/inference_facade.hpp"
#include "test_utils.hpp"

namespace {

struct FakeSpec {
    bool ready = true;
    std::string message{};
};

class FakeRunner final : public rmcs_laser_guidance::runtime_internal::IInferRunner {
public:
    explicit FakeRunner(FakeSpec spec)
        : spec_(std::move(spec)) {}

    [[nodiscard]] auto is_ready() const -> bool override { return spec_.ready; }
    [[nodiscard]] auto startup_message() const -> const std::string& override { return spec_.message; }

    auto infer(const rmcs_laser_guidance::Frame&) const -> rmcs_laser_guidance::ModelInferResult override {
        return rmcs_laser_guidance::ModelInferResult{
            .enabled = true,
            .success = true,
            .contract_supported = true,
            .message = "ok",
        };
    }

private:
    FakeSpec spec_;
};

auto make_config(rmcs_laser_guidance::InferenceBackendKind backend)
    -> rmcs_laser_guidance::Config {
    rmcs_laser_guidance::Config config;
    config.inference.backend = backend;
    config.inference.model_path = "models/mock.onnx";
    config.runtime.engine_path = "models/mock.engine";
    return config;
}

} // namespace

int main() {
    try {
        using rmcs_laser_guidance::Frame;
        using rmcs_laser_guidance::InferenceBackendKind;
        using rmcs_laser_guidance::RuntimeBackend;
        using rmcs_laser_guidance::runtime_internal::InferenceFacade;
        using rmcs_laser_guidance::runtime_internal::InferRunnerFactory;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_contains;

        auto make_factory = [](FakeSpec onnx, FakeSpec trt) -> InferRunnerFactory {
            return [onnx = std::move(onnx), trt = std::move(trt)](
                       const rmcs_laser_guidance::InferenceConfig& config)
                       -> std::unique_ptr<rmcs_laser_guidance::runtime_internal::IInferRunner> {
                if (config.backend == InferenceBackendKind::tensorrt) {
                    return std::make_unique<FakeRunner>(trt);
                }
                return std::make_unique<FakeRunner>(onnx);
            };
        };

        {
            InferenceFacade facade(
                make_config(InferenceBackendKind::tensorrt),
                make_factory(FakeSpec{.ready = true}, FakeSpec{.ready = true}));
            const auto start_result = facade.start();
            require(start_result.has_value(), "preferred TRT backend should start");
            require(
                facade.active_backend() == RuntimeBackend::tensorrt,
                "preferred backend should stay tensorrt");
            require(facade.active_backend_name() == "TensorRT", "backend name mismatch");
            require(facade.set_active_backend(RuntimeBackend::onnx), "switch to onnx should work");
            require(facade.active_backend() == RuntimeBackend::onnx, "backend switch mismatch");
        }

        {
            InferenceFacade facade(
                make_config(InferenceBackendKind::tensorrt),
                make_factory(FakeSpec{.ready = true}, FakeSpec{.ready = false, .message = "trt down"}));
            const auto start_result = facade.start();
            require(start_result.has_value(), "fallback ONNX backend should start");
            require(
                facade.active_backend() == RuntimeBackend::onnx,
                "fallback backend should land on onnx");
            require(!facade.set_active_backend(RuntimeBackend::tensorrt), "dead backend switch must fail");
        }

        {
            InferenceFacade facade(
                make_config(InferenceBackendKind::model),
                make_factory(
                    FakeSpec{.ready = false, .message = "onnx unavailable"},
                    FakeSpec{.ready = false, .message = "trt unavailable"}));
            const auto start_result = facade.start();
            require(!start_result.has_value(), "no backend should fail start");
            require_contains(start_result.error(), "onnx unavailable", "start error");
            require(!facade.active_backend().has_value(), "failed start should have no active backend");
        }

        {
            InferenceFacade facade(
                make_config(InferenceBackendKind::model),
                make_factory(FakeSpec{.ready = true}, FakeSpec{.ready = false}));
            const auto start_result = facade.start();
            require(start_result.has_value(), "onnx-only backend should start");
            const Frame frame{.image = cv::Mat(4, 4, CV_8UC3), .timestamp = rmcs_laser_guidance::Clock::now()};
            require(facade.infer(frame).has_value(), "active backend should infer");
            facade.stop();
            facade.stop();
            require(!facade.infer(frame).has_value(), "stopped facade should not infer");
            require(!facade.active_backend().has_value(), "stopped facade should clear active backend");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "inference_facade_test failed: {}", e.what());
        return 1;
    }
}
