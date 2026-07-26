#include "vision/model_infer.hpp"

#include "vision/cuda_check.hpp"

#include <filesystem>
#include <memory>
#include <print>
#include <string>
#include <utility>

#include "vision/model_adapter.hpp"
#include "vision/model_runtime.hpp"

#include "vision/tensorrt_engine.hpp"

namespace rmcs_laser_guidance {

namespace {

constexpr int kInputWidth = 640;
constexpr int kInputHeight = 640;

auto build_tensorrt_run_result(
    const ModelRunResult& base, const std::vector<float>& output, std::int32_t input_w,
    std::int32_t input_h, float scale, float pad_x, float pad_y) -> ModelRunResult {
    ModelRunResult result;
    result.success = true;
    result.transform = ModelImageTransform{
        .original_width = input_w,
        .original_height = input_h,
        .input_width = kInputWidth,
        .input_height = kInputHeight,
        .scale = scale,
        .pad_x = pad_x,
        .pad_y = pad_y,
    };
    result.outputs.push_back(
        ModelTensorData{
            .name = "output0",
            .shape = {1, 300, 6},
            .element_type = "float32",
            .values = output,
        });
    return result;
}

} // namespace

struct ModelInfer::Details {
    explicit Details(InferenceConfig config_in)
        : config(std::move(config_in))
        , runtime_enabled(model_runtime_enabled_in_build())
        , runtime(config.backend == InferenceBackendKind::tensorrt
              ? std::filesystem::path{} : config.model_path) {
        initialize();
    }

    auto initialize() -> void {
        if (config.backend == InferenceBackendKind::tensorrt) {
            if (!cuda_device_available()) {
                message = "TensorRT requires CUDA, but no CUDA device available";
                return;
            }
            auto engine_result = TensorRTEngine::load(config.model_path.string());
            if (!engine_result) {
                message = "TensorRT: " + engine_result.error();
                return;
            }
            tensorrt_engine = std::make_unique<TensorRTEngine>(std::move(*engine_result));
            auto meta = tensorrt_engine->meta();
            std::println(
                "TensorRT engine loaded: {} ({} inputs, {} outputs)", meta.engine_path,
                meta.inputs.size(), meta.outputs.size());
            startup_ready = true;
            return;
        }

        if (config.model_path.empty()) {
            message = "model backend requires inference.model_path to be set";
            return;
        }
        if (!std::filesystem::exists(config.model_path)) {
            message = "configured ONNX model does not exist: " + config.model_path.string();
            return;
        }
        message = runtime.load();
        if (!message.empty())
            return;
        startup_ready = true;
    }

    auto make_base_result() const -> ModelInferResult {
        return {
            .enabled = runtime_enabled,
            .success = false,
            .contract_supported = false,
            .observation = {},
            .candidates = {},
            .inputs = runtime.input_values(),
            .outputs = runtime.output_values(),
            .message = message,
        };
    }

    auto infer_tensorrt(const Frame& frame, ModelInferResult result) const -> ModelInferResult {
        auto [input_data, transform] = preprocess_blob(frame.image, kInputWidth, kInputHeight);
        std::vector<float> output(300 * 6);
        auto run_result = tensorrt_engine->run(input_data, output);
        if (!run_result) {
            result.message = "TensorRT inference: " + run_result.error();
            return result;
        }
        auto run_model = build_tensorrt_run_result(
            {}, output, frame.image.cols, frame.image.rows, transform.scale,
            transform.pad_x, transform.pad_y);
        auto adapter_result = adapt_yolo_outputs(frame, run_model);
        result.success = adapter_result.success;
        result.contract_supported = adapter_result.contract_supported;
        result.observation = adapter_result.observation;
        result.candidates = adapter_result.candidates;
        result.message = adapter_result.message;
        return result;
    }

    auto infer_onnx(const Frame& frame, ModelInferResult result) const -> ModelInferResult {
        const auto adapter_result = adapt_yolo_outputs(frame, runtime);
        result.success = adapter_result.success;
        result.contract_supported = adapter_result.contract_supported;
        result.observation = adapter_result.observation;
        result.candidates = adapter_result.candidates;
        result.message = adapter_result.message;
        return result;
    }

    InferenceConfig config;
    bool runtime_enabled = false;
    bool startup_ready = false;
    std::string message{};
    ModelRuntime runtime;
    std::unique_ptr<TensorRTEngine> tensorrt_engine;
};

ModelInfer::ModelInfer(InferenceConfig config)
    : details_(std::make_unique<Details>(std::move(config))) {}

ModelInfer::~ModelInfer() = default;
ModelInfer::ModelInfer(ModelInfer&&) noexcept = default;
auto ModelInfer::operator=(ModelInfer&&) noexcept -> ModelInfer& = default;

auto ModelInfer::infer(const Frame& frame) const -> ModelInferResult {
    if (!details_->startup_ready)
        return details_->make_base_result();

    auto result = details_->make_base_result();
    if (frame.image.empty()) {
        result.message = "model backend received an empty frame";
        return result;
    }

    if (details_->tensorrt_engine)
        return details_->infer_tensorrt(frame, std::move(result));

    return details_->infer_onnx(frame, std::move(result));
}

auto ModelInfer::is_ready() const -> bool { return details_->startup_ready; }

auto ModelInfer::startup_message() const -> const std::string& { return details_->message; }

} // namespace rmcs_laser_guidance
