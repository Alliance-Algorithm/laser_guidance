#include "vision/model_infer.hpp"

#include "vision/cuda_check.hpp"
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <memory>
#include <print>
#include <string>
#include <utility>

#include "vision/model_adapter.hpp"
#include "vision/model_runtime.hpp"
#include "vision/preprocess_cuda.hpp"

#include "vision/tensorrt_engine.hpp"

namespace rmcs_laser_guidance {

namespace {

constexpr int kDeploymentInputSize = 1216;

auto model_value_infos(const std::vector<TensorRTMeta::TensorInfo>& tensors)
    -> std::vector<ModelValueInfo> {
    std::vector<ModelValueInfo> values;
    values.reserve(tensors.size());
    for (const auto& tensor : tensors) {
        values.push_back(
            ModelValueInfo{
                .name = tensor.name,
                .shape = tensor.shape,
                .element_type = "float32",
            });
    }
    return values;
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
        std::vector<ModelValueInfo> inputs;
        std::vector<ModelValueInfo> outputs;
        if (tensorrt_engine) {
            const auto& meta = tensorrt_engine->meta();
            inputs = model_value_infos(meta.inputs);
            outputs = model_value_infos(meta.outputs);
        } else {
            inputs = runtime.input_values();
            outputs = runtime.output_values();
        }
        return {
            .enabled = runtime_enabled,
            .success = false,
            .contract_supported = false,
            .observation = {},
            .candidates = {},
            .inputs = std::move(inputs),
            .outputs = std::move(outputs),
            .message = message,
        };
    }

    auto infer_tensorrt(const Frame& frame, ModelInferResult result) const -> ModelInferResult {
        const auto& meta = tensorrt_engine->meta();
        if (meta.outputs.size() != 1 || meta.outputs.front().name != "output0") {
            result.message = "TensorRT output contract requires exactly one output named output0";
            return result;
        }

        std::vector<float> output;
        std::vector<std::int64_t> output_shape;
        const int out_size = kDeploymentInputSize;

        // Fast GPU path: bypass CPU preprocess_blob entirely.
        // Require continuous CV_8UC3 BGR; normalize if needed.
        cv::Mat bgr = frame.image;
        if (bgr.channels() == 1) {
            cv::cvtColor(bgr, bgr, cv::COLOR_GRAY2BGR);
        } else if (bgr.channels() == 4) {
            cv::cvtColor(bgr, bgr, cv::COLOR_BGRA2BGR);
        }
        if (!bgr.isContinuous() || bgr.type() != CV_8UC3) {
            bgr = bgr.clone();
        }

        auto run_result = tensorrt_engine->run_from_bgr(
            bgr.data,
            bgr.cols, bgr.rows,
            out_size, out_size,
            output, output_shape);

        if (!run_result) {
            result.message = "TensorRT inference (GPU preprocess): " + run_result.error();
            return result;
        }
        if (output_shape != std::vector<std::int64_t>{1, 300, 6}) {
            result.message = "TensorRT output contract requires resolved shape {1, 300, 6}";
            return result;
        }

        // Compute the transform that maps model coords back to original image.
        int scaled_w{}, scaled_h{}, pad_left{}, pad_top{};
        preprocess_cuda_letterbox_params(
            frame.image.cols, frame.image.rows, out_size, out_size,
            scaled_w, scaled_h, pad_left, pad_top);
        const float scale = static_cast<float>(scaled_w) / static_cast<float>(frame.image.cols);

        ModelRunResult run_model;
        run_model.success = true;
        run_model.transform = ModelImageTransform{
            .original_width  = frame.image.cols,
            .original_height = frame.image.rows,
            .input_width     = out_size,
            .input_height    = out_size,
            .scale           = scale,
            .pad_x           = static_cast<float>(pad_left),
            .pad_y           = static_cast<float>(pad_top),
        };
        run_model.outputs.push_back(
            ModelTensorData{
                .name = meta.outputs.front().name,
                .shape = std::move(output_shape),
                .element_type = "float32",
                .values = output,
            });
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
