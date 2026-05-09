#include "vision/model_infer.hpp"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <print>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "vision/model_adapter.hpp"
#include "vision/model_runtime.hpp"

#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT
#include "vision/tensorrt_engine.hpp"
#endif

namespace rmcs_laser_guidance {

#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT
namespace {

constexpr int kInputWidth = 640;
constexpr int kInputHeight = 640;

auto preprocess_for_tensorrt(const cv::Mat& image) -> std::vector<float> {
    cv::Mat bgr;
    if (image.channels() == 4) cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
    else if (image.channels() == 1) cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    else bgr = image;

    float scale = std::min(static_cast<float>(kInputWidth) / bgr.cols,
                           static_cast<float>(kInputHeight) / bgr.rows);
    int rw = std::max(1, static_cast<int>(std::lround(bgr.cols * scale)));
    int rh = std::max(1, static_cast<int>(std::lround(bgr.rows * scale)));

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(rw, rh), 0.0, 0.0, cv::INTER_LINEAR);

    cv::Mat letterbox(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(114, 114, 114));
    int pad_x = (kInputWidth - rw) / 2;
    int pad_y = (kInputHeight - rh) / 2;
    resized.copyTo(letterbox(cv::Rect(pad_x, pad_y, rw, rh)));

    cv::Mat rgb;
    cv::cvtColor(letterbox, rgb, cv::COLOR_BGR2RGB);
    cv::Mat rgb_float;
    rgb.convertTo(rgb_float, CV_32F, 1.0 / 255.0);

    std::vector<cv::Mat> channels;
    cv::split(rgb_float, channels);

    std::vector<float> input(3 * kInputHeight * kInputWidth);
    std::size_t ch_size = kInputHeight * kInputWidth;
    for (std::size_t c = 0; c < 3; ++c)
        std::copy(channels[c].ptr<float>(0), channels[c].ptr<float>(0) + ch_size,
                  input.begin() + c * ch_size);
    return input;
}

auto build_tensorrt_run_result(
    const ModelRunResult& base, const std::vector<float>& output,
    std::int32_t input_w, std::int32_t input_h, float scale, float pad_x, float pad_y)
    -> ModelRunResult {
    ModelRunResult result;
    result.success = true;
    result.transform = ModelImageTransform{
        .original_width = input_w, .original_height = input_h,
        .input_width = kInputWidth, .input_height = kInputHeight,
        .scale = scale, .pad_x = pad_x, .pad_y = pad_y,
    };
    result.outputs.push_back(ModelTensorData{
        .name = "output0", .shape = {1, 300, 6},
        .element_type = "float32", .values = output,
    });
    return result;
}

}
#endif

struct ModelInfer::Details {
    explicit Details(InferenceConfig config_in)
        : config(std::move(config_in))
        , runtime_enabled(model_runtime_enabled_in_build())
        , runtime(config.model_path)
        , adapter(make_default_model_adapter()) {
        initialize();
    }

    auto initialize() -> void {
        if (config.backend == InferenceBackendKind::tensorrt) {
#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT
            auto engine_result = TensorRTEngine::load(config.model_path.string());
            if (!engine_result) {
                message = "TensorRT: " + engine_result.error();
                return;
            }
            tensorrt_engine = std::make_unique<TensorRTEngine>(std::move(*engine_result));
            auto meta = tensorrt_engine->meta();
            std::println("TensorRT engine loaded: {} ({} inputs, {} outputs)",
                         meta.engine_path, meta.inputs.size(), meta.outputs.size());
            startup_ready = true;
#else
            message = "tensorrt backend requires -DRMCS_LASER_GUIDANCE_WITH_TENSORRT=ON";
#endif
            return;
        }

        if (!runtime_enabled) {
            message =
                "model backend requires ONNX Runtime support; reconfigure with "
                "-DRMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON";
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
        if (!message.empty()) return;
        startup_ready = true;
    }

    auto make_base_result() const -> ModelInferResult {
        return {
            .enabled = runtime_enabled,
            .success = false, .contract_supported = false,
            .observation = {}, .candidates = {},
            .inputs = runtime.input_values(),
            .outputs = runtime.output_values(),
            .message = message,
        };
    }

    InferenceConfig config;
    bool runtime_enabled = false;
    bool startup_ready = false;
    std::string message {};
    ModelRuntime runtime;
    std::unique_ptr<ModelAdapter> adapter;
#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT
    std::unique_ptr<TensorRTEngine> tensorrt_engine;
#endif
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

#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT
    if (details_->tensorrt_engine) {
        std::vector<float> input = preprocess_for_tensorrt(frame.image);
        std::vector<float> output(300 * 6);
        auto run_result = details_->tensorrt_engine->run(input, output);
        if (!run_result) {
            result.message = "TensorRT inference: " + run_result.error();
            return result;
        }
        float scale = std::min(kInputWidth / static_cast<float>(frame.image.cols),
                               kInputHeight / static_cast<float>(frame.image.rows));
        int rw = std::max(1, static_cast<int>(std::lround(frame.image.cols * scale)));
        int rh = std::max(1, static_cast<int>(std::lround(frame.image.rows * scale)));
        float pad_x = (kInputWidth - rw) / 2.0f;
        float pad_y = (kInputHeight - rh) / 2.0f;
        auto run_model = build_tensorrt_run_result(
            {}, output, frame.image.cols, frame.image.rows, scale, pad_x, pad_y);
        auto adapter_result = adapt_yolov5_outputs(frame, run_model);
        result.success = adapter_result.success;
        result.contract_supported = adapter_result.contract_supported;
        result.observation = adapter_result.observation;
        result.candidates = adapter_result.candidates;
        result.message = adapter_result.message;
        return result;
    }
#endif

    const auto adapter_result = details_->adapter->adapt(frame, details_->runtime);
    result.success = adapter_result.success;
    result.contract_supported = adapter_result.contract_supported;
    result.observation = adapter_result.observation;
    result.candidates = adapter_result.candidates;
    result.message = adapter_result.message;
    return result;
}

}
