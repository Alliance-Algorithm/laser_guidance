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
# include "vision/tensorrt_engine.hpp"
#endif

namespace rmcs_laser_guidance {

#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT
namespace {

constexpr int kInputWidth = 640;
constexpr int kInputHeight = 640;

struct LetterboxParams {
    float scale = 1.0F;
    int resized_width = kInputWidth;
    int resized_height = kInputHeight;
    float pad_x = 0.0F;
    float pad_y = 0.0F;
};

struct TensorrtPreprocessResult {
    std::vector<float> input;
    LetterboxParams params;
};

auto compute_letterbox_params(int width, int height) -> LetterboxParams {
    LetterboxParams params;
    params.scale = std::min(
        static_cast<float>(kInputWidth) / width, static_cast<float>(kInputHeight) / height);
    params.resized_width = std::max(1, static_cast<int>(std::lround(width * params.scale)));
    params.resized_height = std::max(1, static_cast<int>(std::lround(height * params.scale)));
    params.pad_x = static_cast<float>((kInputWidth - params.resized_width) / 2);
    params.pad_y = static_cast<float>((kInputHeight - params.resized_height) / 2);
    return params;
}

auto preprocess_for_tensorrt(const cv::Mat& image) -> TensorrtPreprocessResult {
    cv::Mat bgr;
    if (image.channels() == 4)
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
    else if (image.channels() == 1)
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    else
        bgr = image;

    const auto params = compute_letterbox_params(bgr.cols, bgr.rows);

    cv::Mat resized;
    cv::resize(
        bgr, resized, cv::Size(params.resized_width, params.resized_height), 0.0, 0.0,
        cv::INTER_LINEAR);

    cv::Mat letterbox(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterbox(
        cv::Rect(
            static_cast<int>(params.pad_x), static_cast<int>(params.pad_y), params.resized_width,
            params.resized_height)));

    cv::Mat rgb;
    cv::cvtColor(letterbox, rgb, cv::COLOR_BGR2RGB);
    cv::Mat rgb_float;
    rgb.convertTo(rgb_float, CV_32F, 1.0 / 255.0);

    std::vector<cv::Mat> channels;
    cv::split(rgb_float, channels);

    std::vector<float> input(3 * kInputHeight * kInputWidth);
    std::size_t ch_size = kInputHeight * kInputWidth;
    for (std::size_t c = 0; c < 3; ++c)
        std::copy(
            channels[c].ptr<float>(0), channels[c].ptr<float>(0) + ch_size,
            input.begin() + c * ch_size);
    return {
        .input = std::move(input),
        .params = params,
    };
}

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
            std::println(
                "TensorRT engine loaded: {} ({} inputs, {} outputs)", meta.engine_path,
                meta.inputs.size(), meta.outputs.size());
            startup_ready = true;
#else
            message = "tensorrt backend requires -DRMCS_LASER_GUIDANCE_WITH_TENSORRT=ON";
#endif
            return;
        }

        if (!runtime_enabled) {
            message = "model backend requires ONNX Runtime support; reconfigure with "
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
#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT
        const auto preprocess = preprocess_for_tensorrt(frame.image);
        std::vector<float> output(300 * 6);
        auto run_result = tensorrt_engine->run(preprocess.input, output);
        if (!run_result) {
            result.message = "TensorRT inference: " + run_result.error();
            return result;
        }
        auto run_model = build_tensorrt_run_result(
            {}, output, frame.image.cols, frame.image.rows, preprocess.params.scale,
            preprocess.params.pad_x, preprocess.params.pad_y);
        auto adapter_result = adapt_yolo_outputs(frame, run_model);
        result.success = adapter_result.success;
        result.contract_supported = adapter_result.contract_supported;
        result.observation = adapter_result.observation;
        result.candidates = adapter_result.candidates;
        result.message = adapter_result.message;
#else
        (void)frame;
#endif
        return result;
    }

    auto infer_onnx(const Frame& frame, ModelInferResult result) const -> ModelInferResult {
        const auto adapter_result = adapter->adapt(frame, runtime);
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
    if (details_->tensorrt_engine)
        return details_->infer_tensorrt(frame, std::move(result));
#endif

    return details_->infer_onnx(frame, std::move(result));
}

auto ModelInfer::is_ready() const -> bool { return details_->startup_ready; }

auto ModelInfer::startup_message() const -> const std::string& { return details_->message; }

} // namespace rmcs_laser_guidance
