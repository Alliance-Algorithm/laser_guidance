#include "vision/model_runtime.hpp"

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#endif

namespace rmcs_laser_guidance {

#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
namespace {

    auto tensor_element_type_name(const ONNXTensorElementDataType type) -> std::string {
        switch (type) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            return "float32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            return "uint8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            return "int8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
            return "uint16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
            return "int16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            return "int32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
            return "int64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
            return "float64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
            return "bool";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            return "float16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
            return "bfloat16";
        default:
            return "unknown";
        }
    }

    auto extract_value_info(Ort::Session& session, Ort::AllocatorWithDefaultOptions& allocator,
        const bool is_input) -> std::vector<ModelValueInfo> {
        const std::size_t count = is_input ? session.GetInputCount() : session.GetOutputCount();
        std::vector<ModelValueInfo> values;
        values.reserve(count);

        for (std::size_t index = 0; index < count; ++index) {
            Ort::AllocatedStringPtr name = is_input
                ? session.GetInputNameAllocated(index, allocator)
                : session.GetOutputNameAllocated(index, allocator);
            Ort::TypeInfo type_info =
                is_input ? session.GetInputTypeInfo(index) : session.GetOutputTypeInfo(index);
            Ort::ConstTensorTypeAndShapeInfo tensor_info = type_info.GetTensorTypeAndShapeInfo();

            ModelValueInfo value;
            value.name         = name ? name.get() : "";
            value.shape        = tensor_info.GetShape();
            value.element_type = tensor_element_type_name(tensor_info.GetElementType());
            values.push_back(std::move(value));
        }

        return values;
    }

    auto input_dimensions(const ModelValueInfo& input) -> std::pair<int, int> {
        if (input.shape.size() != 4) throw std::runtime_error("model input must be a 4D tensor");
        if (input.shape[1] != 3)
            throw std::runtime_error("model input must use NCHW with 3 channels");
        if (input.shape[2] <= 0 || input.shape[3] <= 0) {
            throw std::runtime_error("model input height/width must be statically known and "
                                     "positive");
        }

        return {
            static_cast<int>(input.shape[2]),
            static_cast<int>(input.shape[3]),
        };
    }

    struct PreparedInput {
        std::vector<float> values { };
        std::vector<std::int64_t> shape { };
        ModelImageTransform transform { };
    };

    auto ensure_bgr_image(const cv::Mat& image) -> cv::Mat {
        if (image.empty()) throw std::runtime_error("model runtime received an empty image");

        if (image.channels() == 3) return image;

        cv::Mat converted;
        if (image.channels() == 1) {
            cv::cvtColor(image, converted, cv::COLOR_GRAY2BGR);
            return converted;
        }
        if (image.channels() == 4) {
            cv::cvtColor(image, converted, cv::COLOR_BGRA2BGR);
            return converted;
        }

        throw std::runtime_error("model runtime received an unsupported image channel count");
    }

    auto prepare_input_tensor(const cv::Mat& image, const ModelValueInfo& input) -> PreparedInput {
        const auto [input_height, input_width] = input_dimensions(input);
        const cv::Mat bgr                      = ensure_bgr_image(image);

        const float scale_x      = static_cast<float>(input_width) / static_cast<float>(bgr.cols);
        const float scale_y      = static_cast<float>(input_height) / static_cast<float>(bgr.rows);
        const float scale        = std::min(scale_x, scale_y);
        const int resized_width  = std::max(1, static_cast<int>(std::lround(bgr.cols * scale)));
        const int resized_height = std::max(1, static_cast<int>(std::lround(bgr.rows * scale)));
        const int pad_x          = (input_width - resized_width) / 2;
        const int pad_y          = (input_height - resized_height) / 2;

        cv::Mat resized;
        cv::resize(
            bgr, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);

        cv::Mat letterboxed(input_height, input_width, CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(letterboxed(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

        cv::Mat rgb;
        cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);
        cv::Mat rgb_float;
        rgb.convertTo(rgb_float, CV_32F, 1.0 / 255.0);

        std::vector<cv::Mat> channels;
        cv::split(rgb_float, channels);
        if (channels.size() != 3)
            throw std::runtime_error("failed to split model input into RGB channels");

        PreparedInput prepared {
            .values = std::vector<float>(3ULL * static_cast<std::size_t>(input_height)
                * static_cast<std::size_t>(input_width)),
            .shape  = { 1, 3, input_height, input_width },
            .transform =
                ModelImageTransform {
                    .original_width  = bgr.cols,
                    .original_height = bgr.rows,
                    .input_width     = input_width,
                    .input_height    = input_height,
                    .scale           = scale,
                    .pad_x           = static_cast<float>(pad_x),
                    .pad_y           = static_cast<float>(pad_y),
                },
        };

        const std::size_t channel_size =
            static_cast<std::size_t>(input_height) * static_cast<std::size_t>(input_width);
        for (std::size_t index = 0; index < channels.size(); ++index) {
            const auto* begin = channels[index].ptr<float>(0);
            std::copy(begin, begin + channel_size, prepared.values.begin() + index * channel_size);
        }

        return prepared;
    }

    auto extract_tensor_data(const ModelValueInfo& metadata, const Ort::Value& value)
        -> ModelTensorData {
        if (!value.IsTensor()) throw std::runtime_error("model output is not a tensor");

        const auto tensor_info                = value.GetTensorTypeAndShapeInfo();
        const std::vector<std::int64_t> shape = tensor_info.GetShape();
        const std::size_t element_count       = tensor_info.GetElementCount();

        ModelTensorData tensor {
            .name         = metadata.name,
            .shape        = shape,
            .element_type = tensor_element_type_name(tensor_info.GetElementType()),
            .values       = std::vector<float>(element_count),
        };

        switch (tensor_info.GetElementType()) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: {
            const float* data = value.GetTensorData<float>();
            std::copy(data, data + element_count, tensor.values.begin());
            break;
        }
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE: {
            const double* data = value.GetTensorData<double>();
            std::transform(data, data + element_count, tensor.values.begin(),
                [](const double value_in) { return static_cast<float>(value_in); });
            break;
        }
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64: {
            const std::int64_t* data = value.GetTensorData<std::int64_t>();
            std::transform(data, data + element_count, tensor.values.begin(),
                [](const std::int64_t value_in) { return static_cast<float>(value_in); });
            break;
        }
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32: {
            const std::int32_t* data = value.GetTensorData<std::int32_t>();
            std::transform(data, data + element_count, tensor.values.begin(),
                [](const std::int32_t value_in) { return static_cast<float>(value_in); });
            break;
        }
        default:
            throw std::runtime_error(
                "unsupported ONNX output element type: " + tensor.element_type);
        }

        return tensor;
    }

} // namespace

struct ModelRuntime::Details {
    Details()
        : env(ORT_LOGGING_LEVEL_WARNING, "rmcs_laser_guidance")
        , session_options() {
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    }

    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session { };
    std::vector<ModelValueInfo> inputs { };
    std::vector<ModelValueInfo> outputs { };
};

#else

struct ModelRuntime::Details {
    std::vector<ModelValueInfo> inputs { };
    std::vector<ModelValueInfo> outputs { };
};

#endif

ModelRuntime::ModelRuntime(std::filesystem::path model_path)
    : model_path_(std::move(model_path))
    , details_(std::make_unique<Details>()) { }

ModelRuntime::~ModelRuntime() = default;

ModelRuntime::ModelRuntime(ModelRuntime&&) noexcept = default;

auto ModelRuntime::operator=(ModelRuntime&&) noexcept -> ModelRuntime& = default;

auto ModelRuntime::load() -> std::string {
#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
    if (details_->session) return { };

    try {
        details_->session = std::make_unique<Ort::Session>(
            details_->env, model_path_.string().c_str(), details_->session_options);
        Ort::AllocatorWithDefaultOptions allocator;
        details_->inputs  = extract_value_info(*details_->session, allocator, true);
        details_->outputs = extract_value_info(*details_->session, allocator, false);
        return { };
    } catch (const Ort::Exception& e) {
        std::ostringstream oss;
        oss << "failed to load ONNX model '" << model_path_.string() << "': " << e.what();
        return oss.str();
    }
#else
    return "model backend was built without ONNX Runtime support";
#endif
}

auto ModelRuntime::run(const cv::Mat& image) const -> ModelRunResult {
#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
    ModelRunResult result;
    if (!details_->session) {
        result.message = "model runtime is not loaded";
        return result;
    }
    if (details_->inputs.size() != 1) {
        result.message = "model runtime currently supports exactly one input tensor, got "
            + std::to_string(details_->inputs.size());
        return result;
    }

    try {
        const PreparedInput prepared = prepare_input_tensor(image, details_->inputs.front());
        result.transform             = prepared.transform;

        Ort::MemoryInfo memory_info =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor =
            Ort::Value::CreateTensor<float>(memory_info, const_cast<float*>(prepared.values.data()),
                prepared.values.size(), prepared.shape.data(), prepared.shape.size());

        std::vector<const char*> input_names { details_->inputs.front().name.c_str() };
        std::vector<const char*> output_names;
        output_names.reserve(details_->outputs.size());
        for (const auto& output : details_->outputs)
            output_names.push_back(output.name.c_str());

        const auto ort_outputs =
            details_->session->Run(Ort::RunOptions { nullptr }, input_names.data(), &input_tensor,
                input_names.size(), output_names.data(), output_names.size());
        if (ort_outputs.size() != details_->outputs.size()) {
            result.message = "model runtime returned an unexpected number of output tensors";
            return result;
        }

        result.outputs.reserve(ort_outputs.size());
        for (std::size_t index = 0; index < ort_outputs.size(); ++index) {
            result.outputs.push_back(
                extract_tensor_data(details_->outputs[index], ort_outputs[index]));
        }
        result.success = true;
        return result;
    } catch (const std::exception& e) {
        result.message = "model runtime inference failed: " + std::string(e.what());
        return result;
    }
#else
    (void)image;
    return ModelRunResult {
        .success = false,
        .message = "model backend was built without ONNX Runtime support",
    };
#endif
}

auto ModelRuntime::is_loaded() const noexcept -> bool {
#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
    return static_cast<bool>(details_->session);
#else
    return false;
#endif
}

auto ModelRuntime::model_path() const noexcept -> const std::filesystem::path& {
    return model_path_;
}

auto ModelRuntime::input_values() const noexcept -> const std::vector<ModelValueInfo>& {
    return details_->inputs;
}

auto ModelRuntime::output_values() const noexcept -> const std::vector<ModelValueInfo>& {
    return details_->outputs;
}

auto model_runtime_enabled_in_build() noexcept -> bool {
#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
    return true;
#else
    return false;
#endif
}

} // namespace rmcs_laser_guidance
