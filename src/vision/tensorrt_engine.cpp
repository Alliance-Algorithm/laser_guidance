#ifdef RMCS_LASER_GUIDANCE_WITH_TENSORRT

#include "vision/tensorrt_engine.hpp"

#include <cuda_runtime_api.h>

#include <NvInfer.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <print>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string_view>

namespace rmcs_laser_guidance {

namespace {

class TensorRTLogger final : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* message) noexcept override {
        if (severity > Severity::kWARNING || message == nullptr) return;
        std::println(stderr, "[TensorRT] {}", message);
    }
};

auto logger() -> TensorRTLogger& {
    static TensorRTLogger instance;
    return instance;
}

auto cuda_error_message(const cudaError_t code, const std::string_view action) -> std::string {
    std::ostringstream oss;
    oss << action << " failed: " << cudaGetErrorString(code);
    return oss.str();
}

auto tensor_shape_string(const std::vector<std::int64_t>& shape) -> std::string {
    std::ostringstream oss;
    oss << '[';
    for (std::size_t index = 0; index < shape.size(); ++index) {
        if (index != 0) oss << ',';
        oss << shape[index];
    }
    oss << ']';
    return oss.str();
}

auto meta_string(const TensorRTMeta& meta) -> std::string {
    std::ostringstream oss;
    oss << "engine='" << meta.engine_path << "' device='" << meta.device_name << "'";
    oss << " inputs={";
    for (std::size_t index = 0; index < meta.inputs.size(); ++index) {
        if (index != 0) oss << ", ";
        oss << meta.inputs[index].name << tensor_shape_string(meta.inputs[index].shape);
    }
    oss << "} outputs={";
    for (std::size_t index = 0; index < meta.outputs.size(); ++index) {
        if (index != 0) oss << ", ";
        oss << meta.outputs[index].name << tensor_shape_string(meta.outputs[index].shape);
    }
    oss << '}';
    return oss.str();
}

auto tensor_element_count(const std::vector<std::int64_t>& shape) -> std::expected<std::size_t, std::string> {
    std::size_t count = 1;
    for (const std::int64_t dim : shape) {
        if (dim <= 0) {
            std::ostringstream oss;
            oss << "TensorRT engine has unsupported dynamic or invalid dimension in shape "
                << tensor_shape_string(shape);
            return std::unexpected(oss.str());
        }
        if (count > std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(dim)) {
            return std::unexpected("TensorRT tensor element count overflow");
        }
        count *= static_cast<std::size_t>(dim);
    }
    return count;
}

auto tensor_info_message(const TensorRTMeta::TensorInfo& tensor) -> std::string {
    std::ostringstream oss;
    oss << tensor.name << tensor_shape_string(tensor.shape);
    return oss.str();
}

auto cleanup_cuda(void*& device_ptr) noexcept -> void {
    if (device_ptr != nullptr) {
        cudaFree(device_ptr);
        device_ptr = nullptr;
    }
}

} // namespace

struct TensorRTEngine::Impl {
    ~Impl() {
        cleanup();
    }

    auto cleanup() noexcept -> void {
        cleanup_cuda(device_input);
        cleanup_cuda(device_output);

        if (stream != nullptr) {
            cudaStreamDestroy(stream);
            stream = nullptr;
        }

        if (context != nullptr) {
            delete context;
            context = nullptr;
        }
        if (engine != nullptr) {
            delete engine;
            engine = nullptr;
        }
        if (runtime != nullptr) {
            delete runtime;
            runtime = nullptr;
        }
    }

    TensorRTMeta meta { };
    nvinfer1::IRuntime* runtime { nullptr };
    nvinfer1::ICudaEngine* engine { nullptr };
    nvinfer1::IExecutionContext* context { nullptr };
    cudaStream_t stream { nullptr };
    void* device_input { nullptr };
    void* device_output { nullptr };
    std::size_t input_element_count { 0 };
    std::size_t output_element_count { 0 };
};

TensorRTEngine::~TensorRTEngine() = default;

TensorRTEngine::TensorRTEngine(TensorRTEngine&&) noexcept = default;

auto TensorRTEngine::operator=(TensorRTEngine&&) noexcept -> TensorRTEngine& = default;

auto TensorRTEngine::load(const std::string& path) -> std::expected<TensorRTEngine, std::string> {
    TensorRTEngine engine_wrapper;
    engine_wrapper.impl_ = std::make_unique<Impl>();
    auto& impl           = *engine_wrapper.impl_;

    std::error_code ec;
    const std::filesystem::path engine_path(path);
    if (!std::filesystem::exists(engine_path, ec) || ec) {
        std::ostringstream oss;
        oss << "TensorRT engine file not found: '" << path << "'";
        return std::unexpected(oss.str());
    }

    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::ostringstream oss;
        oss << "failed to open TensorRT engine file '" << path << "': " << std::strerror(errno);
        return std::unexpected(oss.str());
    }

    std::vector<char> blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (blob.empty()) {
        std::ostringstream oss;
        oss << "TensorRT engine file is empty: '" << path << "'";
        return std::unexpected(oss.str());
    }

    impl.runtime = nvinfer1::createInferRuntime(logger());
    if (impl.runtime == nullptr) {
        return std::unexpected("failed to create TensorRT runtime");
    }

    impl.engine = impl.runtime->deserializeCudaEngine(blob.data(), blob.size());
    if (impl.engine == nullptr) {
        std::ostringstream oss;
        oss << "failed to deserialize TensorRT engine '" << path << "'";
        return std::unexpected(oss.str());
    }

    impl.context = impl.engine->createExecutionContext();
    if (impl.context == nullptr) {
        return std::unexpected("failed to create TensorRT execution context");
    }

    impl.meta.engine_path = path;
    impl.meta.device_name  = "cuda-device-0";

    const int tensor_count = impl.engine->getNbIOTensors();
    if (tensor_count <= 0) {
        return std::unexpected("TensorRT engine has no I/O tensors");
    }

    for (int index = 0; index < tensor_count; ++index) {
        const char* tensor_name = impl.engine->getIOTensorName(index);
        if (tensor_name == nullptr) {
            return std::unexpected("TensorRT engine returned a null tensor name");
        }

        const nvinfer1::Dims dims = impl.engine->getTensorShape(tensor_name);
        std::vector<std::int64_t> shape;
        shape.reserve(static_cast<std::size_t>(dims.nbDims));
        for (int dim_index = 0; dim_index < dims.nbDims; ++dim_index) {
            shape.push_back(static_cast<std::int64_t>(dims.d[dim_index]));
        }

        const auto element_count = tensor_element_count(shape);
        if (!element_count) {
            return std::unexpected(element_count.error());
        }

        const nvinfer1::DataType tensor_type = impl.engine->getTensorDataType(tensor_name);
        if (tensor_type != nvinfer1::DataType::kFLOAT) {
            std::ostringstream oss;
            oss << "TensorRT tensor '" << tensor_name << "' must use float32 for this skeleton, got "
                << static_cast<int>(tensor_type);
            return std::unexpected(oss.str());
        }

        TensorRTMeta::TensorInfo info;
        info.name  = tensor_name;
        info.shape = std::move(shape);

        const nvinfer1::TensorIOMode mode = impl.engine->getTensorIOMode(tensor_name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            impl.meta.inputs.push_back(std::move(info));
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            impl.meta.outputs.push_back(std::move(info));
        } else {
            return std::unexpected("TensorRT tensor has unsupported I/O mode");
        }
    }

    if (impl.meta.inputs.size() != 1 || impl.meta.outputs.size() != 1) {
        std::ostringstream oss;
        oss << "TensorRT skeleton currently supports exactly one input and one output; "
            << meta_string(impl.meta);
        return std::unexpected(oss.str());
    }

    const auto input_elements = tensor_element_count(impl.meta.inputs.front().shape);
    if (!input_elements) return std::unexpected(input_elements.error());
    const auto output_elements = tensor_element_count(impl.meta.outputs.front().shape);
    if (!output_elements) return std::unexpected(output_elements.error());

    impl.input_element_count  = *input_elements;
    impl.output_element_count = *output_elements;

    if (const cudaError_t status = cudaStreamCreate(&impl.stream); status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaStreamCreate"));
    }

    if (const cudaError_t status = cudaMalloc(&impl.device_input, impl.input_element_count * sizeof(float));
        status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaMalloc(input)"));
    }

    if (const cudaError_t status = cudaMalloc(&impl.device_output, impl.output_element_count * sizeof(float));
        status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaMalloc(output)"));
    }

    if (!impl.context->setTensorAddress(impl.meta.inputs.front().name.c_str(), impl.device_input)) {
        return std::unexpected("failed to bind TensorRT input buffer to execution context");
    }
    if (!impl.context->setTensorAddress(impl.meta.outputs.front().name.c_str(), impl.device_output)) {
        return std::unexpected("failed to bind TensorRT output buffer to execution context");
    }

    return engine_wrapper;
}

auto TensorRTEngine::run(const std::vector<float>& input, std::vector<float>& output)
    -> std::expected<void, std::string> {
    if (impl_ == nullptr || impl_->context == nullptr || impl_->stream == nullptr) {
        return std::unexpected("TensorRT engine is not loaded");
    }

    const auto& meta = impl_->meta;
    if (input.size() != impl_->input_element_count) {
        std::ostringstream oss;
        oss << "TensorRT input size mismatch: got " << input.size() << ", expected "
            << impl_->input_element_count << " for " << tensor_info_message(meta.inputs.front())
            << " with output " << tensor_info_message(meta.outputs.front()) << "; "
            << meta_string(meta);
        return std::unexpected(oss.str());
    }

    output.resize(impl_->output_element_count);

    if (const cudaError_t status = cudaMemcpyAsync(impl_->device_input, input.data(),
            impl_->input_element_count * sizeof(float), cudaMemcpyHostToDevice, impl_->stream);
        status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaMemcpyAsync(H2D)"));
    }

    if (!impl_->context->enqueueV3(impl_->stream)) {
        std::ostringstream oss;
        oss << "TensorRT enqueueV3 failed for " << meta_string(meta);
        return std::unexpected(oss.str());
    }

    if (const cudaError_t status = cudaMemcpyAsync(output.data(), impl_->device_output,
            impl_->output_element_count * sizeof(float), cudaMemcpyDeviceToHost, impl_->stream);
        status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaMemcpyAsync(D2H)"));
    }

    if (const cudaError_t status = cudaStreamSynchronize(impl_->stream); status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaStreamSynchronize"));
    }

    return { };
}

auto TensorRTEngine::meta() const -> const TensorRTMeta& {
    static const TensorRTMeta empty_meta { };
    if (impl_ == nullptr) return empty_meta;
    return impl_->meta;
}

} // namespace rmcs_laser_guidance

#else

// TensorRT not enabled — provide no-op stubs that report the build was compiled
// without TensorRT support. The compile-time flag RMCS_LASER_GUIDANCE_WITH_TENSORRT
// controls the real implementation above.

#include "vision/tensorrt_engine.hpp"

namespace rmcs_laser_guidance {

struct TensorRTEngine::Impl {};

TensorRTEngine::~TensorRTEngine() = default;

auto TensorRTEngine::load(const std::string&) -> std::expected<TensorRTEngine, std::string> {
    return std::unexpected(std::string("TensorRT support was not enabled at build time"));
}

auto TensorRTEngine::run(const std::vector<float>&, std::vector<float>&) -> std::expected<void, std::string> {
    return std::unexpected(std::string("TensorRT is not available in this build"));
}

auto TensorRTEngine::meta() const -> const TensorRTMeta& {
    static const TensorRTMeta empty{};
    return empty;
}

} // namespace rmcs_laser_guidance

#endif
