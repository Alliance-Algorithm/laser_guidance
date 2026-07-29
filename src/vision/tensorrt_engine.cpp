
#include "vision/tensorrt_engine.hpp"

#include "vision/preprocess_cuda.hpp"
#include <cuda_runtime_api.h>

#include <NvInfer.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <print>
#include <ranges>
#include <sstream>
#include <string_view>

namespace rmcs_laser_guidance {

namespace {

class TensorRTLogger final : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* message) noexcept override {
        if (severity > Severity::kWARNING || message == nullptr)
            return;
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
        if (index != 0)
            oss << ',';
        oss << shape[index];
    }
    oss << ']';
    return oss.str();
}

auto dims_to_shape(const nvinfer1::Dims& dims, const std::string_view label)
    -> std::expected<std::vector<std::int64_t>, std::string> {
    if (dims.nbDims <= 0 || dims.nbDims > nvinfer1::Dims::MAX_DIMS) {
        std::ostringstream oss;
        oss << "TensorRT returned invalid " << label << " dimensions (rank " << dims.nbDims
            << ')';
        return std::unexpected(oss.str());
    }

    std::vector<std::int64_t> shape;
    shape.reserve(static_cast<std::size_t>(dims.nbDims));
    for (int index = 0; index < dims.nbDims; ++index)
        shape.push_back(dims.d[index]);
    return shape;
}

auto shape_is_fixed(const std::vector<std::int64_t>& shape) -> bool {
    return std::ranges::all_of(shape, [](const std::int64_t dim) { return dim > 0; });
}

const std::vector<std::int64_t> kOutputShape{1, 300, 6};

auto meta_string(const TensorRTMeta& meta) -> std::string {
    std::ostringstream oss;
    oss << "engine='" << meta.engine_path << "' device='" << meta.device_name << "'";
    oss << " inputs={";
    for (std::size_t index = 0; index < meta.inputs.size(); ++index) {
        if (index != 0)
            oss << ", ";
        const auto& tensor = meta.inputs[index];
        oss << tensor.name << tensor_shape_string(tensor.shape)
            << " profile=" << tensor_shape_string(tensor.min_shape) << ".."
            << tensor_shape_string(tensor.max_shape);
    }
    oss << "} outputs={";
    for (std::size_t index = 0; index < meta.outputs.size(); ++index) {
        if (index != 0)
            oss << ", ";
        oss << meta.outputs[index].name << tensor_shape_string(meta.outputs[index].shape);
    }
    oss << '}';
    return oss.str();
}

auto tensor_element_count(const std::vector<std::int64_t>& shape)
    -> std::expected<std::size_t, std::string> {
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
    oss << tensor.name << tensor_shape_string(tensor.shape) << " profile="
        << tensor_shape_string(tensor.min_shape) << ".." << tensor_shape_string(tensor.max_shape);
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
    ~Impl() { cleanup(); }

    auto cleanup() noexcept -> void {
        cleanup_cuda(device_input);
        cleanup_cuda(device_output);
        cleanup_cuda(device_src_bgr);

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

    TensorRTMeta meta{};
    nvinfer1::IRuntime* runtime{nullptr};
    nvinfer1::ICudaEngine* engine{nullptr};
    nvinfer1::IExecutionContext* context{nullptr};
    cudaStream_t stream{nullptr};
    void* device_input{nullptr};
    void* device_output{nullptr};
    void* device_src_bgr{nullptr};   // uint8 BGR source upload buffer (GPU preprocess path)
    std::size_t input_capacity{0};
    std::size_t output_capacity{0};
    std::size_t src_bgr_capacity{0}; // bytes
};

TensorRTEngine::~TensorRTEngine() = default;

TensorRTEngine::TensorRTEngine(TensorRTEngine&&) noexcept = default;

auto TensorRTEngine::operator=(TensorRTEngine&&) noexcept -> TensorRTEngine& = default;

auto TensorRTEngine::load(const std::string& path) -> std::expected<TensorRTEngine, std::string> {
    TensorRTEngine engine_wrapper;
    engine_wrapper.impl_ = std::make_unique<Impl>();
    auto& impl = *engine_wrapper.impl_;

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

    std::vector<char> blob(
        (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
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
    impl.meta.device_name = "cuda-device-0";

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
        const auto shape_result = dims_to_shape(dims, "engine");
        if (!shape_result)
            return std::unexpected(shape_result.error());

        const nvinfer1::DataType tensor_type = impl.engine->getTensorDataType(tensor_name);
        if (tensor_type != nvinfer1::DataType::kFLOAT) {
            std::ostringstream oss;
            oss << "TensorRT tensor '" << tensor_name
                << "' must use float32 for this skeleton, got " << static_cast<int>(tensor_type);
            return std::unexpected(oss.str());
        }

        TensorRTMeta::TensorInfo info;
        info.name = tensor_name;
        info.shape = *shape_result;

        const nvinfer1::TensorIOMode mode = impl.engine->getTensorIOMode(tensor_name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            const bool has_dynamic_dimension = !shape_is_fixed(info.shape);
            if (has_dynamic_dimension && impl.engine->getNbOptimizationProfiles() <= 0) {
                std::ostringstream oss;
                oss << "TensorRT input '" << tensor_name
                    << "' has dynamic dimensions but no optimization profile";
                return std::unexpected(oss.str());
            }

            auto profile_shape = [&](const nvinfer1::OptProfileSelector selector,
                                     const std::string_view label)
                -> std::expected<std::vector<std::int64_t>, std::string> {
                if (!has_dynamic_dimension)
                    return info.shape;
                return dims_to_shape(impl.engine->getProfileShape(tensor_name, 0, selector), label);
            };

            const auto min_shape = profile_shape(nvinfer1::OptProfileSelector::kMIN, "minimum profile");
            if (!min_shape)
                return std::unexpected(min_shape.error());
            const auto opt_shape = profile_shape(nvinfer1::OptProfileSelector::kOPT, "optimum profile");
            if (!opt_shape)
                return std::unexpected(opt_shape.error());
            const auto max_shape = profile_shape(nvinfer1::OptProfileSelector::kMAX, "maximum profile");
            if (!max_shape)
                return std::unexpected(max_shape.error());

            if (min_shape->size() != info.shape.size() || opt_shape->size() != info.shape.size()
                || max_shape->size() != info.shape.size()
                || !shape_is_fixed(*min_shape) || !shape_is_fixed(*opt_shape)
                || !shape_is_fixed(*max_shape)) {
                std::ostringstream oss;
                oss << "TensorRT input '" << tensor_name
                    << "' has invalid profile shapes: minimum=" << tensor_shape_string(*min_shape)
                    << " optimum=" << tensor_shape_string(*opt_shape)
                    << " maximum=" << tensor_shape_string(*max_shape);
                return std::unexpected(oss.str());
            }

            if (info.shape.size() != 4 || min_shape->size() != 4
                || opt_shape->size() != 4 || max_shape->size() != 4
                || min_shape->at(0) != 1 || min_shape->at(1) != 3
                || opt_shape->at(0) != 1 || opt_shape->at(1) != 3
                || max_shape->at(0) != 1 || max_shape->at(1) != 3) {
                std::ostringstream oss;
                oss << "TensorRT input '" << tensor_name
                    << "' must use the float32 {1,3,H,W} contract: engine="
                    << tensor_shape_string(info.shape) << " minimum="
                    << tensor_shape_string(*min_shape) << " optimum="
                    << tensor_shape_string(*opt_shape) << " maximum="
                    << tensor_shape_string(*max_shape);
                return std::unexpected(oss.str());
            }

            for (std::size_t dim = 0; dim < info.shape.size(); ++dim) {
                if (dim < 2 && info.shape[dim] > 0
                    && info.shape[dim] != static_cast<std::int64_t>(dim == 0 ? 1 : 3)) {
                    std::ostringstream oss;
                    oss << "TensorRT input '" << tensor_name
                        << "' has an incompatible fixed batch/channel dimension at index "
                        << dim << ": engine=" << tensor_shape_string(info.shape)
                        << " profile=" << tensor_shape_string(*min_shape) << ".."
                        << tensor_shape_string(*max_shape);
                    return std::unexpected(oss.str());
                }
                if (info.shape[dim] > 0
                    && (min_shape->at(dim) != info.shape[dim]
                        || opt_shape->at(dim) != info.shape[dim]
                        || max_shape->at(dim) != info.shape[dim])) {
                    std::ostringstream oss;
                    oss << "TensorRT input '" << tensor_name
                        << "' has profile shape inconsistent with fixed engine dimension at index "
                        << dim;
                    return std::unexpected(oss.str());
                }
                if (min_shape->at(dim) > opt_shape->at(dim) || opt_shape->at(dim) > max_shape->at(dim)) {
                    std::ostringstream oss;
                    oss << "TensorRT input '" << tensor_name
                        << "' has non-monotonic profile shapes: minimum="
                        << tensor_shape_string(*min_shape) << " optimum="
                        << tensor_shape_string(*opt_shape) << " maximum="
                        << tensor_shape_string(*max_shape);
                    return std::unexpected(oss.str());
                }
            }

            info.min_shape = *min_shape;
            info.opt_shape = *opt_shape;
            info.max_shape = *max_shape;
            impl.meta.inputs.push_back(std::move(info));
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            info.min_shape = info.shape;
            info.opt_shape = info.shape;
            info.max_shape = info.shape;
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

    if (impl.meta.outputs.front().name != "output0"
        || (shape_is_fixed(impl.meta.outputs.front().shape)
            && impl.meta.outputs.front().shape != kOutputShape)) {
        std::ostringstream oss;
        oss << "TensorRT output must be named 'output0' with shape "
            << tensor_shape_string(kOutputShape) << "; " << meta_string(impl.meta);
        return std::unexpected(oss.str());
    }

    if (const cudaError_t status = cudaStreamCreate(&impl.stream); status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaStreamCreate"));
    }

    return engine_wrapper;
}

auto TensorRTEngine::run(
    const std::vector<float>& input,
    const std::vector<std::int64_t>& input_shape,
    std::vector<float>& output,
    std::vector<std::int64_t>& output_shape)
    -> std::expected<void, std::string> {
    if (impl_ == nullptr || impl_->context == nullptr || impl_->stream == nullptr) {
        return std::unexpected("TensorRT engine is not loaded");
    }

    output_shape.clear();
    const auto& meta = impl_->meta;
    const auto& input_info = meta.inputs.front();
    const auto& output_info = meta.outputs.front();
    if (input_shape.size() != 4 || input_shape[0] != 1 || input_shape[1] != 3
        || input_shape[2] <= 0 || input_shape[3] <= 0 || input_shape[2] % 32 != 0
        || input_shape[3] % 32 != 0) {
        std::ostringstream oss;
        oss << "TensorRT input shape mismatch: got " << tensor_shape_string(input_shape)
            << ", expected {1,3,H,W} with positive H/W divisible by 32 for "
            << tensor_info_message(input_info) << " with output " << tensor_info_message(output_info)
            << "; " << meta_string(meta);
        return std::unexpected(oss.str());
    }

    for (std::size_t index = 0; index < input_shape.size(); ++index) {
        if (input_shape[index] < input_info.min_shape[index]
            || input_shape[index] > input_info.max_shape[index]) {
            std::ostringstream oss;
            oss << "TensorRT input shape " << tensor_shape_string(input_shape)
                << " is outside profile bounds " << tensor_shape_string(input_info.min_shape)
                << ".." << tensor_shape_string(input_info.max_shape) << " for "
                << tensor_info_message(input_info) << "; " << meta_string(meta);
            return std::unexpected(oss.str());
        }
    }

    nvinfer1::Dims input_dims{};
    input_dims.nbDims = static_cast<int32_t>(input_shape.size());
    for (int index = 0; index < input_dims.nbDims; ++index)
        input_dims.d[index] = input_shape[static_cast<std::size_t>(index)];
    if (!impl_->context->setInputShape(input_info.name.c_str(), input_dims)) {
        std::ostringstream oss;
        oss << "TensorRT setInputShape failed for input " << tensor_shape_string(input_shape)
            << " with profile bounds " << tensor_shape_string(input_info.min_shape) << ".."
            << tensor_shape_string(input_info.max_shape) << "; " << meta_string(meta);
        return std::unexpected(oss.str());
    }

    const auto resolved_output_result = dims_to_shape(
        impl_->context->getTensorShape(output_info.name.c_str()), "resolved output");
    if (!resolved_output_result || *resolved_output_result != kOutputShape) {
        std::ostringstream oss;
        oss << "TensorRT output shape did not resolve for input " << tensor_shape_string(input_shape)
            << ": expected " << tensor_shape_string(kOutputShape) << ", got "
            << (resolved_output_result ? tensor_shape_string(*resolved_output_result)
                                       : resolved_output_result.error())
            << "; " << meta_string(meta);
        return std::unexpected(oss.str());
    }

    const auto input_elements = tensor_element_count(input_shape);
    if (!input_elements)
        return std::unexpected(input_elements.error());
    const auto output_elements = tensor_element_count(*resolved_output_result);
    if (!output_elements)
        return std::unexpected(output_elements.error());

    if (input.size() != *input_elements) {
        std::ostringstream oss;
        oss << "TensorRT input size mismatch: got " << input.size() << ", expected "
            << *input_elements << " for input shape " << tensor_shape_string(input_shape)
            << " with profile bounds " << tensor_shape_string(input_info.min_shape) << ".."
            << tensor_shape_string(input_info.max_shape) << " and output shape "
            << tensor_shape_string(*resolved_output_result) << "; " << meta_string(meta);
        return std::unexpected(oss.str());
    }

    // ensure_buffer: capacity and required are always in BYTES (same unit as run_from_bgr).
    auto ensure_buffer = [&](void*& device_ptr, std::size_t& capacity, const std::size_t required,
                             const std::string_view label) -> std::expected<void, std::string> {
        if (capacity >= required)
            return {};
        cleanup_cuda(device_ptr);
        capacity = 0;
        if (const cudaError_t status = cudaMalloc(&device_ptr, required); status != cudaSuccess) {
            return std::unexpected(cuda_error_message(status, std::string("cudaMalloc(") +
                                                                     std::string(label) + ")"));
        }
        capacity = required;
        return {};
    };

    if (const auto result = ensure_buffer(
            impl_->device_input, impl_->input_capacity, *input_elements * sizeof(float), "input");
        !result) {
        return std::unexpected(result.error());
    }
    if (const auto result = ensure_buffer(
            impl_->device_output, impl_->output_capacity, *output_elements * sizeof(float), "output");
        !result) {
        return std::unexpected(result.error());
    }

    if (!impl_->context->setTensorAddress(input_info.name.c_str(), impl_->device_input)) {
        return std::unexpected("failed to bind TensorRT input buffer to execution context");
    }
    if (!impl_->context->setTensorAddress(output_info.name.c_str(), impl_->device_output)) {
        return std::unexpected("failed to bind TensorRT output buffer to execution context");
    }

    output.resize(*output_elements);

    if (const cudaError_t status = cudaMemcpyAsync(
            impl_->device_input, input.data(), *input_elements * sizeof(float),
            cudaMemcpyHostToDevice, impl_->stream);
        status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaMemcpyAsync(H2D)"));
    }

    if (!impl_->context->enqueueV3(impl_->stream)) {
        std::ostringstream oss;
        oss << "TensorRT enqueueV3 failed for " << meta_string(meta);
        return std::unexpected(oss.str());
    }

    if (const cudaError_t status = cudaMemcpyAsync(
            output.data(), impl_->device_output, *output_elements * sizeof(float),
            cudaMemcpyDeviceToHost, impl_->stream);
        status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaMemcpyAsync(D2H)"));
    }

    if (const cudaError_t status = cudaStreamSynchronize(impl_->stream); status != cudaSuccess) {
        return std::unexpected(cuda_error_message(status, "cudaStreamSynchronize"));
    }

    output_shape = *resolved_output_result;
    return {};
}

auto TensorRTEngine::run_from_bgr(
    const unsigned char* src_bgr,
    int src_w, int src_h,
    int out_w, int out_h,
    std::vector<float>& output,
    std::vector<std::int64_t>& output_shape)
    -> std::expected<void, std::string> {
    if (impl_ == nullptr || impl_->context == nullptr || impl_->stream == nullptr)
        return std::unexpected("TensorRT engine is not loaded");
    if (src_bgr == nullptr || src_w <= 0 || src_h <= 0)
        return std::unexpected("run_from_bgr: invalid source image");
    if (out_w <= 0 || out_h <= 0 || out_w % 32 != 0 || out_h % 32 != 0)
        return std::unexpected("run_from_bgr: output dims must be positive and divisible by 32");

    output_shape.clear();
    const auto& meta = impl_->meta;
    const auto& input_info = meta.inputs.front();
    const auto& output_info = meta.outputs.front();

    const std::vector<std::int64_t> input_shape{1, 3, out_h, out_w};
    for (std::size_t index = 0; index < input_shape.size(); ++index) {
        if (input_shape[index] < input_info.min_shape[index]
            || input_shape[index] > input_info.max_shape[index]) {
            std::ostringstream oss;
            oss << "run_from_bgr: input shape " << tensor_shape_string(input_shape)
                << " outside profile bounds " << tensor_shape_string(input_info.min_shape)
                << ".." << tensor_shape_string(input_info.max_shape);
            return std::unexpected(oss.str());
        }
    }

    nvinfer1::Dims input_dims{};
    input_dims.nbDims = static_cast<int32_t>(input_shape.size());
    for (int index = 0; index < input_dims.nbDims; ++index)
        input_dims.d[index] = input_shape[static_cast<std::size_t>(index)];
    if (!impl_->context->setInputShape(input_info.name.c_str(), input_dims))
        return std::unexpected("run_from_bgr: setInputShape failed");

    const auto resolved_output_result = dims_to_shape(
        impl_->context->getTensorShape(output_info.name.c_str()), "resolved output");
    if (!resolved_output_result || *resolved_output_result != kOutputShape)
        return std::unexpected("run_from_bgr: output shape did not resolve to expected contract");

    const std::size_t input_elements =
        static_cast<std::size_t>(1) * 3 * static_cast<std::size_t>(out_h) * out_w;
    const auto output_elements = tensor_element_count(*resolved_output_result);
    if (!output_elements)
        return std::unexpected(output_elements.error());

    auto ensure_buffer = [&](void*& device_ptr, std::size_t& capacity, const std::size_t required,
                             const std::string_view label) -> std::expected<void, std::string> {
        if (capacity >= required)
            return {};
        cleanup_cuda(device_ptr);
        capacity = 0;
        if (const cudaError_t status = cudaMalloc(&device_ptr, required); status != cudaSuccess)
            return std::unexpected(cuda_error_message(status, std::string("cudaMalloc(") +
                                                              std::string(label) + ")"));
        capacity = required;
        return {};
    };

    // device buffers: uint8 source, float input, float output
    const std::size_t src_bytes = static_cast<std::size_t>(src_w) * src_h * 3;
    if (const auto r = ensure_buffer(impl_->device_src_bgr, impl_->src_bgr_capacity, src_bytes,
                                     "src_bgr"); !r)
        return std::unexpected(r.error());
    if (const auto r = ensure_buffer(impl_->device_input, impl_->input_capacity,
                                     input_elements * sizeof(float), "input"); !r)
        return std::unexpected(r.error());
    if (const auto r = ensure_buffer(impl_->device_output, impl_->output_capacity,
                                     *output_elements * sizeof(float), "output"); !r)
        return std::unexpected(r.error());

    if (!impl_->context->setTensorAddress(input_info.name.c_str(), impl_->device_input))
        return std::unexpected("run_from_bgr: failed to bind input buffer");
    if (!impl_->context->setTensorAddress(output_info.name.c_str(), impl_->device_output))
        return std::unexpected("run_from_bgr: failed to bind output buffer");

    output.resize(*output_elements);

    // 1. upload uint8 BGR source (smaller than a float CHW blob)
    if (const cudaError_t status = cudaMemcpyAsync(
            impl_->device_src_bgr, src_bgr, src_bytes, cudaMemcpyHostToDevice, impl_->stream);
        status != cudaSuccess)
        return std::unexpected(cuda_error_message(status, "cudaMemcpyAsync(src H2D)"));

    // 2. fused preprocess kernel writes directly into the TRT input buffer
    if (const int kerr = preprocess_cuda_bgr_to_chw(
            static_cast<const unsigned char*>(impl_->device_src_bgr), src_w, src_h,
            static_cast<float*>(impl_->device_input), out_w, out_h, impl_->stream);
        kerr != cudaSuccess)
        return std::unexpected(cuda_error_message(static_cast<cudaError_t>(kerr),
                                                  "preprocess_cuda_bgr_to_chw"));

    // 3. inference
    if (!impl_->context->enqueueV3(impl_->stream))
        return std::unexpected("run_from_bgr: enqueueV3 failed");

    // 4. download output
    if (const cudaError_t status = cudaMemcpyAsync(
            output.data(), impl_->device_output, *output_elements * sizeof(float),
            cudaMemcpyDeviceToHost, impl_->stream);
        status != cudaSuccess)
        return std::unexpected(cuda_error_message(status, "cudaMemcpyAsync(D2H)"));

    if (const cudaError_t status = cudaStreamSynchronize(impl_->stream); status != cudaSuccess)
        return std::unexpected(cuda_error_message(status, "cudaStreamSynchronize"));

    output_shape = *resolved_output_result;
    return {};
}

auto TensorRTEngine::meta() const -> const TensorRTMeta& {
    static const TensorRTMeta empty_meta{};
    if (impl_ == nullptr)
        return empty_meta;
    return impl_->meta;
}

} // namespace rmcs_laser_guidance
