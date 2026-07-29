#include <print>

#include <cstdint>
#include <filesystem>
#include <vector>

#include "config.hpp"
#include "core/replay.hpp"
#include "test_utils.hpp"
#include "vision/cuda_check.hpp"
#include "vision/model_infer.hpp"
#include "vision/tensorrt_engine.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        const auto dataset = rmcs_laser_guidance::load_replay_dataset(default_sample_replay_path());
        require(!dataset.frames.empty(), "sample dataset should not be empty");

        const auto frame = rmcs_laser_guidance::load_replay_frame(dataset, dataset.frames.front());
        auto config = rmcs_laser_guidance::load_config(default_config_path());
        config.inference.backend = rmcs_laser_guidance::InferenceBackendKind::model;
        config.inference.model_path.clear();

        rmcs_laser_guidance::ModelInfer default_infer(config.inference);
        const auto default_result = default_infer.infer(frame);
        require(!default_result.success, "default model infer should not succeed");
        require(
            !default_result.contract_supported,
            "default model infer should not support a contract");

        require(default_result.enabled, "onnxruntime build should report enabled");
        require_contains(
            default_result.message, "inference.model_path", "missing model path message");

        config.inference.model_path = "models/mock_detector.onnx";
        rmcs_laser_guidance::ModelInfer missing_model_infer(config.inference);
        const auto missing_model_result = missing_model_infer.infer(frame);
        require(!missing_model_result.success, "missing model path should not succeed");
        require(
            !missing_model_result.contract_supported,
            "missing model should not support a contract");

        require_contains(
            missing_model_result.message, "does not exist", "missing model file message");

        config.inference.model_path = "tests/vision/mock/tiny_dynamic_shape.onnx";
        rmcs_laser_guidance::ModelInfer dynamic_model_infer(config.inference);
        const auto dynamic_model_result = dynamic_model_infer.infer(frame);
        require(dynamic_model_result.success, "dynamic-shape ONNX model infer should succeed");
        require(
            dynamic_model_result.contract_supported,
            "dynamic-shape ONNX model should support the current adapter contract");

        const std::filesystem::path engine_path = "models/exp-900-901.engine";
        if (!std::filesystem::exists(engine_path)) {
            std::println(stderr, "model_infer_test: SKIP missing {}", engine_path.string());
            return 0;
        }
        if (!rmcs_laser_guidance::cuda_device_available()) {
            std::println(stderr, "model_infer_test: SKIP CUDA device unavailable");
            return 0;
        }

        auto engine_result = rmcs_laser_guidance::TensorRTEngine::load(engine_path.string());
        require(static_cast<bool>(engine_result), "TensorRT engine should be ready");
        const auto& meta = engine_result->meta();
        require(meta.inputs.size() == 1, "TensorRT engine should have one input");
        require(meta.outputs.size() == 1, "TensorRT engine should have one output");

        const auto& input_info = meta.inputs.front();
        require(
            input_info.shape.size() == 4 && input_info.shape[2] < 0 && input_info.shape[3] < 0,
            "TensorRT engine should retain dynamic spatial dimensions");
        require(
            input_info.min_shape == std::vector<std::int64_t>{1, 3, 640, 640},
            "TensorRT input minimum profile shape");
        require(
            input_info.opt_shape == std::vector<std::int64_t>{1, 3, 1216, 1216},
            "TensorRT input optimum profile shape");
        require(
            input_info.max_shape == std::vector<std::int64_t>{1, 3, 1536, 1536},
            "TensorRT input maximum profile shape");
        require(meta.outputs.front().name == "output0", "TensorRT output name");
        require(
            meta.outputs.front().shape == std::vector<std::int64_t>{1, 300, 6},
            "TensorRT output metadata shape");

        const std::vector<std::int64_t> input_shape{1, 3, 1536, 1536};
        std::vector<float> input_data(1U * 3U * 1536U * 1536U);
        std::vector<float> output;
        std::vector<std::int64_t> output_shape;
        const auto run_result = engine_result->run(input_data, input_shape, output, output_shape);
        require(static_cast<bool>(run_result), "TensorRT 1536 inference should succeed");
        require(
            output_shape == std::vector<std::int64_t>{1, 300, 6},
            "TensorRT resolved output shape");
        require(output.size() == 1800, "TensorRT output should contain 1800 elements");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "model_infer_test failed: {}", e.what());
        return 1;
    }
}
