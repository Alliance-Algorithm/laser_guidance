#include <print>

#include "config.hpp"
#include "vision/model_infer.hpp"
#include "core/replay.hpp"
#include "test_utils.hpp"

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
        require(!default_result.contract_supported, "default model infer should not support a contract");

#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
        require(default_result.enabled, "onnxruntime-enabled build should report enabled");
        require_contains(
            default_result.message, "inference.model_path", "missing model path message");
#else
        require(!default_result.enabled, "default build should report model backend disabled");
        require_contains(default_result.message, "ONNX Runtime", "build-disabled message");
#endif

        config.inference.model_path = "models/mock_detector.onnx";
        rmcs_laser_guidance::ModelInfer missing_model_infer(config.inference);
        const auto missing_model_result = missing_model_infer.infer(frame);
        require(!missing_model_result.success, "missing model path should not succeed");
        require(!missing_model_result.contract_supported, "missing model should not support a contract");

#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
        require_contains(
            missing_model_result.message, "does not exist", "missing model file message");
#else
        require_contains(
            missing_model_result.message, "ONNX Runtime", "build-disabled missing model message");
#endif

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "model_infer_test failed: {}", e.what());
        return 1;
    }
}
