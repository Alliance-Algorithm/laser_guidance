#include <cstdio>
#include <filesystem>
#include <print>
#include <sstream>

#include "config.hpp"
#include "example_support.hpp"
#include "vision/model_infer.hpp"
#include "core/replay.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1) return argv[1];
    return rmcs_laser_guidance::examples::default_config_path();
}

auto resolve_replay_dir(int argc, char** argv) -> std::filesystem::path {
    if (argc > 2) return argv[2];
    return rmcs_laser_guidance::examples::default_sample_replay_path();
}

auto resolve_model_path(const rmcs_laser_guidance::InferenceConfig& config, int argc, char** argv)
    -> std::filesystem::path {
    if (argc > 3) return argv[3];
    return config.model_path;
}

auto format_shape(const std::vector<std::int64_t>& shape) -> std::string {
    std::ostringstream oss;
    oss << '[';
    for (std::size_t index = 0; index < shape.size(); ++index) {
        if (index != 0) oss << ", ";
        oss << shape[index];
    }
    oss << ']';
    return oss.str();
}

auto print_value_infos(std::string_view label,
    const std::vector<rmcs_laser_guidance::ModelValueInfo>& values) -> void {
    std::println("{} count={}", label, values.size());
    for (const auto& value : values) {
        std::println("  name={} dtype={} shape={}", value.name, value.element_type,
            format_shape(value.shape));
    }
}

auto print_candidates(const std::vector<rmcs_laser_guidance::ModelCandidate>& candidates) -> void {
    std::println("candidates count={}", candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto& candidate = candidates[index];
        std::println("  [{}] score={} bbox=[{}, {}, {}, {}] center=[{}, {}]", index,
            candidate.score, candidate.bbox.x, candidate.bbox.y, candidate.bbox.width,
            candidate.bbox.height, candidate.center.x, candidate.center.y);
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path      = resolve_config_path(argc, argv);
        const auto replay_dir       = resolve_replay_dir(argc, argv);
        auto config                 = rmcs_laser_guidance::load_config(config_path);
        config.inference.backend    = rmcs_laser_guidance::InferenceBackendKind::model;
        config.inference.model_path = resolve_model_path(config.inference, argc, argv);

        const auto dataset = rmcs_laser_guidance::load_replay_dataset(replay_dir);
        if (dataset.frames.empty()) {
            std::println(stderr, "No sample frames available for model infer example.");
            return 1;
        }

        const auto frame = rmcs_laser_guidance::load_replay_frame(dataset, dataset.frames.front());
        rmcs_laser_guidance::ModelInfer model_infer(config.inference);
        const auto result = model_infer.infer(frame);

        std::println("backend={} model_path={} runtime_enabled={} success={} contract_supported={}",
            rmcs_laser_guidance::examples::inference_backend_name(config.inference.backend),
            config.inference.model_path.empty() ? "<unset>" : config.inference.model_path.string(),
            result.enabled, result.success, result.contract_supported);
        print_value_infos("inputs", result.inputs);
        print_value_infos("outputs", result.outputs);
        print_candidates(result.candidates);
        std::println("observation detected={} center=[{}, {}] contour_size={} brightness={}",
            result.observation.detected, result.observation.center.x, result.observation.center.y,
            result.observation.contour.size(), result.observation.brightness);
        std::println("message={}", result.message.empty() ? "<none>" : result.message);
        return result.success ? 0 : 1;
    } catch (const std::exception& e) {
        std::println(stderr, "example_model_infer failed: {}", e.what());
        return 1;
    }
}
