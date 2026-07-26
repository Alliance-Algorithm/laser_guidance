// Standalone helper: run TensorRT (or ONNX) inference on a list of image files
// and dump annotated copies next to the source images.
//
// Usage:
//   tool_detect_dump <model_path> <image1.jpg> [image2.jpg ...]

#include <cstdio>
#include <filesystem>
#include <print>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "config.hpp"
#include "core/debug_renderer.hpp"
#include "types.hpp"
#include "vision/model_infer.hpp"

int main(int argc, char** argv) {
    using namespace rmcs_laser_guidance;

    if (argc < 3) {
        std::println(stderr, "usage: tool_detect_dump <model_path> <image1.jpg> [image2.jpg ...]");
        return 1;
    }

    InferenceConfig config;
    config.model_path = argv[1];
    const std::string ext = config.model_path.extension().string();
    config.backend =
        (ext == ".engine") ? InferenceBackendKind::tensorrt : InferenceBackendKind::model;

    std::vector<std::filesystem::path> images;
    for (int i = 2; i < argc; ++i)
        images.emplace_back(argv[i]);

    ModelInfer model_infer(config);
    if (!model_infer.is_ready()) {
        std::println(stderr, "model not ready: {}", model_infer.startup_message());
        return 1;
    }

    for (const auto& path : images) {
        cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
        if (image.empty()) {
            std::println(stderr, "skip (unreadable): {}", path.string());
            continue;
        }

        Frame frame{.image = image, .timestamp = Clock::now()};
        const auto result = model_infer.infer(frame);

        std::println(
            "{} success={} candidates={} observation_detected={}", path.filename().string(),
            result.success, result.candidates.size(), result.observation.detected);
        for (std::size_t i = 0; i < result.candidates.size(); ++i) {
            const auto& c = result.candidates[i];
            std::println(
                "  [{}] score={:.3f} class={} bbox=[{:.1f},{:.1f},{:.1f},{:.1f}] center=[{:.1f},{:.1f}]",
                i, c.score, c.class_id, c.bbox.x, c.bbox.y, c.bbox.width, c.bbox.height, c.center.x,
                c.center.y);
        }

        cv::Mat annotated = image.clone();
        draw_candidates(annotated, result.candidates);

        const auto out_path =
            path.parent_path() / (path.stem().string() + "_detect.jpg");
        if (!cv::imwrite(out_path.string(), annotated))
            std::println(stderr, "failed to write: {}", out_path.string());
        else
            std::println("  -> {}", out_path.string());
    }

    return 0;
}
