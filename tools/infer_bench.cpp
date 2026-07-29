// tools/infer_bench.cpp
// Benchmark tool: measures per-stage inference latency using real frames.
// Usage: ./tool_infer_bench [engine_path] [frames_dir] [n_warmup] [n_bench]
// Default: uses config engine path and /home/yukikaze/Downloads/random_frames/

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <format>
#include <numeric>
#include <print>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "config.hpp"
#include "laser_guidance/support.hpp"
#include "vision/model_adapter.hpp"
#include "vision/model_runtime.hpp"
#include "vision/tensorrt_engine.hpp"

using namespace rmcs_laser_guidance;
using Clock = std::chrono::steady_clock;

namespace {

struct Stats {
    double min_ms{}, max_ms{}, mean_ms{}, p95_ms{}, p99_ms{};
};

auto compute_stats(std::vector<double>& samples) -> Stats {
    if (samples.empty())
        return {};
    std::sort(samples.begin(), samples.end());
    const double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    auto pct = [&](double p) -> double {
        const std::size_t idx =
            static_cast<std::size_t>(p / 100.0 * static_cast<double>(samples.size() - 1));
        return samples[std::min(idx, samples.size() - 1)];
    };
    return {
        .min_ms = samples.front(),
        .max_ms = samples.back(),
        .mean_ms = sum / static_cast<double>(samples.size()),
        .p95_ms = pct(95.0),
        .p99_ms = pct(99.0),
    };
}

auto print_stats(std::string_view label, Stats s) -> void {
    std::println("  {:20s}  mean={:6.2f}ms  min={:6.2f}ms  max={:6.2f}ms  p95={:6.2f}ms  p99={:6.2f}ms",
                 label, s.mean_ms, s.min_ms, s.max_ms, s.p95_ms, s.p99_ms);
}

auto ms_since(const Clock::time_point& t) -> double {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

auto load_frames(const std::filesystem::path& dir, int limit) -> std::vector<cv::Mat> {
    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        const auto ext = entry.path().extension().string();
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg")
            paths.push_back(entry.path());
        if (static_cast<int>(paths.size()) >= limit)
            break;
    }
    std::sort(paths.begin(), paths.end());

    std::vector<cv::Mat> frames;
    for (const auto& p : paths) {
        auto img = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (!img.empty())
            frames.push_back(std::move(img));
    }
    return frames;
}

} // namespace

int main(int argc, char** argv) {
    const std::string engine_path = argc > 1
        ? argv[1]
        : "/home/yukikaze/Documents/workspace/laser_guidance/models/exp-900-901.engine";
    const std::string frames_dir = argc > 2
        ? argv[2]
        : "/home/yukikaze/Downloads/random_frames";
    const int n_warmup = argc > 3 ? std::stoi(argv[3]) : 5;
    const int n_bench  = argc > 4 ? std::stoi(argv[4]) : 50;

    std::println("=== infer_bench ===");
    std::println("Engine:     {}", engine_path);
    std::println("Frames dir: {}", frames_dir);
    std::println("Warmup:     {}  Bench: {}", n_warmup, n_bench);
    std::println("");

    // --- load engine ---
    std::println("Loading TensorRT engine...");
    auto engine_result = TensorRTEngine::load(engine_path);
    if (!engine_result) {
        std::println(stderr, "Failed: {}", engine_result.error());
        return 1;
    }
    TensorRTEngine engine = std::move(*engine_result);
    const auto& meta = engine.meta();
    if (!meta.inputs.empty()) {
        const auto& in = meta.inputs.front();
        std::println("  input:  {} shape=[{}]", in.name,
                     in.opt_shape.empty() ? "?" : std::format("{},{},{},{}",
                         in.opt_shape[0], in.opt_shape[1], in.opt_shape[2], in.opt_shape[3]));
    }
    if (!meta.outputs.empty()) {
        const auto& out = meta.outputs.front();
        std::println("  output: {} shape=[{}]", out.name,
                     out.shape.size() >= 3
                         ? std::format("{},{},{}", out.shape[0], out.shape[1], out.shape[2])
                         : "?");
    }
    std::println("");

    // --- load frames ---
    std::println("Loading frames from {}...", frames_dir);
    auto frames = load_frames(frames_dir, n_warmup + n_bench + 10);
    if (frames.empty()) {
        std::println(stderr, "No images found in {}", frames_dir);
        return 1;
    }
    std::println("  Loaded {} frames ({}x{})", frames.size(),
                 frames.front().cols, frames.front().rows);
    std::println("");

    constexpr int kInputSize = 1216;
    constexpr std::int64_t kInputShapeH = kInputSize;
    constexpr std::int64_t kInputShapeW = kInputSize;

    const int total_iters = n_warmup + n_bench;

    // =====================================================================
    // PATH A: CPU preprocess_blob + run()  (original)
    // =====================================================================
    {
        std::vector<double> t_preprocess, t_infer_run, t_postprocess, t_total;
        for (int i = 0; i < total_iters; ++i) {
            const cv::Mat& img = frames[static_cast<std::size_t>(i) % frames.size()];
            const bool is_warmup = i < n_warmup;

            const auto t0 = Clock::now();
            auto [input_data, transform] = preprocess_blob(img, kInputSize, kInputSize);
            const double dt_pre = ms_since(t0);

            const auto t1 = Clock::now();
            std::vector<float> output;
            std::vector<std::int64_t> output_shape;
            const std::vector<std::int64_t> input_shape{1, 3, kInputShapeH, kInputShapeW};
            auto run_result = engine.run(input_data, input_shape, output, output_shape);
            const double dt_infer = ms_since(t1);
            if (!run_result) {
                std::println(stderr, "[CPU path] TensorRT failed: {}", run_result.error());
                return 1;
            }

            const auto t2 = Clock::now();
            Frame frame{.image = img, .timestamp = {}};
            ModelRunResult run_model;
            run_model.success = true;
            run_model.transform = transform;
            run_model.outputs.push_back(ModelTensorData{
                .name = "output0", .shape = std::move(output_shape),
                .element_type = "float32", .values = output});
            [[maybe_unused]] auto ar = adapt_yolo_outputs(frame, run_model);
            const double dt_post = ms_since(t2);
            const double dt_total = ms_since(t0);

            if (!is_warmup) {
                t_preprocess.push_back(dt_pre);
                t_infer_run.push_back(dt_infer);
                t_postprocess.push_back(dt_post);
                t_total.push_back(dt_total);
            }
        }
        std::println("=== PATH A: CPU preprocess + run()  (n={}) ===", n_bench);
        print_stats("preprocess_blob (CPU)", compute_stats(t_preprocess));
        print_stats("trt run (floatH2D+gpu+D2H)", compute_stats(t_infer_run));
        print_stats("adapt_yolo_outputs", compute_stats(t_postprocess));
        print_stats("TOTAL", compute_stats(t_total));
        const double mean = std::accumulate(t_total.begin(), t_total.end(), 0.0)
                            / static_cast<double>(t_total.size());
        std::println("  → {:.1f} FPS (sequential)", 1000.0 / mean);
        std::println("");
    }

    // =====================================================================
    // PATH B: GPU fused preprocess + run_from_bgr()  (new)
    // =====================================================================
    {
        std::vector<double> t_run, t_postprocess, t_total;
        for (int i = 0; i < total_iters; ++i) {
            const cv::Mat& img = frames[static_cast<std::size_t>(i) % frames.size()];
            const bool is_warmup = i < n_warmup;

            const auto t0 = Clock::now();
            std::vector<float> output;
            std::vector<std::int64_t> output_shape;
            auto run_result = engine.run_from_bgr(
                img.data, img.cols, img.rows, kInputSize, kInputSize, output, output_shape);
            const double dt_run = ms_since(t0);
            if (!run_result) {
                std::println(stderr, "[GPU path] run_from_bgr failed: {}", run_result.error());
                return 1;
            }

            const auto t2 = Clock::now();
            Frame frame{.image = img, .timestamp = {}};
            ModelRunResult run_model;
            run_model.success = true;
            run_model.transform = ModelImageTransform{
                .original_width = img.cols, .original_height = img.rows,
                .input_width = kInputSize, .input_height = kInputSize,
                .scale = 1.0F, .pad_x = 0.0F, .pad_y = 0.0F};
            run_model.outputs.push_back(ModelTensorData{
                .name = "output0", .shape = std::move(output_shape),
                .element_type = "float32", .values = output});
            [[maybe_unused]] auto ar = adapt_yolo_outputs(frame, run_model);
            const double dt_post = ms_since(t2);
            const double dt_total = ms_since(t0);

            if (!is_warmup) {
                t_run.push_back(dt_run);
                t_postprocess.push_back(dt_post);
                t_total.push_back(dt_total);
            }
        }
        std::println("=== PATH B: GPU preprocess + run_from_bgr()  (n={}) ===", n_bench);
        print_stats("run_from_bgr (u8H2D+kernel+gpu+D2H)", compute_stats(t_run));
        print_stats("adapt_yolo_outputs", compute_stats(t_postprocess));
        print_stats("TOTAL", compute_stats(t_total));
        const double mean = std::accumulate(t_total.begin(), t_total.end(), 0.0)
                            / static_cast<double>(t_total.size());
        std::println("  → {:.1f} FPS (sequential)", 1000.0 / mean);
        std::println("");
    }

    return 0;
}
