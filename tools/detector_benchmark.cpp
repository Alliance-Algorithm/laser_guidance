#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <print>
#include <numeric>
#include <vector>

#include "example_support.hpp"
#include "config.hpp"
#include "vision/detector.hpp"
#include "core/replay.hpp"

namespace {

using SampleClock = std::chrono::steady_clock;

auto resolve_replay_dir(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
    return rmcs_laser_guidance::examples::default_sample_replay_path();
}

auto resolve_iterations(int argc, char** argv) -> std::size_t {
    if (argc > 2)
        return static_cast<std::size_t>(std::stoull(argv[2]));
    return 100;
}

auto percentile_index(const std::size_t size, const double fraction) -> std::size_t {
    if (size == 0)
        return 0;
    const double raw_index = fraction * static_cast<double>(size - 1);
    return static_cast<std::size_t>(raw_index);
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto replay_dir = resolve_replay_dir(argc, argv);
        const std::size_t iterations = resolve_iterations(argc, argv);
        const auto dataset = rmcs_laser_guidance::load_replay_dataset(replay_dir);

        std::vector<rmcs_laser_guidance::Frame> frames;
        frames.reserve(dataset.frames.size());
        for (const auto& frame_info : dataset.frames)
            frames.push_back(rmcs_laser_guidance::load_replay_frame(dataset, frame_info));

        rmcs_laser_guidance::Config config;
        rmcs_laser_guidance::Detector detector(config);

        std::vector<double> latencies_us;
        latencies_us.reserve(frames.size() * iterations);

        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            for (const auto& frame : frames) {
                const auto begin = SampleClock::now();
                (void)detector.detect(frame);
                const auto end = SampleClock::now();
                latencies_us.push_back(
                    std::chrono::duration<double, std::micro>(end - begin).count());
            }
        }

        if (latencies_us.empty()) {
            std::println(stderr, "No benchmark samples were collected.");
            return 1;
        }

        std::sort(latencies_us.begin(), latencies_us.end());
        const double min_latency = latencies_us.front();
        const double max_latency = latencies_us.back();
        const double mean_latency =
            std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0) / latencies_us.size();
        const double p50 = latencies_us[percentile_index(latencies_us.size(), 0.50)];
        const double p95 = latencies_us[percentile_index(latencies_us.size(), 0.95)];

        std::println(
            "frames={} iterations={} samples={} mean_us={} min_us={} max_us={} p50_us={} p95_us={}",
            frames.size(),
            iterations,
            latencies_us.size(),
            mean_latency,
            min_latency,
            max_latency,
            p50,
            p95);
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "example_detector_benchmark failed: {}", e.what());
        return 1;
    }
}
