#include <array>
#include <cstdio>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <print>

#include "test_utils.hpp"
#include "config.hpp"
#include "pipeline.hpp"

namespace {

struct SampleExpectation {
    bool detected;
    float center_x;
    float center_y;
    float tolerance;
};

constexpr std::array<SampleExpectation, 4> kExpectations{{
    {.detected = true, .center_x = 320.0F, .center_y = 240.0F, .tolerance = 3.0F},
    {.detected = false, .center_x = 0.0F, .center_y = 0.0F, .tolerance = 0.0F},
    {.detected = true, .center_x = 120.0F, .center_y = 90.0F, .tolerance = 4.0F},
    {.detected = false, .center_x = 0.0F, .center_y = 0.0F, .tolerance = 0.0F},
}};

constexpr std::array<const char*, 4> kSampleFrameNames{{
    "frame_000000.png",
    "frame_000001.png",
    "frame_000002.png",
    "frame_000003.png",
}};

auto load_sample_frame(const std::filesystem::path& root, const std::size_t index)
    -> rmcs_laser_guidance::Frame {
    const auto image_path = root / "frames" / kSampleFrameNames.at(index);
    cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);
    if (image.empty())
        throw std::runtime_error("failed to load sample image");

    return {
        .image = image,
        .timestamp = rmcs_laser_guidance::Clock::time_point{},
    };
}

} // namespace

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        auto config = rmcs_laser_guidance::load_config(default_config_path());
        config.inference.backend = rmcs_laser_guidance::InferenceBackendKind::bright_spot;
        rmcs_laser_guidance::Pipeline pipeline(config);
        const auto sample_root = default_sample_replay_path();

        for (std::size_t index = 0; index < kExpectations.size(); ++index) {
            const auto expectation = kExpectations[index];
            const auto frame = load_sample_frame(sample_root, index);
            const auto observation = pipeline.process(frame);

            require(
                observation.detected == expectation.detected,
                "sample image detected flag mismatch");
            if (expectation.detected) {
                require_near(
                    observation.center.x, expectation.center_x, expectation.tolerance, "sample center.x");
                require_near(
                    observation.center.y, expectation.center_y, expectation.tolerance, "sample center.y");
            }
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "sample_images_test failed: {}", e.what());
        return 1;
    }
}
