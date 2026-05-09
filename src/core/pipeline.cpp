#include "pipeline.hpp"

#include <memory>
#include <stdexcept>

#include "core/debug_renderer.hpp"
#include "vision/detector.hpp"
#include "vision/model_infer.hpp"

namespace rmcs_laser_guidance {
namespace {

    class ObservationBackend {
    public:
        virtual ~ObservationBackend() = default;

        virtual auto process(const Frame& frame) const -> TargetObservation = 0;
    };

    class BrightSpotBackend final : public ObservationBackend {
    public:
        explicit BrightSpotBackend(const Config& config)
            : detector_(config) { }

        auto process(const Frame& frame) const -> TargetObservation override {
            return detector_.detect(frame);
        }

    private:
        Detector detector_;
    };

    class ModelBackend final : public ObservationBackend {
    public:
        explicit ModelBackend(const Config& config)
            : model_infer_(config.inference) { }

        auto process(const Frame& frame) const -> TargetObservation override {
            const auto result = model_infer_.infer(frame);
            if (!result.success)
                throw std::runtime_error(result.message);
            auto observation = result.observation;
            observation.candidates = result.candidates;
            return observation;
        }

    private:
        ModelInfer model_infer_;
    };

    auto make_observation_backend(const Config& config) -> std::unique_ptr<ObservationBackend> {
        switch (config.inference.backend) {
        case InferenceBackendKind::bright_spot:
            return std::make_unique<BrightSpotBackend>(config);
        case InferenceBackendKind::model:
            return std::make_unique<ModelBackend>(config);
        }

        throw std::runtime_error("unsupported inference backend");
    }

} // namespace

struct Pipeline::Details {
    explicit Details(const Config& config)
        : backend(make_observation_backend(config))
        , debug_renderer(config.debug) { }

    std::unique_ptr<ObservationBackend> backend;
    DebugRenderer debug_renderer;
};

Pipeline::Pipeline(const Config& config)
    : details_(std::make_unique<Details>(config)) { }

Pipeline::~Pipeline() = default;

Pipeline::Pipeline(Pipeline&&) noexcept = default;

auto Pipeline::operator=(Pipeline&&) noexcept -> Pipeline& = default;

auto Pipeline::process(const Frame& frame) const -> TargetObservation {
    return details_->backend->process(frame);
}

auto Pipeline::draw_debug_overlay(cv::Mat& image, const TargetObservation& observation) const
    -> void {
    details_->debug_renderer.draw(image, observation);
}

} // namespace rmcs_laser_guidance
