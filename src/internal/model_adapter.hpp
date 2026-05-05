#pragma once

#include <memory>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>

#include "internal/model_runtime.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

struct ModelAdapterResult {
    bool success            = false;
    bool contract_supported = false;
    TargetObservation observation;
    std::vector<ModelCandidate> candidates;
    std::string message;
};

class ModelAdapter {
public:
    virtual ~ModelAdapter() = default;
    ModelAdapter()                                  = default;
    ModelAdapter(const ModelAdapter&)               = delete;
    auto operator=(const ModelAdapter&) -> ModelAdapter& = delete;
    ModelAdapter(ModelAdapter&&) noexcept            = delete;
    auto operator=(ModelAdapter&&) noexcept -> ModelAdapter& = delete;

    virtual auto adapt(const Frame& frame, const ModelRuntime& runtime) const
        -> ModelAdapterResult = 0;
};

auto adapt_yolov5_outputs(const Frame& frame, const ModelRunResult& run_result)
    -> ModelAdapterResult;
auto make_default_model_adapter() -> std::unique_ptr<ModelAdapter>;

} // namespace rmcs_laser_guidance
