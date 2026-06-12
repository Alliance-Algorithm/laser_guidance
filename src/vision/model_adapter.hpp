#pragma once

#include <string>
#include <vector>

#include "types.hpp"
#include "vision/model_runtime.hpp"

namespace rmcs_laser_guidance {

struct ModelAdapterResult {
    bool success = false;
    bool contract_supported = false;
    TargetObservation observation;
    std::vector<ModelCandidate> candidates;
    std::string message;
};

auto adapt_yolo_outputs(const Frame& frame, const ModelRuntime& runtime) -> ModelAdapterResult;
auto adapt_yolo_outputs(const Frame& frame, const ModelRunResult& run_result)
    -> ModelAdapterResult;

} // namespace rmcs_laser_guidance
