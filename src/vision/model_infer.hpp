#pragma once

#include <memory>
#include <string>
#include <vector>

#include "config.hpp"
#include "types.hpp"
#include "vision/model_adapter.hpp"
#include "vision/model_runtime.hpp"

namespace rmcs_laser_guidance {

struct ModelInferResult;

class ModelInfer {
public:
    explicit ModelInfer(InferenceConfig config = {});
    ~ModelInfer();

    ModelInfer(const ModelInfer&) = delete;
    auto operator=(const ModelInfer&) -> ModelInfer& = delete;

    ModelInfer(ModelInfer&&) noexcept;
    auto operator=(ModelInfer&&) noexcept -> ModelInfer&;

    auto infer(const Frame& frame) const -> ModelInferResult;

private:
    struct Details;
    std::unique_ptr<Details> details_;
};

struct ModelInferResult {
    bool enabled = false;
    bool success = false;
    bool contract_supported = false;
    TargetObservation observation{};
    std::vector<ModelCandidate> candidates{};
    std::vector<ModelValueInfo> inputs{};
    std::vector<ModelValueInfo> outputs{};
    std::string message{};
};

} // namespace rmcs_laser_guidance
