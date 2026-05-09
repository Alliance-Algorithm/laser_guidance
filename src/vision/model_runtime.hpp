#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace rmcs_laser_guidance {

struct ModelValueInfo {
    std::string name { };
    std::vector<std::int64_t> shape { };
    std::string element_type { };
};

struct ModelTensorData {
    std::string name { };
    std::vector<std::int64_t> shape { };
    std::string element_type { };
    std::vector<float> values { };
};

struct ModelImageTransform {
    int original_width  = 0;
    int original_height = 0;
    int input_width     = 0;
    int input_height    = 0;
    float scale         = 1.0F;
    float pad_x         = 0.0F;
    float pad_y         = 0.0F;
};

struct ModelRunResult {
    bool success = false;
    std::string message { };
    ModelImageTransform transform { };
    std::vector<ModelTensorData> outputs { };
};

class ModelRuntime {
public:
    explicit ModelRuntime(std::filesystem::path model_path);
    ~ModelRuntime();

    ModelRuntime(const ModelRuntime&)                    = delete;
    auto operator=(const ModelRuntime&) -> ModelRuntime& = delete;

    ModelRuntime(ModelRuntime&&) noexcept;
    auto operator=(ModelRuntime&&) noexcept -> ModelRuntime&;

    auto load() -> std::string;
    auto run(const cv::Mat& image) const -> ModelRunResult;
    auto is_loaded() const noexcept -> bool;
    auto model_path() const noexcept -> const std::filesystem::path&;
    auto input_values() const noexcept -> const std::vector<ModelValueInfo>&;
    auto output_values() const noexcept -> const std::vector<ModelValueInfo>&;

private:
    struct Details;
    std::filesystem::path model_path_;
    std::unique_ptr<Details> details_;
};

auto model_runtime_enabled_in_build() noexcept -> bool;

} // namespace rmcs_laser_guidance
