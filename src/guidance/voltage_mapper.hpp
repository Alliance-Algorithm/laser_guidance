#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

struct VoltageCommand {
    float vx = 0.0F;
    float vy = 0.0F;
    bool valid = false;
};

struct VoltageFeatures {
    float center_x = 0.0F;
    float center_y = 0.0F;
    float bbox_w = 0.0F;
    float bbox_h = 0.0F;
    float bbox_area = 0.0F;
    float score = 0.0F;
    int class_id = -1;
    float image_width = 1.0F;
    float image_height = 1.0F;
};

class VoltageMapper {
public:
    explicit VoltageMapper(const GuidanceConfig& config);

    auto predict(const VoltageFeatures& features) const -> std::optional<VoltageCommand>;

private:
    enum class ModelKind : int {
        lut = 0,
        poly3,
    };

    struct LutModel {
        std::vector<float> u_axis {};
        std::vector<float> v_axis {};
        std::vector<float> log_area_axis {};
        std::vector<float> vx_values {};
        std::vector<float> vy_values {};
    };

    struct Poly3Model {
        float log_area_mean = 0.0F;
        float log_area_std = 1.0F;
        std::vector<float> vx_coeffs {};
        std::vector<float> vy_coeffs {};
    };

    static auto load_lut_model(const std::filesystem::path& path) -> LutModel;
    static auto load_poly3_model(const YAML::Node& model) -> Poly3Model;

    [[nodiscard]] auto sample_lut(const std::vector<float>& values,
                                  float u_norm,
                                  float v_norm,
                                  float log_area) const -> float;
    [[nodiscard]] auto sample_poly3(float u_norm,
                                    float v_norm,
                                    float log_area) const -> VoltageCommand;
    [[nodiscard]] auto lut_index(std::size_t a_idx,
                                 std::size_t v_idx,
                                 std::size_t u_idx) const -> std::size_t;

    GuidanceConfig config_;
    ModelKind model_kind_ = ModelKind::lut;
    LutModel lut_ {};
    Poly3Model poly3_ {};
};

} // namespace rmcs_laser_guidance
