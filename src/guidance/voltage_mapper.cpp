#include "guidance/voltage_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rmcs_laser_guidance {
namespace {

constexpr float kMinDim = 1.0F;
constexpr float kMinArea = 1.0F;

struct AxisSample {
    std::size_t lo = 0;
    std::size_t hi = 0;
    float t = 0.0F;
};

auto parse_float_array(const YAML::Node& node, const char* key) -> std::vector<float> {
    const auto child = node[key];
    if (!child || !child.IsSequence() || child.size() == 0) {
        throw std::runtime_error(std::string("voltage LUT missing non-empty sequence: ") + key);
    }

    std::vector<float> out;
    out.reserve(child.size());
    for (const auto& item : child) out.push_back(item.as<float>());
    if (!std::is_sorted(out.begin(), out.end())) {
        throw std::runtime_error(std::string("voltage LUT axis must be sorted: ") + key);
    }
    return out;
}

auto parse_exact_float_array(const YAML::Node& node,
                             const char* key,
                             std::size_t expected_size) -> std::vector<float> {
    const auto child = node[key];
    if (!child || !child.IsSequence() || child.size() != expected_size) {
        throw std::runtime_error(std::string("polynomial coefficient count mismatch: ") + key);
    }
    std::vector<float> out;
    out.reserve(expected_size);
    for (const auto& item : child) out.push_back(item.as<float>());
    return out;
}

auto flatten_volume(const YAML::Node& node,
                    std::size_t area_n,
                    std::size_t v_n,
                    std::size_t u_n,
                    const char* key) -> std::vector<float> {
    const auto child = node[key];
    if (!child || !child.IsSequence() || child.size() != area_n) {
        throw std::runtime_error(std::string("voltage LUT volume size mismatch: ") + key);
    }

    std::vector<float> values;
    values.reserve(area_n * v_n * u_n);
    for (std::size_t a = 0; a < area_n; ++a) {
        const auto plane = child[a];
        if (!plane.IsSequence() || plane.size() != v_n) {
            throw std::runtime_error(std::string("voltage LUT plane row count mismatch: ") + key);
        }
        for (std::size_t v = 0; v < v_n; ++v) {
            const auto row = plane[v];
            if (!row.IsSequence() || row.size() != u_n) {
                throw std::runtime_error(std::string("voltage LUT row width mismatch: ") + key);
            }
            for (std::size_t u = 0; u < u_n; ++u) values.push_back(row[u].as<float>());
        }
    }
    return values;
}

auto axis_sample(const std::vector<float>& axis, float x) -> AxisSample {
    if (axis.size() == 1) return {};
    const float clamped = std::clamp(x, axis.front(), axis.back());
    auto upper = std::lower_bound(axis.begin(), axis.end(), clamped);
    if (upper == axis.begin()) return {0, 0, 0.0F};
    if (upper == axis.end()) {
        const auto last = axis.size() - 1;
        return {last, last, 0.0F};
    }
    if (*upper == clamped) {
        const auto idx = static_cast<std::size_t>(std::distance(axis.begin(), upper));
        return {idx, idx, 0.0F};
    }

    const auto hi = static_cast<std::size_t>(std::distance(axis.begin(), upper));
    const auto lo = hi - 1;
    const float denom = axis[hi] - axis[lo];
    const float t = denom > 0.0F ? (clamped - axis[lo]) / denom : 0.0F;
    return {lo, hi, t};
}

auto lerp(float a, float b, float t) -> float {
    return a + (b - a) * t;
}

auto cubic_feature_basis(float u_norm, float v_norm, float log_area_norm) -> std::vector<float> {
    const float x = u_norm * 2.0F - 1.0F;
    const float y = v_norm * 2.0F - 1.0F;
    const float z = log_area_norm;

    std::vector<float> out;
    out.reserve(20);
    for (int total = 0; total <= 3; ++total) {
        for (int i = 0; i <= total; ++i) {
            for (int j = 0; j <= total - i; ++j) {
                const int k = total - i - j;
                out.push_back(std::pow(x, static_cast<float>(i))
                            * std::pow(y, static_cast<float>(j))
                            * std::pow(z, static_cast<float>(k)));
            }
        }
    }
    return out;
}

auto dot(const std::vector<float>& a, const std::vector<float>& b) -> float {
    float sum = 0.0F;
    for (std::size_t i = 0; i < a.size(); ++i) sum += a[i] * b[i];
    return sum;
}

} // namespace

VoltageMapper::VoltageMapper(const GuidanceConfig& config)
    : config_(config) {
    if (config_.voltage_model_path.empty()) {
        throw std::runtime_error("direct_voltage mode requires guidance.voltage_model_path");
    }
    const auto yaml = YAML::LoadFile(config_.voltage_model_path.string());
    const auto model = yaml["model"];
    if (!model || !model.IsMap()) {
        throw std::runtime_error("voltage model YAML missing 'model' map");
    }

    const auto type = model["type"] ? model["type"].as<std::string>() : std::string{};
    if (type == "lut") {
        model_kind_ = ModelKind::lut;
        lut_ = load_lut_model(config_.voltage_model_path);
    } else if (type == "poly3") {
        model_kind_ = ModelKind::poly3;
        poly3_ = load_poly3_model(model);
    } else {
        throw std::runtime_error("voltage model.type must be 'lut' or 'poly3'");
    }
}

auto VoltageMapper::load_lut_model(const std::filesystem::path& path) -> LutModel {
    const auto yaml = YAML::LoadFile(path.string());
    const auto model = yaml["model"];
    if (!model || !model.IsMap()) {
        throw std::runtime_error("voltage LUT YAML missing 'model' map");
    }

    const auto type = model["type"] ? model["type"].as<std::string>() : std::string{};
    if (type != "lut") throw std::runtime_error("voltage LUT model.type must be 'lut'");

    LutModel lut;
    lut.u_axis = parse_float_array(model, "u_axis");
    lut.v_axis = parse_float_array(model, "v_axis");
    lut.log_area_axis = parse_float_array(model, "log_area_axis");

    const auto area_n = lut.log_area_axis.size();
    const auto v_n = lut.v_axis.size();
    const auto u_n = lut.u_axis.size();
    lut.vx_values = flatten_volume(model, area_n, v_n, u_n, "vx_values");
    lut.vy_values = flatten_volume(model, area_n, v_n, u_n, "vy_values");
    return lut;
}

auto VoltageMapper::load_poly3_model(const YAML::Node& model) -> Poly3Model {
    constexpr std::size_t kCoeffCount = 20;
    Poly3Model poly;
    if (model["log_area_mean"]) poly.log_area_mean = model["log_area_mean"].as<float>();
    if (model["log_area_std"]) poly.log_area_std = model["log_area_std"].as<float>();
    if (poly.log_area_std <= 0.0F) poly.log_area_std = 1.0F;
    poly.vx_coeffs = parse_exact_float_array(model, "vx_coeffs", kCoeffCount);
    poly.vy_coeffs = parse_exact_float_array(model, "vy_coeffs", kCoeffCount);
    return poly;
}

auto VoltageMapper::lut_index(std::size_t a_idx,
                              std::size_t v_idx,
                              std::size_t u_idx) const -> std::size_t {
    return (a_idx * lut_.v_axis.size() + v_idx) * lut_.u_axis.size() + u_idx;
}

auto VoltageMapper::sample_lut(const std::vector<float>& values,
                               float u_norm,
                               float v_norm,
                               float log_area) const -> float {
    const auto su = axis_sample(lut_.u_axis, u_norm);
    const auto sv = axis_sample(lut_.v_axis, v_norm);
    const auto sa = axis_sample(lut_.log_area_axis, log_area);

    const auto sample = [&](std::size_t a, std::size_t v, std::size_t u) {
        return values[lut_index(a, v, u)];
    };

    const float c000 = sample(sa.lo, sv.lo, su.lo);
    const float c001 = sample(sa.lo, sv.lo, su.hi);
    const float c010 = sample(sa.lo, sv.hi, su.lo);
    const float c011 = sample(sa.lo, sv.hi, su.hi);
    const float c100 = sample(sa.hi, sv.lo, su.lo);
    const float c101 = sample(sa.hi, sv.lo, su.hi);
    const float c110 = sample(sa.hi, sv.hi, su.lo);
    const float c111 = sample(sa.hi, sv.hi, su.hi);

    const float c00 = lerp(c000, c001, su.t);
    const float c01 = lerp(c010, c011, su.t);
    const float c10 = lerp(c100, c101, su.t);
    const float c11 = lerp(c110, c111, su.t);
    const float c0 = lerp(c00, c01, sv.t);
    const float c1 = lerp(c10, c11, sv.t);
    return lerp(c0, c1, sa.t);
}

auto VoltageMapper::sample_poly3(float u_norm,
                                 float v_norm,
                                 float log_area) const -> VoltageCommand {
    const float log_area_norm = (log_area - poly3_.log_area_mean) / poly3_.log_area_std;
    const auto features = cubic_feature_basis(u_norm, v_norm, log_area_norm);
    VoltageCommand out;
    out.vx = dot(features, poly3_.vx_coeffs);
    out.vy = dot(features, poly3_.vy_coeffs);
    out.vx = std::clamp(out.vx, -config_.voltage_limit_v, config_.voltage_limit_v);
    out.vy = std::clamp(out.vy, -config_.voltage_limit_v, config_.voltage_limit_v);
    out.valid = true;
    return out;
}

auto VoltageMapper::predict(const VoltageFeatures& features) const -> std::optional<VoltageCommand> {
    if (features.image_width < kMinDim || features.image_height < kMinDim) return std::nullopt;
    if (features.bbox_w <= 0.0F || features.bbox_h <= 0.0F) return std::nullopt;

    const float u_norm = std::clamp(features.center_x / features.image_width, 0.0F, 1.0F);
    const float v_norm = std::clamp(features.center_y / features.image_height, 0.0F, 1.0F);
    const float area = std::max(features.bbox_area, kMinArea);
    const float log_area = std::log(area);

    if (model_kind_ == ModelKind::poly3) {
        return sample_poly3(u_norm, v_norm, log_area);
    }

    VoltageCommand out;
    out.vx = sample_lut(lut_.vx_values, u_norm, v_norm, log_area);
    out.vy = sample_lut(lut_.vy_values, u_norm, v_norm, log_area);
    out.vx = std::clamp(out.vx, -config_.voltage_limit_v, config_.voltage_limit_v);
    out.vy = std::clamp(out.vy, -config_.voltage_limit_v, config_.voltage_limit_v);
    out.valid = true;
    return out;
}

} // namespace rmcs_laser_guidance
