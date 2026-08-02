#pragma once

#include <expected>
#include <memory>
#include <optional>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "laser_guidance/runtime.hpp"

namespace rmcs_laser_guidance {

class DepthEstimator;
class DepthFilter;
class CameraProjection;
class GalvoKinematics;
class VoltageMapper;

struct AimSolveTelemetry {
    std::optional<float> measured_depth_mm{};
    std::optional<float> active_depth_mm{};
    std::optional<cv::Point3f> selected_target_point{};
    bool used_cached_depth = false;
};

struct AimSolveResult {
    AimOutput aim_output{};
    AimSolveTelemetry telemetry{};
};

class AimSolver {
public:
    AimSolver(const Config& config, int image_width, int image_height);
    ~AimSolver();

    [[nodiscard]] auto is_initialized() const noexcept -> bool { return initialized_; }
    [[nodiscard]] auto initialization_error() const -> const std::string& { return init_error_; }

    auto solve(const AimInput& input) -> AimSolveResult;
    auto observe_target(const Detection* detection, double dt_seconds) -> AimSolveTelemetry;
    auto estimate_depth(const Detection& detection) const -> std::optional<float>;
    auto project_to_camera(const cv::Point2f& pixel, float depth_mm) const -> cv::Point3f;
    auto reset_depth_cache() noexcept -> void;
    [[nodiscard]] auto cached_depth_mm() const -> std::optional<float>;
    auto set_offset(float x_deg, float y_deg) -> void {
        config_.angle_offset_x_deg = x_deg;
        config_.angle_offset_y_deg = y_deg;
    }

private:
    auto load_geometry_calibration(const std::filesystem::path& path) -> std::string;
    auto solve_geometry(const AimInput& input) -> AimSolveResult;
    auto solve_direct_voltage(const AimInput& input) -> AimSolveResult;

    GuidanceConfig config_;
    std::unique_ptr<DepthEstimator> depth_estimator_;
    std::unique_ptr<DepthFilter> depth_filter_;
    std::unique_ptr<CameraProjection> projection_;
    std::unique_ptr<GalvoKinematics> kinematics_;
    std::unique_ptr<VoltageMapper> voltage_mapper_;
    bool initialized_ = false;
    std::string init_error_{};
    float image_width_ = 0.0F;
    float image_height_ = 0.0F;
    float last_valid_depth_mm_ = 0.0F;
};

} // namespace rmcs_laser_guidance
