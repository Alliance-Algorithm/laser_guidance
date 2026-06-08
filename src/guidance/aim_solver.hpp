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
class LidarDepthEstimator;
class CameraProjection;
class GalvoKinematics;
class VoltageMapper;

class AimSolver {
public:
    AimSolver(const Config& config, int image_width, int image_height);
    ~AimSolver();

    [[nodiscard]] auto is_initialized() const noexcept -> bool { return initialized_; }
    [[nodiscard]] auto initialization_error() const -> const std::string& { return init_error_; }

    auto solve(AimInput& input) -> AimOutput;
    auto estimate_depth(const Detection& detection, const LidarFrame* lidar_frame) const
        -> std::optional<float>;
    auto project_to_camera(const cv::Point2f& pixel, float depth_mm) const -> cv::Point3f;

private:
    auto load_geometry_calibration(const std::filesystem::path& path) -> std::string;
    auto solve_geometry(AimInput& input) -> AimOutput;
    auto solve_direct_voltage(AimInput& input) -> AimOutput;

    GuidanceConfig config_;
    std::unique_ptr<DepthEstimator> depth_estimator_;
    std::unique_ptr<LidarDepthEstimator> lidar_depth_estimator_;
    std::unique_ptr<CameraProjection> projection_;
    std::unique_ptr<GalvoKinematics> kinematics_;
    std::unique_ptr<VoltageMapper> voltage_mapper_;
    bool initialized_ = false;
    std::string init_error_{};
    float image_width_ = 0.0F;
    float image_height_ = 0.0F;
};

} // namespace rmcs_laser_guidance
