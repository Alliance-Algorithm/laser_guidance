#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "guidance/voltage_mapper.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class DepthEstimator;
class CameraProjection;
class GalvoKinematics;
class GalvoDriver;
class Ft4222Spi;

class GuidancePipeline {
public:
    GuidancePipeline(const Config& config, Ft4222Spi& spi);
    ~GuidancePipeline();

    GuidancePipeline(const GuidancePipeline&) = delete;
    auto operator=(const GuidancePipeline&) -> GuidancePipeline& = delete;

    auto process(const TargetObservation& observation) -> std::string;

    auto process_ekf_guided(const cv::Point2f& ekf_center,
                             const ModelCandidate* candidate,
                             float& io_depth_mm) -> std::string;

    auto set_center() -> std::string;

    auto process_calib() -> std::string;
    auto process_calib_angle(float angle_x_deg, float angle_y_deg) -> std::string;
    auto process_calib_voltage(float x_voltage, float y_voltage) -> std::string;

    auto project_to_camera(const cv::Point2f& pixel, float depth_mm) const
        -> cv::Point3f;

    auto estimate_depth(const ModelCandidate& candidate) const
        -> std::optional<float>;

    [[nodiscard]] auto latest_output_angles() const -> std::optional<cv::Point2f>;
    [[nodiscard]] auto latest_output_voltages() const -> std::optional<cv::Point2f>;

    [[nodiscard]] auto is_initialized() const noexcept -> bool { return initialized_; }

private:
    auto initialize_driver() -> std::string;
    auto load_calibration(const std::filesystem::path& path) -> std::string;
    auto write_single(float theta_x, float theta_y, float depth_mm,
                      const cv::Point2f& center) -> std::string;
    auto write_voltage_single(float x_voltage, float y_voltage,
                              const cv::Point2f& center) -> std::string;
    auto process_direct_voltage_guided(const cv::Point2f& ekf_center,
                                       const ModelCandidate* candidate) -> std::string;
    auto update_scan_center(float theta_x, float theta_y, float depth_mm,
                            const cv::Point2f& center) -> std::string;
    auto scan_rectangle_once(float cx_deg, float cy_deg) -> std::string;
    auto scan_rectangle_once_voltage(float cx_v, float cy_v) -> std::string;
    auto start_scan_thread() -> void;
    auto stop_scan_thread() -> void;
    auto scan_loop() -> void;

    Ft4222Spi& spi_;
    GuidanceConfig config_;
    std::unique_ptr<DepthEstimator> depth_estimator_;
    std::unique_ptr<CameraProjection> projection_;
    std::unique_ptr<GalvoKinematics> kinematics_;
    std::unique_ptr<GalvoDriver> driver_;
    std::unique_ptr<VoltageMapper> voltage_mapper_;
    bool initialized_ = false;
    std::mutex driver_mutex_;
    std::mutex scan_mutex_;
    std::condition_variable scan_cv_;
    std::thread scan_thread_;
    bool scan_stop_ = false;
    bool scan_active_ = false;
    float scan_center_x_deg_ = 0.0F;
    float scan_center_y_deg_ = 0.0F;
    float scan_center_vx_ = 0.0F;
    float scan_center_vy_ = 0.0F;
    std::atomic<float> last_output_theta_x_deg_ { 0.0F };
    std::atomic<float> last_output_theta_y_deg_ { 0.0F };
    std::atomic<float> last_output_vx_ { 0.0F };
    std::atomic<float> last_output_vy_ { 0.0F };
    std::atomic<bool> has_output_angles_ { false };
    std::atomic<bool> has_output_voltages_ { false };
    float image_width_ = 0.0F;
    float image_height_ = 0.0F;
};

} // namespace rmcs_laser_guidance
