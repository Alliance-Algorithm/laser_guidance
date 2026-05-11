#include "guidance/guidance_pipeline.hpp"

#include <algorithm>
#include <filesystem>
#include <print>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "guidance/camera_projection.hpp"
#include "guidance/depth_estimator.hpp"
#include "guidance/galvo_driver.hpp"
#include "guidance/galvo_kinematics.hpp"
#include "io/ft4222_spi.hpp"

namespace rmcs_laser_guidance {
namespace {

    auto load_yaml_calibration(const std::filesystem::path& path)
        -> std::pair<cv::Mat, cv::Mat> {
        const auto yaml = YAML::LoadFile(path.string());
        const auto calib = yaml["calibration"];
        if (!calib)
            throw std::runtime_error("YAML missing 'calibration' key");

        cv::Mat camera = cv::Mat::eye(3, 3, CV_64F);
        const auto mat_node = calib["camera_matrix"];
        if (!mat_node || mat_node.size() != 3)
            throw std::runtime_error("missing camera_matrix");
        for (int i = 0; i < 3; ++i) {
            if (mat_node[i].size() != 3)
                throw std::runtime_error("camera_matrix row must have 3 columns");
            for (int j = 0; j < 3; ++j)
                camera.at<double>(i, j) = mat_node[i][j].as<double>();
        }

        const auto dc_node = calib["dist_coeffs"];
        cv::Mat dist;
        if (dc_node) {
            const int n = static_cast<int>(dc_node.size());
            dist = cv::Mat::zeros(1, n, CV_64F);
            for (int i = 0; i < n; ++i)
                dist.at<double>(i) = dc_node[i].as<double>();
        }

        return { camera, dist };
    }

} // namespace

GuidancePipeline::GuidancePipeline(const Config& config, Ft4222Spi& spi)
    : spi_(spi)
    , config_(config.guidance) {
    if (!config_.enabled) return;

    const auto init_err = load_calibration(config_.camera_calib_path);
    if (!init_err.empty()) {
        std::println(stderr, "guidance: {}", init_err);
        return;
    }
    initialized_ = true;
    if (config_.scan_mode == ScanMode::rectangle && !config_.calib_mode) {
        start_scan_thread();
    }
    if (config_.calib_mode) {
        std::println("guidance: CALIB MODE — galvo locked at θ=({:.2f}°,{:.2f}°)",
                     config_.calib_angle_x_deg, config_.calib_angle_y_deg);
    }
}

GuidancePipeline::~GuidancePipeline() {
    stop_scan_thread();
}

auto GuidancePipeline::load_calibration(const std::filesystem::path& path)
    -> std::string {
    try {
        auto [camera, dist] = load_yaml_calibration(path);

        projection_ = std::make_unique<CameraProjection>(
            camera.clone(), dist.clone());
        depth_estimator_ = std::make_unique<DepthEstimator>(
            config_, camera.clone());
        kinematics_ = std::make_unique<GalvoKinematics>(config_);
        driver_ = std::make_unique<GalvoDriver>(spi_, config_);

        if (auto r = driver_->enable_reference(); !r)
            return "DAC reference enable failed: " + r.error();

        if (auto r = driver_->set_center(); !r)
            return "galvo center failed: " + r.error();

        std::println("guidance: initialized, K=[{}x{}] mirror_d={:.1f}mm t=[{:.1f},{:.1f},{:.1f}]mm",
                     camera.cols, camera.rows,
                     config_.mirror_separation_mm,
                     config_.t_x_mm, config_.t_y_mm, config_.t_z_mm);
        return {};
    } catch (const std::exception& e) {
        return std::string("calibration load failed: ") + e.what();
    }
}

auto GuidancePipeline::process(const TargetObservation& observation) -> std::string {
    if (!initialized_) return "guidance not initialized";
    if (config_.calib_mode) return "";
    if (!observation.detected) return "";
    if (observation.candidates.empty()) return "no candidates";

    const auto& top = observation.candidates.front();
    const auto depth = depth_estimator_->estimate(top);
    if (!depth) return "depth estimate failed";

    const auto P_c = projection_->project(top.center, *depth);
    const auto angles = kinematics_->compute(P_c);
    if (!angles.valid) return "kinematics failed";
    if (config_.scan_mode == ScanMode::rectangle) {
        return update_scan_center(angles.theta_x_optical_deg,
                                  angles.theta_y_optical_deg,
                                  *depth, top.center);
    }
    return write_single(angles.theta_x_optical_deg,
                        angles.theta_y_optical_deg,
                        *depth, top.center);
}

auto GuidancePipeline::process_ekf_guided(const cv::Point2f& ekf_center,
                                           const ModelCandidate* candidate,
                                           float& io_depth_mm) -> std::string {
    if (!initialized_) return "guidance not initialized";
    if (config_.calib_mode) return "";

    if (candidate != nullptr
        && candidate->bbox.width > 0.0F
        && candidate->bbox.height > 0.0F) {
        const auto depth = depth_estimator_->estimate(*candidate);
        if (depth) {
            io_depth_mm = *depth;
        }
    }

    if (io_depth_mm <= 0.0F) return "no valid depth";

    const auto P_c = projection_->project(ekf_center, io_depth_mm);
    const auto angles = kinematics_->compute(P_c);
    if (!angles.valid) return "kinematics failed";

    if (config_.scan_mode == ScanMode::rectangle) {
        return update_scan_center(angles.theta_x_optical_deg,
                                  angles.theta_y_optical_deg,
                                  io_depth_mm, ekf_center);
    }

    return write_single(angles.theta_x_optical_deg,
                        angles.theta_y_optical_deg,
                        io_depth_mm, ekf_center);
}

auto GuidancePipeline::set_center() -> std::string {
    if (!initialized_) return "guidance not initialized";
    std::println("guidance: centering galvo");
    {
        std::scoped_lock scan_lock(scan_mutex_);
        scan_active_ = false;
    }
    scan_cv_.notify_all();
    std::scoped_lock driver_lock(driver_mutex_);
    if (auto r = driver_->set_center(); !r) {
        return "galvo center failed: " + r.error();
    }
    return "";
}

auto GuidancePipeline::write_single(float theta_x, float theta_y,
                                     float depth_mm, const cv::Point2f& center)
    -> std::string {
    static int log_counter = 0;
    if (++log_counter % 30 == 0) {
        std::println("guidance: depth={:.1f}mm aim=({:.1f},{:.1f}) θ=[{:.2f}°,{:.2f}°]",
                     depth_mm, center.x, center.y, theta_x, theta_y);
    }
    last_output_theta_x_deg_.store(theta_x, std::memory_order_relaxed);
    last_output_theta_y_deg_.store(theta_y, std::memory_order_relaxed);
    has_output_angles_.store(true, std::memory_order_relaxed);
    std::scoped_lock driver_lock(driver_mutex_);
    if (auto r = driver_->set_angles(theta_x, theta_y); !r) {
        return "galvo write failed: " + r.error();
    }
    return "";
}

auto GuidancePipeline::update_scan_center(float theta_x, float theta_y,
                                          float depth_mm, const cv::Point2f& center)
    -> std::string {
    static int log_counter = 0;
    if (++log_counter % 30 == 0) {
        std::println("guidance: scan rect {:.1f}°×{:.1f}°({}×{}) depth={:.1f}mm",
                     config_.scan_width_deg, config_.scan_height_deg,
                     std::max(2, config_.scan_grid_n),
                     std::max(2, config_.scan_grid_n),
                     depth_mm);
    }

    {
        std::scoped_lock lock(scan_mutex_);
        scan_center_x_deg_ = theta_x;
        scan_center_y_deg_ = theta_y;
        scan_active_ = true;
    }
    last_output_theta_x_deg_.store(theta_x, std::memory_order_relaxed);
    last_output_theta_y_deg_.store(theta_y, std::memory_order_relaxed);
    has_output_angles_.store(true, std::memory_order_relaxed);
    scan_cv_.notify_one();
    (void)center;
    return "";
}

auto GuidancePipeline::scan_rectangle_once(float cx_deg, float cy_deg)
    -> std::string {
    const float hw = config_.scan_width_deg * 0.5F;
    const float hh = config_.scan_height_deg * 0.5F;
    const int n = std::max(2, config_.scan_grid_n);
    const float sx = config_.scan_width_deg / static_cast<float>(n - 1);
    const float sy = config_.scan_height_deg / static_cast<float>(n - 1);

    for (int row = 0; row < n; ++row) {
        {
            std::scoped_lock scan_lock(scan_mutex_);
            if (scan_stop_ || !scan_active_) return "";
        }
        const float y = cy_deg - hh + static_cast<float>(row) * sy;
        if (row % 2 == 0) {
            for (int col = 0; col < n; ++col) {
                {
                    std::scoped_lock scan_lock(scan_mutex_);
                    if (scan_stop_ || !scan_active_) return "";
                }
                const float x = cx_deg - hw + static_cast<float>(col) * sx;
                std::scoped_lock driver_lock(driver_mutex_);
                if (auto r = driver_->set_angles(x, y); !r)
                    return "scan write failed: " + r.error();
            }
        } else {
            for (int col = n - 1; col >= 0; --col) {
                {
                    std::scoped_lock scan_lock(scan_mutex_);
                    if (scan_stop_ || !scan_active_) return "";
                }
                const float x = cx_deg - hw + static_cast<float>(col) * sx;
                std::scoped_lock driver_lock(driver_mutex_);
                if (auto r = driver_->set_angles(x, y); !r)
                    return "scan write failed: " + r.error();
            }
        }
    }
    return "";
}

auto GuidancePipeline::start_scan_thread() -> void {
    scan_stop_ = false;
    scan_thread_ = std::thread([this] { scan_loop(); });
}

auto GuidancePipeline::stop_scan_thread() -> void {
    {
        std::scoped_lock lock(scan_mutex_);
        scan_stop_ = true;
        scan_active_ = false;
    }
    scan_cv_.notify_all();
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }
}

auto GuidancePipeline::scan_loop() -> void {
    while (true) {
        float cx_deg = 0.0F;
        float cy_deg = 0.0F;
        {
            std::unique_lock lock(scan_mutex_);
            scan_cv_.wait(lock, [this] { return scan_stop_ || scan_active_; });
            if (scan_stop_) return;
            cx_deg = scan_center_x_deg_;
            cy_deg = scan_center_y_deg_;
        }
        if (auto err = scan_rectangle_once(cx_deg, cy_deg); !err.empty()) {
            std::println(stderr, "guidance: {}", err);
        }
    }
}

auto GuidancePipeline::process_calib() -> std::string {
    if (!initialized_) return "guidance not initialized";
    if (!config_.calib_mode) return "";
    return process_calib_angle(config_.calib_angle_x_deg,
                               config_.calib_angle_y_deg);
}

auto GuidancePipeline::process_calib_angle(float angle_x_deg, float angle_y_deg) -> std::string {
    if (!initialized_) return "guidance not initialized";
    last_output_theta_x_deg_.store(angle_x_deg, std::memory_order_relaxed);
    last_output_theta_y_deg_.store(angle_y_deg, std::memory_order_relaxed);
    has_output_angles_.store(true, std::memory_order_relaxed);
    std::scoped_lock driver_lock(driver_mutex_);
    return driver_->set_angles(angle_x_deg, angle_y_deg).error_or("");
}

auto GuidancePipeline::project_to_camera(const cv::Point2f& pixel, float depth_mm) const
    -> cv::Point3f {
    if (!projection_) return { -1, -1, -1 };
    return projection_->project(pixel, depth_mm);
}

auto GuidancePipeline::estimate_depth(const ModelCandidate& candidate) const
    -> std::optional<float> {
    if (!depth_estimator_) return std::nullopt;
    return depth_estimator_->estimate(candidate);
}

auto GuidancePipeline::latest_output_angles() const -> std::optional<cv::Point2f> {
    if (!has_output_angles_.load(std::memory_order_relaxed)) return std::nullopt;
    return cv::Point2f {
        last_output_theta_x_deg_.load(std::memory_order_relaxed),
        last_output_theta_y_deg_.load(std::memory_order_relaxed),
    };
}

} // namespace rmcs_laser_guidance
