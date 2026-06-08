#include "guidance/scan_controller.hpp"

#include <algorithm>
#include <print>

#include "guidance/galvo_driver.hpp"
#include "guidance/galvo_executor.hpp"

namespace rmcs_laser_guidance {

ScanController::ScanController(const GuidanceConfig& config, GalvoExecutor& executor)
    : config_(config)
    , executor_(executor)
    , enabled_(config.scan_mode == ScanMode::rectangle && executor.is_initialized())
    , voltage_mode_(config.command_model == GuidanceCommandModelKind::direct_voltage) {
    if (enabled_) {
        worker_ = std::thread([this] { run(); });
    }
}

ScanController::~ScanController() { stop(); }

auto ScanController::update_angles_center(const cv::Point2f& center) -> void {
    if (!enabled_ || voltage_mode_) {
        return;
    }
    {
        std::scoped_lock lock(mutex_);
        angle_center_ = center;
        active_ = true;
    }
    cv_.notify_one();
}

auto ScanController::update_voltage_center(const cv::Point2f& center) -> void {
    if (!enabled_ || !voltage_mode_) {
        return;
    }
    {
        std::scoped_lock lock(mutex_);
        voltage_center_ = center;
        active_ = true;
    }
    cv_.notify_one();
}

auto ScanController::deactivate() -> void {
    std::scoped_lock lock(mutex_);
    active_ = false;
}

auto ScanController::stop() -> void {
    {
        std::scoped_lock lock(mutex_);
        stop_requested_ = true;
        active_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

auto ScanController::scan_rectangle_once(const float cx_deg, const float cy_deg) -> std::string {
    auto* driver = executor_.driver();
    if (driver == nullptr) {
        return "scan driver unavailable";
    }
    const float hw = config_.scan_width_deg * 0.5F;
    const float hh = config_.scan_height_deg * 0.5F;
    const int n = std::max(2, config_.scan_grid_n);
    const float sx = config_.scan_width_deg / static_cast<float>(n - 1);
    const float sy = config_.scan_height_deg / static_cast<float>(n - 1);

    for (int row = 0; row < n; ++row) {
        {
            std::scoped_lock lock(mutex_);
            if (stop_requested_ || !active_) {
                return {};
            }
        }
        const float y = cy_deg - hh + static_cast<float>(row) * sy;
        for (int step = 0; step < n; ++step) {
            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_ || !active_) {
                    return {};
                }
            }
            const int col = row % 2 == 0 ? step : (n - 1 - step);
            const float x = cx_deg - hw + static_cast<float>(col) * sx;
            if (auto result = driver->set_angles(x, y); !result) {
                return "scan write failed: " + result.error();
            }
        }
    }

    return {};
}

auto ScanController::scan_rectangle_once_voltage(const float cx_v, const float cy_v) -> std::string {
    auto* driver = executor_.driver();
    if (driver == nullptr) {
        return "scan driver unavailable";
    }
    const float hw = driver->optical_to_voltage(config_.scan_width_deg * 0.5F);
    const float hh = driver->optical_to_voltage(config_.scan_height_deg * 0.5F);
    const int n = std::max(2, config_.scan_grid_n);
    const float sx = (hw * 2.0F) / static_cast<float>(n - 1);
    const float sy = (hh * 2.0F) / static_cast<float>(n - 1);

    for (int row = 0; row < n; ++row) {
        {
            std::scoped_lock lock(mutex_);
            if (stop_requested_ || !active_) {
                return {};
            }
        }
        const float y = cy_v - hh + static_cast<float>(row) * sy;
        for (int step = 0; step < n; ++step) {
            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_ || !active_) {
                    return {};
                }
            }
            const int col = row % 2 == 0 ? step : (n - 1 - step);
            const float x = cx_v - hw + static_cast<float>(col) * sx;
            if (auto result = driver->set_voltages(x, y); !result) {
                return "scan voltage write failed: " + result.error();
            }
        }
    }

    return {};
}

auto ScanController::run() -> void {
    while (true) {
        std::optional<cv::Point2f> angle_center;
        std::optional<cv::Point2f> voltage_center;
        {
            std::unique_lock lock(mutex_);
            cv_.wait(lock, [this] { return stop_requested_ || active_; });
            if (stop_requested_) {
                return;
            }
            angle_center = angle_center_;
            voltage_center = voltage_center_;
        }

        std::string error;
        if (voltage_mode_ && voltage_center.has_value()) {
            error = scan_rectangle_once_voltage(voltage_center->x, voltage_center->y);
        } else if (!voltage_mode_ && angle_center.has_value()) {
            error = scan_rectangle_once(angle_center->x, angle_center->y);
        }

        if (!error.empty()) {
            std::println(stderr, "guidance: {}", error);
        }
    }
}

} // namespace rmcs_laser_guidance
