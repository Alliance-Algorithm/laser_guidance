#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/types.hpp>

#include "config.hpp"

namespace rmcs_laser_guidance {

class GalvoExecutor;
class GalvoDriver;

class ScanController {
public:
    ScanController(const GuidanceConfig& config, GalvoExecutor& executor);
    ~ScanController();

    ScanController(const ScanController&) = delete;
    auto operator=(const ScanController&) -> ScanController& = delete;

    [[nodiscard]] auto enabled() const noexcept -> bool { return enabled_; }
    auto update_angles_center(const cv::Point2f& center) -> void;
    auto update_voltage_center(const cv::Point2f& center) -> void;
    auto deactivate() -> void;
    auto stop() -> void;

private:
    auto scan_rectangle_once(float cx_deg, float cy_deg) -> std::string;
    auto scan_rectangle_once_voltage(float cx_v, float cy_v) -> std::string;
    auto run() -> void;

    GuidanceConfig config_;
    GalvoExecutor& executor_;
    bool enabled_ = false;
    bool voltage_mode_ = false;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    bool stop_requested_ = false;
    bool active_ = false;
    std::optional<cv::Point2f> angle_center_{};
    std::optional<cv::Point2f> voltage_center_{};
};

} // namespace rmcs_laser_guidance
