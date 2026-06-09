#pragma once

#include <expected>
#include <memory>
#include <optional>
#include <string>

#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "laser_guidance/runtime.hpp"

namespace rmcs_laser_guidance {

class Ft4222Spi;
class GalvoDriver;

class GalvoExecutor {
public:
    GalvoExecutor(const Config& config, Ft4222Spi& spi);
    ~GalvoExecutor();

    [[nodiscard]] auto is_initialized() const noexcept -> bool { return initialized_; }
    [[nodiscard]] auto initialization_error() const -> const std::string& { return init_error_; }
    [[nodiscard]] auto command_model() const noexcept -> GuidanceCommandModelKind {
        return config_.command_model;
    }

    auto apply(const AimOutput& aim) -> std::string;
    auto set_center() -> std::string;
    auto process_calib_angle(float angle_x_deg, float angle_y_deg) -> std::string;
    auto process_calib_voltage(float x_voltage, float y_voltage) -> std::string;

    [[nodiscard]] auto latest_output_angles() const -> std::optional<cv::Point2f>;
    [[nodiscard]] auto latest_output_voltages() const -> std::optional<cv::Point2f>;
    [[nodiscard]] auto driver() const -> GalvoDriver* { return driver_.get(); }

private:
    GuidanceConfig config_;
    std::unique_ptr<GalvoDriver> driver_;
    bool initialized_ = false;
    std::string init_error_{};
    std::optional<cv::Point2f> latest_output_angles_{};
    std::optional<cv::Point2f> latest_output_voltages_{};
};

} // namespace rmcs_laser_guidance
