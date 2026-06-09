#include "guidance/galvo_executor.hpp"

#include "guidance/galvo_driver.hpp"
#include "io/ft4222_spi.hpp"

namespace rmcs_laser_guidance {

GalvoExecutor::GalvoExecutor(const Config& config, Ft4222Spi& spi)
    : config_(config.guidance) {
    try {
        driver_ = std::make_unique<GalvoDriver>(spi, config_);
        if (auto result = driver_->enable_reference(); !result) {
            init_error_ = "DAC reference enable failed: " + result.error();
            return;
        }
        if (auto result = driver_->set_center(); !result) {
            init_error_ = "galvo center failed: " + result.error();
            return;
        }
        initialized_ = true;
    } catch (const std::exception& e) {
        init_error_ = std::string("driver init failed: ") + e.what();
    }
}

GalvoExecutor::~GalvoExecutor() = default;

auto GalvoExecutor::apply(const AimOutput& aim) -> std::string {
    if (!initialized_) {
        return init_error_.empty() ? "galvo executor not initialized" : init_error_;
    }
    if (!aim.command_issued) {
        return aim.message;
    }

    if (aim.output_voltages.has_value()) {
        latest_output_voltages_ = aim.output_voltages;
        latest_output_angles_.reset();
        return driver_->set_voltages(aim.output_voltages->x, aim.output_voltages->y).error_or("");
    }

    if (aim.output_angles.has_value()) {
        latest_output_angles_ = aim.output_angles;
        latest_output_voltages_ = cv::Point2f{
            driver_->optical_to_voltage(aim.output_angles->x),
            driver_->optical_to_voltage(aim.output_angles->y),
        };
        return driver_->set_angles(aim.output_angles->x, aim.output_angles->y).error_or("");
    }

    return "aim output missing command payload";
}

auto GalvoExecutor::set_center() -> std::string {
    if (!initialized_) {
        return init_error_.empty() ? "galvo executor not initialized" : init_error_;
    }
    latest_output_angles_.reset();
    latest_output_voltages_.reset();
    return driver_->set_center().error_or("");
}

auto GalvoExecutor::process_calib_angle(const float angle_x_deg, const float angle_y_deg)
    -> std::string {
    if (!initialized_) {
        return init_error_.empty() ? "galvo executor not initialized" : init_error_;
    }
    latest_output_angles_ = cv::Point2f{angle_x_deg, angle_y_deg};
    latest_output_voltages_ =
        cv::Point2f{driver_->optical_to_voltage(angle_x_deg), driver_->optical_to_voltage(angle_y_deg)};
    return driver_->set_angles(angle_x_deg, angle_y_deg).error_or("");
}

auto GalvoExecutor::process_calib_voltage(const float x_voltage, const float y_voltage)
    -> std::string {
    if (!initialized_) {
        return init_error_.empty() ? "galvo executor not initialized" : init_error_;
    }
    latest_output_angles_.reset();
    latest_output_voltages_ = cv::Point2f{x_voltage, y_voltage};
    return driver_->set_voltages(x_voltage, y_voltage).error_or("");
}

auto GalvoExecutor::latest_output_angles() const -> std::optional<cv::Point2f> {
    return latest_output_angles_;
}

auto GalvoExecutor::latest_output_voltages() const -> std::optional<cv::Point2f> {
    return latest_output_voltages_;
}

} // namespace rmcs_laser_guidance
