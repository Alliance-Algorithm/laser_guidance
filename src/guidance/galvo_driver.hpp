#pragma once

#include <expected>
#include <string>

#include "config.hpp"

namespace rmcs_laser_guidance {

class Ft4222Spi;

class GalvoDriver {
public:
    explicit GalvoDriver(Ft4222Spi& spi, const GuidanceConfig& config);
    ~GalvoDriver() = default;

    GalvoDriver(const GalvoDriver&) = delete;
    auto operator=(const GalvoDriver&) -> GalvoDriver& = delete;

    auto enable_reference() -> std::expected<void, std::string>;
    auto set_center() -> std::expected<void, std::string>;
    auto set_angles(float optical_x_deg, float optical_y_deg)
        -> std::expected<void, std::string>;

    [[nodiscard]] auto negotiated_clock_hz() const noexcept -> uint32_t;

private:
    auto optical_to_voltage(float angle_deg) const -> float;
    auto write_voltage(uint8_t channel, float voltage, const char* label)
        -> std::expected<void, std::string>;

    Ft4222Spi& spi_;
    GuidanceConfig config_;
    bool reference_enabled_ = false;
};

} // namespace rmcs_laser_guidance
