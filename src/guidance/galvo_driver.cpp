#include "guidance/galvo_driver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <print>

#include "io/ft4222_spi.hpp"

namespace rmcs_laser_guidance {
namespace {

    auto clamp_voltage(double voltage) -> double {
        return std::clamp(voltage, -10.0, 10.0);
    }

    auto voltage_to_code(double voltage) -> std::uint16_t {
        const double clipped = clamp_voltage(voltage);
        const double code = ((clipped + 10.0) / 20.0)
            * static_cast<double>(std::numeric_limits<std::uint16_t>::max());
        return static_cast<std::uint16_t>(std::lround(code));
    }

    auto build_payload(std::uint8_t control, std::uint8_t address,
                       std::uint16_t data, std::uint8_t feature = 0) -> std::uint32_t {
        return (static_cast<std::uint32_t>(control) << 24)
             | (static_cast<std::uint32_t>(address) << 20)
             | (static_cast<std::uint32_t>(data) << 4)
             | static_cast<std::uint32_t>(feature);
    }

    auto payload_bytes(std::uint32_t payload) -> std::array<std::uint8_t, 4> {
        return {
            static_cast<std::uint8_t>((payload >> 24) & 0xFF),
            static_cast<std::uint8_t>((payload >> 16) & 0xFF),
            static_cast<std::uint8_t>((payload >> 8) & 0xFF),
            static_cast<std::uint8_t>(payload & 0xFF),
        };
    }

} // namespace

GalvoDriver::GalvoDriver(Ft4222Spi& spi, const GuidanceConfig& config)
    : spi_(spi), config_(config) {}

auto GalvoDriver::enable_reference() -> std::expected<void, std::string> {
    constexpr std::uint32_t kEnableInternalRef = 0x08000001u;
    const auto bytes = payload_bytes(kEnableInternalRef);
    std::println("galvo: enabling DAC internal reference");
    auto result = spi_.write(bytes.data(), 4);
    if (result) reference_enabled_ = true;
    return result;
}

auto GalvoDriver::negotiated_clock_hz() const noexcept -> std::uint32_t {
    return spi_.negotiated_clock_hz();
}

auto GalvoDriver::set_center() -> std::expected<void, std::string> {
    return set_angles(0.0F, 0.0F);
}

auto GalvoDriver::optical_to_voltage(float angle_deg) const -> float {
    const float frac = angle_deg / config_.max_optical_angle_deg;
    return frac * config_.input_voltage_range_v;
}

auto GalvoDriver::write_voltage(std::uint8_t channel, float voltage, const char*)
    -> std::expected<void, std::string> {

    const double clipped = clamp_voltage(static_cast<double>(voltage));
    const auto code = voltage_to_code(clipped);
    const auto payload = build_payload(0x03, channel, code, 0x00);
    const auto bytes = payload_bytes(payload);
    return spi_.write(bytes.data(), 4);
}

auto GalvoDriver::set_angles(float optical_x_deg, float optical_y_deg)
    -> std::expected<void, std::string> {

    if (!reference_enabled_) {
        return std::unexpected("DAC internal reference not enabled");
    }

    const float vx = optical_to_voltage(optical_x_deg);
    const float vy = optical_to_voltage(optical_y_deg);

    const auto& w = config_.wiring;
    const bool diff = w.mode == GalvoWiringMode::differential;

    const float x_pos_v = diff ? vx : vx;
    const float x_neg_v = diff ? -vx : 0.0F;
    const float y_pos_v = diff ? vy : vy;
    const float y_neg_v = diff ? -vy : 0.0F;

    if (auto r = write_voltage(static_cast<std::uint8_t>(w.x_plus_channel),
                               x_pos_v, "x_plus"); !r)
        return r;
    if (diff) {
        if (auto r = write_voltage(static_cast<std::uint8_t>(w.x_minus_channel),
                                   x_neg_v, "x_minus"); !r)
            return r;
    }
    if (auto r = write_voltage(static_cast<std::uint8_t>(w.y_plus_channel),
                               y_pos_v, "y_plus"); !r)
        return r;
    if (diff) {
        if (auto r = write_voltage(static_cast<std::uint8_t>(w.y_minus_channel),
                                   y_neg_v, "y_minus"); !r)
            return r;
    }
    return {};
}

} // namespace rmcs_laser_guidance
