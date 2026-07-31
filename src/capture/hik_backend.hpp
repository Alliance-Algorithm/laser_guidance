#pragma once

#include <expected>

#include <hikcamera/capturer.hpp>

#include "capture/capture_backend.hpp"
#include "config.hpp"
#include "laser_guidance/error.hpp"

namespace rmcs_laser_guidance {

struct CaptureFormat;

auto select_startup_profile(
    const HikCameraConfig& config, HikProfileKind kind) -> std::expected<HikRuntimeProfile, Error>;

auto to_capture_format(
    const hikcamera::DeviceInfo& device_info, const hikcamera::StreamFormat& format)
    -> CaptureFormat;

struct HikBackend : public CaptureBackend {
    explicit HikBackend(HikCameraConfig config_in)
        : config(std::move(config_in)) {}

    auto open() -> std::expected<CaptureFormat, Error> override;
    auto read_frame() -> std::expected<Frame, Error> override;
    auto close() noexcept -> void override;
    [[nodiscard]] auto is_open() const noexcept -> bool override;
    auto reconnect() -> std::expected<CaptureFormat, Error> override;
    auto apply_runtime_profile(const HikRuntimeProfile& profile)
        -> std::expected<void, Error> override;

    hikcamera::Camera camera{};
    HikCameraConfig config{};
};

} // namespace rmcs_laser_guidance
