#pragma once

#include <optional>
#include <string>

#include "capture/capture_device.hpp"

#ifdef WITH_HIKCAMERA
#include <hikcamera/capturer.hpp>
#endif

namespace rmcs_laser_guidance {

struct HikCameraCaptureConfig {
    std::string device_id{};
    unsigned int timeout_ms = 2000;
    float exposure_us = 2000.0F;
    float framerate = 80.0F;
    float gain = 16.9807F;
    bool invert_image = false;
    bool software_sync = false;
    bool trigger_mode = false;
    bool fixed_framerate = true;
};

#ifdef WITH_HIKCAMERA
auto to_capture_format(
    const hikcamera::DeviceInfo& device_info, const hikcamera::StreamFormat& format)
    -> CaptureFormat;
#endif

class HikCameraCaptureDevice final : public ICaptureDevice {
public:
    explicit HikCameraCaptureDevice(HikCameraCaptureConfig config);

    auto open() -> std::expected<CaptureFormat, std::string> override;
    auto read_frame() -> std::expected<Frame, std::string> override;
    auto close() noexcept -> void override;
    [[nodiscard]] auto is_open() const noexcept -> bool override;
    [[nodiscard]] auto negotiated_format() const noexcept -> const std::optional<CaptureFormat>& override;

private:
    HikCameraCaptureConfig config_;
    std::optional<CaptureFormat> negotiated_{};
#ifdef WITH_HIKCAMERA
    hikcamera::Camera camera_{};
#endif
};

} // namespace rmcs_laser_guidance
