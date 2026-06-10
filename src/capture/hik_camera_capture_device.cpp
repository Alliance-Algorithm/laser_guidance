#include "capture/hik_camera_capture_device.hpp"

#include <utility>

namespace rmcs_laser_guidance {

#ifdef WITH_HIKCAMERA
namespace {

auto to_hik_config(const HikCameraCaptureConfig& config) -> hikcamera::Config {
    return hikcamera::Config{
        .device_id = config.device_id,
        .timeout_ms = config.timeout_ms,
        .exposure_us = config.exposure_us,
        .framerate = config.framerate,
        .gain = config.gain,
        .invert_image = config.invert_image,
        .software_sync = config.software_sync,
        .trigger_mode = config.trigger_mode,
        .fixed_framerate = config.fixed_framerate,
    };
}

} // namespace

auto to_capture_format(
    const hikcamera::DeviceInfo& device_info, const hikcamera::StreamFormat& format)
    -> CaptureFormat {
    return CaptureFormat{
        .backend = CaptureBackendKind::hikcamera,
        .device_id = device_info.device_id,
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .format_name = format.pixel_format_name,
    };
}
#endif

HikCameraCaptureDevice::HikCameraCaptureDevice(HikCameraCaptureConfig config)
    : config_(std::move(config)) {}

auto HikCameraCaptureDevice::open() -> std::expected<CaptureFormat, std::string> {
#ifdef WITH_HIKCAMERA
    camera_.configure(to_hik_config(config_));
    auto connected = camera_.connect();
    if (!connected)
        return std::unexpected(connected.error());

    if (!camera_.device_info().has_value())
        return std::unexpected("Hik camera connected but device info is unavailable");
    if (!camera_.stream_format().has_value())
        return std::unexpected("Hik camera connected but stream format is unavailable");

    negotiated_ = to_capture_format(*camera_.device_info(), *camera_.stream_format());
    return *negotiated_;
#else
    return std::unexpected("WITH_HIKCAMERA=OFF: Hik camera support is not compiled in");
#endif
}

auto HikCameraCaptureDevice::read_frame() -> std::expected<Frame, std::string> {
#ifdef WITH_HIKCAMERA
    auto image = camera_.read_image_with_timestamp();
    if (!image)
        return std::unexpected(image.error());

    return Frame{
        .image = std::move(image->mat),
        .timestamp = image->timestamp,
    };
#else
    return std::unexpected("WITH_HIKCAMERA=OFF: Hik camera support is not compiled in");
#endif
}

auto HikCameraCaptureDevice::close() noexcept -> void {
#ifdef WITH_HIKCAMERA
    (void)camera_.disconnect();
#endif
    negotiated_.reset();
}

auto HikCameraCaptureDevice::is_open() const noexcept -> bool {
#ifdef WITH_HIKCAMERA
    return camera_.connected();
#else
    return false;
#endif
}

auto HikCameraCaptureDevice::negotiated_format() const noexcept -> const std::optional<CaptureFormat>& {
    return negotiated_;
}

} // namespace rmcs_laser_guidance
