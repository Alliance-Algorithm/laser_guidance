#include "capture/v4l2_capture_device.hpp"

#include <utility>

namespace rmcs_laser_guidance {

auto to_capture_format(const V4l2NegotiatedFormat& format) -> CaptureFormat {
    return CaptureFormat{
        .backend = CaptureBackendKind::v4l2,
        .device_id = format.device_path.string(),
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .format_name = format.fourcc,
    };
}

V4l2CaptureDevice::V4l2CaptureDevice(V4l2Config config)
    : capture_(std::move(config)) {}

auto V4l2CaptureDevice::open() -> std::expected<CaptureFormat, std::string> {
    auto format = capture_.open();
    if (!format)
        return std::unexpected(format.error());

    negotiated_ = to_capture_format(*format);
    return *negotiated_;
}

auto V4l2CaptureDevice::read_frame() -> std::expected<Frame, std::string> {
    return capture_.read_frame();
}

auto V4l2CaptureDevice::close() noexcept -> void {
    capture_.close();
    negotiated_.reset();
}

auto V4l2CaptureDevice::is_open() const noexcept -> bool { return capture_.is_open(); }

auto V4l2CaptureDevice::negotiated_format() const noexcept -> const std::optional<CaptureFormat>& {
    return negotiated_;
}

} // namespace rmcs_laser_guidance
