#include "capture/capture_device.hpp"

#include <memory>
#include <utility>

namespace rmcs_laser_guidance {
namespace {

auto to_hik_config(const HikCameraConfig& config) -> hikcamera::Config {
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

auto to_capture_format(const V4l2NegotiatedFormat& format) -> CaptureFormat {
    return CaptureFormat{
        .backend = CaptureBackendKind::v4l2,
        .device_id = format.device_path.string(),
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .format_name = format.fourcc,
        .device_path = format.device_path.string(),
        .pixel_encoding = format.fourcc,
    };
}

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
        .device_path = device_info.device_id,
        .pixel_encoding = "BGR8",
    };
}

// ---- V4l2Backend ----------------------------------------------------------------

auto V4l2Backend::open() -> std::expected<CaptureFormat, std::string> {
    auto format = capture.open();
    if (!format) {
        return std::unexpected(format.error());
    }
    return to_capture_format(*format);
}

auto V4l2Backend::read_frame() -> std::expected<Frame, std::string> {
    return capture.read_frame();
}

auto V4l2Backend::close() noexcept -> void { capture.close(); }

auto V4l2Backend::is_open() const noexcept -> bool { return capture.is_open(); }

auto V4l2Backend::reconnect() -> std::expected<CaptureFormat, std::string> {
    capture.close();
    return open();
}

// ---- HikBackend -----------------------------------------------------------------

auto HikBackend::open() -> std::expected<CaptureFormat, std::string> {
    camera.configure(to_hik_config(config));
    auto connected = camera.connect();
    if (!connected) {
        return std::unexpected(connected.error());
    }
    if (!camera.device_info().has_value()) {
        return std::unexpected("Hik camera connected but device info is unavailable");
    }
    if (!camera.stream_format().has_value()) {
        return std::unexpected("Hik camera connected but stream format is unavailable");
    }
    return to_capture_format(*camera.device_info(), *camera.stream_format());
}

auto HikBackend::read_frame() -> std::expected<Frame, std::string> {
    auto image = camera.read_image_with_timestamp();
    if (!image) {
        return std::unexpected(image.error());
    }
    return Frame{
        .image = std::move(image->mat),
        .timestamp = image->timestamp,
    };
}

auto HikBackend::close() noexcept -> void { (void)camera.disconnect(); }

auto HikBackend::is_open() const noexcept -> bool { return camera.connected(); }

auto HikBackend::reconnect() -> std::expected<CaptureFormat, std::string> {
    (void)camera.disconnect();
    return open();
}

// ---- CaptureDevice --------------------------------------------------------------

CaptureDevice::CaptureDevice(Config config)
    : config_(std::move(config))
    , backend_(make_backend()) {}

auto CaptureDevice::open() -> std::expected<CaptureFormat, std::string> {
    negotiated_.reset();

    if (!backend_) {
        return std::unexpected("capture backend is unavailable");
    }

    auto format = backend_->open();
    if (!format) {
        return std::unexpected(format.error());
    }
    negotiated_ = *format;
    return *negotiated_;
}

auto CaptureDevice::read_frame() -> std::expected<Frame, std::string> {
    if (!backend_) {
        return std::unexpected("capture backend is unavailable");
    }
    return backend_->read_frame();
}

auto CaptureDevice::close() noexcept -> void {
    if (backend_) {
        backend_->close();
    }
    negotiated_.reset();
}

auto CaptureDevice::is_open() const noexcept -> bool {
    return backend_ && backend_->is_open();
}

auto CaptureDevice::negotiated_format() const noexcept -> const std::optional<CaptureFormat>& {
    return negotiated_;
}

auto CaptureDevice::reconnect() -> std::expected<void, std::string> {
    if (!backend_) {
        return std::unexpected("capture backend is unavailable");
    }

    auto format = backend_->reconnect();
    if (!format) {
        return std::unexpected(format.error());
    }
    negotiated_ = *format;
    return {};
}

auto CaptureDevice::make_backend() const -> std::unique_ptr<CaptureBackend> {
    switch (config_.capture_backend) {
    case CaptureBackendKind::v4l2:
        return std::make_unique<V4l2Backend>(config_.v4l2);
    case CaptureBackendKind::hikcamera:
        return std::make_unique<HikBackend>(config_.hik);
    }
    return nullptr;
}

} // namespace rmcs_laser_guidance
