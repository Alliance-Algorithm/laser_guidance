#include "capture/capture_device.hpp"

#include <type_traits>
#include <utility>

namespace rmcs_laser_guidance {
namespace {

#ifdef WITH_HIKCAMERA
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
#endif

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

#ifdef WITH_HIKCAMERA
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
#endif

CaptureDevice::CaptureDevice(Config config)
    : config_(std::move(config))
    , backend_(make_backend_state()) {}

auto CaptureDevice::open() -> std::expected<CaptureFormat, std::string> {
    negotiated_.reset();

    if (std::holds_alternative<std::monostate>(backend_)) {
        return std::unexpected("Hik camera support is not compiled in");
    }

    return std::visit(
        [this](auto& backend) -> std::expected<CaptureFormat, std::string> {
            using Backend = std::decay_t<decltype(backend)>;
            if constexpr (std::is_same_v<Backend, std::monostate>) {
                return std::unexpected("capture backend is unavailable");
            } else if constexpr (std::is_same_v<Backend, V4l2BackendState>) {
                auto format = backend.capture.open();
                if (!format) {
                    return std::unexpected(format.error());
                }
                negotiated_ = to_capture_format(*format);
                return *negotiated_;
            }
#ifdef WITH_HIKCAMERA
            else if constexpr (std::is_same_v<Backend, HikBackendState>) {
                backend.camera.configure(to_hik_config(config_.hik));
                auto connected = backend.camera.connect();
                if (!connected) {
                    return std::unexpected(connected.error());
                }
                if (!backend.camera.device_info().has_value()) {
                    return std::unexpected(
                        "Hik camera connected but device info is unavailable");
                }
                if (!backend.camera.stream_format().has_value()) {
                    return std::unexpected(
                        "Hik camera connected but stream format is unavailable");
                }
                negotiated_ = to_capture_format(
                    *backend.camera.device_info(), *backend.camera.stream_format());
                return *negotiated_;
            }
#endif
        },
        backend_);
}

auto CaptureDevice::read_frame() -> std::expected<Frame, std::string> {
    return std::visit(
        [](auto& backend) -> std::expected<Frame, std::string> {
            using Backend = std::decay_t<decltype(backend)>;
            if constexpr (std::is_same_v<Backend, std::monostate>) {
                return std::unexpected("capture backend is unavailable");
            } else if constexpr (std::is_same_v<Backend, V4l2BackendState>) {
                return backend.capture.read_frame();
            }
#ifdef WITH_HIKCAMERA
            else if constexpr (std::is_same_v<Backend, HikBackendState>) {
                auto image = backend.camera.read_image_with_timestamp();
                if (!image) {
                    return std::unexpected(image.error());
                }
                return Frame{
                    .image = std::move(image->mat),
                    .timestamp = image->timestamp,
                };
            }
#endif
        },
        backend_);
}

auto CaptureDevice::close() noexcept -> void {
    std::visit(
        [](auto& backend) -> void {
            using Backend = std::decay_t<decltype(backend)>;
            if constexpr (std::is_same_v<Backend, V4l2BackendState>) {
                backend.capture.close();
            }
#ifdef WITH_HIKCAMERA
            else if constexpr (std::is_same_v<Backend, HikBackendState>) {
                (void)backend.camera.disconnect();
            }
#endif
        },
        backend_);
    negotiated_.reset();
}

auto CaptureDevice::is_open() const noexcept -> bool {
    return std::visit(
        [](const auto& backend) -> bool {
            using Backend = std::decay_t<decltype(backend)>;
            if constexpr (std::is_same_v<Backend, std::monostate>) {
                return false;
            } else if constexpr (std::is_same_v<Backend, V4l2BackendState>) {
                return backend.capture.is_open();
            }
#ifdef WITH_HIKCAMERA
            else if constexpr (std::is_same_v<Backend, HikBackendState>) {
                return backend.camera.connected();
            }
#endif
        },
        backend_);
}

auto CaptureDevice::negotiated_format() const noexcept -> const std::optional<CaptureFormat>& {
    return negotiated_;
}

auto CaptureDevice::make_backend_state() const -> BackendState {
    switch (config_.capture_backend) {
    case CaptureBackendKind::v4l2:
        return BackendState{std::in_place_type<V4l2BackendState>, config_.v4l2};
    case CaptureBackendKind::hikcamera:
#ifdef WITH_HIKCAMERA
        return BackendState{std::in_place_type<HikBackendState>};
#else
        return std::monostate{};
#endif
    }
    return std::monostate{};
}

} // namespace rmcs_laser_guidance
