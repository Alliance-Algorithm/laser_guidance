#include "capture/capture_device.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include "capture/hik_backend.hpp"

namespace rmcs_laser_guidance {

struct V4l2Backend : public CaptureBackend {
    explicit V4l2Backend(V4l2Config config_in)
        : capture(std::move(config_in)) {}

    auto open() -> std::expected<CaptureFormat, Error> override;
    auto read_frame() -> std::expected<Frame, Error> override;
    auto close() noexcept -> void override;
    [[nodiscard]] auto is_open() const noexcept -> bool override;
    auto reconnect() -> std::expected<CaptureFormat, Error> override;

    V4l2Capture capture;
};

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

// ---- V4l2Backend ----------------------------------------------------------------

auto V4l2Backend::open() -> std::expected<CaptureFormat, Error> {
    auto format = capture.open();
    if (!format) {
        return std::unexpected(format.error());
    }
    return to_capture_format(*format);
}

auto V4l2Backend::read_frame() -> std::expected<Frame, Error> {
    return capture.read_frame();
}

auto V4l2Backend::close() noexcept -> void { capture.close(); }

auto V4l2Backend::is_open() const noexcept -> bool { return capture.is_open(); }

auto V4l2Backend::reconnect() -> std::expected<CaptureFormat, Error> {
    capture.close();
    return open();
}

// ---- CaptureDevice --------------------------------------------------------------

CaptureDevice::CaptureDevice(Config config)
    : config_(std::move(config))
    , backend_(make_backend()) {}

CaptureDevice::~CaptureDevice() noexcept { close(); }

auto CaptureDevice::open() -> std::expected<CaptureFormat, Error> {
    if (is_open() || capture_thread_.joinable()) {
        return std::unexpected(
            make_error(ErrorKind::internal, "capture device is already open"));
    }
    negotiated_.reset();

    if (!backend_) {
        return std::unexpected(
            make_error(ErrorKind::unavailable, "capture backend is unavailable"));
    }

    auto format = backend_->open();
    if (!format) {
        return std::unexpected(format.error());
    }
    negotiated_ = *format;
    start_capture_thread();
    return *negotiated_;
}

auto CaptureDevice::read_frame() -> std::expected<Frame, Error> {
    if (!frame_queue_) {
        return std::unexpected(
            make_error(ErrorKind::unavailable, "capture device is not open"));
    }
    auto item = frame_queue_->pop();
    if (!item.has_value()) {
        return std::unexpected(make_error(ErrorKind::unavailable, "capture stopped"));
    }
    return std::move(*item);
}

auto CaptureDevice::close() noexcept -> void {
    stop_capture_thread();
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

auto CaptureDevice::reconnect() -> std::expected<void, Error> {
    if (!backend_) {
        return std::unexpected(
            make_error(ErrorKind::unavailable, "capture backend is unavailable"));
    }

    stop_capture_thread();

    auto format = backend_->reconnect();
    if (!format) {
        return std::unexpected(format.error());
    }
    negotiated_ = *format;
    start_capture_thread();
    return {};
}

auto CaptureDevice::apply_runtime_profile(const HikRuntimeProfile& profile)
    -> std::expected<void, Error> {
    if (!backend_) {
        return std::unexpected(
            make_error(ErrorKind::unavailable, "capture backend is unavailable"));
    }
    std::scoped_lock lock(backend_mutex_);
    return backend_->apply_runtime_profile(profile);
}

auto CaptureDevice::start_capture_thread() -> void {
    frame_queue_ = std::make_unique<LatestValue<std::expected<Frame, Error>>>();
    capture_thread_ = std::jthread([this] { capture_loop(); });
}

auto CaptureDevice::stop_capture_thread() noexcept -> void {
    capture_thread_.request_stop();
    if (frame_queue_) {
        frame_queue_->shutdown();
    }
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    frame_queue_.reset();
}

auto CaptureDevice::capture_loop() -> void {
    // backend_->read_frame() can return an error immediately (e.g. camera not
    // connected) instead of blocking on hardware. Without a backoff, that turns
    // into a tight busy-loop hammering the driver. ControlLoop's own retry/backoff
    // now only paces "how often it drains the queue", not "how often the hardware
    // is polled", so the delay must live here.
    constexpr auto kErrorBackoff = std::chrono::milliseconds(100);
    auto stoken = capture_thread_.get_stop_token();

    while (!stoken.stop_requested()) {
        std::expected<Frame, Error> result;
        {
            std::scoped_lock lock(backend_mutex_);
            result = backend_->read_frame();
        }
        if (stoken.stop_requested()) {
            break;
        }
        const bool failed = !result.has_value();
        frame_queue_->push(std::move(result));
        if (frame_queue_->is_shutdown()) {
            break;
        }
        if (failed) {
            std::this_thread::sleep_for(kErrorBackoff);
        }
    }
}

auto CaptureDevice::make_backend() const -> std::unique_ptr<CaptureBackend> {
    switch (config_.capture_backend) {
    case CaptureBackendKind::v4l2:
        return std::make_unique<V4l2Backend>(config_.v4l2);
    case CaptureBackendKind::hikcamera:
        return std::make_unique<HikBackend>(config_.hik);
    }
    __builtin_unreachable();
}

} // namespace rmcs_laser_guidance
