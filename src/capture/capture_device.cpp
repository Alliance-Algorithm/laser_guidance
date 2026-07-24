#include "capture/capture_device.hpp"

#include <chrono>
#include <memory>
#include <print>
#include <thread>
#include <utility>

#include <hikcamera/parameters.hpp>

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

auto apply_hik_runtime_profile(hikcamera::Camera& camera, const HikRuntimeProfile& profile)
    -> std::expected<void, Error> {
    using hikcamera::auto_mode;

    if (auto ret = camera.parameter<hikcamera::param::exposure_auto>().set(auto_mode::off); !ret)
        return std::unexpected(make_error(ErrorKind::device, ret.error()));
    if (auto ret = camera.parameter<hikcamera::param::gain_auto>().set(auto_mode::off); !ret)
        return std::unexpected(make_error(ErrorKind::device, ret.error()));
    if (auto ret = camera.parameter<hikcamera::param::exposure_time_us>().set(profile.exposure_us);
        !ret)
        return std::unexpected(make_error(ErrorKind::device, ret.error()));
    if (auto ret = camera.parameter<hikcamera::param::gain>().set(profile.gain); !ret)
        return std::unexpected(make_error(ErrorKind::device, ret.error()));
    if (profile.framerate > 0.0F) {
        if (auto ret =
                camera.parameter<hikcamera::param::frame_rate_fps>().set(profile.framerate);
            !ret)
            return std::unexpected(make_error(ErrorKind::device, ret.error()));
    }
    if (profile.set_white_balance) {
        if (auto ret =
                camera.parameter<hikcamera::param::white_balance_auto>().set(auto_mode::off);
            !ret)
            return std::unexpected(make_error(ErrorKind::device, ret.error()));
        if (auto ret = camera.parameter<hikcamera::param::white_balance_ratio_red>().set(
                profile.white_balance_ratio_red);
            !ret)
            return std::unexpected(make_error(ErrorKind::device, ret.error()));
        if (auto ret = camera.parameter<hikcamera::param::white_balance_ratio_green>().set(
                profile.white_balance_ratio_green);
            !ret)
            return std::unexpected(make_error(ErrorKind::device, ret.error()));
        if (auto ret = camera.parameter<hikcamera::param::white_balance_ratio_blue>().set(
                profile.white_balance_ratio_blue);
            !ret)
            return std::unexpected(make_error(ErrorKind::device, ret.error()));
    } else {
        if (auto ret = camera.parameter<hikcamera::param::white_balance_auto>().set(
                auto_mode::continuous);
            !ret)
            return std::unexpected(make_error(ErrorKind::device, ret.error()));
    }
    return {};
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

// ---- HikBackend -----------------------------------------------------------------

auto HikBackend::open() -> std::expected<CaptureFormat, Error> {
    camera.configure(to_hik_config(config));
    auto connected = camera.connect();
    if (!connected) {
        return std::unexpected(make_error(ErrorKind::device, connected.error()));
    }
    if (!camera.device_info().has_value()) {
        return std::unexpected(
            make_error(ErrorKind::device, "Hik camera connected but device info is unavailable"));
    }
    if (!camera.stream_format().has_value()) {
        return std::unexpected(
            make_error(ErrorKind::device, "Hik camera connected but stream format is unavailable"));
    }
    if (auto applied = apply_hik_runtime_profile(camera, config.lit_profile()); !applied) {
        std::println(stderr, "Hik lit profile apply: {}", format_error(applied.error()));
    }
    return to_capture_format(*camera.device_info(), *camera.stream_format());
}

auto HikBackend::apply_runtime_profile(const HikRuntimeProfile& profile)
    -> std::expected<void, Error> {
    if (!camera.connected()) {
        return std::unexpected(
            make_error(ErrorKind::unavailable, "Hik camera is not connected"));
    }
    return apply_hik_runtime_profile(camera, profile);
}

auto HikBackend::read_frame() -> std::expected<Frame, Error> {
    auto image = camera.read_image_with_timestamp();
    if (!image) {
        return std::unexpected(make_error(ErrorKind::device, image.error()));
    }
    return Frame{
        .image = std::move(image->mat),
        .timestamp = image->timestamp,
    };
}

auto HikBackend::close() noexcept -> void { (void)camera.disconnect(); }

auto HikBackend::is_open() const noexcept -> bool { return camera.connected(); }

auto HikBackend::reconnect() -> std::expected<CaptureFormat, Error> {
    (void)camera.disconnect();
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
    try {
        return frame_queue_->pop();
    } catch (const std::exception& e) {
        return std::unexpected(make_error(ErrorKind::internal, e.what()));
    }
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
    capture_stop_.store(false, std::memory_order_relaxed);
    capture_thread_ = std::thread([this] { capture_loop(); });
}

auto CaptureDevice::stop_capture_thread() noexcept -> void {
    capture_stop_.store(true, std::memory_order_relaxed);
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

    while (!capture_stop_.load(std::memory_order_relaxed)) {
        std::expected<Frame, Error> result;
        {
            std::scoped_lock lock(backend_mutex_);
            result = backend_->read_frame();
        }
        if (capture_stop_.load(std::memory_order_relaxed)) {
            break;
        }
        const bool failed = !result.has_value();
        try {
            frame_queue_->push(std::move(result));
        } catch (const std::exception&) {
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
