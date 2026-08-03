#include "capture/hik_backend.hpp"

#include <print>
#include <utility>

#include <hikcamera/parameters.hpp>

#include "capture/capture_device.hpp"

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
    } else if (profile.white_balance_off) {
        if (auto ret = camera.parameter<hikcamera::param::white_balance_auto>().set(
                auto_mode::off);
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

auto select_startup_profile(
    const HikCameraConfig& config, const HikProfileKind kind) -> std::expected<HikRuntimeProfile, Error> {
    switch (kind) {
    case HikProfileKind::lit:
        return config.lit_profile();
    case HikProfileKind::unlit:
        if (!config.has_unlit_profile) {
            return std::unexpected(
                make_error(ErrorKind::config, "hik.profile_unlit is not configured"));
        }
        return config.unlit;
    }
    __builtin_unreachable();
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
        .pixel_encoding = format.pixel_format_name,
    };
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
    auto startup = select_startup_profile(config, config.startup_profile_kind);
    if (!startup) {
        std::println(stderr, "Hik startup profile: {}", format_error(startup.error()));
        return std::unexpected(startup.error());
    }
    if (auto applied = apply_hik_runtime_profile(camera, *startup); !applied) {
        std::println(stderr, "Hik startup profile apply: {}", format_error(applied.error()));
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

} // namespace rmcs_laser_guidance
