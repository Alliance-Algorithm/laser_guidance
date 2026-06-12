#pragma once

#include <expected>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "capture/v4l2_capture.hpp"
#include "config.hpp"
#include "types.hpp"

#ifdef WITH_HIKCAMERA
#include <hikcamera/capturer.hpp>
#endif

namespace rmcs_laser_guidance {

struct CaptureFormat {
    CaptureBackendKind backend = CaptureBackendKind::v4l2;
    std::string device_id{};
    int width = 0;
    int height = 0;
    double framerate = 0.0;
    std::string format_name{};
    std::string device_path{};
    std::string pixel_encoding{};
};

auto to_capture_format(const V4l2NegotiatedFormat& format) -> CaptureFormat;

#ifdef WITH_HIKCAMERA
auto to_capture_format(
    const hikcamera::DeviceInfo& device_info, const hikcamera::StreamFormat& format)
    -> CaptureFormat;
#endif

class CaptureDevice {
public:
    explicit CaptureDevice(Config config);

    auto open() -> std::expected<CaptureFormat, std::string>;
    auto read_frame() -> std::expected<Frame, std::string>;
    auto close() noexcept -> void;
    [[nodiscard]] auto is_open() const noexcept -> bool;
    [[nodiscard]] auto negotiated_format() const noexcept -> const std::optional<CaptureFormat>&;

private:
    struct V4l2BackendState {
        explicit V4l2BackendState(V4l2Config config_in)
            : capture(std::move(config_in)) {}

        V4l2Capture capture;
    };

#ifdef WITH_HIKCAMERA
    struct HikBackendState {
        hikcamera::Camera camera{};
    };
#endif

    using BackendState = std::variant<
        std::monostate,
        V4l2BackendState
#ifdef WITH_HIKCAMERA
        ,
        HikBackendState
#endif
        >;

    [[nodiscard]] auto make_backend_state() const -> BackendState;

    Config config_{};
    BackendState backend_{};
    std::optional<CaptureFormat> negotiated_{};
};

} // namespace rmcs_laser_guidance
