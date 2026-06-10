#pragma once

#include <optional>

#include "capture/capture_device.hpp"
#include "capture/v4l2_capture.hpp"

namespace rmcs_laser_guidance {

auto to_capture_format(const V4l2NegotiatedFormat& format) -> CaptureFormat;

class V4l2CaptureDevice final : public ICaptureDevice {
public:
    explicit V4l2CaptureDevice(V4l2Config config);

    auto open() -> std::expected<CaptureFormat, std::string> override;
    auto read_frame() -> std::expected<Frame, std::string> override;
    auto close() noexcept -> void override;
    [[nodiscard]] auto is_open() const noexcept -> bool override;
    [[nodiscard]] auto negotiated_format() const noexcept -> const std::optional<CaptureFormat>& override;

private:
    V4l2Capture capture_;
    std::optional<CaptureFormat> negotiated_{};
};

} // namespace rmcs_laser_guidance
