#pragma once

#include <expected>
#include <optional>
#include <string>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

struct CaptureFormat {
    CaptureBackendKind backend = CaptureBackendKind::v4l2;
    std::string device_id{};
    int width = 0;
    int height = 0;
    double framerate = 0.0;
    std::string format_name{};

    // 录制/推流链路所需，替代原 V4l2NegotiatedFormat 字段
    std::string device_path{};       // 设备路径或标识（V4L2: /dev/video0; HikCamera: device_id）
    std::string pixel_encoding{};    // 像素编码（V4L2: fourcc 如 "MJPG"; HikCamera: "BGR8"）
};

class ICaptureDevice {
public:
    virtual ~ICaptureDevice() = default;

    virtual auto open() -> std::expected<CaptureFormat, std::string> = 0;
    virtual auto read_frame() -> std::expected<Frame, std::string> = 0;
    virtual auto close() noexcept -> void = 0;
    [[nodiscard]] virtual auto is_open() const noexcept -> bool = 0;
    [[nodiscard]] virtual auto negotiated_format() const noexcept
        -> const std::optional<CaptureFormat>& = 0;
};

} // namespace rmcs_laser_guidance
