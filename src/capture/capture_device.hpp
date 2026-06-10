#pragma once

#include <expected>
#include <optional>
#include <string>

#include "types.hpp"

namespace rmcs_laser_guidance {

enum class CaptureBackendKind {
    v4l2,
    hikcamera,
};

struct CaptureFormat {
    CaptureBackendKind backend = CaptureBackendKind::v4l2;
    std::string device_id{};
    int width = 0;
    int height = 0;
    double framerate = 0.0;
    std::string format_name{};
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
