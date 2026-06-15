#pragma once

#include <expected>
#include <string>

#include "types.hpp"

namespace rmcs_laser_guidance {

struct CaptureFormat;

class CaptureBackend {
public:
    virtual ~CaptureBackend() noexcept = default;
    virtual auto open() -> std::expected<CaptureFormat, std::string> = 0;
    virtual auto read_frame() -> std::expected<Frame, std::string> = 0;
    virtual auto close() noexcept -> void = 0;
    [[nodiscard]] virtual auto is_open() const noexcept -> bool = 0;
    virtual auto reconnect() -> std::expected<CaptureFormat, std::string> = 0;
};

} // namespace rmcs_laser_guidance
