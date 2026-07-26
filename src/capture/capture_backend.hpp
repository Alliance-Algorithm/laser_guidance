#pragma once

#include <expected>
#include <string>

#include "config.hpp"
#include "laser_guidance/error.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

struct CaptureFormat;

class CaptureBackend {
public:
    virtual ~CaptureBackend() noexcept = default;
    CaptureBackend() = default;
    CaptureBackend(const CaptureBackend&) = delete;
    CaptureBackend& operator=(const CaptureBackend&) = delete;
    virtual auto open() -> std::expected<CaptureFormat, Error> = 0;
    virtual auto read_frame() -> std::expected<Frame, Error> = 0;
    virtual auto close() noexcept -> void = 0;
    [[nodiscard]] virtual auto is_open() const noexcept -> bool = 0;
    virtual auto reconnect() -> std::expected<CaptureFormat, Error> = 0;
    virtual auto apply_runtime_profile(const HikRuntimeProfile& /*profile*/)
        -> std::expected<void, Error> {
        return std::unexpected(
            make_error(ErrorKind::unavailable,
                       "runtime camera profile switch is not supported on this backend"));
    }
};

} // namespace rmcs_laser_guidance
