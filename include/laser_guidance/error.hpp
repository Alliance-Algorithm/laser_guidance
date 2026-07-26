#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace rmcs_laser_guidance {

enum class ErrorKind : std::uint8_t {
    config,
    device,
    unavailable,
    timeout,
    internal,
};

struct Error {
    ErrorKind kind = ErrorKind::internal;
    std::string message{};
};

inline auto make_error(ErrorKind kind, std::string message) -> Error {
    return Error{.kind = kind, .message = std::move(message)};
}

inline auto error_kind_name(ErrorKind kind) -> std::string_view {
    switch (kind) {
    case ErrorKind::config: return "config";
    case ErrorKind::device: return "device";
    case ErrorKind::unavailable: return "unavailable";
    case ErrorKind::timeout: return "timeout";
    case ErrorKind::internal: return "internal";
    }
    return "internal";
}

inline auto format_error(const Error& error) -> std::string {
    return std::string("[") + std::string(error_kind_name(error.kind)) + "] " + error.message;
}

} // namespace rmcs_laser_guidance
