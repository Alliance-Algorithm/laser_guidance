#pragma once

#include <cstdint>
#include <optional>

namespace laser_shim {

// USB 控制请求的 recipient（bmRequestType 低 2 位）：
//   0 = 设备, 1 = 接口, 2 = 端点, 3 = other
inline constexpr std::uint8_t kInterfaceRecipient = 0x01;

// 判断控制请求是否以接口为目标；若是，返回接口号（wIndex 低字节）。
// 用于 libusb claim shim：MVS SDK 的 U3V 传输层对接口做控制传输前不 claim
// 接口，内核 usbfs 会告警并伴随设备 reset。
inline auto control_target_interface(const std::uint8_t bm_request_type, const std::uint16_t w_index)
    -> std::optional<int> {
    if ((bm_request_type & 0x03U) != kInterfaceRecipient)
        return std::nullopt;
    return static_cast<int>(w_index & 0xFFU);
}

} // namespace laser_shim
