#include <cstdint>
#include <optional>
#include <print>

#include "io/libusb_claim_policy.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::tests::require;
        using laser_shim::control_target_interface;

        // 标准请求：目标=接口 0（wIndex 低字节）
        {
            const auto target = control_target_interface(0x01, 0x0000);
            require(target.has_value(), "standard interface-0 request targets interface");
            require(*target == 0, "interface number = wIndex low byte");
        }

        // 目标=接口 2（wIndex=2，即日志中的 interface 2；vendor 类型+接口 recipient）
        {
            const auto target = control_target_interface(0x41, 0x0002);
            require(target.has_value() && *target == 2, "vendor interface-2 request detected");
        }

        // 目标=设备：不 claim
        {
            require(!control_target_interface(0x00, 0x0000).has_value(),
                "device-recipient request needs no claim");
            require(!control_target_interface(0x80, 0x0000).has_value(),
                "device-recipient IN request needs no claim");
        }

        // 目标=端点：不 claim
        {
            require(!control_target_interface(0x82, 0x0000).has_value(),
                "endpoint-recipient request needs no claim");
        }

        // 目标=other：不 claim
        {
            require(!control_target_interface(0x03, 0x0000).has_value(),
                "other-recipient request needs no claim");
        }

        // wIndex 高字节不参与接口号提取
        {
            const auto target = control_target_interface(0x21, 0x0302);
            require(target.has_value() && *target == 2, "interface number ignores high byte");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "libusb_claim_shim_test failed: {}", e.what());
        return 1;
    }
}
