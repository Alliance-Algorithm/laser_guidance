#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class UdpSender {
public:
    explicit UdpSender(UdpConfig config);
    ~UdpSender();

    UdpSender(const UdpSender&) = delete;
    auto operator=(const UdpSender&) -> UdpSender& = delete;

    auto send(const TargetObservation& observation) -> void;

private:
    UdpConfig config_;
    int sock_ = -1;
    std::uint8_t seq_ = 0;
};

}
