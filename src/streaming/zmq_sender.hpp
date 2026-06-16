#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class ZmqSender {
public:
    explicit ZmqSender(ZmqConfig config);
    ~ZmqSender();

    ZmqSender(const ZmqSender&) = delete;
    auto operator=(const ZmqSender&) -> ZmqSender& = delete;

    auto send(const TargetObservation& observation) -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rmcs_laser_guidance
