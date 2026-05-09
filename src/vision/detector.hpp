#pragma once

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class Detector {
public:
    explicit Detector(const Config& config);

    auto detect(const Frame& frame) const -> TargetObservation;
};

} // namespace rmcs_laser_guidance
