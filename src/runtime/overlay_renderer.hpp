#pragma once

#include "laser_guidance/runtime.hpp"
#include "runtime/control_loop_types.hpp"
#include "runtime/guidance_calibration.hpp"
#include "tracking/hit_progress.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct OverlayRenderContext {
    bool guidance_enabled = false;
    bool guidance_ready = false;
    bool calibration_mode = false;
    GuidanceCommandModelKind command_model = GuidanceCommandModelKind::geometry;
    const GuidanceCalibrationState* calibration_state = nullptr;
    const HitProgress* hit_progress = nullptr;
    bool streaming_active = false;
    bool recording_active = false;
    EnemyColor enemy_color = EnemyColor::auto_select;
    bool using_tensorrt = false;
    const RefereeSnapshot* referee = nullptr;
};

class OverlayRenderer {
public:
    auto render(ControlLoopFrame& frame, const OverlayRenderContext& context) const -> void;
};

} // namespace rmcs_laser_guidance::runtime_internal
