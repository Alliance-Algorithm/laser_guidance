#pragma once

namespace rmcs_laser_guidance::runtime_internal {

struct GuidanceCalibrationState {
    float angle_x_deg = 0.0F;
    float angle_y_deg = 0.0F;
    float angle_step_deg = 0.01F;
    float voltage_x = 0.0F;
    float voltage_y = 0.0F;
    float voltage_step_v = 0.005F;

    static constexpr float kMinAngleStepDeg = 0.001F;
    static constexpr float kMaxAngleStepDeg = 0.5F;
    static constexpr float kMinVoltageStepV = 0.001F;
    static constexpr float kMaxVoltageStepV = 0.5F;
};

} // namespace rmcs_laser_guidance::runtime_internal
