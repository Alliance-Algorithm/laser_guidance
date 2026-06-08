#include <print>
#include <stdexcept>

#include "runtime/guidance_tool_runtime.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::GuidanceCommandModelKind;
        using rmcs_laser_guidance::GuidanceConfig;
        using rmcs_laser_guidance::HitState;
        using rmcs_laser_guidance::ModelCandidate;
        using rmcs_laser_guidance::TargetObservation;
        using rmcs_laser_guidance::runtime_internal::GuidanceCalibrationState;
        using rmcs_laser_guidance::runtime_internal::GuidanceToolRuntime;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_contains;
        using rmcs_laser_guidance::tests::require_near;

        {
            GuidanceCalibrationState state;
            GuidanceConfig config;
            config.command_model = GuidanceCommandModelKind::geometry;
            GuidanceToolRuntime::apply_calibration_key(state, config, 'w');
            require_near(state.angle_y_deg, -0.01F, 1e-6F, "geometry W should move up");
            GuidanceToolRuntime::apply_calibration_key(state, config, 'd');
            require_near(state.angle_x_deg, 0.01F, 1e-6F, "geometry D should move right");
            auto message = GuidanceToolRuntime::apply_calibration_key(state, config, '<');
            require(message.has_value(), "geometry step shrink should report message");
            require_near(state.angle_step_deg, 0.005F, 1e-6F, "geometry step shrink mismatch");
            message = GuidanceToolRuntime::apply_calibration_key(state, config, '>');
            require(message.has_value(), "geometry step expand should report message");
            require_near(state.angle_step_deg, 0.01F, 1e-6F, "geometry step expand mismatch");
        }

        {
            GuidanceCalibrationState state;
            GuidanceConfig config;
            config.command_model = GuidanceCommandModelKind::direct_voltage;
            config.voltage_limit_v = 0.02F;
            GuidanceToolRuntime::apply_calibration_key(state, config, 'a');
            GuidanceToolRuntime::apply_calibration_key(state, config, 'a');
            GuidanceToolRuntime::apply_calibration_key(state, config, 'a');
            require_near(state.voltage_x, -0.015F, 1e-6F, "voltage A step mismatch");
            GuidanceToolRuntime::apply_calibration_key(state, config, 'a');
            GuidanceToolRuntime::apply_calibration_key(state, config, 'a');
            require_near(state.voltage_x, -0.02F, 1e-6F, "voltage clamp mismatch");
            auto message = GuidanceToolRuntime::apply_calibration_key(state, config, '>');
            require(message.has_value(), "voltage step grow should report message");
            require_near(state.voltage_step_v, 0.01F, 1e-6F, "voltage step growth mismatch");
        }

        {
            ModelCandidate candidate{
                .score = 0.875F,
                .class_id = 2,
                .bbox = {10.0F, 20.0F, 30.0F, 40.0F},
                .center = {25.0F, 40.0F},
            };
            const auto line = GuidanceToolRuntime::format_voltage_record(
                rmcs_laser_guidance::Clock::time_point(std::chrono::nanoseconds(1234)), candidate,
                1.25F, -0.75F);
            require_contains(line, "1234,", "voltage csv timestamp");
            require_contains(line, ",25.000,40.000,", "voltage csv center");
            require_contains(line, ",1200.000,", "voltage csv area");
            require_contains(line, ",0.87500,2,1.25000,-0.75000\n", "voltage csv tail");
        }

        {
            const auto line = GuidanceToolRuntime::format_geometry_record(
                1.25F, -0.5F, cv::Point3f{100.0F, 200.0F, 300.0F});
            require(line == "1.250,-0.500,100.000,200.000,300.000\n", "geometry csv mismatch");
        }

        {
            TargetObservation observation;
            observation.detected = true;
            observation.candidates.push_back(ModelCandidate{
                .score = 0.8F,
                .class_id = 0,
                .bbox = {0.0F, 0.0F, 10.0F, 10.0F},
                .center = {5.0F, 5.0F},
            });
            require(
                GuidanceToolRuntime::should_record_hit_edge(
                    HitState::Candidate, HitState::Confirmed, observation),
                "first confirm edge should record");
            require(
                !GuidanceToolRuntime::should_record_hit_edge(
                    HitState::Confirmed, HitState::Confirmed, observation),
                "steady confirmed state should not duplicate records");
            observation.candidates.front().class_id = 1;
            require(
                !GuidanceToolRuntime::should_record_hit_edge(
                    HitState::Candidate, HitState::Confirmed, observation),
                "non-purple should not record hit edge");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "guidance_tool_runtime_test failed: {}", e.what());
        return 1;
    }
}
