#pragma once

#include <cstdint>
#include <expected>
#include <memory>

#include "capture/capture_device.hpp"
#include "config.hpp"
#include "laser_guidance/error.hpp"
#include "runtime/control_loop_types.hpp"
#include "runtime/guidance_calibration.hpp"
#include "runtime/guidance_state_machine.hpp"

namespace rmcs_laser_guidance {

class AimSolver;
class Ft4222Spi;
class GalvoExecutor;
class ScanController;

} // namespace rmcs_laser_guidance

namespace rmcs_laser_guidance::runtime_internal {

class GuidanceSession {
public:
    enum class Mode : std::uint8_t {
        auto_guidance,
        manual_calibration,
    };

    GuidanceSession(GuidanceSession&&) noexcept;
    auto operator=(GuidanceSession&&) noexcept -> GuidanceSession&;
    ~GuidanceSession();

    GuidanceSession(const GuidanceSession&) = delete;
    auto operator=(const GuidanceSession&) -> GuidanceSession& = delete;

    static auto create_auto(const Config& config, const CaptureFormat& format)
        -> std::expected<GuidanceSession, Error>;
    static auto create_manual(
        const Config& config, const CaptureFormat& format,
        std::shared_ptr<GuidanceCalibrationState> calibration_state)
        -> std::expected<GuidanceSession, Error>;

    [[nodiscard]] auto mode() const -> Mode { return mode_; }
    // Session existence implies readiness — factory methods return
    // std::expected<GuidanceSession, Error> so callers check
    // guidance_.has_value() instead of calling enabled()/ready().
    [[nodiscard]] auto calibration_state() const -> const GuidanceCalibrationState*;
    auto mutable_calibration_state() -> GuidanceCalibrationState*;
    auto execute(const TargetTrack& track) -> GuidanceFrameResult;
    auto set_offset(float x_deg, float y_deg) -> void;
    auto shutdown() -> void;

private:
    GuidanceSession(
        GuidanceConfig config, Mode mode,
        std::shared_ptr<GuidanceCalibrationState> calibration_state,
        std::unique_ptr<Ft4222Spi> spi, std::unique_ptr<AimSolver> solver,
        std::unique_ptr<GalvoExecutor> executor, std::unique_ptr<ScanController> scanner);

    GuidanceConfig config_{};
    Mode mode_ = Mode::auto_guidance;
    std::shared_ptr<GuidanceCalibrationState> calibration_state_{};
    std::unique_ptr<Ft4222Spi> spi_{};
    std::unique_ptr<AimSolver> solver_{};
    std::unique_ptr<GalvoExecutor> executor_{};
    std::unique_ptr<ScanController> scanner_{};
    GuidanceStateMachine state_machine_{};
    bool ekf_was_lost_ = false;
};

// Shared by ControlLoop and GuidanceOpsApp: picks create_manual() when a
// calibration state is supplied (tool_guidance's calib mode), otherwise
// create_auto(). ControlLoop always passes nullptr.
inline auto try_create_guidance_session(
    const Config& config, const CaptureFormat& format,
    const std::shared_ptr<GuidanceCalibrationState>& calibration_state)
    -> std::expected<GuidanceSession, Error> {
    if (calibration_state) {
        return GuidanceSession::create_manual(config, format, calibration_state);
    }
    return GuidanceSession::create_auto(config, format);
}

} // namespace rmcs_laser_guidance::runtime_internal
