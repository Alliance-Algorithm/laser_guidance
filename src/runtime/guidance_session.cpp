#include "runtime/guidance_session.hpp"

#include <utility>

#include "guidance/aim_solver.hpp"
#include "guidance/galvo_executor.hpp"
#include "guidance/scan_controller.hpp"
#include "io/ft4222_spi.hpp"

namespace rmcs_laser_guidance::runtime_internal {
namespace {

auto open_ft4222() -> std::expected<Ft4222Spi, std::string> {
    return Ft4222Spi::open(
        Ft4222Config{
            .sys_clock = Ft4222SysClock::k60MHz,
            .clock_div = Ft4222SpiDiv::kDiv2,
            .cpol = Ft4222Cpol::kIdleLow,
            .cpha = Ft4222Cpha::kTrailing,
            .cs_active = Ft4222CsActive::kLow,
            .cs_channel = 0,
        });
}

auto make_solver(const Config& config, const CaptureFormat& format)
    -> std::expected<std::unique_ptr<AimSolver>, std::string> {
    auto solver = std::make_unique<AimSolver>(config, format.width, format.height);
    if (!solver->is_initialized()) {
        return std::unexpected(solver->initialization_error());
    }
    return solver;
}

auto make_executor(const Config& config, Ft4222Spi& spi)
    -> std::expected<std::unique_ptr<GalvoExecutor>, std::string> {
    auto executor = std::make_unique<GalvoExecutor>(config, spi);
    if (!executor->is_initialized()) {
        return std::unexpected(executor->initialization_error());
    }
    return executor;
}

auto make_scan_controller(
    const GuidanceConfig& config, GalvoExecutor& executor) -> std::unique_ptr<ScanController> {
    if (config.scan_mode != ScanMode::rectangle) {
        return nullptr;
    }
    return std::make_unique<ScanController>(config, executor);
}

} // namespace

GuidanceSession::GuidanceSession(
    GuidanceConfig config, const Mode mode,
    std::shared_ptr<GuidanceCalibrationState> calibration_state, std::unique_ptr<Ft4222Spi> spi,
    std::unique_ptr<AimSolver> solver, std::unique_ptr<GalvoExecutor> executor,
    std::unique_ptr<ScanController> scanner)
    : config_(std::move(config))
    , mode_(mode)
    , calibration_state_(std::move(calibration_state))
    , spi_(std::move(spi))
    , solver_(std::move(solver))
    , executor_(std::move(executor))
    , scanner_(std::move(scanner)) {}

GuidanceSession::GuidanceSession(GuidanceSession&&) noexcept = default;
auto GuidanceSession::operator=(GuidanceSession&&) noexcept -> GuidanceSession& = default;
GuidanceSession::~GuidanceSession() = default;

auto GuidanceSession::create_auto(const Config& config, const CaptureFormat& format)
    -> std::expected<GuidanceSession, std::string> {
    auto spi = open_ft4222();
    if (!spi) {
        return std::unexpected("FT4222 open failed: " + spi.error());
    }

    auto spi_ptr = std::make_unique<Ft4222Spi>(std::move(*spi));
    auto solver = make_solver(config, format);
    if (!solver) {
        return std::unexpected(solver.error());
    }

    auto executor = make_executor(config, *spi_ptr);
    if (!executor) {
        return std::unexpected(executor.error());
    }

    auto executor_ptr = std::move(*executor);
    auto scanner = make_scan_controller(config.guidance, *executor_ptr);
    return GuidanceSession(
        config.guidance, Mode::auto_guidance, nullptr, std::move(spi_ptr), std::move(*solver),
        std::move(executor_ptr), std::move(scanner));
}

auto GuidanceSession::create_manual(
    const Config& config, const CaptureFormat& format,
    std::shared_ptr<GuidanceCalibrationState> calibration_state)
    -> std::expected<GuidanceSession, std::string> {
    auto spi = open_ft4222();
    if (!spi) {
        return std::unexpected("FT4222 open failed: " + spi.error());
    }

    auto spi_ptr = std::make_unique<Ft4222Spi>(std::move(*spi));
    auto solver = make_solver(config, format);
    if (!solver) {
        return std::unexpected(solver.error());
    }

    auto executor = make_executor(config, *spi_ptr);
    if (!executor) {
        return std::unexpected(executor.error());
    }

    auto executor_ptr = std::move(*executor);
    return GuidanceSession(
        config.guidance, Mode::manual_calibration, std::move(calibration_state),
        std::move(spi_ptr), std::move(*solver), std::move(executor_ptr), nullptr);
}

auto GuidanceSession::calibration_state() const -> const GuidanceCalibrationState* {
    return calibration_state_.get();
}

auto GuidanceSession::mutable_calibration_state() -> GuidanceCalibrationState* {
    return calibration_state_.get();
}

auto GuidanceSession::execute(const TargetTrack& track) -> GuidanceFrameResult {
    if (mode_ == Mode::manual_calibration) {
        GuidanceFrameResult result;
        if (config_.command_model == GuidanceCommandModelKind::direct_voltage) {
            result.aim_output.message = executor_->process_calib_voltage(
                calibration_state_->voltage_x, calibration_state_->voltage_y);
        } else {
            result.aim_output.message = executor_->process_calib_angle(
                calibration_state_->angle_x_deg, calibration_state_->angle_y_deg);
        }

        result.aim_output.output_angles = executor_->latest_output_angles();
        result.aim_output.output_voltages = executor_->latest_output_voltages();

        const auto* selected =
            track.selected_detection.has_value() ? &*track.selected_detection : nullptr;
        if (selected == nullptr || selected->bbox.width <= 0.0F) {
            return result;
        }

        const auto telemetry = solver_->observe_target(selected, track.dt_seconds);
        result.telemetry = GuidanceTelemetry{
            .measured_depth_mm = telemetry.measured_depth_mm,
            .active_depth_mm = telemetry.active_depth_mm,
            .selected_target_point = telemetry.selected_target_point,
            .used_cached_depth = telemetry.used_cached_depth,
        };
        if (result.telemetry.active_depth_mm.has_value()) {
            result.aim_output.depth_valid = true;
            result.aim_output.depth_mm = *result.telemetry.active_depth_mm;
        }
        return result;
    }

    const auto decision = state_machine_.decide(
        track, ekf_was_lost_, solver_->cached_depth_mm().value_or(0.0F));
    GuidanceFrameResult result;

    switch (decision.action) {
    case GuidanceAction::idle:
        ekf_was_lost_ = track.ekf_enabled && track.lost;
        return result;
    case GuidanceAction::recenter:
        result.aim_output.recentered = true;
        result.aim_output.message = executor_->set_center();
        solver_->reset_depth_cache();
        if (scanner_) {
            scanner_->deactivate();
        }
        ekf_was_lost_ = track.ekf_enabled && track.lost;
        return result;
    case GuidanceAction::solve: break;
    }

    auto solve_result = solver_->solve(AimInput{
        .ekf_enabled = track.ekf_enabled,
        .track = track,
    });
    result.aim_output = std::move(solve_result.aim_output);
    result.telemetry = GuidanceTelemetry{
        .measured_depth_mm = solve_result.telemetry.measured_depth_mm,
        .active_depth_mm = solve_result.telemetry.active_depth_mm,
        .selected_target_point = solve_result.telemetry.selected_target_point,
        .used_cached_depth = solve_result.telemetry.used_cached_depth,
    };

    if (result.aim_output.command_issued) {
        if (scanner_ && scanner_->enabled()) {
            if (result.aim_output.output_angles.has_value()) {
                scanner_->update_angles_center(*result.aim_output.output_angles);
            } else if (result.aim_output.output_voltages.has_value()) {
                scanner_->update_voltage_center(*result.aim_output.output_voltages);
            }
        } else if (const auto message = executor_->apply(result.aim_output); !message.empty()) {
            result.aim_output.message = message;
        }
    }

    result.aim_output.output_angles = executor_->latest_output_angles();
    result.aim_output.output_voltages = executor_->latest_output_voltages();
    ekf_was_lost_ = track.ekf_enabled && track.lost;
    return result;
}

auto GuidanceSession::shutdown() -> void {
    if (scanner_) {
        scanner_->stop();
    }
    (void)executor_->set_center();
}

} // namespace rmcs_laser_guidance::runtime_internal
