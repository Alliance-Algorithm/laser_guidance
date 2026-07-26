#include "guidance/aim_solver.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "guidance/camera_projection.hpp"
#include "guidance/depth_estimator.hpp"
#include "guidance/depth_filter.hpp"
#include "guidance/galvo_kinematics.hpp"
#include "guidance/voltage_mapper.hpp"

namespace rmcs_laser_guidance {
namespace {

auto load_yaml_calibration(const std::filesystem::path& path) -> std::pair<cv::Mat, cv::Mat> {
    const auto yaml = YAML::LoadFile(path.string());
    const auto calib = yaml["calibration"];
    if (!calib) {
        throw std::runtime_error("YAML missing 'calibration' key");
    }

    cv::Mat camera = cv::Mat::eye(3, 3, CV_64F);
    const auto mat_node = calib["camera_matrix"];
    if (!mat_node || mat_node.size() != 3) {
        throw std::runtime_error("missing camera_matrix");
    }
    for (int row = 0; row < 3; ++row) {
        if (mat_node[row].size() != 3) {
            throw std::runtime_error("camera_matrix row must have 3 columns");
        }
        for (int col = 0; col < 3; ++col) {
            camera.at<double>(row, col) = mat_node[row][col].as<double>();
        }
    }

    cv::Mat dist;
    const auto dc_node = calib["dist_coeffs"];
    if (dc_node) {
        dist = cv::Mat::zeros(1, static_cast<int>(dc_node.size()), CV_64F);
        for (int idx = 0; idx < dc_node.size(); ++idx) {
            dist.at<double>(idx) = dc_node[idx].as<double>();
        }
    }

    return {camera, dist};
}

auto to_model_candidate(const Detection& detection) -> ModelCandidate {
    return ModelCandidate{
        .score = detection.score,
        .class_id = detection.class_id,
        .bbox = detection.bbox,
        .center = detection.center,
    };
}

} // namespace

AimSolver::AimSolver(const Config& config, const int image_width, const int image_height)
    : config_(config.guidance)
    , image_width_(static_cast<float>(image_width))
    , image_height_(static_cast<float>(image_height)) {
    if (!config_.enabled) {
        init_error_ = "guidance disabled";
        return;
    }

    if (config_.command_model == GuidanceCommandModelKind::direct_voltage) {
        try {
            voltage_mapper_ = std::make_unique<VoltageMapper>(config_);
            if (config_.scan_mode == ScanMode::rectangle) {
                init_error_ = load_geometry_calibration(config_.camera_calib_path);
                if (!init_error_.empty()) {
                    return;
                }
            }
        } catch (const std::exception& e) {
            init_error_ = std::string("voltage mapper init failed: ") + e.what();
            return;
        }
    } else {
        init_error_ = load_geometry_calibration(config_.camera_calib_path);
        if (!init_error_.empty()) {
            return;
        }
    }

    initialized_ = true;
}

AimSolver::~AimSolver() = default;

auto AimSolver::load_geometry_calibration(const std::filesystem::path& path) -> std::string {
    try {
        auto [camera, dist] = load_yaml_calibration(path);
        projection_ = std::make_unique<CameraProjection>(camera.clone(), dist.clone());
        depth_estimator_ = std::make_unique<DepthEstimator>(config_, camera.clone());
        if (config_.depth_filter_enabled) {
            depth_filter_ = std::make_unique<DepthFilter>(config_);
        }
        kinematics_ = std::make_unique<GalvoKinematics>(config_);
        return {};
    } catch (const std::exception& e) {
        return std::string("calibration load failed: ") + e.what();
    }
}

auto AimSolver::solve(const AimInput& input) -> AimSolveResult {
    if (!initialized_) {
        return AimSolveResult{
            .aim_output =
                AimOutput{
                    .message = init_error_.empty() ? "aim solver not initialized" : init_error_,
                },
        };
    }
    if (config_.calib_mode) {
        return AimSolveResult{.aim_output = AimOutput{.message = ""}};
    }
    if (input.track.aim_center.x < 0.0F || input.track.aim_center.y < 0.0F
        || input.track.aim_center.x >= std::max(1.0F, image_width_)
        || input.track.aim_center.y >= std::max(1.0F, image_height_)) {
        return AimSolveResult{.aim_output = AimOutput{.message = "invalid aim center"}};
    }

    if (config_.command_model == GuidanceCommandModelKind::direct_voltage) {
        return solve_direct_voltage(input);
    }
    return solve_geometry(input);
}

auto AimSolver::observe_target(const Detection* detection, const double dt_seconds)
    -> AimSolveTelemetry {
    AimSolveTelemetry telemetry;
    if (detection != nullptr) {
        if (const auto depth = estimate_depth(*detection)) {
            last_valid_depth_mm_ = *depth;
            telemetry.measured_depth_mm = depth;
        }
    }

    if (depth_filter_) {
        // See depth_filter.hpp for why raw measurements are filtered here.
        depth_filter_->predict(dt_seconds);
        if (telemetry.measured_depth_mm.has_value()) {
            depth_filter_->update(*telemetry.measured_depth_mm);
        }
        if (depth_filter_->is_initialized()) {
            telemetry.active_depth_mm = depth_filter_->state().depth_mm;
            telemetry.used_cached_depth = !telemetry.measured_depth_mm.has_value();
        }
    } else if (last_valid_depth_mm_ > 0.0F) {
        telemetry.active_depth_mm = last_valid_depth_mm_;
        telemetry.used_cached_depth = !telemetry.measured_depth_mm.has_value();
    }

    if (detection != nullptr && telemetry.active_depth_mm.has_value() && projection_) {
        telemetry.selected_target_point = projection_->project(detection->center, *telemetry.active_depth_mm);
    }

    return telemetry;
}

auto AimSolver::estimate_depth(const Detection& detection) const -> std::optional<float> {
    const auto model_candidate = to_model_candidate(detection);
    if (depth_estimator_) {
        return depth_estimator_->estimate(model_candidate);
    }
    return std::nullopt;
}

auto AimSolver::project_to_camera(const cv::Point2f& pixel, const float depth_mm) const
    -> cv::Point3f {
    if (!projection_) {
        return {-1.0F, -1.0F, -1.0F};
    }
    return projection_->project(pixel, depth_mm);
}

auto AimSolver::reset_depth_cache() noexcept -> void {
    last_valid_depth_mm_ = 0.0F;
    if (depth_filter_) {
        depth_filter_->reset();
    }
}

auto AimSolver::cached_depth_mm() const -> std::optional<float> {
    if (last_valid_depth_mm_ <= 0.0F) {
        return std::nullopt;
    }
    return last_valid_depth_mm_;
}

auto AimSolver::solve_geometry(const AimInput& input) -> AimSolveResult {
    if (!projection_ || !kinematics_) {
        return AimSolveResult{
            .aim_output = AimOutput{.message = "geometry guidance not initialized"},
        };
    }

    const auto* selected =
        input.track.selected_detection.has_value() ? &*input.track.selected_detection : nullptr;
    auto result = AimSolveResult{
        .telemetry = observe_target(selected, input.track.dt_seconds),
    };
    if (!result.telemetry.active_depth_mm.has_value()) {
        result.aim_output.message = "no valid depth";
        return result;
    }

    const auto P_c = projection_->project(input.track.aim_center, *result.telemetry.active_depth_mm);
    const auto angles = kinematics_->compute(P_c);
    if (!angles.valid) {
        result.aim_output.message = "kinematics failed";
        return result;
    }

    const cv::Point2f output_angles{
        angles.theta_x_optical_deg + config_.angle_offset_x_deg,
        angles.theta_y_optical_deg + config_.angle_offset_y_deg,
    };
    result.aim_output = AimOutput{
        .command_issued = true,
        .depth_valid = true,
        .depth_mm = *result.telemetry.active_depth_mm,
        .message = "",
        .output_angles = output_angles,
    };
    return result;
}

auto AimSolver::solve_direct_voltage(const AimInput& input) -> AimSolveResult {
    const auto* selected =
        input.track.selected_detection.has_value() ? &*input.track.selected_detection : nullptr;
    auto result = AimSolveResult{
        .telemetry = observe_target(selected, input.track.dt_seconds),
    };
    if (!voltage_mapper_) {
        result.aim_output.message = "direct voltage mapper not initialized";
        return result;
    }
    if (!input.track.selected_detection.has_value()) {
        result.aim_output.message = "no candidate for direct voltage";
        return result;
    }
    const auto& detection = *input.track.selected_detection;
    if (detection.bbox.width <= 0.0F || detection.bbox.height <= 0.0F) {
        result.aim_output.message = "invalid bbox for direct voltage";
        return result;
    }

    VoltageFeatures features;
    features.center_x = config_.voltage_use_ekf_center ? input.track.aim_center.x : detection.center.x;
    features.center_y = config_.voltage_use_ekf_center ? input.track.aim_center.y : detection.center.y;
    features.bbox_w = detection.bbox.width;
    features.bbox_h = detection.bbox.height;
    features.bbox_area = detection.bbox.width * detection.bbox.height;
    features.score = detection.score;
    features.class_id = detection.class_id;
    features.image_width = std::max(1.0F, image_width_);
    features.image_height = std::max(1.0F, image_height_);

    const auto command = voltage_mapper_->predict(features);
    if (!command || !command->valid) {
        result.aim_output.message = "direct voltage predict failed";
        return result;
    }

    result.aim_output = AimOutput{
        .command_issued = true,
        .depth_valid = result.telemetry.active_depth_mm.has_value(),
        .depth_mm = result.telemetry.active_depth_mm.value_or(0.0F),
        .message = "",
        .output_voltages = cv::Point2f{
            command->vx + config_.voltage_offset_vx,
            command->vy + config_.voltage_offset_vy,
        },
    };
    return result;
}

} // namespace rmcs_laser_guidance
