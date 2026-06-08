#include "guidance/aim_solver.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "guidance/camera_projection.hpp"
#include "guidance/depth_estimator.hpp"
#include "guidance/galvo_kinematics.hpp"
#include "guidance/lidar_depth_estimator.hpp"
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
        lidar_depth_estimator_ = std::make_unique<LidarDepthEstimator>(config_);
        kinematics_ = std::make_unique<GalvoKinematics>(config_);
        return {};
    } catch (const std::exception& e) {
        return std::string("calibration load failed: ") + e.what();
    }
}

auto AimSolver::solve(AimInput& input) -> AimOutput {
    if (!initialized_) {
        return AimOutput{
            .message = init_error_.empty() ? "aim solver not initialized" : init_error_,
        };
    }
    if (config_.calib_mode) {
        return AimOutput{.message = ""};
    }
    if (input.track.aim_center.x < 0.0F || input.track.aim_center.y < 0.0F
        || input.track.aim_center.x >= std::max(1.0F, image_width_)
        || input.track.aim_center.y >= std::max(1.0F, image_height_)) {
        return AimOutput{.message = "invalid aim center"};
    }

    if (config_.command_model == GuidanceCommandModelKind::direct_voltage) {
        return solve_direct_voltage(input);
    }
    return solve_geometry(input);
}

auto AimSolver::estimate_depth(const Detection& detection, const LidarFrame* lidar_frame) const
    -> std::optional<float> {
    const auto model_candidate = to_model_candidate(detection);
    if (config_.depth_source == GuidanceDepthSourceKind::lidar_target_cluster
        && lidar_depth_estimator_ && lidar_frame != nullptr) {
        if (const auto lidar_depth = lidar_depth_estimator_->estimate(model_candidate, *lidar_frame)) {
            return lidar_depth;
        }
    }
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

auto AimSolver::solve_geometry(AimInput& input) -> AimOutput {
    if (!projection_ || !kinematics_) {
        return AimOutput{.message = "geometry guidance not initialized"};
    }

    if (input.track.selected_detection != nullptr) {
        if (const auto depth = estimate_depth(*input.track.selected_detection, &input.lidar_frame)) {
            input.last_valid_depth_mm = *depth;
        }
    }

    if (input.last_valid_depth_mm <= 0.0F) {
        return AimOutput{
            .message = "no valid depth",
        };
    }

    const auto P_c = projection_->project(input.track.aim_center, input.last_valid_depth_mm);
    const auto angles = kinematics_->compute(P_c);
    if (!angles.valid) {
        return AimOutput{.message = "kinematics failed"};
    }

    const cv::Point2f output_angles{
        angles.theta_x_optical_deg + config_.angle_offset_x_deg,
        angles.theta_y_optical_deg + config_.angle_offset_y_deg,
    };
    return AimOutput{
        .command_issued = true,
        .depth_valid = true,
        .depth_mm = input.last_valid_depth_mm,
        .message = "",
        .output_angles = output_angles,
    };
}

auto AimSolver::solve_direct_voltage(AimInput& input) -> AimOutput {
    if (!voltage_mapper_) {
        return AimOutput{.message = "direct voltage mapper not initialized"};
    }
    if (input.track.selected_detection == nullptr) {
        return AimOutput{.message = "no candidate for direct voltage"};
    }
    const auto& detection = *input.track.selected_detection;
    if (detection.bbox.width <= 0.0F || detection.bbox.height <= 0.0F) {
        return AimOutput{.message = "invalid bbox for direct voltage"};
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
        return AimOutput{.message = "direct voltage predict failed"};
    }

    return AimOutput{
        .command_issued = true,
        .depth_valid = input.last_valid_depth_mm > 0.0F,
        .depth_mm = input.last_valid_depth_mm,
        .message = "",
        .output_voltages = cv::Point2f{
            command->vx + config_.voltage_offset_vx,
            command->vy + config_.voltage_offset_vy,
        },
    };
}

} // namespace rmcs_laser_guidance
