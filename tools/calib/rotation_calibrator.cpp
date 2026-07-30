#include "calib/rotation_calibrator.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <memory>
#include <numbers>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs_laser_guidance::calibration {
namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;

auto split_csv(const std::string& line) -> std::vector<std::string_view> {
    std::vector<std::string_view> fields;
    const std::string_view view(line);
    std::size_t begin = 0;
    while (begin <= view.size()) {
        const auto end = view.find(',', begin);
        fields.push_back(
            view.substr(begin, end == std::string_view::npos ? view.size() - begin : end - begin));
        if (end == std::string_view::npos)
            break;
        begin = end + 1;
    }
    return fields;
}

auto parse_double(const std::string_view text, double& value) -> bool {
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto result = std::from_chars(begin, end, value);
    return result.ec == std::errc{} && result.ptr == end && std::isfinite(value);
}

auto source_weight(const DepthSource source, const CalibrationOptions& options) -> double {
    return source == DepthSource::rangefinder ? options.rangefinder_weight : options.bbox_weight;
}

auto galvo_direction(const double theta_x_deg, const double theta_y_deg) -> Eigen::Vector3d {
    const double theta_x = theta_x_deg * kDegToRad;
    const double theta_y = theta_y_deg * kDegToRad;
    Eigen::Vector3d result(std::tan(theta_x) / std::cos(theta_y), std::tan(theta_y), 1.0);
    return result.normalized();
}

auto percentile(const std::vector<double>& sorted, const double quantile) -> double {
    if (sorted.empty())
        return 0.0;
    const auto index =
        static_cast<std::size_t>(std::ceil(quantile * static_cast<double>(sorted.size())) - 1.0);
    return sorted[std::min(index, sorted.size() - 1)];
}

auto residual_stats(
    const std::vector<CalibrationRecord>& records, const CameraIntrinsics& intrinsics,
    const CalibrationOptions& options, const Eigen::Quaterniond& rotation, const DepthSource source)
    -> ResidualStats {
    std::vector<std::pair<double, std::size_t>> values;
    for (const auto& record : records) {
        if (record.depth_source != source)
            continue;
        const auto prediction = predict_angles(
            rotation, options.fixed_translation_camera_mm, options.fixed_mirror_separation_mm,
            camera_point(record, intrinsics));
        if (!prediction.valid)
            continue;
        const double dx = prediction.x_rad - record.theta_x_deg * kDegToRad;
        const double dy = prediction.y_rad - record.theta_y_deg * kDegToRad;
        values.emplace_back(std::hypot(dx, dy) * kRadToDeg, record.line_number);
    }
    ResidualStats stats;
    stats.count = values.size();
    if (values.empty())
        return stats;
    double sum_sq = 0.0;
    for (const auto& [value, line] : values) {
        sum_sq += value * value;
        if (value >= stats.max_deg) {
            stats.max_deg = value;
            stats.worst_line = line;
        }
    }
    stats.rms_deg = std::sqrt(sum_sq / static_cast<double>(values.size()));
    std::vector<double> sorted;
    sorted.reserve(values.size());
    for (const auto& [value, line] : values) {
        static_cast<void>(line);
        sorted.push_back(value);
    }
    std::ranges::sort(sorted);
    stats.p50_deg = percentile(sorted, 0.50);
    stats.p95_deg = percentile(sorted, 0.95);
    return stats;
}

struct AngleResidual {
    Eigen::Vector3d point_camera_mm;
    Eigen::Vector3d translation_camera_mm;
    double mirror_separation_mm;
    double measured_x_rad;
    double measured_y_rad;
    double residual_scale_x;
    double residual_scale_y;

    template <typename T>
    auto operator()(const T* const q_wxyz, T* residuals) const -> bool {
        const Eigen::Quaternion<T> rotation(q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3]);
        const Eigen::Matrix<T, 3, 1> point = point_camera_mm.cast<T>();
        const Eigen::Matrix<T, 3, 1> translation = translation_camera_mm.cast<T>();
        const Eigen::Matrix<T, 3, 1> point_galvo = rotation * (point - translation);
        const T z_effective = point_galvo.z() + T(mirror_separation_mm);
        const T radial_yz =
            ceres::sqrt(point_galvo.y() * point_galvo.y() + z_effective * z_effective);
        const T predicted_x = ceres::atan2(point_galvo.x(), radial_yz);
        const T predicted_y = ceres::atan2(point_galvo.y(), z_effective);
        const T difference_x = predicted_x - T(measured_x_rad);
        const T difference_y = predicted_y - T(measured_y_rad);
        residuals[0] =
            ceres::atan2(ceres::sin(difference_x), ceres::cos(difference_x)) * T(residual_scale_x);
        residuals[1] =
            ceres::atan2(ceres::sin(difference_y), ceres::cos(difference_y)) * T(residual_scale_y);
        return true;
    }
};

auto angular_sigmas(
    const CalibrationRecord& record, const CameraIntrinsics& intrinsics,
    const CalibrationOptions& options, const Eigen::Quaterniond& rotation) -> Eigen::Vector2d {
    const double step = std::max(1.0, record.depth_mm * 1e-4);
    CalibrationRecord lower = record;
    CalibrationRecord upper = record;
    lower.depth_mm -= step;
    upper.depth_mm += step;
    if (lower.depth_mm <= 0.0) {
        return Eigen::Vector2d::Constant(options.angle_sigma_floor_rad);
    }
    const auto low = predict_angles(
        rotation, options.fixed_translation_camera_mm, options.fixed_mirror_separation_mm,
        camera_point(lower, intrinsics));
    const auto high = predict_angles(
        rotation, options.fixed_translation_camera_mm, options.fixed_mirror_separation_mm,
        camera_point(upper, intrinsics));
    if (!low.valid || !high.valid) {
        return Eigen::Vector2d::Constant(options.angle_sigma_floor_rad);
    }
    const Eigen::Vector2d derivative(
        (high.x_rad - low.x_rad) / (2.0 * step), (high.y_rad - low.y_rad) / (2.0 * step));
    return Eigen::Vector2d(
        std::hypot(options.angle_sigma_floor_rad, derivative.x() * record.depth_sigma_mm),
        std::hypot(options.angle_sigma_floor_rad, derivative.y() * record.depth_sigma_mm));
}

} // namespace

auto load_calibration_records(const std::filesystem::path& path) -> RecordLoadResult {
    RecordLoadResult result;
    std::ifstream input(path);
    if (!input) {
        result.errors.push_back("cannot open calibration CSV: " + path.string());
        return result;
    }
    std::string line;
    if (!std::getline(input, line) || line != kCalibrationCsvHeader) {
        result.errors.push_back("depth-tagged seven-column CSV required");
        return result;
    }
    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty())
            continue;
        const auto fields = split_csv(line);
        if (fields.size() != 7) {
            result.errors.push_back("line " + std::to_string(line_number) + ": expected 7 columns");
            continue;
        }
        CalibrationRecord record;
        record.line_number = line_number;
        if (!parse_double(fields[0], record.theta_x_deg)
            || !parse_double(fields[1], record.theta_y_deg)
            || !parse_double(fields[2], record.pixel.x) || !parse_double(fields[3], record.pixel.y)
            || !parse_double(fields[4], record.depth_mm)
            || !parse_double(fields[6], record.depth_sigma_mm)) {
            result.errors.push_back("line " + std::to_string(line_number) + ": invalid number");
            continue;
        }
        if (fields[5] == "rangefinder")
            record.depth_source = DepthSource::rangefinder;
        else if (fields[5] == "bbox")
            record.depth_source = DepthSource::bbox;
        else {
            result.errors.push_back(
                "line " + std::to_string(line_number) + ": invalid depth_source");
            continue;
        }
        if (record.depth_mm <= 0.0 || record.depth_sigma_mm <= 0.0) {
            result.errors.push_back(
                "line " + std::to_string(line_number) + ": depth and sigma must be positive");
            continue;
        }
        result.records.push_back(record);
    }
    return result;
}

auto load_camera_intrinsics(const std::filesystem::path& path) -> CameraIntrinsics {
    const auto yaml = YAML::LoadFile(path.string());
    const auto calibration = yaml["calibration"];
    if (!calibration)
        throw std::runtime_error("missing calibration node");
    CameraIntrinsics result;
    result.image_width = calibration["image_width"].as<int>();
    result.image_height = calibration["image_height"].as<int>();
    const auto matrix = calibration["camera_matrix"];
    if (!matrix || matrix.size() != 3)
        throw std::runtime_error("invalid camera_matrix");
    result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            result.camera_matrix.at<double>(row, column) = matrix[row][column].as<double>();
    if (const auto distortion = calibration["dist_coeffs"]) {
        result.dist_coeffs = cv::Mat::zeros(1, static_cast<int>(distortion.size()), CV_64F);
        for (int index = 0; index < static_cast<int>(distortion.size()); ++index)
            result.dist_coeffs.at<double>(index) = distortion[index].as<double>();
    }
    return result;
}

auto camera_direction(const cv::Point2d& pixel, const CameraIntrinsics& intrinsics)
    -> Eigen::Vector3d {
    std::vector<cv::Point2d> source{pixel};
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(source, normalized, intrinsics.camera_matrix, intrinsics.dist_coeffs);
    Eigen::Vector3d direction(normalized[0].x, normalized[0].y, 1.0);
    return direction.normalized();
}

auto camera_point(const CalibrationRecord& record, const CameraIntrinsics& intrinsics)
    -> Eigen::Vector3d {
    const Eigen::Vector3d direction = camera_direction(record.pixel, intrinsics);
    const double axial_depth = record.depth_source == DepthSource::rangefinder
                                 ? record.depth_mm * direction.z()
                                 : record.depth_mm;
    return direction * (axial_depth / direction.z());
}

auto predict_angles(
    const Eigen::Quaterniond& rotation_galvo_from_camera,
    const Eigen::Vector3d& translation_camera_mm, const double mirror_separation_mm,
    const Eigen::Vector3d& point_camera_mm) -> PredictedAngles {
    if (!point_camera_mm.allFinite() || point_camera_mm.z() <= 0.0)
        return {};
    const Eigen::Vector3d point_galvo =
        rotation_galvo_from_camera * (point_camera_mm - translation_camera_mm);
    if (!point_galvo.allFinite() || point_galvo.z() <= 0.0)
        return {};
    const double z_effective = point_galvo.z() + mirror_separation_mm;
    return {
        std::atan2(point_galvo.x(), std::hypot(point_galvo.y(), z_effective)),
        std::atan2(point_galvo.y(), z_effective), true};
}

auto quaternion_from_config_euler(const double r_x_deg, const double r_y_deg, const double r_z_deg)
    -> Eigen::Quaterniond {
    Eigen::Quaterniond rotation = Eigen::AngleAxisd(-r_z_deg * kDegToRad, Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(-r_x_deg * kDegToRad, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(-r_y_deg * kDegToRad, Eigen::Vector3d::UnitX());
    rotation.normalize();
    return rotation;
}

auto config_euler_from_quaternion(const Eigen::Quaterniond& rotation) -> Eigen::Vector3d {
    const Eigen::Matrix3d matrix = rotation.normalized().toRotationMatrix();
    const double sine_beta = std::clamp(-matrix(2, 0), -1.0, 1.0);
    const double beta = std::asin(sine_beta);
    const double cosine_beta = std::cos(beta);
    double alpha = 0.0;
    double gamma = 0.0;
    if (std::abs(cosine_beta) > 1e-10) {
        alpha = std::atan2(matrix(1, 0), matrix(0, 0));
        gamma = std::atan2(matrix(2, 1), matrix(2, 2));
    } else {
        gamma = sine_beta > 0.0 ? std::atan2(matrix(0, 1), matrix(1, 1))
                                : std::atan2(-matrix(0, 1), matrix(1, 1));
    }
    return {-beta * kRadToDeg, -gamma * kRadToDeg, -alpha * kRadToDeg};
}

auto solve_wahba(
    const std::vector<Eigen::Vector3d>& camera_directions,
    const std::vector<Eigen::Vector3d>& galvo_directions, const std::vector<double>& weights)
    -> WahbaResult {
    WahbaResult result;
    if (camera_directions.size() < 3 || camera_directions.size() != galvo_directions.size()
        || camera_directions.size() != weights.size())
        return result;
    Eigen::Matrix3d correlation = Eigen::Matrix3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    double total_weight = 0.0;
    for (std::size_t index = 0; index < camera_directions.size(); ++index) {
        correlation +=
            weights[index] * galvo_directions[index] * camera_directions[index].transpose();
        mean += weights[index] * camera_directions[index];
        total_weight += weights[index];
    }
    mean /= total_weight;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (std::size_t index = 0; index < camera_directions.size(); ++index) {
        const Eigen::Vector3d centered = camera_directions[index] - mean;
        covariance += weights[index] * centered * centered.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    const auto eigenvalues = eigen.eigenvalues();
    result.coverage_ratio = eigenvalues[2] > 0.0 ? eigenvalues[1] / eigenvalues[2] : 0.0;
    result.observable = result.coverage_ratio >= 1e-3;

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        correlation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = (U * V.transpose()).determinant() < 0.0 ? -1.0 : 1.0;
    result.rotation_galvo_from_camera =
        Eigen::Quaterniond(U * correction * V.transpose()).normalized();
    result.singular_values = svd.singularValues();
    return result;
}

auto calibrate_rotation(
    const std::vector<CalibrationRecord>& records, const CameraIntrinsics& intrinsics,
    const CalibrationOptions& options) -> CalibrationResult {
    CalibrationResult result;
    if (records.size() < 6) {
        result.message = "need at least 6 depth-tagged records";
        return result;
    }
    double min_x = static_cast<double>(intrinsics.image_width);
    double max_x = 0.0;
    double min_y = static_cast<double>(intrinsics.image_height);
    double max_y = 0.0;
    std::vector<Eigen::Vector3d> camera_directions;
    std::vector<Eigen::Vector3d> galvo_directions;
    std::vector<double> weights;
    for (const auto& record : records) {
        if (record.pixel.x < 0.0 || record.pixel.y < 0.0 || record.pixel.x >= intrinsics.image_width
            || record.pixel.y >= intrinsics.image_height || std::abs(record.theta_x_deg) >= 89.0
            || std::abs(record.theta_y_deg) >= 89.0) {
            result.message = "record is outside valid pixel/angle bounds at line "
                           + std::to_string(record.line_number);
            return result;
        }
        const double depth_step = std::max(1.0, record.depth_mm * 1e-4);
        if (record.depth_mm <= depth_step) {
            result.message = "depth is too small for uncertainty propagation at line "
                           + std::to_string(record.line_number);
            return result;
        }
        const auto camera = camera_direction(record.pixel, intrinsics);
        const auto galvo = galvo_direction(record.theta_x_deg, record.theta_y_deg);
        if (!camera.allFinite() || !galvo.allFinite()) {
            result.message =
                "record direction is not finite at line " + std::to_string(record.line_number);
            return result;
        }
        min_x = std::min(min_x, record.pixel.x);
        max_x = std::max(max_x, record.pixel.x);
        min_y = std::min(min_y, record.pixel.y);
        max_y = std::max(max_y, record.pixel.y);
        camera_directions.push_back(camera);
        galvo_directions.push_back(galvo);
        weights.push_back(source_weight(record.depth_source, options));
    }
    if ((max_x - min_x) < intrinsics.image_width * 0.05
        && (max_y - min_y) < intrinsics.image_height * 0.05) {
        result.message = "depth samples do not cover enough of the image";
        return result;
    }
    result.wahba = solve_wahba(camera_directions, galvo_directions, weights);
    if (!result.wahba.observable) {
        result.message = "calibration directions are degenerate";
        return result;
    }

    result.initial_rotation = options.initial_rotation == InitialRotation::wahba
                                ? result.wahba.rotation_galvo_from_camera
                                : options.configured_rotation.normalized();
    std::array<double, 4> quaternion{
        result.initial_rotation.w(), result.initial_rotation.x(), result.initial_rotation.y(),
        result.initial_rotation.z()};
    ceres::Problem problem;
    for (const auto& record : records) {
        const auto point = camera_point(record, intrinsics);
        const auto initial_prediction = predict_angles(
            result.initial_rotation, options.fixed_translation_camera_mm,
            options.fixed_mirror_separation_mm, point);
        if (!initial_prediction.valid) {
            result.message = "record predicts invalid forward geometry at line "
                           + std::to_string(record.line_number);
            return result;
        }
        const Eigen::Vector2d sigmas =
            angular_sigmas(record, intrinsics, options, result.initial_rotation);
        const double weight = std::sqrt(source_weight(record.depth_source, options));
        auto* cost = new ceres::AutoDiffCostFunction<AngleResidual, 2, 4>(new AngleResidual{
            point,
            options.fixed_translation_camera_mm,
            options.fixed_mirror_separation_mm,
            record.theta_x_deg * kDegToRad,
            record.theta_y_deg * kDegToRad,
            weight / sigmas.x(),
            weight / sigmas.y(),
        });
        problem.AddResidualBlock(
            cost, new ceres::HuberLoss(options.huber_delta), quaternion.data());
    }
    problem.SetManifold(quaternion.data(), new ceres::QuaternionManifold());
    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.max_num_iterations = 100;
    solver_options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);
    result.solver_report = summary.BriefReport();
    result.optimized_rotation =
        Eigen::Quaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]).normalized();
    if (!summary.IsSolutionUsable() || !result.optimized_rotation.coeffs().allFinite()) {
        result.message = "Ceres did not produce a usable rotation";
        return result;
    }
    for (const auto& record : records) {
        const auto prediction = predict_angles(
            result.optimized_rotation, options.fixed_translation_camera_mm,
            options.fixed_mirror_separation_mm, camera_point(record, intrinsics));
        if (!prediction.valid || !std::isfinite(prediction.x_rad)
            || !std::isfinite(prediction.y_rad)) {
            result.message = "optimized rotation invalidates record at line "
                           + std::to_string(record.line_number);
            return result;
        }
    }
    result.rangefinder = residual_stats(
        records, intrinsics, options, result.optimized_rotation, DepthSource::rangefinder);
    result.bbox =
        residual_stats(records, intrinsics, options, result.optimized_rotation, DepthSource::bbox);
    result.usable = true;
    result.message = "ok";
    return result;
}

} // namespace rmcs_laser_guidance::calibration
