#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <print>
#include <string>
#include <vector>

#include "calib/rotation_calibrator.hpp"
#include "guidance/camera_galvo_geometry.hpp"

namespace {

using namespace rmcs_laser_guidance::calibration;

#define CHECK(condition)                                               \
    do {                                                               \
        if (!(condition)) {                                            \
            std::println(stderr, "FAIL:{}: {}", __LINE__, #condition); \
            std::abort();                                              \
        }                                                              \
    } while (false)

auto temp_csv(const std::string& name, const std::string& content) -> std::filesystem::path {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream(path) << content;
    return path;
}

auto rotation_error(const Eigen::Quaterniond& lhs, const Eigen::Quaterniond& rhs) -> double {
    return Eigen::AngleAxisd(lhs.conjugate() * rhs).angle();
}

void test_strict_records() {
    const auto valid = temp_csv(
        "laser_rotation_valid.csv", std::string(kCalibrationCsvHeader)
                                        + "\n"
                                          "0.25,-0.5,1200.5,900.25,13500,rangefinder,150\n"
                                          "0.5,0.25,1500,700,14250,bbox,1140\n");
    const auto loaded = load_calibration_records(valid);
    CHECK(loaded.errors.empty());
    CHECK(loaded.records.size() == 2);
    CHECK(loaded.records[0].depth_source == DepthSource::rangefinder);
    CHECK(loaded.records[1].depth_source == DepthSource::bbox);

    const auto old = temp_csv("laser_rotation_old.csv", "0.25,-0.5,1200.5,900.25,13500\n");
    CHECK(!load_calibration_records(old).errors.empty());

    const auto invalid = temp_csv(
        "laser_rotation_invalid.csv", std::string(kCalibrationCsvHeader)
                                          + "\n"
                                            "0.25,-0.5,1200.5,900.25,0,bbox,100\n");
    CHECK(!load_calibration_records(invalid).errors.empty());
}

auto test_intrinsics() -> CameraIntrinsics {
    CameraIntrinsics intrinsics;
    intrinsics.image_width = 1920;
    intrinsics.image_height = 1080;
    intrinsics.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    intrinsics.camera_matrix.at<double>(0, 0) = 1000.0;
    intrinsics.camera_matrix.at<double>(1, 1) = 1000.0;
    intrinsics.camera_matrix.at<double>(0, 2) = 960.0;
    intrinsics.camera_matrix.at<double>(1, 2) = 540.0;
    return intrinsics;
}

void test_depth_semantics_and_prediction() {
    const auto intrinsics = test_intrinsics();
    CalibrationRecord bbox{1, 0.0, 0.0, {1160.0, 640.0}, 10000.0, DepthSource::bbox, 800.0};
    CalibrationRecord range = bbox;
    range.depth_source = DepthSource::rangefinder;
    const auto bbox_point = camera_point(bbox, intrinsics);
    const auto range_point = camera_point(range, intrinsics);
    CHECK(std::abs(bbox_point.z() - 10000.0) < 1e-9);
    CHECK(std::abs(range_point.norm() - 10000.0) < 1e-8);

    const Eigen::Vector3d translation(85.5, 15.0, 20.0);
    const auto rotation = quaternion_from_config_euler(0.6, 0.82, 0.31);
    const auto angles = predict_angles(rotation, translation, 15.0, bbox_point);
    CHECK(angles.valid);
    const auto point_galvo = rotation * (bbox_point - translation);
    CHECK(
        std::abs(
            angles.x_rad
            - std::atan2(point_galvo.x(), std::hypot(point_galvo.y(), point_galvo.z() + 15.0)))
        < 1e-12);
    CHECK(std::abs(angles.y_rad - std::atan2(point_galvo.y(), point_galvo.z() + 15.0)) < 1e-12);

    rmcs_laser_guidance::CameraGalvoGeometry runtime_geometry(
        85.5F, 15.0F, 20.0F, 0.6F, 0.82F, 0.31F, 15.0F);
    const auto runtime_angles = runtime_geometry.solve_angles(bbox_point.cast<float>());
    CHECK(runtime_angles.valid);
    CHECK(
        std::abs(runtime_angles.theta_x_optical_deg * std::numbers::pi / 180.0 - angles.x_rad)
        < 1e-6);
    CHECK(
        std::abs(runtime_angles.theta_y_optical_deg * std::numbers::pi / 180.0 - angles.y_rad)
        < 1e-6);
}

void test_wahba_direction() {
    std::vector<Eigen::Vector3d> camera;
    for (const double x : {-0.3, 0.0, 0.25})
        for (const double y : {-0.2, 0.15, 0.35})
            camera.emplace_back(Eigen::Vector3d(x, y, 1.0).normalized());
    const auto truth = quaternion_from_config_euler(1.3, -2.1, 0.7);
    std::vector<Eigen::Vector3d> galvo;
    for (const auto& direction : camera)
        galvo.push_back(truth * direction);
    const auto result = solve_wahba(camera, galvo, std::vector<double>(camera.size(), 1.0));
    CHECK(result.observable);
    CHECK(rotation_error(result.rotation_galvo_from_camera, truth) < 1e-10);
    CHECK(rotation_error(result.rotation_galvo_from_camera, truth.conjugate()) > 1e-3);

    for (const double pitch : {-90.0, 90.0}) {
        const auto singular = quaternion_from_config_euler(pitch, 12.0, -7.0);
        const auto recovered = config_euler_from_quaternion(singular);
        const auto round_trip =
            quaternion_from_config_euler(recovered.x(), recovered.y(), recovered.z());
        CHECK(rotation_error(singular, round_trip) < 1e-9);
    }
}

void test_depth_aware_rotation_recovery() {
    const auto intrinsics = test_intrinsics();
    const auto truth = quaternion_from_config_euler(0.7, -0.9, 0.4);
    CalibrationOptions options;
    options.configured_rotation = quaternion_from_config_euler(0.2, -0.4, 0.1);
    options.fixed_translation_camera_mm = {85.5, 15.0, 20.0};
    options.fixed_mirror_separation_mm = 15.0;

    std::vector<CalibrationRecord> records;
    std::size_t line = 2;
    for (const double depth : {5000.0, 13500.0, 25000.0}) {
        for (const double u : {400.0, 960.0, 1500.0}) {
            for (const double v : {250.0, 540.0, 850.0}) {
                CalibrationRecord record{line++,      0.0, 0.0, {u, v}, depth, DepthSource::bbox,
                                         depth * 0.08};
                const auto angles = predict_angles(
                    truth, options.fixed_translation_camera_mm, options.fixed_mirror_separation_mm,
                    camera_point(record, intrinsics));
                record.theta_x_deg = angles.x_rad * 180.0 / std::numbers::pi;
                record.theta_y_deg = angles.y_rad * 180.0 / std::numbers::pi;
                records.push_back(record);
            }
        }
    }
    const auto result = calibrate_rotation(records, intrinsics, options);
    CHECK(result.usable);
    CHECK(rotation_error(result.optimized_rotation, truth) < 1e-7);
    CHECK(result.bbox.rms_deg < 1e-6);

    auto invalid_pixel = records;
    invalid_pixel.front().pixel.x = -1.0;
    CHECK(!calibrate_rotation(invalid_pixel, intrinsics, options).usable);

    auto narrow = records;
    for (auto& record : narrow)
        record.pixel = {960.0, 540.0};
    CHECK(!calibrate_rotation(narrow, intrinsics, options).usable);
}

} // namespace

int main() {
    test_strict_records();
    test_depth_semantics_and_prediction();
    test_wahba_direction();
    test_depth_aware_rotation_recovery();
    std::println("rotation_calibrator_test: PASSED");
}
