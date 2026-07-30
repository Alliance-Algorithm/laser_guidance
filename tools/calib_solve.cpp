#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>

#include "calib/rotation_calibrator.hpp"
#include "config.hpp"

namespace {

using namespace rmcs_laser_guidance;
using namespace rmcs_laser_guidance::calibration;

struct CliOptions {
    std::filesystem::path records_path{};
    std::filesystem::path runtime_path{};
    std::filesystem::path intrinsics_path{};
    int image_width = 0;
    int image_height = 0;
    InitialRotation initial_rotation = InitialRotation::config;
    double rangefinder_weight = 1.0;
    double bbox_weight = 0.25;
};

[[noreturn]] auto usage_error(const std::string& message) -> void {
    throw std::runtime_error(
        message
        + "\nUsage: tool_calib_solve <records.csv> <runtime.yaml> <camera_calib.yaml> "
          "--image-width <W> --image-height <H> "
          "[--initial-rotation config|wahba] "
          "[--rangefinder-weight <positive>] [--bbox-weight <positive>]");
}

auto parse_positive_int(const std::string_view value, const std::string_view name) -> int {
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(std::string(value), &consumed);
        if (consumed != value.size())
            usage_error(std::string(name) + " must be a positive integer");
        if (parsed <= 0)
            usage_error(std::string(name) + " must be positive");
        return parsed;
    } catch (const std::exception&) {
        usage_error(std::string(name) + " must be a positive integer");
    }
}

auto parse_positive_double(const std::string_view value, const std::string_view name) -> double {
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(std::string(value), &consumed);
        if (consumed != value.size())
            usage_error(std::string(name) + " must be positive and finite");
        if (!std::isfinite(parsed) || parsed <= 0.0)
            usage_error(std::string(name) + " must be positive and finite");
        return parsed;
    } catch (const std::exception&) {
        usage_error(std::string(name) + " must be positive and finite");
    }
}

auto parse_cli(const int argc, char** argv) -> CliOptions {
    if (argc < 4)
        usage_error("missing required paths");
    CliOptions options{
        .records_path = argv[1],
        .runtime_path = argv[2],
        .intrinsics_path = argv[3],
    };
    for (int index = 4; index < argc; ++index) {
        const std::string_view argument(argv[index]);
        if (index + 1 >= argc)
            usage_error("missing value for " + std::string(argument));
        const std::string_view value(argv[++index]);
        if (argument == "--image-width")
            options.image_width = parse_positive_int(value, argument);
        else if (argument == "--image-height")
            options.image_height = parse_positive_int(value, argument);
        else if (argument == "--rangefinder-weight")
            options.rangefinder_weight = parse_positive_double(value, argument);
        else if (argument == "--bbox-weight")
            options.bbox_weight = parse_positive_double(value, argument);
        else if (argument == "--initial-rotation") {
            if (value == "config")
                options.initial_rotation = InitialRotation::config;
            else if (value == "wahba")
                options.initial_rotation = InitialRotation::wahba;
            else
                usage_error("--initial-rotation must be config or wahba");
        } else {
            usage_error("unknown option: " + std::string(argument));
        }
    }
    if (options.image_width <= 0 || options.image_height <= 0)
        usage_error("--image-width and --image-height are required");
    return options;
}

auto print_stats(const std::string_view name, const ResidualStats& stats) -> void {
    std::println(
        "{}: n={} rms={:.6f}deg p50={:.6f}deg p95={:.6f}deg max={:.6f}deg "
        "worst_line={}",
        name, stats.count, stats.rms_deg, stats.p50_deg, stats.p95_deg, stats.max_deg,
        stats.worst_line);
}

} // namespace

int main(int argc, char** argv) {
    try {
        const CliOptions cli = parse_cli(argc, argv);
        const RecordLoadResult loaded = load_calibration_records(cli.records_path);
        if (!loaded.errors.empty()) {
            for (const auto& error : loaded.errors)
                std::println(stderr, "CSV error: {}", error);
            return EXIT_FAILURE;
        }

        const CameraIntrinsics intrinsics = load_camera_intrinsics(cli.intrinsics_path);
        if (intrinsics.image_width != cli.image_width
            || intrinsics.image_height != cli.image_height) {
            std::println(
                stderr, "image size mismatch: records={}x{} calibration={}x{}", cli.image_width,
                cli.image_height, intrinsics.image_width, intrinsics.image_height);
            return EXIT_FAILURE;
        }

        const Config config = load_config(cli.runtime_path);
        const auto& guidance = config.guidance;
        const Eigen::Vector3d translation(guidance.t_x_mm, guidance.t_y_mm, guidance.t_z_mm);
        if (!translation.allFinite() || !std::isfinite(guidance.mirror_separation_mm)) {
            std::println(stderr, "fixed translation and mirror separation must be finite");
            return EXIT_FAILURE;
        }

        CalibrationOptions options;
        options.configured_rotation =
            quaternion_from_config_euler(guidance.r_x_deg, guidance.r_y_deg, guidance.r_z_deg);
        options.fixed_translation_camera_mm = translation;
        options.fixed_mirror_separation_mm = guidance.mirror_separation_mm;
        options.initial_rotation = cli.initial_rotation;
        options.rangefinder_weight = cli.rangefinder_weight;
        options.bbox_weight = cli.bbox_weight;

        const CalibrationResult result = calibrate_rotation(loaded.records, intrinsics, options);
        std::println("Loaded {} depth-tagged records", loaded.records.size());
        std::println(
            "Fixed translation (FIXED, NOT OPTIMIZED): [{:.9g}, {:.9g}, {:.9g}] mm",
            translation.x(), translation.y(), translation.z());
        std::println("Calibration offsets forced to zero: [0, 0] deg");
        std::println(
            "Residual weights: rangefinder={:.9g} bbox={:.9g}", options.rangefinder_weight,
            options.bbox_weight);
        std::println(
            "Wahba diagnostic: observable={} coverage={:.9g} singular=[{:.9g}, {:.9g}, "
            "{:.9g}]",
            result.wahba.observable, result.wahba.coverage_ratio, result.wahba.singular_values.x(),
            result.wahba.singular_values.y(), result.wahba.singular_values.z());
        std::println("Ceres: {}", result.solver_report);
        if (!result.usable) {
            std::println(stderr, "calibration failed: {}", result.message);
            return EXIT_FAILURE;
        }

        print_stats("rangefinder", result.rangefinder);
        print_stats("bbox", result.bbox);
        std::println(
            "Initial q_wxyz: [{:.12g}, {:.12g}, {:.12g}, {:.12g}]", result.initial_rotation.w(),
            result.initial_rotation.x(), result.initial_rotation.y(), result.initial_rotation.z());
        std::println(
            "Final q_wxyz: [{:.12g}, {:.12g}, {:.12g}, {:.12g}]", result.optimized_rotation.w(),
            result.optimized_rotation.x(), result.optimized_rotation.y(),
            result.optimized_rotation.z());
        const Eigen::Vector3d euler = config_euler_from_quaternion(result.optimized_rotation);
        const auto& quaternion = result.optimized_rotation;
        std::println("\nFINAL ROTATION CONFIG");
        std::println("r_x_deg: {:.12g}", euler.x());
        std::println("r_y_deg: {:.12g}", euler.y());
        std::println("r_z_deg: {:.12g}", euler.z());
        std::println(
            "q_wxyz: [{:.12g}, {:.12g}, {:.12g}, {:.12g}]", quaternion.w(), quaternion.x(),
            quaternion.y(), quaternion.z());
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::println(stderr, "tool_calib_solve: {}", error.what());
        return EXIT_FAILURE;
    }
}
