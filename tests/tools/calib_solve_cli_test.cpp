#include <array>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <print>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include "calib/rotation_calibrator.hpp"

namespace {

using namespace rmcs_laser_guidance::calibration;

#define CHECK(condition)                                               \
    do {                                                               \
        if (!(condition)) {                                            \
            std::println(stderr, "FAIL:{}: {}", __LINE__, #condition); \
            std::abort();                                              \
        }                                                              \
    } while (false)

struct ProcessResult {
    int exit_code = -1;
    std::string output{};
};

auto run_tool(const std::vector<std::string>& arguments, const std::string& name) -> ProcessResult {
    const auto output_path = std::filesystem::temp_directory_path() / (name + ".out");
    const pid_t child = fork();
    CHECK(child >= 0);
    if (child == 0) {
        const int output = open(output_path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0600);
        dup2(output, STDOUT_FILENO);
        dup2(output, STDERR_FILENO);
        close(output);
        std::vector<char*> argv;
        argv.reserve(arguments.size() + 1);
        for (const auto& argument : arguments)
            argv.push_back(const_cast<char*>(argument.c_str()));
        argv.push_back(nullptr);
        execv(argv[0], argv.data());
        _exit(127);
    }
    int status = 0;
    CHECK(waitpid(child, &status, 0) == child);
    std::ifstream output(output_path);
    const std::string text{
        std::istreambuf_iterator<char>(output), std::istreambuf_iterator<char>()};
    return {WIFEXITED(status) ? WEXITSTATUS(status) : -1, text};
}

auto write_intrinsics() -> std::filesystem::path {
    const auto path = std::filesystem::temp_directory_path() / "laser_cli_intrinsics.yaml";
    std::ofstream(path) << "calibration:\n"
                           "  image_width: 1920\n"
                           "  image_height: 1080\n"
                           "  camera_matrix:\n"
                           "    - [1000.0, 0.0, 960.0]\n"
                           "    - [0.0, 1000.0, 540.0]\n"
                           "    - [0.0, 0.0, 1.0]\n"
                           "  dist_coeffs: []\n";
    return path;
}

auto write_runtime() -> std::filesystem::path {
    const auto path = std::filesystem::temp_directory_path() / "laser_cli_runtime.yaml";
    std::ofstream(path) << "guidance:\n"
                           "  t_x_mm: 85.5\n"
                           "  t_y_mm: 15.0\n"
                           "  t_z_mm: 20.0\n"
                           "  r_x_deg: 0.2\n"
                           "  r_y_deg: -0.4\n"
                           "  r_z_deg: 0.1\n"
                           "  mirror_separation_mm: 15.0\n";
    return path;
}

auto write_records(const CameraIntrinsics& intrinsics) -> std::filesystem::path {
    const auto path = std::filesystem::temp_directory_path() / "laser_cli_records.csv";
    std::ofstream output(path);
    output << kCalibrationCsvHeader << '\n';
    const auto truth = quaternion_from_config_euler(0.7, -0.9, 0.4);
    const Eigen::Vector3d translation(85.5, 15.0, 20.0);
    std::size_t line = 0;
    for (const double depth : {5000.0, 13500.0}) {
        for (const double u : {400.0, 960.0, 1500.0}) {
            for (const double v : {250.0, 850.0}) {
                CalibrationRecord record{++line,      0.0, 0.0, {u, v}, depth, DepthSource::bbox,
                                         depth * 0.08};
                const auto angles =
                    predict_angles(truth, translation, 15.0, camera_point(record, intrinsics));
                output << angles.x_rad * 180.0 / std::numbers::pi << ','
                       << angles.y_rad * 180.0 / std::numbers::pi << ',' << u << ',' << v << ','
                       << depth << ",bbox," << depth * 0.08 << '\n';
            }
        }
    }
    return path;
}

} // namespace

int main() {
    const auto intrinsics_path = write_intrinsics();
    const auto runtime_path = write_runtime();
    const auto intrinsics = load_camera_intrinsics(intrinsics_path);
    const auto records_path = write_records(intrinsics);
    const std::string tool = CALIB_SOLVE_PATH;

    const auto valid = run_tool(
        {tool, records_path.string(), runtime_path.string(), intrinsics_path.string(),
         "--image-width", "1920", "--image-height", "1080"},
        "laser_cli_valid");
    CHECK(valid.exit_code == 0);
    CHECK(valid.output.contains("FINAL ROTATION CONFIG"));
    CHECK(valid.output.contains("FIXED, NOT OPTIMIZED"));

    const auto old_path = std::filesystem::temp_directory_path() / "laser_cli_old.csv";
    std::ofstream(old_path) << "0.1,0.2,100,200,10000\n";
    const auto old = run_tool(
        {tool, old_path.string(), runtime_path.string(), intrinsics_path.string(), "--image-width",
         "1920", "--image-height", "1080"},
        "laser_cli_old");
    CHECK(old.exit_code != 0);
    CHECK(old.output.contains("depth-tagged seven-column CSV required"));
    CHECK(!old.output.contains("FINAL ROTATION CONFIG"));

    const auto malformed = run_tool(
        {tool, records_path.string(), runtime_path.string(), intrinsics_path.string(),
         "--image-width", "1920junk", "--image-height", "1080"},
        "laser_cli_malformed");
    CHECK(malformed.exit_code != 0);
    CHECK(!malformed.output.contains("FINAL ROTATION CONFIG"));

    std::println("calib_solve_cli_test: PASSED");
}
