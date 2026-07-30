#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <print>
#include <string>

#include "runtime/guidance_calibration.hpp"

int main() {
    using namespace rmcs_laser_guidance::runtime_internal;
    if (geometry_calibration_csv_header()
        != "theta_x_deg,theta_y_deg,pixel_x,pixel_y,depth_mm,depth_source,depth_sigma_mm\n")
        std::abort();
    const std::string row =
        format_bbox_geometry_calibration_record(0.25, -0.5, 1200.5, 900.25, 10000.0);
    if (!row.contains(",10000.000,bbox,800.000\n"))
        std::abort();
    const auto old_path =
        std::filesystem::temp_directory_path() / "laser_guidance_old_geometry.csv";
    std::ofstream(old_path) << "0.1,0.2,100,200,10000\n";
    if (geometry_calibration_file_is_compatible(old_path))
        std::abort();
    std::println("guidance_calibration_record_test: PASSED");
}
