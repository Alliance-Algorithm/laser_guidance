#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <print>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "guidance/galvo_kinematics.hpp"
#include "config.hpp"

namespace {

using namespace rmcs_laser_guidance;

constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;
constexpr float kRadToDeg = 180.0F / 3.14159265358979323846F;

struct CalibRecord {
    float theta_x_deg = 0.0F;
    float theta_y_deg = 0.0F;
    float p_x_mm = 0.0F;
    float p_y_mm = 0.0F;
    float p_z_mm = 0.0F;
};

auto load_records(const std::string& path) -> std::vector<CalibRecord> {
    std::vector<CalibRecord> recs;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        CalibRecord r;
        ss >> r.theta_x_deg >> r.theta_y_deg >> r.p_x_mm >> r.p_y_mm >> r.p_z_mm;
        if (r.p_z_mm > 0.0F) recs.push_back(r);
    }
    return recs;
}

auto sq(float v) -> float { return v * v; }

auto error_for_extrinsics(const std::vector<CalibRecord>& recs,
                           GuidanceConfig cfg) -> float {
    GalvoKinematics kin(cfg);
    float total = 0.0F;
    for (const auto& r : recs) {
        auto angles = kin.compute({r.p_x_mm, r.p_y_mm, r.p_z_mm});
        if (!angles.valid) return 1e9F;
        total += sq(angles.theta_x_optical_deg - r.theta_x_deg)
               + sq(angles.theta_y_optical_deg - r.theta_y_deg);
    }
    return std::sqrt(total / static_cast<float>(recs.size()));
}

auto optimize_extrinsics(const std::vector<CalibRecord>& recs,
                         GuidanceConfig init) -> GuidanceConfig {
    GuidanceConfig best = init;
    float best_err = error_for_extrinsics(recs, best);

    constexpr int kIters = 200;
    constexpr float kInitStep = 10.0F;
    constexpr float kMinStep = 0.01F;

    struct Param { float GuidanceConfig::*ptr; float step; };
    Param params[] = {
        {&GuidanceConfig::r_x_deg, 0.5F},
        {&GuidanceConfig::r_y_deg, 0.5F},
        {&GuidanceConfig::r_z_deg, 0.5F},
    };

    for (int iter = 0; iter < kIters; ++iter) {
        bool improved = false;
        for (auto& param : params) {
            GuidanceConfig trial = best;
            trial.*(param.ptr) += param.step;
            float err = error_for_extrinsics(recs, trial);
            if (err < best_err) {
                best = trial;
                best_err = err;
                improved = true;
            } else {
                trial = best;
                trial.*(param.ptr) -= param.step;
                err = error_for_extrinsics(recs, trial);
                if (err < best_err) {
                    best = trial;
                    best_err = err;
                    improved = true;
                }
            }
        }
        if (!improved) {
            bool any_reduced = false;
            for (auto& param : params) {
                if (param.step > kMinStep) {
                    param.step *= 0.5F;
                    any_reduced = true;
                }
            }
            if (!any_reduced) break;
        }
    }
    return best;
}

} // namespace

int main(int argc, char** argv) {
    const char* csv_path = (argc > 1) ? argv[1] : "calib_records.csv";
    const char* config_path = (argc > 2) ? argv[2] : "config/calib_guidance.yaml";

    auto recs = load_records(csv_path);
    if (recs.size() < 3) {
        std::println(stderr, "Need at least 3 records, got {}", recs.size());
        return 1;
    }
    std::println("Loaded {} records from {}", recs.size(), csv_path);

    auto config = load_config(config_path);
    auto init = config.guidance;
    std::println("Initial: t=({:.1f},{:.1f},{:.1f})mm r=({:.2f},{:.2f},{:.2f})° err={:.3f}°",
                 init.t_x_mm, init.t_y_mm, init.t_z_mm,
                 init.r_x_deg, init.r_y_deg, init.r_z_deg,
                 error_for_extrinsics(recs, init));

    auto best = optimize_extrinsics(recs, init);
    std::println("");
    std::println("=== OPTIMIZED EXTRINSICS ===");
    std::println("t_x_mm: {:.1f}", best.t_x_mm);
    std::println("t_y_mm: {:.1f}", best.t_y_mm);
    std::println("t_z_mm: {:.1f}", best.t_z_mm);
    std::println("r_x_deg: {:.2f}", best.r_x_deg);
    std::println("r_y_deg: {:.2f}", best.r_y_deg);
    std::println("r_z_deg: {:.2f}", best.r_z_deg);
    std::println("residual: {:.3f}°", error_for_extrinsics(recs, best));
    return 0;
}
