#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <print>
#include <sstream>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "config.hpp"
#include "guidance/camera_galvo_geometry.hpp"

namespace {

using namespace rmcs_laser_guidance;

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

auto compute_residual(const std::vector<CalibRecord>& recs,
                      const CameraGalvoGeometry& geom) -> double {
    double total = 0.0;
    for (const auto& r : recs) {
        auto angles = geom.solve_angles({r.p_x_mm, r.p_y_mm, r.p_z_mm});
        if (!angles.valid) continue;
        double dx = static_cast<double>(angles.theta_x_optical_deg) - static_cast<double>(r.theta_x_deg);
        double dy = static_cast<double>(angles.theta_y_optical_deg) - static_cast<double>(r.theta_y_deg);
        total += dx * dx + dy * dy;
    }
    return std::sqrt(total / static_cast<double>(recs.size()));
}

// Ceres AutoDiff residual. The kinematics matches CameraGalvoGeometry::solve_angles()
// but is templated on T to support automatic differentiation.
struct ExtrinsicResidual {
    float p_x, p_y, p_z;
    float th_x, th_y;
    float m_sep;

    template <typename T>
    bool operator()(const T* const q_ptr, const T* const t_ptr, T* res) const {
        T qw = q_ptr[0], qx = q_ptr[1], qy = q_ptr[2], qz = q_ptr[3];

        T dx = T(p_x) - t_ptr[0];
        T dy = T(p_y) - t_ptr[1];
        T dz = T(p_z) - t_ptr[2];

        if (dz <= T(0)) { res[0] = T(0); res[1] = T(0); return true; }

        T cx = qy * dz - qz * dy;
        T cy = qz * dx - qx * dz;
        T cz = qx * dy - qy * dx;

        T ax = qy * cz - qz * cy;
        T ay = qz * cx - qx * cz;
        T az = qx * cy - qy * cx;

        T x_g = dx + T(2) * (qw * cx + ax);
        T y_g = dy + T(2) * (qw * cy + ay);
        T z_g = dz + T(2) * (qw * cz + az);

        if (z_g <= T(0)) { res[0] = T(0); res[1] = T(0); return true; }

        T z_eff = z_g + T(m_sep);
        T r_yz = ceres::sqrt(y_g * y_g + z_eff * z_eff);
        T th_x_m = T(0.5) * ceres::atan2(x_g, r_yz);
        T th_y_m = T(0.5) * ceres::atan2(y_g, z_eff);
        T deg = T(180.0 / 3.14159265358979323846);

        res[0] = T(2.0) * th_x_m * deg - T(th_x);
        res[1] = T(2.0) * th_y_m * deg - T(th_y);
        return true;
    }
};

} // namespace

int main(int argc, char** argv) {
    const char* csv_path = argc > 1 ? argv[1] : "test_data/calib/geometry_calib_records.csv";
    const char* config_path = argc > 2 ? argv[2] : "config/calib_guidance.yaml";

    auto recs = load_records(csv_path);
    if (recs.size() < 3) {
        std::println(stderr, "Need at least 3 records, got {}", recs.size());
        return 1;
    }
    std::println("Loaded {} records from {}", recs.size(), csv_path);

    auto config = load_config(config_path);
    auto init = config.guidance;

    CameraGalvoGeometry init_geom(init.t_x_mm, init.t_y_mm, init.t_z_mm,
                                   init.r_x_deg, init.r_y_deg, init.r_z_deg,
                                   init.mirror_separation_mm);
    auto qf = init_geom.rotation();
    auto tf = init_geom.translation();

    std::array<double, 4> q_opt = {static_cast<double>(qf.w()), static_cast<double>(qf.x()),
                                    static_cast<double>(qf.y()), static_cast<double>(qf.z())};
    std::array<double, 3> t_opt = {static_cast<double>(tf.x()), static_cast<double>(tf.y()),
                                    static_cast<double>(tf.z())};

    std::println("Initial: t=({:.1f},{:.1f},{:.1f}) q=({:.4f},{:.4f},{:.4f},{:.4f}) err={:.3f}deg",
                 init.t_x_mm, init.t_y_mm, init.t_z_mm,
                 q_opt[0], q_opt[1], q_opt[2], q_opt[3],
                 compute_residual(recs, init_geom));

    ceres::Problem problem;
    for (const auto& r : recs) {
        auto* cost = new ceres::AutoDiffCostFunction<ExtrinsicResidual, 2, 4, 3>(
            new ExtrinsicResidual{r.p_x_mm, r.p_y_mm, r.p_z_mm,
                                  r.theta_x_deg, r.theta_y_deg,
                                  init.mirror_separation_mm});
        problem.AddResidualBlock(cost, nullptr, q_opt.data(), t_opt.data());
    }

    ceres::QuaternionManifold quat_manifold;
    problem.SetManifold(q_opt.data(), &quat_manifold);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::println("{}", summary.BriefReport());

    std::println("");
    std::println("=== OPTIMIZED EXTRINSICS ===");
    std::println("t_x_mm: {:.1f}", t_opt[0]);
    std::println("t_y_mm: {:.1f}", t_opt[1]);
    std::println("t_z_mm: {:.1f}", t_opt[2]);
    std::println("q_w: {:.6f}", q_opt[0]);
    std::println("q_x: {:.6f}", q_opt[1]);
    std::println("q_y: {:.6f}", q_opt[2]);
    std::println("q_z: {:.6f}", q_opt[3]);
    return 0;
}
