#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <print>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include "config.hpp"

namespace {

using namespace rmcs_laser_guidance;

constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;
constexpr float kRadToDeg = 180.0F / 3.14159265358979323846F;

// CSV: theta_x_deg, theta_y_deg, pixel_x, pixel_y
struct CalibRecord {
    float theta_x_deg = 0.0F;
    float theta_y_deg = 0.0F;
    float pixel_x = 0.0F;
    float pixel_y = 0.0F;
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
        ss >> r.theta_x_deg >> r.theta_y_deg >> r.pixel_x >> r.pixel_y;
        recs.push_back(r);
    }
    return recs;
}

// Load camera intrinsics from OpenCV-format YAML
struct CameraIntrinsics {
    float fx = 0.0F, fy = 0.0F, cx = 0.0F, cy = 0.0F;
};

auto load_intrinsics(const std::string& path) -> CameraIntrinsics {
    const auto yaml = YAML::LoadFile(path);
    const auto calib = yaml["calibration"];
    if (!calib) throw std::runtime_error("missing 'calibration' key");
    const auto mat = calib["camera_matrix"];
    if (!mat || mat.size() < 3) throw std::runtime_error("missing camera_matrix");
    CameraIntrinsics k;
    k.fx = mat[0][0].as<float>();
    k.fy = mat[1][1].as<float>();
    k.cx = mat[0][2].as<float>();
    k.cy = mat[1][2].as<float>();
    return k;
}

// Back-project pixel to unit direction in camera frame (no distortion correction)
auto pixel_to_direction(float px, float py, const CameraIntrinsics& K) -> Eigen::Vector3f {
    Eigen::Vector3f d;
    d.x() = (px - K.cx) / K.fx;
    d.y() = (py - K.cy) / K.fy;
    d.z() = 1.0F;
    d.normalize();
    return d;
}

// Reverse mirror model: galvo optical angles → unit direction in galvo frame.
// Mirror model: θ_opt = 2 · θ_mech,  θ_mech = atan2(coord, ...)
// For far-field direction (z_eff → ∞): d = (tan(θx)/cos(θy), tan(θy), 1) normalized.
auto galvo_angles_to_direction(float theta_x_opt_deg, float theta_y_opt_deg) -> Eigen::Vector3f {
    float thx = theta_x_opt_deg * 0.5F * kDegToRad;
    float thy = theta_y_opt_deg * 0.5F * kDegToRad;
    float cx = std::cos(thx), sx = std::tan(thx);
    float cy = std::cos(thy), sy = std::tan(thy);
    // d_gal = (tan(thx)/cos(thy), tan(thy), 1) normalized
    Eigen::Vector3f d(sx / cy, sy, 1.0F);
    d.normalize();
    return d;
}

// Wahba problem: given pairs of unit direction vectors (d_cam, d_gal),
// find optimal rotation R minimizing Σ||R·d_cam - d_gal||².
// SVD closed-form solution, no depth required.
auto solve_wahba(const std::vector<Eigen::Vector3f>& d_cam,
                 const std::vector<Eigen::Vector3f>& d_gal) -> Eigen::Matrix3f {
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < d_cam.size(); ++i) {
        H += d_gal[i] * d_cam[i].transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0.0F) {
        Eigen::Matrix3f V = svd.matrixV();
        V.col(2) = -V.col(2);
        R = V * svd.matrixU().transpose();
    }
    return R;
}

auto rotation_to_euler(const Eigen::Matrix3f& R)
    -> std::tuple<float, float, float> {
    // R = Rz(-rz) * Ry(-rx) * Rx(-ry)  (camera→galvo convention)
    // Decompose into ZYX Euler: R = Rz(α) * Ry(β) * Rx(γ)
    // Then rz = -α, rx = -β, ry = -γ
    float sy = -R(2, 0);
    float cos_beta = std::sqrt(std::max(1.0F - sy * sy, 0.0F));
    float alpha, gamma;
    if (cos_beta > 1e-7F) {
        alpha = std::atan2(R(1, 0), R(0, 0));
        gamma = std::atan2(R(2, 1), R(2, 2));
    } else {
        alpha = 0.0F;
        gamma = std::atan2(-R(0, 1), R(1, 1));
    }
    float rx = -std::atan2(sy, cos_beta) * kRadToDeg;
    float ry = -gamma * kRadToDeg;
    float rz = -alpha * kRadToDeg;
    return {rx, ry, rz};
}

} // namespace

int main(int argc, char** argv) {
    const char* csv_path = argc > 1 ? argv[1] : "test_data/calib/pixel_calib_records.csv";
    const char* config_path = argc > 2 ? argv[2] : "config/calib_guidance.yaml";
    const char* calib_path = argc > 3 ? argv[3] : "config/camera_calib.yaml";

    auto recs = load_records(csv_path);
    if (recs.size() < 3) {
        std::println(stderr, "Need at least 3 records, got {}", recs.size());
        std::println(stderr, "CSV format: theta_x_deg, theta_y_deg, pixel_x, pixel_y");
        return 1;
    }
    std::println("Loaded {} records from {}", recs.size(), csv_path);

    auto K = load_intrinsics(calib_path);
    std::println("Camera intrinsics: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}", K.fx, K.fy, K.cx, K.cy);

    // Compute direction pairs
    std::vector<Eigen::Vector3f> d_cam, d_gal;
    for (const auto& r : recs) {
        d_cam.push_back(pixel_to_direction(r.pixel_x, r.pixel_y, K));
        d_gal.push_back(galvo_angles_to_direction(r.theta_x_deg, r.theta_y_deg));
    }

    // SVD solve for rotation
    Eigen::Matrix3f R = solve_wahba(d_cam, d_gal);
    auto [rx, ry, rz] = rotation_to_euler(R);
    Eigen::Quaternionf q(R);

    std::println("");
    std::println("=== WAHBA SVD ROTATION ===");
    std::println("r_x_deg: {:.2f}", rx);
    std::println("r_y_deg: {:.2f}", ry);
    std::println("r_z_deg: {:.2f}", rz);
    std::println("q: ({:.6f}, {:.6f}, {:.6f}, {:.6f})", q.w(), q.x(), q.y(), q.z());

    // Load translation from config (fixed, not optimized)
    auto config = load_config(config_path);
    auto init = config.guidance;
    std::println("");
    std::println("=== FIXED TRANSLATION (from config) ===");
    std::println("t_x_mm: {:.1f}", init.t_x_mm);
    std::println("t_y_mm: {:.1f}", init.t_y_mm);
    std::println("t_z_mm: {:.1f}", init.t_z_mm);

    // Compute direction-matching error
    double dir_err = 0.0;
    for (size_t i = 0; i < d_cam.size(); ++i) {
        Eigen::Vector3f pred = R * d_cam[i];
        double dot = std::max(-1.0, std::min(1.0, static_cast<double>(pred.dot(d_gal[i]))));
        dir_err += std::acos(dot) * kRadToDeg;
    }
    dir_err /= d_cam.size();
    std::println("");
    std::println("Direction error (mean angle): {:.3f}deg", dir_err);

    return 0;
}
