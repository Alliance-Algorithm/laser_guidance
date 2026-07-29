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

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <yaml-cpp/yaml.h>

#include "config.hpp"

namespace {

using namespace rmcs_laser_guidance;

constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;
constexpr float kRadToDeg = 180.0F / 3.14159265358979323846F;

// CSV: theta_x_deg, theta_y_deg, pixel_x, pixel_y [, depth_mm]
struct CalibRecord {
    float theta_x_deg = 0.0F;
    float theta_y_deg = 0.0F;
    float pixel_x = 0.0F;
    float pixel_y = 0.0F;
    float depth_mm = 0.0F;   // optional, 0 = unknown
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
        if (ss >> r.depth_mm) { /* optional 5th column */ }
        recs.push_back(r);
    }
    return recs;
}

// Load camera intrinsics from OpenCV-format YAML
struct CameraIntrinsics {
    float fx = 0.0F, fy = 0.0F, cx = 0.0F, cy = 0.0F;
    cv::Mat camera_matrix{};   // CV_64F 3x3, for cv::undistortPoints
    cv::Mat dist_coeffs{};     // CV_64F 1xN, empty = no distortion
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

    k.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            k.camera_matrix.at<double>(r, c) = mat[r][c].as<double>();

    const auto dc = calib["dist_coeffs"];
    if (dc && dc.size() > 0) {
        k.dist_coeffs = cv::Mat::zeros(1, static_cast<int>(dc.size()), CV_64F);
        for (int i = 0; i < static_cast<int>(dc.size()); ++i)
            k.dist_coeffs.at<double>(i) = dc[i].as<double>();
    }
    return k;
}

// Back-project pixel to unit direction in camera frame.
// Applies distortion correction (matching CameraProjection::project at runtime)
// so the calibrated R is consistent with how the runtime uses pixels.
auto pixel_to_direction(float px, float py, const CameraIntrinsics& K) -> Eigen::Vector3f {
    float ux = px, uy = py;
    if (!K.dist_coeffs.empty()) {
        std::vector<cv::Point2f> src{{px, py}}, dst;
        cv::undistortPoints(src, dst, K.camera_matrix, K.dist_coeffs,
                            cv::noArray(), K.camera_matrix);
        ux = dst[0].x;
        uy = dst[0].y;
    }
    Eigen::Vector3f d;
    d.x() = (ux - K.cx) / K.fx;
    d.y() = (uy - K.cy) / K.fy;
    d.z() = 1.0F;
    d.normalize();
    return d;
}

// Reverse mirror model: galvo optical angles → unit direction in galvo frame.
// Mirror model: θ_opt = 2 · θ_mech,  θ_mech = atan2(coord, ...)
// For far-field direction (z_eff → ∞): d = (tan(θx)/cos(θy), tan(θy), 1) normalized.
auto galvo_angles_to_direction(float theta_x_opt_deg, float theta_y_opt_deg) -> Eigen::Vector3f {
    float thx = theta_x_opt_deg * kDegToRad;
    float thy = theta_y_opt_deg * kDegToRad;
    float cx = std::cos(thx);
    float cy = std::cos(thy);
    // d_gal = (tan(θx)/cos(θy), tan(θy), 1) normalized
    Eigen::Vector3f d(std::tan(thx) / cy, std::tan(thy), 1.0F);
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

// Translation from coplanarity: det([t, R·d_cam, d_gal]) = 0, t_z=0.
// Returns direction (t_x, t_y) as unit vector. Magnitude indeterminate.
// If known_depth_mm > 0, uses it to fix scale: |t| ≈ known_depth * RMS angular error.
auto solve_translation_coplanar(const std::vector<Eigen::Vector3f>& d_cam,
                                const std::vector<Eigen::Vector3f>& d_gal,
                                const Eigen::Matrix3f& R,
                                float known_depth_mm) -> Eigen::Vector2f {
    if (d_cam.size() < 2) return {0.0F, 0.0F};

    // Build constraint: t · n_i = 0  where n = cross(R*d_cam, d_gal)
    Eigen::MatrixXf A(d_cam.size(), 2);
    for (size_t i = 0; i < d_cam.size(); ++i) {
        Eigen::Vector3f n = (R * d_cam[i]).cross(d_gal[i]);
        // Normalize to unit weight per observation
        float nr = n.norm();
        if (nr > 1e-9F) { n /= nr; }
        A(i, 0) = n.x();
        A(i, 1) = n.y();
    }

    // SVD of A: least-squares solution to A·[tx,ty]ᵀ = 0
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
    Eigen::Vector2f dir = svd.matrixV().col(1);  // smallest singular vector

    // Fix sign: t should put camera at physically measured position.
    // Use majority sign agreement from cross products.
    Eigen::Vector2f sign_ref{0.0F, 0.0F};
    for (size_t i = 0; i < d_cam.size(); ++i) {
        Eigen::Vector3f n = (R * d_cam[i]).cross(d_gal[i]).normalized();
        // t·n = 0, the direction of t is perpendicular to n
        // Use the sign from cross product projection
        Eigen::Vector2f ni(n.x(), n.y());
        sign_ref += ni;
    }
    if (dir.dot(sign_ref) < 0.0F) dir = -dir;

    // Estimate magnitude from known depth if available.
    // Approximate: |t| ≈ mean(|n_z|) * mean_depth / mean(|n_xy|)
    float scale = 10.0F;  // default order-of-magnitude: 10cm
    if (known_depth_mm > 0.0F) {
        float mean_nxy = 0.0F;
        float mean_nz = 0.0F;
        for (size_t i = 0; i < d_cam.size(); ++i) {
            Eigen::Vector3f n = (R * d_cam[i]).cross(d_gal[i]).normalized();
            mean_nxy += std::sqrt(n.x()*n.x() + n.y()*n.y());
            mean_nz += std::abs(n.z());
        }
        mean_nxy /= d_cam.size();
        mean_nz /= d_cam.size();
        if (mean_nxy > 1e-6F && mean_nz > 1e-6F)
            scale = known_depth_mm * mean_nz / mean_nxy;
    }

    return dir * scale;
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

    // Translation from coplanarity: estimate (t_x, t_y) only.
    // Runtime uses measured mount: t = galvo in camera frame (mm), e.g. t_z=20
    // when the camera sits 20mm behind the galvo. This solver still forces t_z=0
    // and is weak on |t|; prefer hand-measured t_x/t_y/t_z for geometry mode.
    float known_depth = 0.0F;
    for (const auto& r : recs)
        if (r.depth_mm > 0.0F) known_depth = r.depth_mm;
    if (known_depth == 0.0F)
        known_depth = 5000.0F;  // 5m default

    Eigen::Vector2f t_xy = solve_translation_coplanar(d_cam, d_gal, R, known_depth);

    auto config = load_config(config_path);
    auto init = config.guidance;
    std::println("");
    std::println("=== TRANSLATION (coplanar SVD; t_z fixed 0 — prefer measured mount) ===");
    std::println("t_x_mm: {:.1f}  (runtime prior t_x={:.1f})", t_xy.x(), init.t_x_mm);
    std::println("t_y_mm: {:.1f}  (runtime prior t_y={:.1f})", t_xy.y(), init.t_y_mm);
    std::println("t_z_mm: 0.0  (runtime prior t_z={:.1f} mm)", init.t_z_mm);

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
