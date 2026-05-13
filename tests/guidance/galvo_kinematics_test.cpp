#include <cmath>
#include <cstdlib>
#include <print>

#include "guidance/galvo_kinematics.hpp"
#include "config.hpp"

namespace {

using namespace rmcs_laser_guidance;

constexpr float kRadToDeg = 180.0F / 3.14159265358979323846F;
constexpr float kEps = 0.1F;

#define CHECK(cond) do { if (!(cond)) { std::println("  FAIL: " #cond); std::abort(); } } while(0)

auto make_test_config(float tx = 0.0F, float ty = 0.0F, float tz = 0.0F, float d = 15.0F) -> GuidanceConfig {
    GuidanceConfig cfg;
    cfg.enabled = true;
    cfg.t_x_mm = tx;
    cfg.t_y_mm = ty;
    cfg.t_z_mm = tz;
    cfg.mirror_separation_mm = d;
    cfg.max_optical_angle_deg = 30.0F;
    return cfg;
}

void test_center_point() {
    auto cfg = make_test_config();
    GalvoKinematics kin(cfg);
    auto angles = kin.compute({0.0F, 0.0F, 10000.0F});
    CHECK(angles.valid);
    CHECK(std::abs(angles.theta_x_optical_deg) < kEps);
    CHECK(std::abs(angles.theta_y_optical_deg) < kEps);
    std::println("  center_point: OK");
}

void test_positive_x() {
    auto cfg = make_test_config();
    GalvoKinematics kin(cfg);
    float z = 10000.0F; float x = 500.0F;
    auto angles = kin.compute({x, 0.0F, z});
    CHECK(angles.valid);
    CHECK(angles.theta_x_optical_deg > 0.0F);
    CHECK(std::abs(angles.theta_y_optical_deg) < kEps);
    std::println("  positive_x: th_x={}deg", angles.theta_x_optical_deg);
}

void test_positive_y() {
    auto cfg = make_test_config();
    GalvoKinematics kin(cfg);
    float z = 10000.0F; float y = 500.0F;
    auto angles = kin.compute({0.0F, y, z});
    CHECK(angles.valid);
    CHECK(std::abs(angles.theta_x_optical_deg) < kEps);
    CHECK(angles.theta_y_optical_deg > 0.0F);
    std::println("  positive_y: th_y={}deg", angles.theta_y_optical_deg);
}

void test_extrinsic_offset() {
    auto cfg = make_test_config(-92.5F, 0.0F, 0.0F);
    GalvoKinematics kin(cfg);
    auto angles = kin.compute({-92.5F, 0.0F, 10000.0F});
    CHECK(angles.valid);
    CHECK(std::abs(angles.theta_x_optical_deg) < kEps);
    CHECK(std::abs(angles.theta_y_optical_deg) < kEps);
    std::println("  extrinsic_offset: OK (t_x=-92.5mm)");
}

void test_behind_camera() {
    GalvoKinematics kin(make_test_config());
    auto angles = kin.compute({0.0F, 0.0F, -100.0F});
    CHECK(!angles.valid);
    std::println("  behind_camera: OK");
}

void test_behind_galvo() {
    GalvoKinematics kin(make_test_config(0.0F, 0.0F, 50.0F));
    auto angles = kin.compute({0.0F, 0.0F, 30.0F});
    CHECK(!angles.valid);
    std::println("  behind_galvo: OK");
}

void test_mirror_separation_effect() {
    GalvoKinematics kin_no(make_test_config(0.0F, 0.0F, 0.0F, 0.0F));
    GalvoKinematics kin_with(make_test_config(0.0F, 0.0F, 0.0F, 100.0F));
    auto a_no = kin_no.compute({0.0F, 200.0F, 1000.0F});
    auto a_with = kin_with.compute({0.0F, 200.0F, 1000.0F});
    CHECK(a_no.valid && a_with.valid);
    CHECK(a_with.theta_y_optical_deg < a_no.theta_y_optical_deg);
    std::println("  mirror_separation_effect: d=0→{}deg d=100→{}deg",
                a_no.theta_y_optical_deg, a_with.theta_y_optical_deg);
}

} // namespace

int main() {
    std::println("galvo_kinematics_test:");
    test_center_point();
    test_positive_x();
    test_positive_y();
    test_extrinsic_offset();
    test_behind_camera();
    test_behind_galvo();
    test_mirror_separation_effect();
    std::println("PASSED");
    return 0;
}
