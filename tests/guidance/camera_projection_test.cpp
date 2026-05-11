#include <cassert>
#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>

#include "guidance/camera_projection.hpp"

namespace {

using namespace rmcs_laser_guidance;

auto make_k_matrix(double fx = 2000.0, double fy = 2000.0, double cx = 960.0, double cy = 540.0) -> cv::Mat {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

void test_project_center() {
    auto K = make_k_matrix(2000.0, 2000.0, 960.0, 540.0);
    CameraProjection proj(K, cv::Mat{});

    cv::Point2f pixel{960.0F, 540.0F};
    auto P = proj.project(pixel, 10000.0F);

    assert(std::abs(P.x) < 1.0F);
    assert(std::abs(P.y) < 1.0F);
    assert(std::abs(P.z - 10000.0F) < 1.0F);

    std::cout << "  project_center: P=(" << P.x << "," << P.y << "," << P.z << ")" << std::endl;
}

void test_project_offset() {
    auto K = make_k_matrix(2000.0, 2000.0, 960.0, 540.0);
    CameraProjection proj(K, cv::Mat{});

    cv::Point2f pixel{1060.0F, 540.0F};
    auto P = proj.project(pixel, 10000.0F);

    float expected_x = (1060.0F - 960.0F) * 10000.0F / 2000.0F;
    assert(std::abs(P.x - expected_x) < 0.1F);
    assert(std::abs(P.y) < 0.1F);

    std::cout << "  project_offset: P.x=" << P.x << " (expected " << expected_x << ")" << std::endl;
}

void test_project_zero_depth() {
    CameraProjection proj(make_k_matrix(), cv::Mat{});
    auto P = proj.project({100, 200}, 0.0F);
    assert(P.x == -1.0F && P.y == -1.0F && P.z == -1.0F);
    std::cout << "  project_zero_depth: OK" << std::endl;
}

void test_project_negative_depth() {
    CameraProjection proj(make_k_matrix(), cv::Mat{});
    auto P = proj.project({100, 200}, -5.0F);
    assert(P.x == -1.0F && P.y == -1.0F && P.z == -1.0F);
    std::cout << "  project_negative_depth: OK" << std::endl;
}

void test_project_different_f() {
    auto K = make_k_matrix(1500.0, 1800.0);
    CameraProjection proj(K, cv::Mat{});

    cv::Point2f pixel{1260.0F, 240.0F};
    auto P = proj.project(pixel, 5000.0F);

    float expected_x = (1260.0F - 960.0F) * 5000.0F / 1500.0F;
    float expected_y = (240.0F - 540.0F) * 5000.0F / 1800.0F;

    assert(std::abs(P.x - expected_x) < 0.1F);
    assert(std::abs(P.y - expected_y) < 0.1F);
    assert(std::abs(P.z - 5000.0F) < 0.1F);

    std::cout << "  project_different_f: OK" << std::endl;
}

void test_camera_matrix_access() {
    auto K = make_k_matrix(1234.0, 5678.0);
    CameraProjection proj(K, cv::Mat{});
    const auto& stored = proj.camera_matrix();
    assert(std::abs(stored.at<double>(0, 0) - 1234.0) < 0.01);
    assert(std::abs(stored.at<double>(1, 1) - 5678.0) < 0.01);
    std::cout << "  camera_matrix_access: OK" << std::endl;
}

} // namespace

int main() {
    std::cout << "camera_projection_test:" << std::endl;
    test_project_center();
    test_project_offset();
    test_project_zero_depth();
    test_project_negative_depth();
    test_project_different_f();
    test_camera_matrix_access();
    std::cout << "PASSED" << std::endl;
    return 0;
}
