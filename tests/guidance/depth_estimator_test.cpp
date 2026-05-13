#include <cassert>
#include <cmath>
#include <print>

#include <opencv2/core.hpp>

#include "guidance/depth_estimator.hpp"
#include "config.hpp"
#include "types.hpp"

namespace {

using namespace rmcs_laser_guidance;

auto make_test_config(float target_width_mm = 150.0F) -> GuidanceConfig {
    GuidanceConfig cfg;
    cfg.enabled = true;
    cfg.target_geometry = {{ .class_id = 0, .width_mm = target_width_mm, .height_mm = target_width_mm }};
    return cfg;
}

auto make_k_matrix(double fx = 2000.0, double fy = 2000.0, double cx = 960.0, double cy = 540.0) -> cv::Mat {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

void test_depth_basic() {
    auto cfg = make_test_config(150.0F);
    auto K = make_k_matrix(2000.0, 2000.0);
    DepthEstimator estimator(cfg, K);

    ModelCandidate candidate;
    candidate.class_id = 0;
    candidate.bbox = cv::Rect2f{400, 200, 15.0F, 15.0F};
    candidate.score = 0.9F;

    auto depth = estimator.estimate(candidate);
    assert(depth.has_value());
    float expected = 2000.0F * 150.0F / 15.0F;
    assert(std::abs(*depth - expected) < 1.0F);

    std::println("  depth_basic: {}mm (expected ~{}mm)", *depth, expected);
}

void test_depth_uses_max_dim() {
    auto cfg = make_test_config(150.0F);
    auto K = make_k_matrix(2000.0, 2000.0);
    DepthEstimator estimator(cfg, K);

    ModelCandidate candidate;
    candidate.class_id = 0;
    candidate.bbox = cv::Rect2f{0, 0, 10.0F, 30.0F};
    float expected = 2000.0F * 150.0F / 30.0F;

    auto depth = estimator.estimate(candidate);
    assert(depth.has_value());
    assert(std::abs(*depth - expected) < 1.0F);

    std::println("  depth_uses_max_dim: OK");
}

void test_depth_unknown_class() {
    auto cfg = make_test_config(150.0F);
    auto K = make_k_matrix();
    DepthEstimator estimator(cfg, K);

    ModelCandidate candidate;
    candidate.class_id = 99;
    candidate.bbox = cv::Rect2f{0, 0, 10.0F, 10.0F};

    auto depth = estimator.estimate(candidate);
    assert(depth.has_value());
    float expected = 2000.0F * 150.0F / 10.0F;
    assert(std::abs(*depth - expected) < 1.0F);

    std::println("  depth_unknown_class: OK (falls back to default)");
}

void test_depth_zero_size() {
    auto cfg = make_test_config();
    auto K = make_k_matrix();
    DepthEstimator estimator(cfg, K);

    ModelCandidate candidate;
    candidate.class_id = 0;
    candidate.bbox = cv::Rect2f{0, 0, 0.0F, 0.0F};

    auto depth = estimator.estimate(candidate);
    assert(!depth.has_value());

    std::println("  depth_zero_size: OK");
}

void test_depth_invalid_fx() {
    auto cfg = make_test_config();
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    K.at<double>(0, 0) = 0.0;
    K.at<double>(1, 1) = 2000.0;

    DepthEstimator estimator(cfg, K);

    ModelCandidate candidate;
    candidate.class_id = 0;
    candidate.bbox = cv::Rect2f{0, 0, 10.0F, 10.0F};

    auto depth = estimator.estimate(candidate);
    assert(!depth.has_value());

    std::println("  depth_invalid_fx: OK");
}

void test_depth_large_distance() {
    auto cfg = make_test_config(200.0F);
    auto K = make_k_matrix(3000.0, 3000.0);
    DepthEstimator estimator(cfg, K);

    ModelCandidate candidate;
    candidate.class_id = 0;
    candidate.bbox = cv::Rect2f{0, 0, 3.0F, 3.0F};

    auto depth = estimator.estimate(candidate);
    assert(depth.has_value());
    float expected = 3000.0F * 200.0F / 3.0F;
    assert(std::abs(*depth - expected) < 1.0F);
    assert(*depth > 100000.0F);

    std::println("  depth_large_distance: {}mm", *depth);
}

} // namespace

int main() {
    std::println("depth_estimator_test:");
    test_depth_basic();
    test_depth_uses_max_dim();
    test_depth_unknown_class();
    test_depth_zero_size();
    test_depth_invalid_fx();
    test_depth_large_distance();
    std::println("PASSED");
    return 0;
}
