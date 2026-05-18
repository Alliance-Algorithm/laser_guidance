#include <print>

#include "config.hpp"
#include "guidance/lidar_depth_estimator.hpp"
#include "test_utils.hpp"
#include "types.hpp"

namespace {

using namespace rmcs_laser_guidance;
using namespace rmcs_laser_guidance::tests;

auto make_config() -> GuidanceConfig {
    GuidanceConfig cfg;
    cfg.depth_source = GuidanceDepthSourceKind::lidar_target_cluster;
    cfg.lidar_bbox_margin_px = 0.0F;
    cfg.lidar_cluster_tolerance_mm = 80.0F;
    cfg.lidar_min_cluster_points = 3;
    cfg.lidar_max_depth_mm = 5000.0F;
    return cfg;
}

auto make_candidate() -> ModelCandidate {
    ModelCandidate cand;
    cand.bbox = cv::Rect2f {95.0F, 45.0F, 20.0F, 20.0F};
    cand.center = cv::Point2f {105.0F, 55.0F};
    cand.score = 0.9F;
    cand.class_id = 0;
    return cand;
}

void test_prefers_compact_centered_cluster() {
    LidarFrame frame;
    frame.points = {
        {100.0F, 20.0F, 2030.0F, 220.0F, 52, 102},
        {110.0F, 10.0F, 2010.0F, 230.0F, 54, 104},
        {95.0F, 18.0F, 2020.0F, 225.0F, 56, 106},
        {380.0F, 220.0F, 1980.0F, 80.0F, 50, 99},
        {430.0F, 240.0F, 1990.0F, 85.0F, 58, 110},
        {410.0F, 210.0F, 1975.0F, 78.0F, 60, 112},
    };

    LidarDepthEstimator estimator(make_config());
    const auto depth = estimator.estimate(make_candidate(), frame);
    require(depth.has_value(), "expected lidar cluster depth");
    require_near(*depth, 2020.0F, 15.0F, "centered compact cluster should win");
}

void test_rejects_body_like_thick_cluster() {
    LidarFrame frame;
    frame.points = {
        {100.0F, 20.0F, 2000.0F, 220.0F, 52, 102},
        {108.0F, 18.0F, 2010.0F, 215.0F, 54, 104},
        {96.0F, 16.0F, 2020.0F, 225.0F, 56, 106},
        {100.0F, 20.0F, 2000.0F, 150.0F, 53, 103},
        {100.0F, 20.0F, 2400.0F, 150.0F, 53, 103},
        {100.0F, 20.0F, 2800.0F, 150.0F, 53, 103},
    };

    LidarDepthEstimator estimator(make_config());
    const auto depth = estimator.estimate(make_candidate(), frame);
    require(depth.has_value(), "thin target-like cluster should survive thick body cluster");
    require_near(*depth, 2010.0F, 20.0F, "target-like cluster depth mismatch");
}

void test_requires_min_cluster_points() {
    auto cfg = make_config();
    cfg.lidar_min_cluster_points = 4;
    LidarDepthEstimator estimator(cfg);

    LidarFrame frame;
    frame.points = {
        {100.0F, 20.0F, 2030.0F, 220.0F, 52, 102},
        {110.0F, 10.0F, 2010.0F, 230.0F, 54, 104},
        {95.0F, 18.0F, 2020.0F, 225.0F, 56, 106},
    };

    const auto depth = estimator.estimate(make_candidate(), frame);
    require(!depth.has_value(), "cluster under min size should fail");
}

void test_filters_outside_bbox() {
    LidarDepthEstimator estimator(make_config());

    LidarFrame frame;
    frame.points = {
        {100.0F, 20.0F, 2030.0F, 220.0F, 10, 10},
        {110.0F, 10.0F, 2010.0F, 230.0F, 11, 11},
        {95.0F, 18.0F, 2020.0F, 225.0F, 12, 12},
    };

    const auto depth = estimator.estimate(make_candidate(), frame);
    require(!depth.has_value(), "points outside bbox ROI should be ignored");
}

} // namespace

int main() {
    std::println("lidar_depth_estimator_test:");
    test_prefers_compact_centered_cluster();
    test_rejects_body_like_thick_cluster();
    test_requires_min_cluster_points();
    test_filters_outside_bbox();
    std::println("PASSED");
    return 0;
}
