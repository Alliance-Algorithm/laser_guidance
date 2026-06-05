#pragma once

#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "config.hpp"
#include "tracking/ekf_tracker.hpp"
#include "tracking/hit_progress.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

auto class_color(int class_id) -> cv::Scalar;
auto class_name(int class_id) -> std::string;
auto draw_candidates(cv::Mat& image, const std::vector<ModelCandidate>& candidates) -> void;
auto draw_ekf_state(cv::Mat& image, const EkfState& state) -> void;
auto draw_guidance_status(cv::Mat& image, bool guidance_active,
                          bool ekf_ok, bool depth_ok, const std::string& msg) -> void;
auto draw_status_bar(cv::Mat& image, bool streaming, bool recording,
                     int enemy_class_id, bool using_trt) -> void;
auto draw_hit_progress(cv::Mat& image, const HitProgress& hp) -> void;

class DebugRenderer {
public:
    explicit DebugRenderer(const DebugConfig& debug_config);

    auto draw(cv::Mat& image, const TargetObservation& observation) const -> void;
    auto draw_ekf_state(cv::Mat& image, const EkfState& state) const -> void;

private:
    DebugConfig debug_;
};

} // namespace rmcs_laser_guidance
