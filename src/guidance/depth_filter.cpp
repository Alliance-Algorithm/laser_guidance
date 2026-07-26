#include "guidance/depth_filter.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/core/mat.hpp>

namespace rmcs_laser_guidance {
namespace {

constexpr int kStateDim = 2; // [depth_mm, velocity_mm_s]

auto squared(const double value) -> double { return value * value; }

} // namespace

struct DepthFilter::Details {
    explicit Details(GuidanceConfig cfg)
        : config(std::move(cfg))
        , x(cv::Mat::zeros(kStateDim, 1, CV_64F))
        , p(cv::Mat::zeros(kStateDim, kStateDim, CV_64F)) {
        reset_covariance();
    }

    auto reset_covariance() -> void {
        p = cv::Mat::zeros(kStateDim, kStateDim, CV_64F);
        p.at<double>(0, 0) = squared(config.depth_initial_pos_std);
        p.at<double>(1, 1) = squared(config.depth_initial_vel_std);
    }

    auto build_f(const double dt) const -> cv::Mat {
        cv::Mat f = cv::Mat::eye(kStateDim, kStateDim, CV_64F);
        f.at<double>(0, 1) = dt;
        return f;
    }

    auto build_q(const double dt) const -> cv::Mat {
        const double q = std::max(config.depth_process_noise_q, 1e-12);
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;

        cv::Mat q_mat = cv::Mat::zeros(kStateDim, kStateDim, CV_64F);
        q_mat.at<double>(0, 0) = dt4 / 4.0;
        q_mat.at<double>(0, 1) = dt3 / 2.0;
        q_mat.at<double>(1, 0) = dt3 / 2.0;
        q_mat.at<double>(1, 1) = dt2;
        return q * q_mat;
    }

    GuidanceConfig config;
    cv::Mat x;
    cv::Mat p;
    bool initialized = false;
    int missed_frames = 0;
};

DepthFilter::DepthFilter(GuidanceConfig config)
    : details_(std::make_unique<Details>(std::move(config))) {}

DepthFilter::~DepthFilter() = default;

DepthFilter::DepthFilter(DepthFilter&&) noexcept = default;

auto DepthFilter::operator=(DepthFilter&&) noexcept -> DepthFilter& = default;

auto DepthFilter::predict(const double dt_seconds) -> void {
    if (!details_->initialized || dt_seconds <= 0.0)
        return;

    const cv::Mat f = details_->build_f(dt_seconds);
    const cv::Mat q = details_->build_q(dt_seconds);
    details_->x = f * details_->x;
    details_->p = f * details_->p * f.t() + q;

    ++details_->missed_frames;
    if (details_->missed_frames > details_->config.depth_max_missed_frames)
        reset();
}

auto DepthFilter::update(const float measured_depth_mm) -> void {
    if (!details_->initialized) {
        details_->x.at<double>(0, 0) = static_cast<double>(measured_depth_mm);
        details_->x.at<double>(1, 0) = 0.0;
        details_->reset_covariance();
        details_->initialized = true;
        details_->missed_frames = 0;
        return;
    }

    const double r = std::max(details_->config.depth_measurement_noise_r, 1e-9);
    const double innovation = static_cast<double>(measured_depth_mm) - details_->x.at<double>(0, 0);
    const double s = details_->p.at<double>(0, 0) + r;
    const double k0 = details_->p.at<double>(0, 0) / s;
    const double k1 = details_->p.at<double>(1, 0) / s;

    details_->x.at<double>(0, 0) += k0 * innovation;
    details_->x.at<double>(1, 0) += k1 * innovation;

    const double p00 = details_->p.at<double>(0, 0);
    const double p01 = details_->p.at<double>(0, 1);
    const double p10 = details_->p.at<double>(1, 0);
    const double p11 = details_->p.at<double>(1, 1);
    details_->p.at<double>(0, 0) = p00 - k0 * p00;
    details_->p.at<double>(0, 1) = p01 - k0 * p01;
    details_->p.at<double>(1, 0) = p10 - k1 * p00;
    details_->p.at<double>(1, 1) = p11 - k1 * p01;

    details_->missed_frames = 0;
}

auto DepthFilter::process(const float measured_depth_mm, const double dt_seconds) -> void {
    predict(dt_seconds);
    update(measured_depth_mm);
}

auto DepthFilter::state() const -> DepthFilterState {
    DepthFilterState state;
    state.initialized = details_->initialized;
    if (!details_->initialized)
        return state;

    state.depth_mm = static_cast<float>(details_->x.at<double>(0, 0));
    state.velocity_mm_s = static_cast<float>(details_->x.at<double>(1, 0));
    return state;
}

auto DepthFilter::is_initialized() const -> bool { return details_->initialized; }

auto DepthFilter::reset() -> void {
    details_->x = cv::Mat::zeros(kStateDim, 1, CV_64F);
    details_->reset_covariance();
    details_->initialized = false;
    details_->missed_frames = 0;
}

} // namespace rmcs_laser_guidance
