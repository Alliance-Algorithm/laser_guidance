#include "internal/ekf_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace rmcs_laser_guidance {
namespace {

    constexpr int kStateDim = 6;
    constexpr int kMeasDim  = 2;

    auto squared(const double value) -> double {
        return value * value;
    }

} // namespace

struct EkfTracker::Details {
    explicit Details(EkfConfig cfg)
        : config(std::move(cfg))
        , x(cv::Mat::zeros(kStateDim, 1, CV_64F))
        , p(cv::Mat::zeros(kStateDim, kStateDim, CV_64F))
        , h(cv::Mat::zeros(kMeasDim, kStateDim, CV_64F))
        , r(cv::Mat::zeros(kMeasDim, kMeasDim, CV_64F)) {
        h.at<double>(0, 0) = 1.0;
        h.at<double>(1, 1) = 1.0;
        r.at<double>(0, 0) = std::max(config.measurement_noise_r, 1e-9);
        r.at<double>(1, 1) = std::max(config.measurement_noise_r, 1e-9);
        reset_covariance();
    }

    auto reset_covariance() -> void {
        p = cv::Mat::zeros(kStateDim, kStateDim, CV_64F);
        p.at<double>(0, 0) = squared(config.initial_pos_std);
        p.at<double>(1, 1) = squared(config.initial_pos_std);
        p.at<double>(2, 2) = squared(config.initial_vel_std);
        p.at<double>(3, 3) = squared(config.initial_vel_std);
        p.at<double>(4, 4) = squared(config.initial_acc_std);
        p.at<double>(5, 5) = squared(config.initial_acc_std);
    }

    auto build_f(const double dt) const -> cv::Mat {
        cv::Mat f = cv::Mat::eye(kStateDim, kStateDim, CV_64F);
        const double dt2 = dt * dt;
        f.at<double>(0, 2) = dt;
        f.at<double>(0, 4) = 0.5 * dt2;
        f.at<double>(1, 3) = dt;
        f.at<double>(1, 5) = 0.5 * dt2;
        f.at<double>(2, 4) = dt;
        f.at<double>(3, 5) = dt;
        return f;
    }

    auto build_q(const double dt) const -> cv::Mat {
        const double q = std::max(config.process_noise_q, 1e-12);
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        const double dt5 = dt4 * dt;

        cv::Mat q_mat = cv::Mat::zeros(kStateDim, kStateDim, CV_64F);
        q_mat.at<double>(0, 0) = dt5 / 20.0;
        q_mat.at<double>(0, 2) = dt4 / 8.0;
        q_mat.at<double>(0, 4) = dt3 / 6.0;
        q_mat.at<double>(1, 1) = dt5 / 20.0;
        q_mat.at<double>(1, 3) = dt4 / 8.0;
        q_mat.at<double>(1, 5) = dt3 / 6.0;

        q_mat.at<double>(2, 0) = dt4 / 8.0;
        q_mat.at<double>(2, 2) = dt3 / 3.0;
        q_mat.at<double>(2, 4) = dt2 / 2.0;
        q_mat.at<double>(3, 1) = dt4 / 8.0;
        q_mat.at<double>(3, 3) = dt3 / 3.0;
        q_mat.at<double>(3, 5) = dt2 / 2.0;

        q_mat.at<double>(4, 0) = dt3 / 6.0;
        q_mat.at<double>(4, 2) = dt2 / 2.0;
        q_mat.at<double>(4, 4) = dt;
        q_mat.at<double>(5, 1) = dt3 / 6.0;
        q_mat.at<double>(5, 3) = dt2 / 2.0;
        q_mat.at<double>(5, 5) = dt;
        return q * q_mat;
    }

    EkfConfig config;
    cv::Mat x;
    cv::Mat p;
    cv::Mat h;
    cv::Mat r;
    Clock::time_point last_timestamp { };
    bool initialized = false;
    bool lost = false;
    int missed_frames = 0;
    double last_dt_seconds = 0.0;
};

EkfTracker::EkfTracker(EkfConfig config)
    : details_(std::make_unique<Details>(std::move(config))) { }

EkfTracker::~EkfTracker() = default;

EkfTracker::EkfTracker(EkfTracker&&) noexcept = default;

auto EkfTracker::operator=(EkfTracker&&) noexcept -> EkfTracker& = default;

auto EkfTracker::predict(const Clock::time_point timestamp) -> void {
    if (!details_->initialized) {
        details_->last_timestamp = timestamp;
        details_->last_dt_seconds = 0.0;
        return;
    }

    if (details_->last_timestamp == Clock::time_point { }) {
        details_->last_timestamp = timestamp;
        details_->last_dt_seconds = 0.0;
        return;
    }

    const double dt = std::chrono::duration<double>(timestamp - details_->last_timestamp).count();
    details_->last_timestamp = timestamp;
    details_->last_dt_seconds = dt > 0.0 ? dt : 0.0;
    if (dt <= 0.0) return;

    const cv::Mat f = details_->build_f(dt);
    const cv::Mat q = details_->build_q(dt);
    details_->x = f * details_->x;
    details_->p = f * details_->p * f.t() + q;

    ++details_->missed_frames;
    details_->lost = details_->missed_frames > details_->config.max_missed_frames;
}

auto EkfTracker::update(const cv::Point2f& measurement) -> void {
    if (!details_->initialized) {
        details_->x.at<double>(0, 0) = static_cast<double>(measurement.x);
        details_->x.at<double>(1, 0) = static_cast<double>(measurement.y);
        details_->x.at<double>(2, 0) = 0.0;
        details_->x.at<double>(3, 0) = 0.0;
        details_->x.at<double>(4, 0) = 0.0;
        details_->x.at<double>(5, 0) = 0.0;
        details_->reset_covariance();
        details_->initialized = true;
        details_->lost = false;
        details_->missed_frames = 0;
        return;
    }

    cv::Mat z = cv::Mat::zeros(kMeasDim, 1, CV_64F);
    z.at<double>(0, 0) = static_cast<double>(measurement.x);
    z.at<double>(1, 0) = static_cast<double>(measurement.y);

    const cv::Mat innovation = z - details_->h * details_->x;
    const cv::Mat s = details_->h * details_->p * details_->h.t() + details_->r;
    const cv::Mat k = details_->p * details_->h.t() * s.inv(cv::DECOMP_SVD);
    details_->x = details_->x + k * innovation;

    const cv::Mat identity = cv::Mat::eye(kStateDim, kStateDim, CV_64F);
    details_->p = (identity - k * details_->h) * details_->p;

    details_->missed_frames = 0;
    details_->lost = false;
}

auto EkfTracker::process(const cv::Point2f& measurement, const Clock::time_point timestamp) -> void {
    predict(timestamp);
    update(measurement);
}

auto EkfTracker::state() const -> EkfState {
    EkfState state;
    state.initialized = details_->initialized;
    state.lost = details_->lost;
    state.missed_frames = details_->missed_frames;
    state.dt_seconds = details_->last_dt_seconds;
    if (!details_->initialized) return state;

    state.position.x = static_cast<float>(details_->x.at<double>(0, 0));
    state.position.y = static_cast<float>(details_->x.at<double>(1, 0));
    state.velocity.x = static_cast<float>(details_->x.at<double>(2, 0));
    state.velocity.y = static_cast<float>(details_->x.at<double>(3, 0));
    state.acceleration.x = static_cast<float>(details_->x.at<double>(4, 0));
    state.acceleration.y = static_cast<float>(details_->x.at<double>(5, 0));
    return state;
}

auto EkfTracker::is_initialized() const -> bool {
    return details_->initialized;
}

auto EkfTracker::is_lost() const -> bool {
    return details_->lost;
}

auto EkfTracker::reset() -> void {
    details_->x = cv::Mat::zeros(kStateDim, 1, CV_64F);
    details_->reset_covariance();
    details_->last_timestamp = Clock::time_point { };
    details_->initialized = false;
    details_->lost = false;
    details_->missed_frames = 0;
    details_->last_dt_seconds = 0.0;
}

}
