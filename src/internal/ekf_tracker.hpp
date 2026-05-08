#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

struct EkfState {
    cv::Point2f position{-1.0F, -1.0F};
    cv::Point2f velocity{0.0F, 0.0F};
    cv::Point2f acceleration{0.0F, 0.0F};
    bool initialized = false;
    bool lost = false;
    int missed_frames = 0;
    double dt_seconds = 0.0;
};

class EkfTracker {
public:
    explicit EkfTracker(EkfConfig config = {});
    ~EkfTracker();

    EkfTracker(const EkfTracker&) = delete;
    auto operator=(const EkfTracker&) -> EkfTracker& = delete;
    EkfTracker(EkfTracker&&) noexcept;
    auto operator=(EkfTracker&&) noexcept -> EkfTracker&;

    auto predict(Clock::time_point timestamp) -> void;
    auto update(const cv::Point2f& measurement) -> void;
    auto process(const cv::Point2f& measurement, Clock::time_point timestamp) -> void;

    [[nodiscard]] auto state() const -> EkfState;
    [[nodiscard]] auto is_initialized() const -> bool;
    [[nodiscard]] auto is_lost() const -> bool;
    auto reset() -> void;

private:
    struct Details;
    std::unique_ptr<Details> details_;
};

}
