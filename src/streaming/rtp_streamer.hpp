#pragma once

#include <memory>

#include <opencv2/core/mat.hpp>

#include "config.hpp"

namespace rmcs_laser_guidance {

class RtpStreamer {
public:
    explicit RtpStreamer(RtpConfig config);
    ~RtpStreamer();

    RtpStreamer(const RtpStreamer&) = delete;
    auto operator=(const RtpStreamer&) -> RtpStreamer& = delete;

    auto start(int width, int height, float framerate) -> bool;
    auto push(const cv::Mat& bgr_frame) -> void;
    auto push(cv::Mat&& bgr_frame) -> void;
    auto stop() -> void;
    [[nodiscard]] auto is_active() const -> bool;

private:
    struct Details;
    std::unique_ptr<Details> details_;
};

}
