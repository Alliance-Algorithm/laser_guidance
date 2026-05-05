#pragma once

#include <cstdio>
#include <string>

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
    auto stop() -> void;
    [[nodiscard]] auto is_active() const -> bool;

private:
    auto launch_ffmpeg(int width, int height, float framerate) -> bool;

    RtpConfig config_;
    std::FILE* pipe_ = nullptr;
};

}
