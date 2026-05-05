#include "internal/rtp_streamer.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <print>
#include <stdexcept>
#include <string>

namespace rmcs_laser_guidance {

RtpStreamer::RtpStreamer(RtpConfig config)
    : config_(std::move(config)) {}

RtpStreamer::~RtpStreamer() {
    stop();
}

auto RtpStreamer::start(const int width, const int height, const float framerate) -> bool {
    if (pipe_) return false;
    if (!config_.enabled) return false;
    return launch_ffmpeg(width, height, framerate);
}

auto RtpStreamer::launch_ffmpeg(const int width, const int height, const float framerate) -> bool {
    const std::string command = std::format(
        "ffmpeg -loglevel error "
        "-f rawvideo -pixel_format bgr24 -video_size {}x{} -framerate {} -i pipe:0 "
        "-c:v libx264 -preset ultrafast -tune zerolatency "
        "-x264-params \"keyint=10:min-keyint=10:scenecut=0\" "
        "-f rtp -sdp_file \"{}\" rtp://{}:{}",
        width, height, framerate,
        config_.sdp_path.string(), config_.host, config_.port);

    pipe_ = popen(command.c_str(), "w");
    if (!pipe_) return false;
    std::println("RTP streaming started: {} -> rtp://{}:{}, SDP={}",
                 width, config_.host, config_.port, config_.sdp_path.string());
    return true;
}

auto RtpStreamer::push(const cv::Mat& bgr_frame) -> void {
    if (!pipe_) return;
    if (bgr_frame.empty()) return;
    if (bgr_frame.type() != CV_8UC3) return;

    const std::size_t size = bgr_frame.total() * bgr_frame.elemSize();
    if (std::fwrite(bgr_frame.data, 1, size, pipe_) != size) {
        std::println(stderr, "RTP streaming: ffmpeg pipe write failed, stopping");
        stop();
    }
}

auto RtpStreamer::stop() -> void {
    if (!pipe_) return;
    pclose(pipe_);
    pipe_ = nullptr;
    std::println("RTP streaming stopped");
}

auto RtpStreamer::is_active() const -> bool {
    return pipe_ != nullptr;
}

}
