#include "internal/rtp_streamer.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <mutex>
#include <print>
#include <string>
#include <thread>

namespace rmcs_laser_guidance {

struct RtpStreamer::Details {
    RtpConfig config;
    std::FILE* pipe = nullptr;
    std::thread writer;
    std::mutex mtx;
    cv::Mat latest_frame;
    uint64_t frame_seq = 0;
    uint64_t written_seq = 0;
    bool running = false;
};

RtpStreamer::RtpStreamer(RtpConfig config)
    : details_(std::make_unique<Details>()) {
    details_->config = std::move(config);
}

RtpStreamer::~RtpStreamer() {
    stop();
}

auto RtpStreamer::start(const int width, const int height, const float framerate) -> bool {
    if (details_->pipe) return false;
    if (!details_->config.enabled) return false;

    const std::string command = std::format(
        "ffmpeg -loglevel error "
        "-f rawvideo -pixel_format bgr24 -video_size {}x{} -framerate {} -i pipe:0 "
        "-c:v {} -preset ultrafast -tune zerolatency "
        "-x264-params \"keyint=10:min-keyint=10:scenecut=0:repeat-headers=1\" "
        "-f rtp -sdp_file \"{}\" \"rtp://{}:{}\"",
        width, height, framerate, details_->config.encoder,
        details_->config.sdp_path.string(), details_->config.host, details_->config.port);

    details_->pipe = popen(command.c_str(), "w");
    if (!details_->pipe) {
        std::println(stderr, "RTP streamer: failed to launch ffmpeg: {}", strerror(errno));
        return false;
    }

    std::println("RTP streaming started: {}x{} -> rtp://{}:{}, SDP={}, encoder={}",
                 width, height, details_->config.host, details_->config.port,
                 details_->config.sdp_path.string(), details_->config.encoder);

    details_->running = true;
    details_->writer = std::thread([this] {
        uint64_t frame_count = 0;

        while (details_->running) {
            cv::Mat frame;
            {
                std::scoped_lock lock(details_->mtx);
                if (details_->frame_seq == details_->written_seq) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                if (details_->latest_frame.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                frame = details_->latest_frame.clone();
                details_->written_seq = details_->frame_seq;
            }

            const std::size_t size = frame.total() * frame.elemSize();
            if (std::fwrite(frame.data, 1, size, details_->pipe) != size) {
                std::println(stderr, "RTP streamer: pipe write failed (ffmpeg exited?)");
                details_->running = false;
                break;
            }
            frame_count++;

            if (frame_count % 300 == 0) {
                std::fprintf(stderr, "RTP streamer: %lu frames sent\n",
                             static_cast<unsigned long>(frame_count));
            }
        }
    });

    return true;
}

auto RtpStreamer::push(const cv::Mat& bgr_frame) -> void {
    if (!details_->running) return;
    if (bgr_frame.empty()) return;
    if (bgr_frame.type() != CV_8UC3) return;
    std::scoped_lock lock(details_->mtx);
    details_->latest_frame = bgr_frame.clone();
    details_->frame_seq++;
}

auto RtpStreamer::stop() -> void {
    details_->running = false;
    if (details_->writer.joinable())
        details_->writer.join();
    if (details_->pipe) {
        const int rc = pclose(details_->pipe);
        if (rc != 0) {
            std::println(stderr, "RTP streamer: ffmpeg exited with code {}", rc);
        }
        details_->pipe = nullptr;
    }
    std::println("RTP streaming stopped");
}

auto RtpStreamer::is_active() const -> bool {
    return details_->running;
}

}
