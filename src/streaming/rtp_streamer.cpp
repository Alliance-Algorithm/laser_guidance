#include "streaming/rtp_streamer.hpp"

#include <condition_variable>
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
    std::condition_variable cv;
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

    const auto& encoder = details_->config.encoder;
    std::string encoder_opts;
    if (encoder.find("nvenc") != std::string::npos) {
        encoder_opts = "-preset p1 -tune ll -rc cbr -g 2 -bf 0";
    } else {
        encoder_opts = "-preset ultrafast -tune zerolatency "
                       "-x264-params \"keyint=2:min-keyint=2:scenecut=0:repeat-headers=1\"";
    }

    const std::string command = std::format(
        "ffmpeg -loglevel error "
        "-f rawvideo -pixel_format bgr24 -video_size {}x{} -framerate {} -i pipe:0 "
        "-c:v {} {} "
        "-f rtp -sdp_file \"{}\" \"rtp://{}:{}\"",
        width, height, framerate, encoder, encoder_opts,
        details_->config.sdp_path.string(), details_->config.host, details_->config.port);

    details_->pipe = popen(command.c_str(), "w");
    if (!details_->pipe) {
        std::println(stderr, "RTP streamer: failed to launch ffmpeg: {}", strerror(errno));
        return false;
    }

    std::println("RTP streaming started: {}x{} -> rtp://{}:{}, SDP={}, encoder={}",
                 width, height, details_->config.host, details_->config.port,
                 details_->config.sdp_path.string(), encoder);

    details_->running = true;
    details_->writer = std::thread([this] {
        uint64_t frame_count = 0;

        while (details_->running) {
            cv::Mat frame;
            {
                std::unique_lock lock(details_->mtx);
                details_->cv.wait(lock, [this] {
                    return !details_->running || details_->frame_seq != details_->written_seq;
                });
                if (!details_->running) break;
                if (details_->latest_frame.empty()) continue;
                frame = std::move(details_->latest_frame);
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
    {
        std::scoped_lock lock(details_->mtx);
        details_->latest_frame = bgr_frame.clone();
        details_->frame_seq++;
    }
    details_->cv.notify_one();
}

auto RtpStreamer::push(cv::Mat&& bgr_frame) -> void {
    if (!details_->running) return;
    if (bgr_frame.empty()) return;
    if (bgr_frame.type() != CV_8UC3) return;
    {
        std::scoped_lock lock(details_->mtx);
        details_->latest_frame = std::move(bgr_frame);
        details_->frame_seq++;
    }
    details_->cv.notify_one();
}

auto RtpStreamer::stop() -> void {
    details_->running = false;
    details_->cv.notify_one();
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
