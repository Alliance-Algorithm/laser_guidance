#include "streaming/rtp_streamer.hpp"

#include <atomic>
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
    std::atomic<bool> running{false};
};

RtpStreamer::RtpStreamer(RtpConfig config)
    : details_(std::make_unique<Details>()) {
    details_->config = std::move(config);
}

RtpStreamer::~RtpStreamer() { stop(); }

auto RtpStreamer::start(const int width, const int height, const float framerate) -> bool {
    if (details_->pipe)
        return false;
    if (!details_->config.enabled)
        return false;

    const auto& encoder = details_->config.encoder;
    const int gop = static_cast<int>(framerate > 0.0F ? framerate : 60.0F);
    const std::string x264_params = std::format(
        "keyint={}:min-keyint={}:scenecut=0:repeat-headers=1", gop, gop);
    std::string encoder_opts;
    if (encoder.find("nvenc") != std::string::npos) {
        encoder_opts = std::format(
            "-preset p1 -tune ll -rc cbr -b:v {} -g {} -bf 0 "
            "-pix_fmt yuv420p -profile:v high -zerolatency 1 -forced-idr 1",
            details_->config.bitrate, gop);
    } else {
        encoder_opts = std::format(
            "-preset ultrafast -tune zerolatency -b:v {} -g {} "
            "-pix_fmt yuv420p -profile:v high "
            "-x264-params \"{}\"",
            details_->config.bitrate, gop, x264_params);
    }

    const std::string command = std::format(
        "ffmpeg -loglevel error "
        "-f rawvideo -pixel_format bgr24 -video_size {}x{} "
        "-framerate {} -i pipe:0 "
        "-c:v {} {} "
        "-flags:v +global_header "
        "-f rtp -sdp_file \"{}\" \"rtp://{}:{}\"",
        width, height, framerate, encoder, encoder_opts, details_->config.sdp_path.string(),
        details_->config.host, details_->config.port);

    details_->pipe = popen(command.c_str(), "w");
    if (!details_->pipe) {
        std::println(stderr, "RTP streamer: failed to launch ffmpeg: {}", strerror(errno));
        return false;
    }

    std::println(
        "RTP streaming started: {}x{} -> rtp://{}:{}, SDP={}, encoder={}", width, height,
        details_->config.host, details_->config.port, details_->config.sdp_path.string(), encoder);

    details_->running = true;
    details_->writer = std::thread([this] {
        uint64_t frame_count = 0;

        while (details_->running.load()) {
            cv::Mat frame;
            {
                std::unique_lock lock(details_->mtx);
                details_->cv.wait(lock, [this] {
                    return !details_->running.load()
                        || details_->frame_seq != details_->written_seq;
                });
                if (!details_->running.load())
                    break;
                if (details_->latest_frame.empty())
                    continue;
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
                std::println(stderr, "RTP streamer: {} frames sent", frame_count);
            }
        }
    });

    return true;
}

auto RtpStreamer::push(const cv::Mat& bgr_frame) -> void {
    if (!details_->running.load())
        return;
    if (bgr_frame.empty())
        return;
    if (bgr_frame.type() != CV_8UC3)
        return;
    {
        std::scoped_lock lock(details_->mtx);
        details_->latest_frame = bgr_frame.clone();
        details_->frame_seq++;
    }
    details_->cv.notify_one();
}

auto RtpStreamer::push(cv::Mat&& bgr_frame) -> void {
    if (!details_->running.load())
        return;
    if (bgr_frame.empty())
        return;
    if (bgr_frame.type() != CV_8UC3)
        return;
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

auto RtpStreamer::is_active() const -> bool { return details_->running.load(); }

} // namespace rmcs_laser_guidance
