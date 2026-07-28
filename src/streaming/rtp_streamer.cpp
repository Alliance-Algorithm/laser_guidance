#include "streaming/rtp_streamer.hpp"

#include "vision/cuda_check.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <mutex>
#include <print>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>

namespace rmcs_laser_guidance {

struct RtpStreamer::Details {
    RtpConfig config;
    std::FILE* pipe = nullptr;
    std::jthread writer;
    std::mutex mtx;
    std::condition_variable cv;
    cv::Mat latest_frame;
    uint64_t frame_seq = 0;
    uint64_t written_seq = 0;
    std::atomic<bool> running{false};
    int stream_width = 0;
    int stream_height = 0;
    int source_width = 0;
    int source_height = 0;
    std::chrono::steady_clock::time_point last_push{};
    std::chrono::microseconds min_push_interval{0};
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

    details_->source_width = width;
    details_->source_height = height;
    details_->stream_width = width;
    details_->stream_height = height;
    if (details_->config.max_width > 0 && width > details_->config.max_width) {
        details_->stream_width = details_->config.max_width;
        details_->stream_height = std::max(
            2, (height * details_->config.max_width / width) & ~1); // even for yuv420
    }

    const float stream_fps =
        details_->config.max_fps > 0
            ? std::min(framerate > 0.0F ? framerate : 30.0F, static_cast<float>(details_->config.max_fps))
            : (framerate > 0.0F ? framerate : 30.0F);
    if (details_->config.max_fps > 0) {
        details_->min_push_interval = std::chrono::microseconds(
            static_cast<int>(1'000'000.0F / stream_fps));
    } else {
        details_->min_push_interval = std::chrono::microseconds::zero();
    }
    details_->last_push = {};

    std::string encoder = details_->config.encoder;
    if (encoder.find("nvenc") != std::string::npos && !cuda_device_available()) {
        std::println(stderr,
                     "RTP streamer: nvenc unavailable (no CUDA device), using libx264");
        encoder = "libx264";
    }

    constexpr int kRtpPktSize = 1200;
    // Short GOP (~0.25s): lower glass-to-glass lag at full resolution.
    const int low_latency_gop = std::max(1, static_cast<int>(stream_fps / 4.0F));
    const std::string x264_ll = std::format(
        "keyint={}:min-keyint={}:scenecut=0:repeat-headers=1:rc-lookahead=0:sync-lookahead=0:"
        "bframes=0:sliced-threads=1",
        low_latency_gop, low_latency_gop);
    std::string encoder_opts;
    if (encoder.find("nvenc") != std::string::npos) {
        // p1 + ll; bufsize = bitrate keeps VBV shallow for low delay.
        encoder_opts = std::format(
            "-preset p1 -tune ll -rc cbr -b:v {} -maxrate {} -bufsize {} "
            "-g {} -bf 0 -pix_fmt yuv420p -profile:v high "
            "-zerolatency 1 -forced-idr 1",
            details_->config.bitrate, details_->config.bitrate, details_->config.bitrate,
            low_latency_gop);
    } else {
        encoder_opts = std::format(
            "-preset ultrafast -tune zerolatency -b:v {} -maxrate {} -bufsize {} "
            "-g {} -pix_fmt yuv420p -profile:v high "
            "-x264-params \"{}\"",
            details_->config.bitrate, details_->config.bitrate, details_->config.bitrate,
            low_latency_gop, x264_ll);
    }

    const std::string command = std::format(
        "ffmpeg -loglevel error "
        "-fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 "
        "-f rawvideo -pixel_format bgr24 -video_size {}x{} "
        "-framerate {} -i pipe:0 "
        "-c:v {} {} "
        "-flush_packets 1 -flags:v +global_header+low_delay "
        "-f rtp -pkt_size {} -sdp_file \"{}\" \"rtp://{}:{}\"",
        details_->stream_width, details_->stream_height, stream_fps, encoder, encoder_opts,
        kRtpPktSize, details_->config.sdp_path.string(), details_->config.host,
        details_->config.port);

    details_->pipe = popen(command.c_str(), "w");
    if (!details_->pipe) {
        std::println(stderr, "RTP streamer: failed to launch ffmpeg: {}", strerror(errno));
        return false;
    }
    setvbuf(details_->pipe, nullptr, _IONBF, 0);

    std::println(
        "RTP streaming started: {}x{}@{} -> {}x{}@{} rtp://{}:{}, SDP={}, encoder={}, bitrate={}",
        width, height, framerate, details_->stream_width, details_->stream_height, stream_fps,
        details_->config.host, details_->config.port, details_->config.sdp_path.string(), encoder,
        details_->config.bitrate);

    details_->running = true;
    details_->writer = std::jthread([this] {
        auto stoken = details_->writer.get_stop_token();
        uint64_t frame_count = 0;

        while (!stoken.stop_requested()) {
            cv::Mat frame;
            {
                std::unique_lock lock(details_->mtx);
                details_->cv.wait(lock, [&] {
                    return stoken.stop_requested()
                        || details_->frame_seq != details_->written_seq;
                });
                if (stoken.stop_requested())
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
            std::fflush(details_->pipe);
            frame_count++;

            if (frame_count % 300 == 0) {
                std::println(stderr, "RTP streamer: {} frames sent", frame_count);
            }
        }
    });

    return true;
}

namespace {

auto prepare_stream_frame(
    const cv::Mat& bgr_frame, const int stream_w, const int stream_h) -> cv::Mat {
    if (bgr_frame.cols == stream_w && bgr_frame.rows == stream_h) {
        return bgr_frame.clone();
    }
    cv::Mat scaled;
    cv::resize(bgr_frame, scaled, cv::Size(stream_w, stream_h), 0.0, 0.0, cv::INTER_AREA);
    return scaled;
}

} // namespace

auto RtpStreamer::push(const cv::Mat& bgr_frame) -> void {
    if (!details_->running.load())
        return;
    if (bgr_frame.empty())
        return;
    if (bgr_frame.type() != CV_8UC3)
        return;

    if (details_->min_push_interval.count() > 0) {
        const auto now = std::chrono::steady_clock::now();
        if (details_->last_push.time_since_epoch().count() != 0
            && now - details_->last_push < details_->min_push_interval) {
            return;
        }
        details_->last_push = now;
    }

    cv::Mat frame = prepare_stream_frame(
        bgr_frame, details_->stream_width, details_->stream_height);
    {
        std::scoped_lock lock(details_->mtx);
        details_->latest_frame = std::move(frame);
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

    if (details_->min_push_interval.count() > 0) {
        const auto now = std::chrono::steady_clock::now();
        if (details_->last_push.time_since_epoch().count() != 0
            && now - details_->last_push < details_->min_push_interval) {
            return;
        }
        details_->last_push = now;
    }

    cv::Mat frame;
    if (bgr_frame.cols == details_->stream_width && bgr_frame.rows == details_->stream_height) {
        frame = std::move(bgr_frame);
    } else {
        frame = prepare_stream_frame(bgr_frame, details_->stream_width, details_->stream_height);
    }
    {
        std::scoped_lock lock(details_->mtx);
        details_->latest_frame = std::move(frame);
        details_->frame_seq++;
    }
    details_->cv.notify_one();
}

auto RtpStreamer::stop() -> void {
    if (!details_->running.load() && !details_->writer.joinable() && details_->pipe == nullptr) {
        return;
    }
    const bool was_running = details_->running.exchange(false);
    if (!was_running && !details_->writer.joinable() && details_->pipe == nullptr) {
        return;
    }
    details_->writer.request_stop();
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
    if (was_running) {
        std::println("RTP streaming stopped");
    }
}

auto RtpStreamer::is_active() const -> bool { return details_->running.load(); }

} // namespace rmcs_laser_guidance
