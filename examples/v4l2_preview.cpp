#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <print>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "example_support.hpp"
#include "config.hpp"
#include "internal/model_infer.hpp"
#include "internal/rtp_streamer.hpp"
#include "internal/v4l2_capture.hpp"

namespace {

constexpr const char* kFifoPath = "/tmp/laser_cmd";

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1) return argv[1];
    return rmcs_laser_guidance::examples::default_config_path();
}

auto print_mode(const rmcs_laser_guidance::V4l2Config& requested,
                const rmcs_laser_guidance::V4l2NegotiatedFormat& actual) -> void {
    std::println("requested device={} mode={}x{}@{} format={}",
                 requested.device_path.string(), requested.width, requested.height,
                 requested.framerate,
                 rmcs_laser_guidance::examples::pixel_format_name(requested.pixel_format));
    std::println("actual    device={} mode={}x{}@{} format={}",
                 actual.device_path.string(), actual.width, actual.height,
                 actual.framerate, actual.fourcc);
}

auto class_color(int class_id) -> cv::Scalar {
    switch (class_id) {
    case 0:  return { 255, 0, 255 };
    case 1:  return { 0, 0, 255 };
    case 2:  return { 255, 0, 0 };
    default: return { 0, 255, 0 };
    }
}

auto class_name(int class_id) -> std::string {
    switch (class_id) {
    case 0:  return "purple";
    case 1:  return "red";
    case 2:  return "blue";
    default: return "?";
    }
}

auto draw_candidates(cv::Mat& image,
                     const std::vector<rmcs_laser_guidance::ModelCandidate>& candidates)
    -> void {
    for (const auto& c : candidates) {
        if (c.score < 0.25F) continue;
        const auto color = class_color(c.class_id);
        const cv::Rect r(static_cast<int>(c.bbox.x), static_cast<int>(c.bbox.y),
                         static_cast<int>(c.bbox.width), static_cast<int>(c.bbox.height));
        cv::rectangle(image, r, color, 2);
        const auto label = std::format("{} {:.0f}%", class_name(c.class_id), c.score * 100.0F);
        cv::putText(image, label, cv::Point(r.x, std::max(r.y - 6, 16)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
    if (candidates.empty()) return;
    const auto& best = candidates.front();
    if (best.score < 0.25F) return;
    const int cx = static_cast<int>(best.center.x);
    const int cy = static_cast<int>(best.center.y);
    const int g = 8;
    cv::line(image, { cx - g, cy }, { cx + g, cy }, { 0, 255, 255 }, 1);
    cv::line(image, { cx, cy - g }, { cx, cy + g }, { 0, 255, 255 }, 1);
}

auto setup_fifo() -> int {
    mkfifo(kFifoPath, 0666);
    int fd = open(kFifoPath, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::println(stderr, "Failed to open FIFO: {}", kFifoPath);
        return -1;
    }
    std::println("FIFO ready: {}", kFifoPath);
    return fd;
}

auto read_command(int fd, char* buf, int size) -> std::string_view {
    ssize_t n = read(fd, buf, size - 1);
    if (n <= 0) return {};
    buf[n] = '\0';
    return std::string_view(buf, static_cast<std::size_t>(n));
}

auto handle_command(std::string_view cmd,
                    rmcs_laser_guidance::RtpStreamer& streamer,
                    const rmcs_laser_guidance::Config& config,
                    bool& running) -> void {
    if (cmd.starts_with("stream on")) {
        if (!streamer.is_active()) {
            streamer.start(config.v4l2.width, config.v4l2.height, config.v4l2.framerate);
        }
    } else if (cmd.starts_with("stream off")) {
        streamer.stop();
    } else if (cmd.starts_with("quit")) {
        running = false;
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path config_path = resolve_config_path(argc, argv);
        const auto config = rmcs_laser_guidance::load_config(config_path);

        rmcs_laser_guidance::V4l2Capture capture(config.v4l2);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open V4L2 capture: {}", open_result.error());
            return 1;
        }
        print_mode(config.v4l2, *open_result);

        std::unique_ptr<rmcs_laser_guidance::ModelInfer> infer;
        if (config.inference.backend == rmcs_laser_guidance::InferenceBackendKind::model)
            infer = std::make_unique<rmcs_laser_guidance::ModelInfer>(config.inference);

        rmcs_laser_guidance::RtpStreamer streamer(config.rtp);
        if (config.rtp.enabled)
            streamer.start(config.v4l2.width, config.v4l2.height, config.v4l2.framerate);

        int fifo_fd = setup_fifo();
        if (fifo_fd < 0) return 1;

        char cmd_buf[256];
        bool running = true;

        while (running) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Failed to read frame: {}", frame.error());
                continue;
            }

            cv::Mat display = frame->image.clone();

            if (infer) {
                const auto result = infer->infer(*frame);
                if (result.success)
                    draw_candidates(display, result.candidates);
            }

            auto cmd = read_command(fifo_fd, cmd_buf, sizeof(cmd_buf));
            if (!cmd.empty())
                handle_command(cmd, streamer, config, running);

            streamer.push(display);

            if (config.debug.show_window) {
                cv::imshow("rmcs_laser_guidance_v4l2", display);
                int key = cv::waitKey(1);
                if (rmcs_laser_guidance::examples::should_exit_from_key(key))
                    running = false;
            }
        }

        streamer.stop();
        close(fifo_fd);
        unlink(kFifoPath);
        capture.close();
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "example_v4l2_preview failed: {}", e.what());
        return 1;
    }
}
