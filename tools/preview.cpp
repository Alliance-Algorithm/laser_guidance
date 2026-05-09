#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <future>
#include <mutex>
#include <print>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "example_support.hpp"
#include "config.hpp"
#include "tracking/ekf_tracker.hpp"
#include "vision/model_infer.hpp"
#include "streaming/rtp_streamer.hpp"
#include "streaming/udp_sender.hpp"
#include "streaming/video_shm.hpp"
#include "capture/v4l2_capture.hpp"

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

auto draw_ekf_state(cv::Mat& image,
                    const rmcs_laser_guidance::EkfState& state) -> void {
    if (!state.initialized)
        return;

    const int cx = static_cast<int>(state.position.x);
    const int cy = static_cast<int>(state.position.y);

    if (state.lost) {
        cv::putText(image, "EKF LOST", {10, 60},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
        return;
    }

    cv::circle(image, {cx, cy}, 5, {0, 255, 0}, -1);

    constexpr float kArrowScale = 0.5F;
    const int vx = static_cast<int>(state.velocity.x * kArrowScale);
    const int vy = static_cast<int>(state.velocity.y * kArrowScale);
    if (vx != 0 || vy != 0)
        cv::arrowedLine(image, {cx, cy}, {cx + vx, cy + vy}, {0, 255, 0}, 2);

    const float speed = std::hypot(state.velocity.x, state.velocity.y);
    const auto label = std::format("EKF {:.0f} px/s", speed);
    cv::putText(image, label, {cx + 10, cy - 10},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
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
        std::future<void> model_ready;
        if (config.inference.backend != rmcs_laser_guidance::InferenceBackendKind::bright_spot) {
            model_ready = std::async(std::launch::async, [&] {
                infer = std::make_unique<rmcs_laser_guidance::ModelInfer>(config.inference);
            });
        }

        rmcs_laser_guidance::RtpStreamer streamer(config.rtp);
        if (config.rtp.enabled)
            streamer.start(open_result->width, open_result->height,
                           static_cast<float>(open_result->framerate));

        rmcs_laser_guidance::UdpSender udp(config.udp);

        rmcs_laser_guidance::VideoShmProducer shm;
        if (!shm.open(open_result->width, open_result->height)) {
            std::println(stderr, "[shm] failed to create shared memory");
        }

        int fifo_fd = setup_fifo();
        if (fifo_fd < 0) return 1;

        char cmd_buf[256];
        bool running = true;

        cv::Mat output; // previous frame with overlay, ready to push
        auto loop_t0 = std::chrono::steady_clock::now();
        unsigned loop_frames = 0;

        // Async inference: main loop runs at camera speed, inference in background
        std::mutex infer_mtx;
        std::condition_variable infer_cv;
        cv::Mat pending_frame;
        bool has_pending = false;
        rmcs_laser_guidance::TargetObservation latest_observation;
        rmcs_laser_guidance::EkfTracker tracker(config.ekf);
        rmcs_laser_guidance::EkfState latest_ekf_state;
        std::thread infer_thread;
        if (config.inference.backend != rmcs_laser_guidance::InferenceBackendKind::bright_spot) {
            infer_thread = std::thread([&] {
                model_ready.wait();
                unsigned infer_cnt = 0;
                double infer_total_us = 0;
                while (running) {
                    cv::Mat frame_to_process;
                    {
                        std::unique_lock lock(infer_mtx);
                        infer_cv.wait(lock, [&] { return has_pending || !running; });
                        if (!running) break;
                        frame_to_process = std::move(pending_frame);
                        has_pending = false;
                    }
                    const auto t0 = std::chrono::steady_clock::now();
                    rmcs_laser_guidance::Frame infer_frame{.image = frame_to_process, .timestamp = rmcs_laser_guidance::Clock::now()};
                    const auto result = infer->infer(infer_frame);
                    const auto t1 = std::chrono::steady_clock::now();

                    if (result.observation.detected) {
                        tracker.process(result.observation.center, infer_frame.timestamp);
                    } else {
                        tracker.predict(infer_frame.timestamp);
                    }

                    {
                        std::scoped_lock lock(infer_mtx);
                        latest_observation = result.observation;
                        latest_observation.candidates = result.candidates;
                        latest_ekf_state = tracker.state();
                    }
                    infer_total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
                    if (++infer_cnt % 30 == 0)
                        std::println(stderr, "[infer] {:.0f} us avg over {} frames",
                                     infer_total_us / infer_cnt, infer_cnt);
                }
            });
        }

        while (running) {
            auto t_cap0 = std::chrono::steady_clock::now();
            auto frame = capture.read_frame();
            auto t_cap1 = std::chrono::steady_clock::now();
            if (!frame) {
                std::println(stderr, "Failed to read frame: {}", frame.error());
                continue;
            }

            // Push previous frame — zero wait, no inference block
            auto t_push0 = std::chrono::steady_clock::now();
            if (!output.empty()) {
                if (output.isContinuous()) {
                    shm.push_frame(output.data, output.cols, output.rows);
                } else {
                    cv::Mat cont;
                    output.copyTo(cont);
                    shm.push_frame(cont.data, cont.cols, cont.rows);
                }

                streamer.push(std::move(output));

                static unsigned frame_count = 0;
                if (++frame_count % 60 == 0)
                    std::println(stderr, "[video] sent {} frames",
                                 frame_count);
            }

            cv::Mat display = frame->image.clone();

            // Single lock: draw overlay, submit frame, grab observation
            rmcs_laser_guidance::TargetObservation observation;
            rmcs_laser_guidance::EkfState ekf_state;
            {
                std::scoped_lock lock(infer_mtx);
                if (latest_observation.detected || !latest_observation.candidates.empty())
                    draw_candidates(display, latest_observation.candidates);
                draw_ekf_state(display, latest_ekf_state);
                if (infer) {
                    pending_frame = std::move(frame->image);
                    has_pending = true;
                }
                observation = latest_observation;
                ekf_state = latest_ekf_state;
            }
            if (infer) infer_cv.notify_one();
            udp.send(observation);

            auto cmd = read_command(fifo_fd, cmd_buf, sizeof(cmd_buf));
            if (!cmd.empty())
                handle_command(cmd, streamer, config, running);

            output = display;

            if (config.debug.show_window) {
                cv::imshow("rmcs_laser_guidance_v4l2", display);
                int key = cv::waitKey(1);
                if (rmcs_laser_guidance::examples::should_exit_from_key(key))
                    running = false;
                if (cv::getWindowProperty("rmcs_laser_guidance_v4l2", cv::WND_PROP_VISIBLE) < 1)
                    running = false;
            }

            if (++loop_frames % 30 == 0) {
                const auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - loop_t0).count();
                const double cap_ms = std::chrono::duration<double, std::milli>(t_cap1 - t_cap0).count();
                const double push_ms = std::chrono::duration<double, std::milli>(t_push0 - t_cap1).count();
                std::println(stderr, "[main] {} frames {:.1f}s ({:.0f}fps) cap={:.0f}ms push={:.0f}ms infer={}",
                             loop_frames, elapsed, loop_frames / elapsed,
                             cap_ms, push_ms,
                             infer ? "ready" : "waiting");
                loop_t0 = std::chrono::steady_clock::now();
                loop_frames = 0;
            }
        }

        running = false;
        infer_cv.notify_one();
        if (infer_thread.joinable())
            infer_thread.join();

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
