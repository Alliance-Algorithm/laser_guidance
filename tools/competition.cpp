#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
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

#include "config.hpp"
#include "types.hpp"
#include "example_support.hpp"
#include "capture/v4l2_capture.hpp"
#include "guidance/guidance_pipeline.hpp"
#include "io/ft4222_spi.hpp"
#include "streaming/rtp_streamer.hpp"
#include "streaming/udp_sender.hpp"
#include "streaming/video_shm.hpp"
#include "tracking/ekf_tracker.hpp"
#include "tracking/hit_progress.hpp"
#include "vision/model_infer.hpp"
#include "vision/training_data.hpp"

namespace {

namespace rg = rmcs_laser_guidance;

constexpr const char* kFifoPath   = "/tmp/laser_cmd";
constexpr const char* kWindowName = "laser_guidance_competition";

volatile std::sig_atomic_t g_stop_requested = 0;

auto handle_signal(int) -> void { g_stop_requested = 1; }

auto install_signal_handlers() -> void {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
}

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1) return argv[1];
    return rg::examples::default_config_path();
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
                     const std::vector<rg::ModelCandidate>& candidates) -> void {
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

auto draw_ekf_state(cv::Mat& image, const rg::EkfState& state) -> void {
    if (!state.initialized) return;
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
    cv::putText(image, std::format("EKF {:.0f} px/s", speed),
                {cx + 10, cy - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
}

auto draw_guidance_status(cv::Mat& image, bool guidance_active,
                          bool ekf_ok, bool depth_ok, const std::string& msg) -> void {
    if (!guidance_active) {
        cv::putText(image, "GUIDANCE: disabled", {10, 90},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 200, 255}, 2);
        return;
    }
    if (!ekf_ok) {
        cv::putText(image, "GUIDANCE: EKF lost/waiting", {10, 90},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 200, 255}, 2);
        return;
    }
    if (!depth_ok) {
        cv::putText(image, "GUIDANCE: waiting for depth", {10, 90},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 200, 255}, 2);
        return;
    }
    if (!msg.empty()) {
        cv::putText(image, std::format("GUIDANCE: {}", msg), {10, 90},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 200, 255}, 2);
        return;
    }
    cv::putText(image, "GUIDANCE OK", {10, 90},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
}

auto draw_hit_progress(cv::Mat& image, const rg::HitProgress& hp) -> void {
    const int bar_x = 10;
    const int bar_y = image.rows - 40;
    const int bar_w = 200;
    const int bar_h = 20;
    const int bar_border = 2;

    cv::rectangle(image, {bar_x - bar_border, bar_y - bar_border},
                  {bar_x + bar_w + bar_border, bar_y + bar_h + bar_border},
                  {80, 80, 80}, cv::FILLED);

    if (!hp.is_locked() && !hp.is_exhausted()) {
        const int fill_w = static_cast<int>(hp.progress_ratio() * static_cast<float>(bar_w));
        cv::rectangle(image, {bar_x, bar_y}, {bar_x + fill_w, bar_y + bar_h},
                      hp.is_hitting() ? cv::Scalar{0, 0, 255} : cv::Scalar{0, 165, 255},
                      cv::FILLED);
    }

    cv::rectangle(image, {bar_x - bar_border, bar_y - bar_border},
                  {bar_x + bar_w + bar_border, bar_y + bar_h + bar_border},
                  {200, 200, 200}, bar_border);

    std::string status_text;
    if (hp.is_exhausted()) {
        status_text = "LOCK EXHAUSTED (3/3)";
    } else if (hp.is_locked()) {
        status_text = std::format("LOCKED {:.0f}s  [{}/3]",
                                  hp.lock_remaining_s(), hp.lock_count());
    } else {
        status_text = std::format("P={:.0f}/{:.0f}  stage={}  locks={}",
                                  hp.progress(), hp.p0(), hp.stage(), hp.lock_count());
    }
    cv::putText(image, status_text, {bar_x, bar_y - 8},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {200, 200, 200}, 1);
}

auto draw_status_bar(cv::Mat& image, bool streaming, bool recording,
                     int enemy_class_id, bool using_trt) -> void {
    std::string line;
    line += using_trt ? " [TRT]" : " [ONNX]";
    if (streaming) line += " [RTP]";
    if (recording) line += " [REC]";
    switch (enemy_class_id) {
    case 1: line += " [BLUE]"; break;
    case 2: line += " [RED]";  break;
    default: break;
    }
    if (line.empty()) return;
    cv::putText(image, line, {image.cols - 220, 25},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
}

auto filter_candidates(std::vector<rg::ModelCandidate>& candidates,
                       int enemy_class_id) -> void {
    if (enemy_class_id < 0) return;
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
            [enemy_class_id](const rg::ModelCandidate& c) {
                return c.class_id != 0 && c.class_id != enemy_class_id;
            }),
        candidates.end());
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

auto start_recording(
    const rg::V4l2NegotiatedFormat& fmt,
    const rg::examples::RecordSessionOptions& opts) -> rg::VideoSessionRecorder {
    const auto capture_start = std::chrono::system_clock::now();
    const auto session_id    = rg::format_session_id(capture_start);
    return rg::VideoSessionRecorder(opts.output_root,
        rg::VideoSessionMetadata{
            .session_id            = session_id,
            .relative_video_path   = "raw.mp4",
            .device_path           = fmt.device_path,
            .width                 = fmt.width,
            .height                = fmt.height,
            .framerate             = fmt.framerate > 0.0
                                         ? fmt.framerate : 60.0,
            .fourcc                = fmt.fourcc,
            .capture_start_unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         capture_start.time_since_epoch()).count(),
            .duration_ms           = 0,
            .lighting_tag          = opts.lighting_tag,
            .background_tag        = opts.background_tag,
            .distance_tag          = opts.distance_tag,
            .target_color          = opts.target_color,
            .operator_note_present  = false,
        });
}

auto flush_recording(std::unique_ptr<rg::VideoSessionRecorder>& recorder,
                     std::chrono::steady_clock::time_point recording_start) -> void {
    if (!recorder) return;
    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - recording_start).count();
    recorder->flush(duration_ms);
    std::println("Recording flushed: {} frames, {}",
                 recorder->recorded_frames(),
                 recorder->session_root().string());
    recorder.reset();
}

}

int main(int argc, char** argv) {
    try {
        install_signal_handlers();

        const auto config_path = resolve_config_path(argc, argv);
        const auto config      = rg::load_config(config_path);

        rg::examples::RecordSessionOptions record_opts;
        try {
            record_opts = rg::examples::load_record_session_options(config_path);
        } catch (const std::exception& e) {
            std::println(stderr, "Record options not available: {} (recording disabled)", e.what());
        }

        rg::V4l2Capture capture(config.v4l2);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open camera: {}", open_result.error());
            return 1;
        }
        const auto& fmt = *open_result;
        std::println("Camera: {} {}x{} @ {:.0f}fps",
                     fmt.device_path.string(), fmt.width, fmt.height, fmt.framerate);

        bool running = true;

        std::unique_ptr<rg::ModelInfer> infer_onnx;
        std::unique_ptr<rg::ModelInfer> infer_trt;
        std::atomic<rg::ModelInfer*> active_infer{nullptr};
        std::future<void> model_ready;

        auto load_backends = [&] {
            auto onnx_cfg = config.inference;
            onnx_cfg.backend = rg::InferenceBackendKind::model;
            onnx_cfg.model_path = "models/exp.onnx";
            try {
                infer_onnx = std::make_unique<rg::ModelInfer>(onnx_cfg);
                std::println("ONNX backend loaded: {}", onnx_cfg.model_path.string());
            } catch (const std::exception& e) {
                std::println(stderr, "ONNX load failed: {}", e.what());
            }

            auto trt_cfg = config.inference;
            trt_cfg.backend = rg::InferenceBackendKind::tensorrt;
            trt_cfg.model_path = "models/exp.engine";
            try {
                infer_trt = std::make_unique<rg::ModelInfer>(trt_cfg);
                std::println("TensorRT backend loaded: {}", trt_cfg.model_path.string());
            } catch (const std::exception& e) {
                std::println(stderr, "TensorRT load failed: {}", e.what());
            }

            if (!infer_onnx && !infer_trt) {
                std::println(stderr, "FATAL: no inference backend available");
                running = false;
                return;
            }

            auto* preferred = (config.inference.backend == rg::InferenceBackendKind::tensorrt)
                ? static_cast<rg::ModelInfer*>(infer_trt.get())
                : static_cast<rg::ModelInfer*>(infer_onnx.get());
            if (preferred == nullptr)
                preferred = infer_onnx ? infer_onnx.get() : infer_trt.get();
            active_infer = preferred;
            std::println("Active backend: {}",
                active_infer.load() == infer_trt.get() ? "TensorRT" : "ONNX");
        };

        if (config.inference.backend != rg::InferenceBackendKind::bright_spot) {
            model_ready = std::async(std::launch::async, load_backends);
        }

        std::unique_ptr<rg::Ft4222Spi> spi;
        std::unique_ptr<rg::GuidancePipeline> guidance;
        if (config.guidance.enabled) {
            auto spi_result = rg::Ft4222Spi::open(rg::Ft4222Config{
                .sys_clock  = rg::Ft4222SysClock::k60MHz,
                .clock_div  = rg::Ft4222SpiDiv::kDiv2,
                .cpol       = rg::Ft4222Cpol::kIdleLow,
                .cpha       = rg::Ft4222Cpha::kTrailing,
                .cs_active  = rg::Ft4222CsActive::kLow,
                .cs_channel = 0,
            });
            if (!spi_result) {
                std::println(stderr, "FT4222 open failed: {} (continuing without galvo)",
                             spi_result.error());
            } else {
                spi = std::make_unique<rg::Ft4222Spi>(std::move(*spi_result));
                std::println("FT4222: SCLK ~{} Hz", spi->negotiated_clock_hz());
                guidance = std::make_unique<rg::GuidancePipeline>(config, *spi);
                if (!guidance->is_initialized()) {
                    std::println(stderr, "GuidancePipeline init failed; check camera_calib_path");
                } else {
                    std::println("GuidancePipeline: ready");
                }
            }
        }

        rg::RtpStreamer streamer(config.rtp);
        bool streaming_active = false;
        if (config.rtp.enabled) {
            streaming_active = streamer.start(fmt.width, fmt.height,
                                              static_cast<float>(fmt.framerate));
            if (streaming_active)
                std::println("RTP streaming: {}:{}", config.rtp.host, config.rtp.port);
        }

        rg::UdpSender udp(config.udp);

        rg::VideoShmProducer shm;
        if (!shm.open(fmt.width, fmt.height)) {
            std::println(stderr, "[shm] failed to open /laser_frame");
        }

        std::unique_ptr<rg::VideoSessionRecorder> recorder;
        std::chrono::steady_clock::time_point recording_start;
        bool recording_active = false;
        if (config.runtime.record_enabled && !record_opts.output_root.empty()) {
            recorder = std::make_unique<rg::VideoSessionRecorder>(
                start_recording(fmt, record_opts));
            recording_start = std::chrono::steady_clock::now();
            recording_active = true;
            const auto& root = recorder->session_root();
            std::println("Recording started: {}", root.string());
        }

        int fifo_fd = setup_fifo();
        if (fifo_fd < 0) return 1;
        char cmd_buf[256];

        rg::EkfTracker tracker(config.ekf);
        rg::HitProgress hit_progress;
        int enemy_class_id = config.inference.enemy_class_id;
        bool ekf_enabled = config.ekf.enabled;

        std::mutex infer_mtx;
        std::condition_variable infer_cv;
        cv::Mat pending_frame;
        bool has_pending = false;
        rg::TargetObservation latest_observation;
        rg::EkfState latest_ekf_state;

        std::thread infer_thread;
        if (config.inference.backend != rg::InferenceBackendKind::bright_spot) {
            infer_thread = std::thread([&] {
                model_ready.wait();
                try {
                    while (running) {
                        cv::Mat frame_to_process;
                        {
                            std::unique_lock lock(infer_mtx);
                            infer_cv.wait(lock, [&] { return has_pending || !running; });
                            if (!running) break;
                            frame_to_process = std::move(pending_frame);
                            has_pending = false;
                        }
                        rg::Frame infer_frame{
                            .image     = frame_to_process,
                            .timestamp = rg::Clock::now(),
                        };
                        auto* engine = active_infer.load();
                        if (engine == nullptr) continue;
                        auto result = engine->infer(infer_frame);
                        filter_candidates(result.candidates, enemy_class_id);
                        if (!result.candidates.empty()
                            && result.candidates.front().score >= 0.25F) {
                            result.observation.detected = true;
                            result.observation.center = result.candidates.front().center;
                        } else {
                            result.observation.detected = false;
                        }
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
                    }
                } catch (const std::exception& e) {
                    std::println(stderr, "[infer] error: {}", e.what());
                }
            });
        }

        cv::Mat output;
        auto loop_t0     = std::chrono::steady_clock::now();
        unsigned loop_frames = 0;

        std::string guidance_msg;
        float last_valid_depth_mm = 0.0F;
        bool depth_valid = false;
        bool ekf_was_lost = false;

        while (running && !g_stop_requested) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Frame read error: {}", frame.error());
                continue;
            }

            if (!output.empty()) {
                if (output.isContinuous()) {
                    shm.push_frame(output.data, output.cols, output.rows);
                } else {
                    cv::Mat cont;
                    output.copyTo(cont);
                    shm.push_frame(cont.data, cont.cols, cont.rows);
                }
                streamer.push(std::move(output));
            }

            cv::Mat display = frame->image.clone();

            rg::TargetObservation observation;
            rg::EkfState ekf_state;
            {
                std::scoped_lock lock(infer_mtx);
                if (latest_observation.detected || !latest_observation.candidates.empty())
                    draw_candidates(display, latest_observation.candidates);
                draw_ekf_state(display, latest_ekf_state);
                if (active_infer.load() != nullptr) {
                    pending_frame = std::move(frame->image);
                    has_pending = true;
                }
                observation = latest_observation;
                ekf_state   = latest_ekf_state;
            }
            if (active_infer.load() != nullptr) infer_cv.notify_one();

            udp.send(observation);

            const auto top_candidate = observation.candidates.empty()
                ? nullptr : &observation.candidates.front();
            const bool is_purple = observation.detected
                && top_candidate != nullptr
                && top_candidate->class_id == 0
                && top_candidate->score >= 0.25F;
            const float frame_dt_s = 1.0F / static_cast<float>(fmt.framerate);
            hit_progress.update(is_purple, frame_dt_s);

            if (guidance && guidance->is_initialized()) {
                if (!ekf_enabled) {
                    if (observation.detected && !observation.candidates.empty()) {
                        auto* cand = &observation.candidates.front();
                        guidance_msg = guidance->process_ekf_guided(
                            observation.center, cand, last_valid_depth_mm);
                        depth_valid = true;
                    } else if (depth_valid) {
                        guidance_msg = guidance->set_center();
                        depth_valid = false;
                    }
                } else if (ekf_state.initialized && !ekf_state.lost) {
                    const rg::ModelCandidate* cand = nullptr;
                    if (observation.detected && !observation.candidates.empty()) {
                        cand = &observation.candidates.front();
                        depth_valid = true;
                    }
                    if (depth_valid) {
                        const float latency_s =
                            static_cast<float>(config.ekf.lookahead_ms) * 0.001F;
                        const cv::Point2f aim_pos(
                            ekf_state.position.x + ekf_state.velocity.x * latency_s,
                            ekf_state.position.y + ekf_state.velocity.y * latency_s);
                        guidance_msg = guidance->process_ekf_guided(
                            aim_pos, cand, last_valid_depth_mm);
                    }
                } else if (ekf_state.lost) {
                    if (!ekf_was_lost) {
                        guidance_msg = guidance->set_center();
                        depth_valid = false;
                        last_valid_depth_mm = 0.0F;
                    }
                }
                ekf_was_lost = ekf_state.lost;
            }

            const bool guidance_ok = guidance && guidance->is_initialized();
            const bool ekf_ok = ekf_enabled
                ? (ekf_state.initialized && !ekf_state.lost)
                : depth_valid;
            draw_guidance_status(display, guidance_ok, ekf_ok, depth_valid,
                                 ekf_enabled ? guidance_msg
                                 : std::string("EKF OFF (raw)"));
            draw_hit_progress(display, hit_progress);
            draw_status_bar(display, streaming_active, recording_active, enemy_class_id,
                            active_infer.load() == infer_trt.get());

            if (recording_active && recorder)
                recorder->record_frame(display);

            auto cmd = read_command(fifo_fd, cmd_buf, sizeof(cmd_buf));
            if (!cmd.empty()) {
                if (cmd.starts_with("stream on")) {
                    if (!streamer.is_active()) {
                        streaming_active = streamer.start(
                            fmt.width, fmt.height,
                            static_cast<float>(fmt.framerate));
                        std::println("FIFO: streaming ON");
                    }
                } else if (cmd.starts_with("stream off")) {
                    streamer.stop();
                    streaming_active = false;
                    std::println("FIFO: streaming OFF");
                } else if (cmd.starts_with("record on")) {
                    if (!recording_active && !record_opts.output_root.empty()) {
                        if (recorder) {
                            flush_recording(recorder, recording_start);
                        }
                        recorder = std::make_unique<rg::VideoSessionRecorder>(
                            start_recording(fmt, record_opts));
                        recording_start = std::chrono::steady_clock::now();
                        recording_active = true;
                        std::println("FIFO: recording ON → {}",
                                     recorder->session_root().string());
                    }
                } else if (cmd.starts_with("record off")) {
                    if (recording_active) {
                        flush_recording(recorder, recording_start);
                        recording_active = false;
                        std::println("FIFO: recording OFF");
                    }
                } else if (cmd.starts_with("enemy red")) {
                    enemy_class_id = 2;
                    std::println("FIFO: enemy → RED");
                } else if (cmd.starts_with("enemy blue")) {
                    enemy_class_id = 1;
                    std::println("FIFO: enemy → BLUE");
                } else if (cmd.starts_with("enemy auto")) {
                    enemy_class_id = -1;
                    std::println("FIFO: enemy → AUTO (all classes)");
                } else if (cmd.starts_with("backend tensorrt")) {
                    if (infer_trt) {
                        active_infer = infer_trt.get();
                        std::println("FIFO: backend → TensorRT");
                    }
                } else if (cmd.starts_with("backend onnx")) {
                    if (infer_onnx) {
                        active_infer = infer_onnx.get();
                        std::println("FIFO: backend → ONNX");
                    }
                } else if (cmd.starts_with("ekf on")) {
                    ekf_enabled = true;
                    std::println("FIFO: EKF ON");
                } else if (cmd.starts_with("ekf off")) {
                    ekf_enabled = false;
                    std::println("FIFO: EKF OFF (raw detection)");
                } else if (cmd.starts_with("quit")) {
                    running = false;
                }
            }

            output = display;

            if (config.debug.show_window) {
                cv::imshow(kWindowName, display);
                int key = cv::waitKey(1);
                if (rg::examples::should_exit_from_key(key))
                    running = false;
                if (cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) < 1)
                    running = false;
            }

            if (++loop_frames % 30 == 0) {
                const auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - loop_t0).count();
                std::println(stderr, "[main] {} frames {:.1f}s ({:.0f}fps)",
                             loop_frames, elapsed, loop_frames / elapsed);
                loop_t0 = std::chrono::steady_clock::now();
                loop_frames = 0;
            }
        }

        running = false;
        infer_cv.notify_one();
        if (infer_thread.joinable()) infer_thread.join();

        if (g_stop_requested) std::println("Signal received, shutting down…");

        if (recording_active)
            flush_recording(recorder, recording_start);

        if (guidance) {
            if (auto r = guidance->set_center(); !r.empty())
                std::println("guidance shutdown: {}", r);
        }

        streamer.stop();
        close(fifo_fd);
        unlink(kFifoPath);
        capture.close();
        if (config.debug.show_window) cv::destroyWindow(kWindowName);

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_competition failed: {}", e.what());
        return 1;
    }
}
