#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <print>
#include <string>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "config.hpp"
#include "types.hpp"
#include "example_support.hpp"
#include "capture/v4l2_capture.hpp"
#include "guidance/guidance_pipeline.hpp"
#include "io/ft4222_spi.hpp"
#include "tracking/ekf_tracker.hpp"
#include "tracking/hit_state.hpp"
#include "vision/model_infer.hpp"

namespace {

namespace rg = rmcs_laser_guidance;

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

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path = resolve_config_path(argc, argv);
        const auto config = rg::load_config(config_path);

        rg::V4l2Capture capture(config.v4l2);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open camera: {}", open_result.error());
            return 1;
        }
        std::println("Camera: {} {}x{} @ {:.0f}fps",
                     open_result->device_path.string(),
                     open_result->width, open_result->height,
                     open_result->framerate);

        std::unique_ptr<rg::ModelInfer> infer;
        std::future<void> model_ready;
        if (config.inference.backend != rg::InferenceBackendKind::bright_spot) {
            model_ready = std::async(std::launch::async, [&] {
                infer = std::make_unique<rg::ModelInfer>(config.inference);
            });
        }

        std::unique_ptr<rg::Ft4222Spi> spi;
        std::unique_ptr<rg::GuidancePipeline> guidance;
        if (config.guidance.enabled) {
            auto spi_result = rg::Ft4222Spi::open(rg::Ft4222Config{
                .sys_clock = rg::Ft4222SysClock::k60MHz,
                .clock_div = rg::Ft4222SpiDiv::kDiv4,
                .cpol = rg::Ft4222Cpol::kIdleLow,
                .cpha = rg::Ft4222Cpha::kTrailing,
                .cs_active = rg::Ft4222CsActive::kLow,
                .cs_channel = 0,
            });
            if (!spi_result) {
                std::println(stderr, "Failed to open FT4222: {}", spi_result.error());
                std::println(stderr, "Continuing without galvo control");
            } else {
                spi = std::make_unique<rg::Ft4222Spi>(std::move(*spi_result));
                std::println("FT4222: opened, SCLK ~{} Hz", spi->negotiated_clock_hz());
                guidance = std::make_unique<rg::GuidancePipeline>(config, *spi);
                if (!guidance->is_initialized()) {
                    std::println(stderr, "Guidance pipeline failed to initialize; "
                                         "check camera_calib_path and FT4222");
                }
            }
        }

        std::mutex infer_mtx;
        std::condition_variable infer_cv;
        cv::Mat pending_frame;
        bool has_pending = false;
        rg::TargetObservation latest_observation;
        rg::EkfTracker tracker(config.ekf);
        rg::EkfState latest_ekf_state;
        std::string guidance_msg;
        bool running = true;

        float last_valid_depth_mm = 0.0F;
        bool depth_valid = false;
        bool ekf_was_lost = false;

        float calib_angle_x = config.guidance.calib_angle_x_deg;
        float calib_angle_y = config.guidance.calib_angle_y_deg;
        constexpr float kCalibStepDeg = 0.1F;
        cv::Point3f calib_last_P_c { -1, -1, -1 };
        bool calib_has_P_c = false;

        std::ofstream calib_file;
        if (config.guidance.calib_mode) {
            calib_file.open("calib_records.csv", std::ios::app);
            std::println("CALIB: saving records to calib_records.csv");
        }

        std::ofstream hit_calib_file;
        if (config.guidance.enabled && !config.guidance.calib_mode) {
            hit_calib_file.open("hit_calib_records.csv", std::ios::app);
            std::println("HIT-CALIB: saving confirmed purple hits to hit_calib_records.csv");
        }

        rg::HitStateMachine hit_state_machine(
            config.runtime.hit_confirm_frames,
            config.runtime.hit_release_frames);
        auto last_hit_state = rg::HitState::None;

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
                        .image = frame_to_process,
                        .timestamp = rg::Clock::now(),
                    };
                    const auto result = infer->infer(infer_frame);
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

        cv::namedWindow("laser_guidance", cv::WINDOW_NORMAL);
        auto loop_t0 = std::chrono::steady_clock::now();
        unsigned loop_frames = 0;

        while (running) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Frame read error: {}", frame.error());
                continue;
            }

            cv::Mat display = frame->image.clone();

            rg::TargetObservation observation;
            rg::EkfState ekf_state;
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

            if (guidance && guidance->is_initialized()) {
                if (config.guidance.calib_mode) {
                    guidance_msg = guidance->process_calib_angle(calib_angle_x, calib_angle_y);
                    if (observation.detected && !observation.candidates.empty()) {
                        const auto& top = observation.candidates.front();
                        if (top.bbox.width > 0.0F) {
                            const auto depth = guidance->estimate_depth(top);
                            if (depth) {
                                last_valid_depth_mm = *depth;
                                depth_valid = true;
                            }
                        }
                        if (depth_valid && last_valid_depth_mm > 0.0F) {
                            calib_last_P_c = guidance->project_to_camera(
                                top.center, last_valid_depth_mm);
                            calib_has_P_c = true;
                            static int calib_log = 0;
                            if (++calib_log % 15 == 0) {
                                std::println("CALIB: θ=({:.2f}°,{:.2f}°) "
                                             "P_c=({:.1f},{:.1f},{:.1f})mm",
                                             calib_angle_x, calib_angle_y,
                                             calib_last_P_c.x, calib_last_P_c.y, calib_last_P_c.z);
                            }
                        }
                    }
                } else if (ekf_state.initialized && !ekf_state.lost) {
                    const rg::ModelCandidate* cand = nullptr;
                    if (observation.detected && !observation.candidates.empty()) {
                        cand = &observation.candidates.front();
                        depth_valid = true;
                    }
                    if (depth_valid) {
                        guidance_msg = guidance->process_ekf_guided(
                            ekf_state.position, cand, last_valid_depth_mm);
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

            const auto top_candidate = observation.candidates.empty()
                ? nullptr
                : &observation.candidates.front();
            const bool is_purple = observation.detected
                && top_candidate != nullptr
                && top_candidate->class_id == 0
                && top_candidate->score >= 0.25F;
            const auto hit_state = hit_state_machine.update(is_purple);
            const bool hit_edge = hit_state == rg::HitState::Confirmed
                && last_hit_state != rg::HitState::Confirmed;
            last_hit_state = hit_state;

            if (hit_edge
                && guidance
                && guidance->is_initialized()
                && !config.guidance.calib_mode
                && top_candidate != nullptr) {
                const auto hit_depth = guidance->estimate_depth(*top_candidate);
                const auto hit_angles = guidance->latest_output_angles();
                if (hit_depth && hit_angles) {
                    const auto P_c = guidance->project_to_camera(
                        top_candidate->center, *hit_depth);
                    std::println(">>> HIT-CALIB RECORD: θ=({:.2f}°,{:.2f}°) P_c=({:.1f},{:.1f},{:.1f})mm class=purple",
                                 hit_angles->x, hit_angles->y,
                                 P_c.x, P_c.y, P_c.z);
                    hit_calib_file << std::format("{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n",
                        hit_angles->x, hit_angles->y,
                        P_c.x, P_c.y, P_c.z);
                    hit_calib_file.flush();
                }
            }

            const bool guidance_active = guidance && guidance->is_initialized();
            const bool calib = guidance_active && config.guidance.calib_mode;
            const bool ekf_ok = ekf_state.initialized && !ekf_state.lost;

            if (calib) {
                const auto label = std::format("CALIB galvo=({:.2f}°,{:.2f}°) [WASD adjust]",
                    calib_angle_x, calib_angle_y);
                cv::putText(display, label, {10, 90},
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
            } else {
                draw_guidance_status(display, guidance_active, ekf_ok, depth_valid, guidance_msg);
            }

            cv::imshow("laser_guidance", display);
            const int key = cv::waitKey(1);

            if (calib) {
                switch (key) {
                case 'w': case 'W': calib_angle_y -= kCalibStepDeg; break;
                case 's': case 'S': calib_angle_y += kCalibStepDeg; break;
                case 'a': case 'A': calib_angle_x -= kCalibStepDeg; break;
                case 'd': case 'D': calib_angle_x += kCalibStepDeg; break;
                case ' ':
                    if (calib_has_P_c) {
                        std::println(">>> RECORD: θ=({:.2f}°,{:.2f}°) "
                                     "P_c=({:.1f},{:.1f},{:.1f})mm",
                                     calib_angle_x, calib_angle_y,
                                     calib_last_P_c.x, calib_last_P_c.y, calib_last_P_c.z);
                        calib_file << std::format("{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n",
                            calib_angle_x, calib_angle_y,
                            calib_last_P_c.x, calib_last_P_c.y, calib_last_P_c.z);
                        calib_file.flush();
                    }
                    break;
                default: break;
                }
            }

            if (rg::examples::should_exit_from_key(key)) running = false;
            if (cv::getWindowProperty("laser_guidance", cv::WND_PROP_VISIBLE) < 1) running = false;

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

        if (guidance) {
            if (auto r = guidance->set_center(); !r.empty())
                std::println("guidance shutdown: {}", r);
        }

        capture.close();
        cv::destroyWindow("laser_guidance");
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_guidance failed: {}", e.what());
        return 1;
    }
}
