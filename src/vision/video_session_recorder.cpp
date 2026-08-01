#include "vision/training_data.hpp"
#include "vision/cuda_check.hpp"

#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <sys/wait.h>
#include <utility>

#include <opencv2/core.hpp>

#include "laser_guidance/support.hpp"

namespace rmcs_laser_guidance {
namespace {

auto validate_video_extension(const std::filesystem::path& path) -> void {
    const std::string extension = to_lower(path.extension().string());
    if (extension == ".mp4")
        return;
    throw std::runtime_error(
        "unsupported session video extension, expected .mp4: " + path.string());
}

auto h264_nvenc_available() -> bool {
    const int status = std::system(
        "ffmpeg -hide_banner -loglevel error -h encoder=h264_nvenc >/dev/null 2>&1");
    return status != -1 && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

auto clamp_h264_qp(const int h264_qp) -> int {
    return std::clamp(h264_qp, 0, 51);
}

auto build_recording_ffmpeg_command(
    const int width, const int height, const double framerate,
    const std::filesystem::path& output_path, const int h264_qp) -> std::string {
    const int gop = std::max(1, static_cast<int>(framerate > 0.0 ? framerate : 80.0));
    const int qp = clamp_h264_qp(h264_qp);
    const std::string quoted_output_path = shell_quote(output_path.string());

    if (cuda_device_available() && h264_nvenc_available()) {
        return std::format(
            "ffmpeg -y -loglevel error "
            "-f rawvideo -pixel_format bgr24 -video_size {}x{} "
            "-framerate {} -i pipe:0 "
            "-c:v h264_nvenc "
            "-preset p4 -tune hq -rc constqp -qp {} "
            "-g {} -bf 0 -rc-lookahead 0 "
            "-spatial_aq 1 -temporal_aq 1 "
            "-profile:v high -pix_fmt yuv420p -movflags +faststart "
            "-f mp4 {}",
            width, height, framerate, qp, gop, quoted_output_path);
    }

    std::println(
        stderr, "VideoSessionRecorder: h264_nvenc unavailable, using libx264 fallback");
    return std::format(
        "ffmpeg -y -loglevel error "
        "-f rawvideo -pixel_format bgr24 -video_size {}x{} "
        "-framerate {} -i pipe:0 "
        "-c:v libx264 "
        "-preset veryfast -crf {} "
        "-g {} -pix_fmt yuv420p -movflags +faststart "
        "-f mp4 {}",
        width, height, framerate, qp, gop, quoted_output_path);
}

auto validate_video_session_metadata(const VideoSessionMetadata& metadata) -> void {
    if (metadata.session_id.empty())
        throw std::runtime_error("video session metadata requires a non-empty session_id");
    if (metadata.relative_video_path.empty())
        throw std::runtime_error("video session metadata requires a relative_video_path");
    if (metadata.relative_video_path.is_absolute()) {
        throw std::runtime_error("video session metadata relative_video_path must be relative");
    }
    validate_video_extension(metadata.relative_video_path);
    if (metadata.width <= 0 || metadata.height <= 0) {
        throw std::runtime_error("video session metadata requires positive frame dimensions");
    }
    if (metadata.framerate <= 0.0)
        throw std::runtime_error("video session metadata requires a positive framerate");
}

auto write_session_notes_template(
    const std::filesystem::path& path, const VideoSessionMetadata& metadata) -> void {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());

    std::ofstream notes(path);
    if (!notes)
        throw std::runtime_error("failed to create session notes file");

    notes << "lighting_tag: " << metadata.lighting_tag << '\n';
    notes << "background_tag: " << metadata.background_tag << '\n';
    notes << "distance_tag: " << metadata.distance_tag << '\n';
    notes << "target_color: " << metadata.target_color << '\n';
    notes << "pollution_light_source: \n";
    notes << "target_motion_note: \n";
    notes << "operator_note: \n";
}

auto remove_file_if_exists(const std::filesystem::path& path) -> void {
    std::error_code error;
    (void)std::filesystem::remove(path, error);
}

auto retime_video_in_place(const std::filesystem::path& video_path, const double itsscale) -> void {
    constexpr double kNoopTolerance = 0.02;
    if (std::abs(itsscale - 1.0) <= kNoopTolerance)
        return;

    const auto temp_path = video_path.parent_path()
                           / (video_path.stem().string() + ".retime_tmp.mp4");
    remove_file_if_exists(temp_path);

    const std::string command = std::format(
        "ffmpeg -y -loglevel error -itsscale {:.6f} -i {} -c copy -movflags +faststart -f mp4 {}",
        itsscale, shell_quote(video_path.string()), shell_quote(temp_path.string()));

    const int status = std::system(command.c_str());
    if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        remove_file_if_exists(temp_path);
        throw std::runtime_error(std::format("recording retime remux failed: scale={:.6f}", itsscale));
    }

    std::filesystem::rename(temp_path, video_path);
}

} // namespace

VideoSessionRecorder::VideoSessionRecorder(
    std::filesystem::path output_root, VideoSessionMetadata metadata, const int h264_qp,
    const int queue_capacity)
    : metadata_(std::move(metadata))
    , queue_capacity_(std::max(1, queue_capacity)) {
    (void)std::signal(SIGPIPE, SIG_IGN);
    validate_video_session_metadata(metadata_);
    metadata_.h264_qp = clamp_h264_qp(h264_qp);

    session_root_ = std::move(output_root) / metadata_.session_id;
    video_path_ = session_root_ / metadata_.relative_video_path;
    metadata_path_ = session_root_ / "session.yaml";
    notes_path_ = session_root_ / "notes.txt";

    std::filesystem::create_directories(session_root_);
    if (video_path_.has_parent_path())
        std::filesystem::create_directories(video_path_.parent_path());

    const std::string command = build_recording_ffmpeg_command(
        metadata_.width, metadata_.height, metadata_.framerate, video_path_, metadata_.h264_qp);
    pipe_ = popen(command.c_str(), "w");
    if (!pipe_) {
        throw std::runtime_error(
            "failed to start recording ffmpeg: " + std::string(strerror(errno)));
    }
    std::println(
        stderr, "VideoSessionRecorder: {}x{} @ {:.1f}fps h264_qp={}", metadata_.width,
        metadata_.height, metadata_.framerate, metadata_.h264_qp);

    writer_ = std::jthread([this](std::stop_token) { writer_run(); });
}

VideoSessionRecorder::~VideoSessionRecorder() {
    if (!flushed_) {
        {
            std::scoped_lock lock(queue_mutex_);
            writer_stop_ = true;
        }
        queue_cv_.notify_all();
        if (writer_.joinable())
            writer_.join();
    }
    if (pipe_)
        (void)pclose(pipe_);
}

auto VideoSessionRecorder::record_frame(const cv::Mat& image) -> void {
    if (flushed_)
        throw std::runtime_error("cannot record frame after video session flush");
    if (writer_failed_.load())
        return;
    if (image.empty())
        throw std::runtime_error("cannot record empty session frame");
    if (image.type() != CV_8UC3)
        throw std::runtime_error("session frame must be CV_8UC3 BGR");
    if (image.cols != metadata_.width || image.rows != metadata_.height) {
        throw std::runtime_error("session frame size does not match negotiated video dimensions");
    }

    {
        std::scoped_lock lock(queue_mutex_);
        if (queue_capacity_ > 0 && queue_.size() >= static_cast<std::size_t>(queue_capacity_)) {
            queue_.pop_front(); // drop the oldest frame under backpressure
        }
        queue_.emplace_back(image.clone());
    }
    queue_cv_.notify_one();
}

auto VideoSessionRecorder::writer_run() -> void {
    while (true) {
        cv::Mat frame;
        {
            std::unique_lock lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return writer_stop_ || !queue_.empty(); });
            if (writer_stop_ && queue_.empty())
                break;
            if (queue_.empty())
                continue;
            frame = std::move(queue_.front());
            queue_.pop_front();
        }
        if (!write_frame(frame)) {
            writer_failed_ = true;
            std::println(stderr, "VideoSessionRecorder: writer stopped (ffmpeg exited?)");
            break;
        }
    }
}

auto VideoSessionRecorder::write_frame(const cv::Mat& image) -> bool {
    const std::size_t row_bytes = static_cast<std::size_t>(image.cols) * image.elemSize();
    if (image.isContinuous()) {
        const std::size_t size = image.total() * image.elemSize();
        if (std::fwrite(image.data, 1, size, pipe_) != size)
            return false;
    } else {
        for (int row = 0; row < image.rows; ++row) {
            if (std::fwrite(image.ptr(row), 1, row_bytes, pipe_) != row_bytes)
                return false;
        }
    }
    ++recorded_frames_;
    return true;
}

auto VideoSessionRecorder::flush(const std::int64_t duration_ms) -> void {
    if (flushed_)
        throw std::runtime_error("video session already flushed");
    if (duration_ms < 0)
        throw std::runtime_error("video session duration must be non-negative");

    // Stop the writer and drain whatever frames are still queued.
    {
        std::scoped_lock lock(queue_mutex_);
        writer_stop_ = true;
    }
    queue_cv_.notify_all();
    if (writer_.joinable())
        writer_.join();

    if (pipe_) {
        const int rc = pclose(pipe_);
        pipe_ = nullptr;
        if (rc == -1)
            throw std::runtime_error("failed to close recording ffmpeg pipe");
        if (WIFEXITED(rc) && WEXITSTATUS(rc) != 0) {
            throw std::runtime_error(
                std::format("recording ffmpeg exited with code {}", WEXITSTATUS(rc)));
        }
        if (WIFSIGNALED(rc)) {
            throw std::runtime_error(
                std::format("recording ffmpeg terminated by signal {}", WTERMSIG(rc)));
        }
    }
    if (recorded_frames_ > 0 && duration_ms > 0) {
        const double actual_fps =
            (static_cast<double>(recorded_frames_) * 1000.0) / static_cast<double>(duration_ms);
        retime_video_in_place(video_path_, metadata_.framerate / actual_fps);
        metadata_.framerate = actual_fps;
    }
    metadata_.duration_ms = duration_ms;
    write_video_session_metadata(metadata_path_, metadata_);
    write_session_notes_template(notes_path_, metadata_);
    flushed_ = true;
}

} // namespace rmcs_laser_guidance
