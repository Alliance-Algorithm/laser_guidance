#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

namespace rmcs_laser_guidance {

struct VideoSessionMetadata {
    std::string session_id { };
    std::filesystem::path relative_video_path { "raw.mp4" };
    std::filesystem::path device_path { };
    int width        = 0;
    int height       = 0;
    double framerate = 0.0;
    std::string fourcc { };
    std::int64_t capture_start_unix_ms = 0;
    std::int64_t duration_ms           = 0;
    std::string lighting_tag { };
    std::string background_tag { };
    std::string distance_tag { };
    std::string target_color { };
    bool operator_note_present = false;
};

struct ExportedTrainingFrame {
    std::string image_name { };
    std::string source_session_id { };
    std::int64_t source_timestamp_ms = 0;
    std::string split { };
    double blur_score = 0.0;
    int width         = 0;
    int height        = 0;
    std::filesystem::path relative_image_path { };
};

struct VideoEncodingInfo {
    std::string codec_name { };
    std::string codec_tag_string { };
    std::string profile { };
    std::string pix_fmt { };
    int width  = 0;
    int height = 0;
};

class VideoSessionRecorder {
public:
    VideoSessionRecorder(std::filesystem::path output_root, VideoSessionMetadata metadata);

    auto record_frame(const cv::Mat& image) -> void;
    auto flush(std::int64_t duration_ms) -> void;

    [[nodiscard]] auto session_root() const noexcept -> const std::filesystem::path& {
        return session_root_;
    }
    [[nodiscard]] auto video_path() const noexcept -> const std::filesystem::path& {
        return video_path_;
    }
    [[nodiscard]] auto metadata_path() const noexcept -> const std::filesystem::path& {
        return metadata_path_;
    }
    [[nodiscard]] auto notes_path() const noexcept -> const std::filesystem::path& {
        return notes_path_;
    }
    [[nodiscard]] auto recorded_frames() const noexcept -> std::size_t { return recorded_frames_; }

private:
    std::filesystem::path session_root_ { };
    std::filesystem::path video_path_ { };
    std::filesystem::path metadata_path_ { };
    std::filesystem::path notes_path_ { };
    VideoSessionMetadata metadata_ { };
    cv::VideoWriter writer_ { };
    std::size_t recorded_frames_ = 0;
    bool flushed_                = false;
};

auto format_session_id(std::chrono::system_clock::time_point capture_start) -> std::string;
auto write_video_session_metadata(
    const std::filesystem::path& path, const VideoSessionMetadata& metadata) -> void;
auto load_video_session_metadata(const std::filesystem::path& path) -> VideoSessionMetadata;

auto probe_video_encoding_info(const std::filesystem::path& video_path) -> VideoEncodingInfo;
auto transcode_video_to_h264_in_place(const std::filesystem::path& video_path) -> void;

auto blur_score_for_frame(const cv::Mat& image) -> double;
auto export_training_frames(const std::filesystem::path& video_path, std::string_view session_id,
    const std::filesystem::path& dataset_root, std::string_view split,
    std::int64_t sample_interval_ms) -> std::vector<ExportedTrainingFrame>;
auto write_export_manifest(
    const std::filesystem::path& path, const std::vector<ExportedTrainingFrame>& frames) -> void;

} // namespace rmcs_laser_guidance
