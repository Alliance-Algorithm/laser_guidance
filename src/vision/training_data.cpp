#include "vision/training_data.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <system_error>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs_laser_guidance {
namespace {

    constexpr const char* kExportManifestHeader = "image_name,source_session_id,source_timestamp_"
                                                  "ms,split,blur_score,width,height,relative_image_"
                                                  "path";

    auto normalize_lower(std::string value) -> std::string {
        std::transform(value.begin(), value.end(), value.begin(),
            [](const unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        return value;
    }

    auto video_writer_fourcc_for_path(const std::filesystem::path& path) -> int {
        const std::string extension = normalize_lower(path.extension().string());
        if (extension == ".mp4") return cv::VideoWriter::fourcc('a', 'v', 'c', '1');
        if (extension == ".avi") return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

        throw std::runtime_error(
            "unsupported session video extension, expected .mp4 or .avi: " + path.string());
    }

    auto shell_quote(std::string_view value) -> std::string {
        std::string quoted;
        quoted.reserve(value.size() + 2);
        quoted.push_back('\'');
        for (const char ch : value) {
            if (ch == '\'') quoted += "'\"'\"'";
            else quoted.push_back(ch);
        }
        quoted.push_back('\'');
        return quoted;
    }

    struct CommandResult {
        int exit_code = -1;
        std::string output { };
    };

    auto run_command_capture(const std::string& command) -> CommandResult {
        const std::string full_command = command + " 2>&1";
        FILE* pipe                     = popen(full_command.c_str(), "r");
        if (pipe == nullptr) throw std::runtime_error("failed to start command: " + command);

        std::string output;
        char buffer[4096];
        while (fgets(buffer, static_cast<int>(sizeof(buffer)), pipe) != nullptr)
            output += buffer;

        const int status = pclose(pipe);
        if (status == -1) throw std::runtime_error("failed to close command pipe: " + command);

        CommandResult result {
            .exit_code = status,
            .output    = std::move(output),
        };
        if (WIFEXITED(status)) result.exit_code = WEXITSTATUS(status);
        return result;
    }

    auto parse_video_encoding_info(const std::string& output) -> VideoEncodingInfo {
        VideoEncodingInfo info;

        std::istringstream stream(output);
        std::string line;
        while (std::getline(stream, line)) {
            const auto separator = line.find('=');
            if (separator == std::string::npos) continue;

            const std::string key   = line.substr(0, separator);
            const std::string value = line.substr(separator + 1);
            if (key == "codec_name") info.codec_name = value;
            else if (key == "codec_tag_string") info.codec_tag_string = value;
            else if (key == "profile") info.profile = value;
            else if (key == "pix_fmt") info.pix_fmt = value;
            else if (key == "width") info.width = std::stoi(value);
            else if (key == "height") info.height = std::stoi(value);
        }

        if (info.codec_name.empty() || info.codec_tag_string.empty() || info.width <= 0
            || info.height <= 0) {
            throw std::runtime_error("failed to parse ffprobe video encoding info");
        }

        return info;
    }

    auto is_h264_avc1(const VideoEncodingInfo& info) -> bool {
        return normalize_lower(info.codec_name) == "h264"
            && normalize_lower(info.codec_tag_string) == "avc1";
    }

    auto normalize_split_name(std::string_view split) -> std::string {
        const std::string normalized = normalize_lower(std::string(split));

        if (normalized != "train" && normalized != "val" && normalized != "test") {
            throw std::runtime_error("split must be one of: train, val, test");
        }

        return normalized;
    }

    auto to_gray_image(const cv::Mat& image) -> cv::Mat {
        if (image.empty()) return { };

        if (image.channels() == 1) return image;

        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            return gray;
        }

        if (image.channels() == 4) {
            cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
            return gray;
        }

        throw std::runtime_error("unsupported image channel count for blur scoring");
    }

    auto make_image_name(std::string_view session_id, const std::int64_t timestamp_ms)
        -> std::string {
        std::ostringstream oss;
        oss << session_id << "_t" << std::setw(8) << std::setfill('0') << timestamp_ms << "ms.png";
        return oss.str();
    }

    auto validate_video_session_metadata(const VideoSessionMetadata& metadata) -> void {
        if (metadata.session_id.empty())
            throw std::runtime_error("video session metadata requires a non-empty session_id");
        if (metadata.relative_video_path.empty())
            throw std::runtime_error("video session metadata requires a relative_video_path");
        if (metadata.relative_video_path.is_absolute()) {
            throw std::runtime_error("video session metadata relative_video_path must be relative");
        }
        (void)video_writer_fourcc_for_path(metadata.relative_video_path);
        if (metadata.width <= 0 || metadata.height <= 0) {
            throw std::runtime_error("video session metadata requires positive frame dimensions");
        }
        if (metadata.framerate <= 0.0)
            throw std::runtime_error("video session metadata requires a positive framerate");
    }

    auto write_session_notes_template(
        const std::filesystem::path& path, const VideoSessionMetadata& metadata) -> void {
        if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());

        std::ofstream notes(path);
        if (!notes) throw std::runtime_error("failed to create session notes file");

        notes << "lighting_tag: " << metadata.lighting_tag << '\n';
        notes << "background_tag: " << metadata.background_tag << '\n';
        notes << "distance_tag: " << metadata.distance_tag << '\n';
        notes << "target_color: " << metadata.target_color << '\n';
        notes << "pollution_light_source: \n";
        notes << "target_motion_note: \n";
        notes << "operator_note: \n";
    }

} // namespace

VideoSessionRecorder::VideoSessionRecorder(
    std::filesystem::path output_root, VideoSessionMetadata metadata)
    : metadata_(std::move(metadata)) {
    validate_video_session_metadata(metadata_);

    session_root_  = std::move(output_root) / metadata_.session_id;
    video_path_    = session_root_ / metadata_.relative_video_path;
    metadata_path_ = session_root_ / "session.yaml";
    notes_path_    = session_root_ / "notes.txt";

    std::filesystem::create_directories(session_root_);
    if (video_path_.has_parent_path())
        std::filesystem::create_directories(video_path_.parent_path());

    writer_.open(video_path_.string(), video_writer_fourcc_for_path(video_path_),
        metadata_.framerate, cv::Size(metadata_.width, metadata_.height));
    if (!writer_.isOpened()) {
        throw std::runtime_error(
            "failed to open session video for writing: " + video_path_.string());
    }
}

auto VideoSessionRecorder::record_frame(const cv::Mat& image) -> void {
    if (flushed_) throw std::runtime_error("cannot record frame after video session flush");
    if (!writer_.isOpened()) throw std::runtime_error("video session writer is not open");
    if (image.empty()) throw std::runtime_error("cannot record empty session frame");
    if (image.cols != metadata_.width || image.rows != metadata_.height) {
        throw std::runtime_error("session frame size does not match negotiated video dimensions");
    }

    writer_.write(image);
    ++recorded_frames_;
}

auto VideoSessionRecorder::flush(const std::int64_t duration_ms) -> void {
    if (flushed_) throw std::runtime_error("video session already flushed");
    if (duration_ms < 0) throw std::runtime_error("video session duration must be non-negative");

    writer_.release();
    metadata_.duration_ms = duration_ms;
    write_video_session_metadata(metadata_path_, metadata_);
    write_session_notes_template(notes_path_, metadata_);
    flushed_ = true;
}

auto format_session_id(const std::chrono::system_clock::time_point capture_start) -> std::string {
    const std::time_t timestamp = std::chrono::system_clock::to_time_t(capture_start);
    const std::tm* local_time   = std::localtime(&timestamp);
    if (local_time == nullptr) throw std::runtime_error("failed to format capture start time");

    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y%m%dT%H%M%S");
    return oss.str();
}

auto write_video_session_metadata(
    const std::filesystem::path& path, const VideoSessionMetadata& metadata) -> void {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "session_id" << YAML::Value << metadata.session_id;
    out << YAML::Key << "relative_video_path" << YAML::Value
        << metadata.relative_video_path.generic_string();
    out << YAML::Key << "device_path" << YAML::Value << metadata.device_path.string();
    out << YAML::Key << "width" << YAML::Value << metadata.width;
    out << YAML::Key << "height" << YAML::Value << metadata.height;
    out << YAML::Key << "framerate" << YAML::Value << metadata.framerate;
    out << YAML::Key << "fourcc" << YAML::Value << metadata.fourcc;
    out << YAML::Key << "capture_start_unix_ms" << YAML::Value << metadata.capture_start_unix_ms;
    out << YAML::Key << "duration_ms" << YAML::Value << metadata.duration_ms;
    out << YAML::Key << "lighting_tag" << YAML::Value << metadata.lighting_tag;
    out << YAML::Key << "background_tag" << YAML::Value << metadata.background_tag;
    out << YAML::Key << "distance_tag" << YAML::Value << metadata.distance_tag;
    out << YAML::Key << "target_color" << YAML::Value << metadata.target_color;
    out << YAML::Key << "operator_note_present" << YAML::Value << metadata.operator_note_present;
    out << YAML::EndMap;

    std::ofstream metadata_file(path);
    if (!metadata_file) throw std::runtime_error("failed to open session metadata for writing");
    metadata_file << out.c_str();
}

auto load_video_session_metadata(const std::filesystem::path& path) -> VideoSessionMetadata {
    const YAML::Node yaml = YAML::LoadFile(path.string());

    VideoSessionMetadata metadata;
    if (yaml["session_id"]) metadata.session_id = yaml["session_id"].as<std::string>();
    if (yaml["relative_video_path"]) {
        metadata.relative_video_path = yaml["relative_video_path"].as<std::string>();
    }
    if (yaml["device_path"]) metadata.device_path = yaml["device_path"].as<std::string>();
    if (yaml["width"]) metadata.width = yaml["width"].as<int>();
    if (yaml["height"]) metadata.height = yaml["height"].as<int>();
    if (yaml["framerate"]) metadata.framerate = yaml["framerate"].as<double>();
    if (yaml["fourcc"]) metadata.fourcc = yaml["fourcc"].as<std::string>();
    if (yaml["capture_start_unix_ms"]) {
        metadata.capture_start_unix_ms = yaml["capture_start_unix_ms"].as<std::int64_t>();
    }
    if (yaml["duration_ms"]) metadata.duration_ms = yaml["duration_ms"].as<std::int64_t>();
    if (yaml["lighting_tag"]) metadata.lighting_tag = yaml["lighting_tag"].as<std::string>();
    if (yaml["background_tag"]) metadata.background_tag = yaml["background_tag"].as<std::string>();
    if (yaml["distance_tag"]) metadata.distance_tag = yaml["distance_tag"].as<std::string>();
    if (yaml["target_color"]) metadata.target_color = yaml["target_color"].as<std::string>();
    if (yaml["operator_note_present"]) {
        metadata.operator_note_present = yaml["operator_note_present"].as<bool>();
    }

    return metadata;
}

auto probe_video_encoding_info(const std::filesystem::path& video_path) -> VideoEncodingInfo {
    if (!std::filesystem::exists(video_path))
        throw std::runtime_error("video path does not exist: " + video_path.string());

    const std::string command = "ffprobe -v error -select_streams v:0 "
                                "-show_entries "
                                "stream=codec_name,codec_tag_string,profile,width,height,pix_fmt "
                                "-of default=noprint_wrappers=1 "
        + shell_quote(video_path.string());
    const auto result = run_command_capture(command);
    if (result.exit_code != 0) {
        throw std::runtime_error(
            "ffprobe failed for video path " + video_path.string() + ": " + result.output);
    }

    return parse_video_encoding_info(result.output);
}

auto transcode_video_to_h264_in_place(const std::filesystem::path& video_path) -> void {
    const auto current_info = probe_video_encoding_info(video_path);
    if (is_h264_avc1(current_info)) return;

    const std::filesystem::path temp_path =
        video_path.parent_path() / (video_path.stem().string() + ".transcoding.mp4");
    const std::filesystem::path backup_path =
        video_path.parent_path() / (video_path.stem().string() + ".backup.mp4");

    std::error_code ec;
    std::filesystem::remove(temp_path, ec);
    std::filesystem::remove(backup_path, ec);

    const std::string command = "ffmpeg -y -loglevel error -i " + shell_quote(video_path.string())
        + " -map 0:v:0 -an -c:v libx264 -pix_fmt yuv420p -movflags +faststart "
        + shell_quote(temp_path.string());
    const auto result = run_command_capture(command);
    if (result.exit_code != 0) {
        throw std::runtime_error(
            "ffmpeg transcode failed for video path " + video_path.string() + ": " + result.output);
    }

    const auto transcoded_info = probe_video_encoding_info(temp_path);
    if (!is_h264_avc1(transcoded_info)) {
        throw std::runtime_error("transcoded video is not H.264/avc1: " + temp_path.string());
    }

    std::filesystem::rename(video_path, backup_path);
    try {
        std::filesystem::rename(temp_path, video_path);
    } catch (...) {
        if (!std::filesystem::exists(video_path) && std::filesystem::exists(backup_path))
            std::filesystem::rename(backup_path, video_path);
        throw;
    }

    std::filesystem::remove(backup_path, ec);
    (void)probe_video_encoding_info(video_path);
}

auto blur_score_for_frame(const cv::Mat& image) -> double {
    const cv::Mat gray = to_gray_image(image);
    if (gray.empty()) return 0.0;

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    (void)mean;
    return stddev[0] * stddev[0];
}

auto export_training_frames(const std::filesystem::path& video_path, std::string_view session_id,
    const std::filesystem::path& dataset_root, std::string_view split,
    const std::int64_t sample_interval_ms) -> std::vector<ExportedTrainingFrame> {
    if (sample_interval_ms <= 0) throw std::runtime_error("sample_interval_ms must be positive");

    const std::string normalized_split = normalize_split_name(split);

    cv::VideoCapture capture(video_path.string());
    if (!capture.isOpened()) {
        throw std::runtime_error("failed to open session video: " + video_path.string());
    }

    const double fps = capture.get(cv::CAP_PROP_FPS);
    if (fps <= 0.0) throw std::runtime_error("session video reports non-positive FPS");

    const std::filesystem::path image_root = dataset_root / "images" / normalized_split;
    std::filesystem::create_directories(image_root);

    std::vector<ExportedTrainingFrame> exported_frames;
    std::int64_t next_export_ms = 0;
    std::size_t frame_index     = 0;

    cv::Mat frame;
    while (capture.read(frame)) {
        if (frame.empty()) {
            ++frame_index;
            continue;
        }

        const auto timestamp_ms =
            static_cast<std::int64_t>(std::llround((1000.0 * frame_index) / fps));
        ++frame_index;

        if (timestamp_ms < next_export_ms) continue;

        const std::string image_name           = make_image_name(session_id, timestamp_ms);
        const std::filesystem::path image_path = image_root / image_name;
        if (!cv::imwrite(image_path.string(), frame))
            throw std::runtime_error("failed to write exported training frame");

        exported_frames.push_back(ExportedTrainingFrame {
            .image_name          = image_name,
            .source_session_id   = std::string(session_id),
            .source_timestamp_ms = timestamp_ms,
            .split               = normalized_split,
            .blur_score          = blur_score_for_frame(frame),
            .width               = frame.cols,
            .height              = frame.rows,
            .relative_image_path = image_path.lexically_relative(dataset_root),
        });

        do {
            next_export_ms += sample_interval_ms;
        } while (next_export_ms <= timestamp_ms);
    }

    return exported_frames;
}

auto write_export_manifest(
    const std::filesystem::path& path, const std::vector<ExportedTrainingFrame>& frames) -> void {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());

    std::ofstream manifest(path);
    if (!manifest) throw std::runtime_error("failed to open export manifest for writing");

    manifest << kExportManifestHeader << '\n';
    for (const auto& frame : frames) {
        manifest << frame.image_name << ',' << frame.source_session_id << ','
                 << frame.source_timestamp_ms << ',' << frame.split << ',' << std::fixed
                 << std::setprecision(4) << frame.blur_score << ',' << frame.width << ','
                 << frame.height << ',' << frame.relative_image_path.generic_string() << '\n';
    }
}

} // namespace rmcs_laser_guidance
