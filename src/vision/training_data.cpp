#include "vision/training_data.hpp"
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

#include "core/frame_format.hpp"
#include "laser_guidance/support.hpp"

namespace rmcs_laser_guidance {
namespace {

constexpr const char* kExportManifestHeader = "image_name,source_session_id,source_timestamp_"
                                              "ms,split,blur_score,width,height,relative_image_"
                                              "path";

auto normalize_split_name(std::string_view split) -> std::string {
    const std::string normalized = to_lower(std::string(split));

    if (normalized != "train" && normalized != "val" && normalized != "test") {
        throw std::runtime_error("split must be one of: train, val, test");
    }

    return normalized;
}

auto make_image_name(std::string_view session_id, const std::int64_t timestamp_ms) -> std::string {
    std::ostringstream oss;
    oss << session_id << "_t" << std::setw(8) << std::setfill('0') << timestamp_ms << "ms.png";
    return oss.str();
}

} // namespace

auto format_session_id(const std::chrono::system_clock::time_point capture_start) -> std::string {
    const std::time_t timestamp = std::chrono::system_clock::to_time_t(capture_start);
    const std::tm* local_time = std::localtime(&timestamp);
    if (local_time == nullptr)
        throw std::runtime_error("failed to format capture start time");

    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y%m%dT%H%M%S");
    return oss.str();
}

auto write_video_session_metadata(
    const std::filesystem::path& path, const VideoSessionMetadata& metadata) -> void {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());

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
    if (!metadata_file)
        throw std::runtime_error("failed to open session metadata for writing");
    metadata_file << out.c_str();
}

auto load_video_session_metadata(const std::filesystem::path& path) -> VideoSessionMetadata {
    const YAML::Node yaml = YAML::LoadFile(path.string());

    VideoSessionMetadata metadata;
    if (yaml["session_id"])
        metadata.session_id = yaml["session_id"].as<std::string>();
    if (yaml["relative_video_path"]) {
        metadata.relative_video_path = yaml["relative_video_path"].as<std::string>();
    }
    if (yaml["device_path"])
        metadata.device_path = yaml["device_path"].as<std::string>();
    if (yaml["width"])
        metadata.width = yaml["width"].as<int>();
    if (yaml["height"])
        metadata.height = yaml["height"].as<int>();
    if (yaml["framerate"])
        metadata.framerate = yaml["framerate"].as<double>();
    if (yaml["fourcc"])
        metadata.fourcc = yaml["fourcc"].as<std::string>();
    if (yaml["capture_start_unix_ms"]) {
        metadata.capture_start_unix_ms = yaml["capture_start_unix_ms"].as<std::int64_t>();
    }
    if (yaml["duration_ms"])
        metadata.duration_ms = yaml["duration_ms"].as<std::int64_t>();
    if (yaml["lighting_tag"])
        metadata.lighting_tag = yaml["lighting_tag"].as<std::string>();
    if (yaml["background_tag"])
        metadata.background_tag = yaml["background_tag"].as<std::string>();
    if (yaml["distance_tag"])
        metadata.distance_tag = yaml["distance_tag"].as<std::string>();
    if (yaml["target_color"])
        metadata.target_color = yaml["target_color"].as<std::string>();
    if (yaml["operator_note_present"]) {
        metadata.operator_note_present = yaml["operator_note_present"].as<bool>();
    }

    return metadata;
}

auto blur_score_for_frame(const cv::Mat& image) -> double {
    const cv::Mat gray = to_gray_image(image);
    if (gray.empty())
        return 0.0;

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    (void)mean;
    return stddev[0] * stddev[0];
}

auto export_training_frames(
    const std::filesystem::path& video_path, std::string_view session_id,
    const std::filesystem::path& dataset_root, std::string_view split,
    const std::int64_t sample_interval_ms) -> std::vector<ExportedTrainingFrame> {
    if (sample_interval_ms <= 0)
        throw std::runtime_error("sample_interval_ms must be positive");

    const std::string normalized_split = normalize_split_name(split);

    cv::VideoCapture capture(video_path.string());
    if (!capture.isOpened()) {
        throw std::runtime_error("failed to open session video: " + video_path.string());
    }

    const double fps = capture.get(cv::CAP_PROP_FPS);
    if (fps <= 0.0)
        throw std::runtime_error("session video reports non-positive FPS");

    const std::filesystem::path image_root = dataset_root / "images" / normalized_split;
    std::filesystem::create_directories(image_root);

    std::vector<ExportedTrainingFrame> exported_frames;
    std::int64_t next_export_ms = 0;
    std::size_t frame_index = 0;

    cv::Mat frame;
    while (capture.read(frame)) {
        if (frame.empty()) {
            ++frame_index;
            continue;
        }

        const auto timestamp_ms =
            static_cast<std::int64_t>(std::llround((1000.0 * frame_index) / fps));
        ++frame_index;

        if (timestamp_ms < next_export_ms)
            continue;

        const std::string image_name = make_image_name(session_id, timestamp_ms);
        const std::filesystem::path image_path = image_root / image_name;
        if (!cv::imwrite(image_path.string(), frame))
            throw std::runtime_error("failed to write exported training frame");

        exported_frames.push_back(
            ExportedTrainingFrame{
                .image_name = image_name,
                .source_session_id = std::string(session_id),
                .source_timestamp_ms = timestamp_ms,
                .split = normalized_split,
                .blur_score = blur_score_for_frame(frame),
                .width = frame.cols,
                .height = frame.rows,
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
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());

    std::ofstream manifest(path);
    if (!manifest)
        throw std::runtime_error("failed to open export manifest for writing");

    manifest << kExportManifestHeader << '\n';
    for (const auto& frame : frames) {
        manifest << frame.image_name << ',' << frame.source_session_id << ','
                 << frame.source_timestamp_ms << ',' << frame.split << ',' << std::fixed
                 << std::setprecision(4) << frame.blur_score << ',' << frame.width << ','
                 << frame.height << ',' << frame.relative_image_path.generic_string() << '\n';
    }
}

} // namespace rmcs_laser_guidance
