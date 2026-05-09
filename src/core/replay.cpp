#include "core/replay.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>

namespace rmcs_laser_guidance {
namespace {

constexpr const char* kManifestHeader = "index,timestamp_ns,relative_image_path";

auto frame_filename(const std::size_t index) -> std::filesystem::path {
    std::ostringstream oss;
    oss << "frame_" << std::setw(6) << std::setfill('0') << index << ".png";
    return std::filesystem::path("frames") / oss.str();
}

auto split_csv_line(const std::string& line) -> std::vector<std::string> {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ','))
        fields.push_back(item);
    return fields;
}

} // namespace

ReplayRecorder::ReplayRecorder(std::filesystem::path root)
    : root_(std::move(root)) {
    std::filesystem::create_directories(root_ / "frames");
}

auto ReplayRecorder::record_frame(const Frame& frame) -> ReplayFrameInfo {
    if (frame.image.empty())
        throw std::runtime_error("cannot record empty frame");

    const ReplayFrameInfo info{
        .index = frames_.size(),
        .timestamp_ns = timestamp_to_nanoseconds(frame.timestamp),
        .relative_image_path = frame_filename(frames_.size()),
    };

    const std::filesystem::path image_path = root_ / info.relative_image_path;
    if (!cv::imwrite(image_path.string(), frame.image))
        throw std::runtime_error("failed to write replay frame image");

    frames_.push_back(info);
    return info;
}

auto ReplayRecorder::flush_manifest() const -> void {
    std::ofstream manifest(root_ / "manifest.csv");
    if (!manifest)
        throw std::runtime_error("failed to open replay manifest for writing");

    manifest << kManifestHeader << '\n';
    for (const ReplayFrameInfo& frame : frames_) {
        manifest << frame.index << ',' << frame.timestamp_ns << ','
                 << frame.relative_image_path.generic_string() << '\n';
    }
}

auto timestamp_to_nanoseconds(const Clock::time_point& timestamp) -> std::int64_t {
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        timestamp.time_since_epoch());
    return duration.count();
}

auto timestamp_from_nanoseconds(const std::int64_t nanoseconds) -> Clock::time_point {
    return Clock::time_point{
        std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds{nanoseconds})};
}

auto load_replay_dataset(const std::filesystem::path& root) -> ReplayDataset {
    std::ifstream manifest(root / "manifest.csv");
    if (!manifest)
        throw std::runtime_error("failed to open replay manifest");

    std::string header;
    if (!std::getline(manifest, header) || header != kManifestHeader)
        throw std::runtime_error("invalid replay manifest header");

    ReplayDataset dataset{.root = root};

    std::string line;
    while (std::getline(manifest, line)) {
        if (line.empty())
            continue;

        const auto fields = split_csv_line(line);
        if (fields.size() != 3)
            throw std::runtime_error("invalid replay manifest row");

        dataset.frames.push_back(ReplayFrameInfo{
            .index = static_cast<std::size_t>(std::stoull(fields[0])),
            .timestamp_ns = std::stoll(fields[1]),
            .relative_image_path = fields[2],
        });
    }

    return dataset;
}

auto load_replay_frame(const ReplayDataset& dataset, const ReplayFrameInfo& frame_info) -> Frame {
    const std::filesystem::path image_path = dataset.root / frame_info.relative_image_path;
    cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);
    if (image.empty())
        throw std::runtime_error("failed to load replay frame image");

    return Frame{
        .image = image,
        .timestamp = timestamp_from_nanoseconds(frame_info.timestamp_ns),
    };
}

} // namespace rmcs_laser_guidance
