#include "vision/training_data.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/wait.h>
#include <system_error>
#include <utility>

#include "laser_guidance/support.hpp"

namespace rmcs_laser_guidance {
namespace {

struct CommandResult {
    int exit_code = -1;
    std::string output{};
};

auto run_command_capture(const std::string& command) -> CommandResult {
    const std::string full_command = command + " 2>&1";
    FILE* pipe = popen(full_command.c_str(), "r");
    if (pipe == nullptr)
        throw std::runtime_error("failed to start command: " + command);

    std::string output;
    char buffer[4096];
    while (fgets(buffer, static_cast<int>(sizeof(buffer)), pipe) != nullptr)
        output += buffer;

    const int status = pclose(pipe);
    if (status == -1)
        throw std::runtime_error("failed to close command pipe: " + command);

    CommandResult result{
        .exit_code = status,
        .output = std::move(output),
    };
    if (WIFEXITED(status))
        result.exit_code = WEXITSTATUS(status);
    return result;
}

auto parse_video_encoding_info(const std::string& output) -> VideoEncodingInfo {
    VideoEncodingInfo info;

    std::istringstream stream(output);
    std::string line;
    while (std::getline(stream, line)) {
        const auto separator = line.find('=');
        if (separator == std::string::npos)
            continue;

        const std::string key = line.substr(0, separator);
        const std::string value = line.substr(separator + 1);
        if (key == "codec_name")
            info.codec_name = value;
        else if (key == "codec_tag_string")
            info.codec_tag_string = value;
        else if (key == "profile")
            info.profile = value;
        else if (key == "pix_fmt")
            info.pix_fmt = value;
        else if (key == "width")
            info.width = std::stoi(value);
        else if (key == "height")
            info.height = std::stoi(value);
    }

    if (info.codec_name.empty() || info.codec_tag_string.empty() || info.width <= 0
        || info.height <= 0) {
        throw std::runtime_error("failed to parse ffprobe video encoding info");
    }

    return info;
}

auto is_h264_avc1(const VideoEncodingInfo& info) -> bool {
    return to_lower(info.codec_name) == "h264"
        && to_lower(info.codec_tag_string) == "avc1";
}

} // namespace

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
    if (is_h264_avc1(current_info))
        return;

    const std::filesystem::path temp_path =
        video_path.parent_path() / (video_path.stem().string() + ".transcoding.mp4");
    const std::filesystem::path backup_path =
        video_path.parent_path() / (video_path.stem().string() + ".backup.mp4");

    std::error_code ec;
    std::filesystem::remove(temp_path, ec);
    std::filesystem::remove(backup_path, ec);

    const std::string command =
        "ffmpeg -y -loglevel error -i " + shell_quote(video_path.string())
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

} // namespace rmcs_laser_guidance
