#include <cstdio>
#include <filesystem>
#include <print>

#include "vision/training_data.hpp"

namespace {

auto resolve_session_root(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1) return argv[1];
    throw std::runtime_error("usage: example_transcode_recorded_session <session_root>");
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto session_root = resolve_session_root(argc, argv);
        const auto metadata =
            rmcs_laser_guidance::load_video_session_metadata(session_root / "session.yaml");
        const std::filesystem::path video_path = session_root / metadata.relative_video_path;

        const auto before = rmcs_laser_guidance::probe_video_encoding_info(video_path);
        std::println("before codec={} tag={} profile={} pix_fmt={} path={}", before.codec_name,
            before.codec_tag_string, before.profile, before.pix_fmt, video_path.string());

        rmcs_laser_guidance::transcode_video_to_h264_in_place(video_path);

        const auto after = rmcs_laser_guidance::probe_video_encoding_info(video_path);
        std::println("after  codec={} tag={} profile={} pix_fmt={} path={}", after.codec_name,
            after.codec_tag_string, after.profile, after.pix_fmt, video_path.string());
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "example_transcode_recorded_session failed: {}", e.what());
        return 1;
    }
}
