#include <cstdio>
#include <filesystem>
#include <print>

#include "example_support.hpp"
#include "vision/training_data.hpp"

namespace {

auto resolve_session_root(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1) return argv[1];
    return rmcs_laser_guidance::examples::default_video_session_root();
}

auto resolve_dataset_root(int argc, char** argv) -> std::filesystem::path {
    if (argc > 2) return argv[2];
    return "/tmp/rmcs_laser_guidance_dataset";
}

auto resolve_split(int argc, char** argv) -> std::string {
    if (argc > 3) return argv[3];
    return "train";
}

auto resolve_sample_interval_ms(int argc, char** argv) -> std::int64_t {
    if (argc > 4) return std::stoll(argv[4]);
    return 200;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto session_root       = resolve_session_root(argc, argv);
        const auto dataset_root       = resolve_dataset_root(argc, argv);
        const std::string split       = resolve_split(argc, argv);
        const auto sample_interval_ms = resolve_sample_interval_ms(argc, argv);

        const auto metadata =
            rmcs_laser_guidance::load_video_session_metadata(session_root / "session.yaml");
        const std::filesystem::path video_path = session_root / metadata.relative_video_path;

        const auto exported_frames = rmcs_laser_guidance::export_training_frames(
            video_path, metadata.session_id, dataset_root, split, sample_interval_ms);
        const std::filesystem::path manifest_path = dataset_root / "manifests"
            / (metadata.session_id + "_" + split + "_export_manifest.csv");
        rmcs_laser_guidance::write_export_manifest(manifest_path, exported_frames);

        std::println("session_id={} split={} exported_frames={} manifest={}", metadata.session_id,
            split, exported_frames.size(), manifest_path.string());
        return exported_frames.empty() ? 1 : 0;
    } catch (const std::exception& e) {
        std::println(stderr, "example_export_training_frames failed: {}", e.what());
        return 1;
    }
}
