#include <cstdio>
#include <filesystem>
#include <print>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "vision/training_data.hpp"
#include "test_utils.hpp"

namespace {

auto make_frame(const cv::Scalar& background, const bool with_edges) -> cv::Mat {
    cv::Mat image(120, 160, CV_8UC3, background);
    if (with_edges) {
        cv::rectangle(image, { 30, 20 }, { 70, 90 }, { 255, 255, 255 }, -1);
        cv::line(image, { 10, 100 }, { 150, 30 }, { 0, 0, 255 }, 3);
    }
    return image;
}

auto write_test_video(const std::filesystem::path& path, const int fourcc, const double fps = 10.0)
    -> void {
    cv::VideoWriter writer(path.string(), fourcc, fps, cv::Size(160, 120));
    if (!writer.isOpened()) throw std::runtime_error("failed to open test video writer");

    writer.write(make_frame({ 0, 0, 0 }, true));
    writer.write(make_frame({ 30, 30, 30 }, false));
    writer.write(make_frame({ 0, 0, 0 }, true));
    writer.write(make_frame({ 50, 50, 50 }, false));
    writer.write(make_frame({ 0, 0, 0 }, true));
    writer.release();
}

struct VideoInspection {
    std::size_t frame_count = 0;
    int width               = 0;
    int height              = 0;
};

auto inspect_video(const std::filesystem::path& path) -> VideoInspection {
    cv::VideoCapture capture(path.string());
    if (!capture.isOpened()) throw std::runtime_error("failed to open recorded test video");

    VideoInspection inspection {
        .frame_count = 0,
        .width       = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
        .height      = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT)),
    };
    cv::Mat frame;
    while (capture.read(frame)) {
        if (!frame.empty()) ++inspection.frame_count;
    }

    return inspection;
}

} // namespace

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        const auto temp_root     = make_temp_dir("rmcs_laser_guidance_training_data");
        const auto metadata_path = temp_root / "session.yaml";

        const rmcs_laser_guidance::VideoSessionMetadata metadata {
            .session_id            = "20260423T123000",
            .relative_video_path   = "raw.mp4",
            .device_path           = "/dev/video2",
            .width                 = 160,
            .height                = 120,
            .framerate             = 10.0,
            .fourcc                = "MJPG",
            .capture_start_unix_ms = 123456789,
            .duration_ms           = 500,
            .lighting_tag          = "lab",
            .background_tag        = "plain",
            .distance_tag          = "20m",
            .target_color          = "red",
            .operator_note_present = false,
        };
        rmcs_laser_guidance::write_video_session_metadata(metadata_path, metadata);
        const auto loaded = rmcs_laser_guidance::load_video_session_metadata(metadata_path);
        require(loaded.session_id == metadata.session_id, "session metadata session_id mismatch");
        require(
            loaded.device_path == metadata.device_path, "session metadata device_path mismatch");
        require(loaded.width == metadata.width, "session metadata width mismatch");
        require(loaded.height == metadata.height, "session metadata height mismatch");
        require(loaded.fourcc == metadata.fourcc, "session metadata fourcc mismatch");
        require(
            loaded.target_color == metadata.target_color, "session metadata target_color mismatch");

        const auto session_root = temp_root / "sessions";
        rmcs_laser_guidance::VideoSessionRecorder recorder(session_root, metadata);
        recorder.record_frame(make_frame({ 0, 0, 0 }, true));
        recorder.record_frame(make_frame({ 30, 30, 30 }, false));
        recorder.record_frame(make_frame({ 0, 0, 0 }, true));
        recorder.record_frame(make_frame({ 50, 50, 50 }, false));
        recorder.record_frame(make_frame({ 0, 0, 0 }, true));
        recorder.flush(metadata.duration_ms);

        require(recorder.recorded_frames() == 5, "recorded session frame count mismatch");
        require(
            std::filesystem::exists(recorder.video_path()), "recorded session video should exist");
        require(std::filesystem::exists(recorder.metadata_path()),
            "recorded session metadata should exist");
        require(
            std::filesystem::exists(recorder.notes_path()), "recorded session notes should exist");

        const auto recorded_metadata =
            rmcs_laser_guidance::load_video_session_metadata(recorder.metadata_path());
        require(recorded_metadata.session_id == metadata.session_id,
            "recorded session metadata session_id mismatch");
        require(recorded_metadata.relative_video_path == metadata.relative_video_path,
            "recorded session metadata video path mismatch");
        require(recorded_metadata.duration_ms == metadata.duration_ms,
            "recorded session metadata duration mismatch");
        require(
            recorded_metadata.width == metadata.width, "recorded session metadata width mismatch");
        require(recorded_metadata.height == metadata.height,
            "recorded session metadata height mismatch");
        require(recorded_metadata.lighting_tag == metadata.lighting_tag,
            "recorded session metadata lighting tag mismatch");

        const std::string notes_text = read_text_file(recorder.notes_path());
        require_contains(notes_text, "lighting_tag: lab", "session notes lighting tag");
        require_contains(notes_text, "target_color: red", "session notes target color");

        const auto recorded_encoding =
            rmcs_laser_guidance::probe_video_encoding_info(recorder.video_path());
        require(recorded_encoding.codec_name == "h264", "recorded session codec mismatch");
        require(
            recorded_encoding.codec_tag_string == "avc1", "recorded session codec tag mismatch");
        require(recorded_encoding.pix_fmt == "yuv420p", "recorded session pix fmt mismatch");

        const auto inspection = inspect_video(recorder.video_path());
        require(inspection.width == metadata.width, "recorded session video width mismatch");
        require(inspection.height == metadata.height, "recorded session video height mismatch");
        require(inspection.frame_count == 5, "recorded session video frame count mismatch");

        cv::Mat sharp = make_frame({ 0, 0, 0 }, true);
        cv::Mat blurred;
        cv::GaussianBlur(sharp, blurred, { 9, 9 }, 0.0);
        require(rmcs_laser_guidance::blur_score_for_frame(sharp)
                > rmcs_laser_guidance::blur_score_for_frame(blurred),
            "sharp frame should have higher blur score than blurred frame");

        const auto legacy_video_path = temp_root / "legacy_raw.mp4";
        write_test_video(legacy_video_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
        const auto legacy_encoding =
            rmcs_laser_guidance::probe_video_encoding_info(legacy_video_path);
        require(legacy_encoding.codec_name == "mpeg4", "legacy session codec mismatch");
        require(legacy_encoding.codec_tag_string == "mp4v", "legacy session codec tag mismatch");

        rmcs_laser_guidance::transcode_video_to_h264_in_place(legacy_video_path);
        const auto transcoded_encoding =
            rmcs_laser_guidance::probe_video_encoding_info(legacy_video_path);
        require(transcoded_encoding.codec_name == "h264", "transcoded session codec mismatch");
        require(transcoded_encoding.codec_tag_string == "avc1",
            "transcoded session codec tag mismatch");
        require(transcoded_encoding.pix_fmt == "yuv420p", "transcoded session pix fmt mismatch");

        const auto dataset_root = temp_root / "dataset";
        const auto exported     = rmcs_laser_guidance::export_training_frames(
            recorder.video_path(), metadata.session_id, dataset_root, "train", 200);
        require(exported.size() == 3, "expected 3 exported frames from 5-frame test video");
        require(exported.front().source_timestamp_ms == 0, "first exported timestamp mismatch");
        require(exported.at(1).source_timestamp_ms == 200, "second exported timestamp mismatch");
        require(exported.at(2).source_timestamp_ms == 400, "third exported timestamp mismatch");

        for (const auto& frame : exported) {
            require(std::filesystem::exists(dataset_root / frame.relative_image_path),
                "exported frame image should exist");
            require(frame.split == "train", "exported frame split mismatch");
        }

        const auto manifest_path = dataset_root / "manifests" / "train_export_manifest.csv";
        rmcs_laser_guidance::write_export_manifest(manifest_path, exported);
        const std::string manifest_text = read_text_file(manifest_path);
        require_contains(manifest_text, "source_session_id", "export manifest header");
        require_contains(manifest_text, metadata.session_id, "export manifest session id");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "training_data_test failed: {}", e.what());
        return 1;
    }
}
