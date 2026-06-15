#pragma once

#include <cstdint>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "laser_guidance/support.hpp"
#include "types.hpp"
namespace rmcs_laser_guidance {

enum class EnemyColor : std::int32_t {
    auto_select = -1,
    red = 1,
    blue = 2,
};

enum class RuntimeBackend : std::uint8_t {
    onnx,
    tensorrt,
};

enum class RuntimeCommandType : std::uint8_t {
    set_streaming,
    set_recording,
    set_enemy_color,
    set_backend,
    set_ekf,
    shutdown,
};

struct RuntimeCommand {
    RuntimeCommandType type = RuntimeCommandType::shutdown;
    bool enabled = false;
    EnemyColor enemy_color = EnemyColor::auto_select;
    RuntimeBackend backend = RuntimeBackend::onnx;

    static auto set_streaming(bool enabled) -> RuntimeCommand;
    static auto set_recording(bool enabled) -> RuntimeCommand;
    static auto set_enemy_color(EnemyColor color) -> RuntimeCommand;
    static auto set_backend(RuntimeBackend backend) -> RuntimeCommand;
    static auto set_ekf(bool enabled) -> RuntimeCommand;
    static auto shutdown() -> RuntimeCommand;
};

struct Detection {
    float score = 0.0F;
    std::int32_t class_id = -1;
    cv::Rect2f bbox{};
    cv::Point2f center{-1.0F, -1.0F};
};

struct DetectionBatch {
    std::vector<Detection> detections{};
    bool detected = false;
    cv::Point2f selected_center{-1.0F, -1.0F};
};

struct TargetTrack {
    bool detected = false;
    bool ekf_enabled = false;
    bool initialized = false;
    bool lost = false;
    int missed_frames = 0;
    double dt_seconds = 0.0;
    cv::Point2f raw_center{-1.0F, -1.0F};
    cv::Point2f aim_center{-1.0F, -1.0F};
    cv::Point2f velocity{0.0F, 0.0F};
    std::optional<Detection> selected_detection{};
    std::optional<cv::Point2f> ekf_position{};
    std::optional<cv::Point2f> ekf_acceleration{};
};

struct AimInput {
    bool ekf_enabled = false;
    TargetTrack track{};
    float last_valid_depth_mm = 0.0F;
};

struct AimOutput {
    bool command_issued = false;
    bool depth_valid = false;
    bool recentered = false;
    float depth_mm = 0.0F;
    std::string message{};
    std::optional<cv::Point2f> output_angles{};
    std::optional<cv::Point2f> output_voltages{};
};

struct CaptureFormatSnapshot {
    std::filesystem::path device_path{};
    int width = 0;
    int height = 0;
    double framerate = 0.0;
    std::string fourcc{};
};

struct HitProgressSnapshot {
    float progress = 0.0F;
    float progress_ratio = 0.0F;
    bool is_hitting = false;
    bool is_locked = false;
    float lock_remaining_s = 0.0F;
    int lock_count = 0;
    int stage = 0;
    float p0 = 0.0F;
    bool is_exhausted = false;
};

struct RuntimeStatus {
    bool running = false;
    bool stop_requested = false;
    bool capture_open = false;
    bool inference_enabled = false;
    bool streaming_active = false;
    bool recording_active = false;
    bool guidance_enabled = false;
    bool guidance_ready = false;
    bool ekf_enabled = false;
    bool backend_uses_tensorrt = false;
    EnemyColor enemy_color = EnemyColor::auto_select;
    std::string last_error{};
    std::string last_guidance_message{};
};

struct RuntimeSnapshot {
    RuntimeStatus status{};
    std::optional<CaptureFormatSnapshot> negotiated_format{};
    DetectionBatch detection{};
    std::optional<TargetTrack> track{};
    AimOutput aim{};
    HitProgressSnapshot hit_progress{};
    std::size_t dropped_frames = 0;
    std::string active_backend_name{};
    std::filesystem::path current_recording_root{};
};

enum class CompetitionProfile : std::uint8_t {
    main,
    preview,
};

struct CompetitionRuntimeOptions {
    CompetitionProfile profile = CompetitionProfile::main;
    RecordSessionOptions record_options{};
};

class CompetitionRuntime {
public:
    explicit CompetitionRuntime(Config config, CompetitionRuntimeOptions options = {});
    ~CompetitionRuntime();

    CompetitionRuntime(const CompetitionRuntime&) = delete;
    auto operator=(const CompetitionRuntime&) -> CompetitionRuntime& = delete;

    auto start() -> std::expected<void, std::string>;
    auto run() -> std::expected<void, std::string>;
    auto stop() -> void;
    auto join() -> void;
    auto submit_command(const RuntimeCommand& command) -> std::expected<void, std::string>;
    [[nodiscard]] auto snapshot() const -> RuntimeSnapshot;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rmcs_laser_guidance
