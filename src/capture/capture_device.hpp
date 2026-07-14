#pragma once

#include <atomic>
#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "capture/capture_backend.hpp"
#include "capture/v4l2_capture.hpp"
#include "config.hpp"
#include "tracking/freshness_queue.hpp"
#include "types.hpp"

#include <hikcamera/capturer.hpp>

namespace rmcs_laser_guidance {

struct CaptureFormat {
    CaptureBackendKind backend = CaptureBackendKind::v4l2;
    std::string device_id{};
    int width = 0;
    int height = 0;
    double framerate = 0.0;
    std::string format_name{};
    std::string device_path{};
    std::string pixel_encoding{};
};

auto to_capture_format(const V4l2NegotiatedFormat& format) -> CaptureFormat;

auto to_capture_format(
    const hikcamera::DeviceInfo& device_info, const hikcamera::StreamFormat& format)
    -> CaptureFormat;

struct V4l2Backend : public CaptureBackend {
    explicit V4l2Backend(V4l2Config config_in)
        : capture(std::move(config_in)) {}

    auto open() -> std::expected<CaptureFormat, std::string> override;
    auto read_frame() -> std::expected<Frame, std::string> override;
    auto close() noexcept -> void override;
    [[nodiscard]] auto is_open() const noexcept -> bool override;
    auto reconnect() -> std::expected<CaptureFormat, std::string> override;

    V4l2Capture capture;
};

struct HikBackend : public CaptureBackend {
    explicit HikBackend(HikCameraConfig config_in)
        : config(std::move(config_in)) {}

    auto open() -> std::expected<CaptureFormat, std::string> override;
    auto read_frame() -> std::expected<Frame, std::string> override;
    auto close() noexcept -> void override;
    [[nodiscard]] auto is_open() const noexcept -> bool override;
    auto reconnect() -> std::expected<CaptureFormat, std::string> override;

    hikcamera::Camera camera{};
    HikCameraConfig config{};
};

class CaptureDevice {
public:
    explicit CaptureDevice(Config config);
    ~CaptureDevice() noexcept;

    CaptureDevice(const CaptureDevice&) = delete;
    auto operator=(const CaptureDevice&) -> CaptureDevice& = delete;

    auto open() -> std::expected<CaptureFormat, std::string>;
    auto read_frame() -> std::expected<Frame, std::string>;
    auto close() noexcept -> void;
    [[nodiscard]] auto is_open() const noexcept -> bool;
    [[nodiscard]] auto negotiated_format() const noexcept -> const std::optional<CaptureFormat>&;

    auto reconnect() -> std::expected<void, std::string>;

private:
    [[nodiscard]] auto make_backend() const -> std::unique_ptr<CaptureBackend>;
    auto start_capture_thread() -> void;
    auto stop_capture_thread() noexcept -> void;
    auto capture_loop() -> void;

    Config config_{};
    std::unique_ptr<CaptureBackend> backend_{};
    std::optional<CaptureFormat> negotiated_{};

    // Dedicated capture thread: runs backend_->read_frame() (which blocks on the
    // SDK's demosaic/convert step) independently of the consumer, so the consumer
    // can overlap its own processing with the next frame's capture instead of
    // paying for both serially. LatestValue gives "newest frame wins" semantics.
    std::unique_ptr<LatestValue<std::expected<Frame, std::string>>> frame_queue_{};
    std::thread capture_thread_{};
    std::atomic<bool> capture_stop_{false};
};

} // namespace rmcs_laser_guidance
