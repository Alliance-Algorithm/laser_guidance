#pragma once

#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "capture/capture_backend.hpp"
#include "capture/v4l2_capture.hpp"
#include "config.hpp"
#include "laser_guidance/error.hpp"
#include "tracking/freshness_queue.hpp"
#include "types.hpp"

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

class CaptureDevice {
public:
    explicit CaptureDevice(Config config);
    ~CaptureDevice() noexcept;

    CaptureDevice(const CaptureDevice&) = delete;
    auto operator=(const CaptureDevice&) -> CaptureDevice& = delete;

    auto open() -> std::expected<CaptureFormat, Error>;
    auto read_frame() -> std::expected<Frame, Error>;
    auto close() noexcept -> void;
    [[nodiscard]] auto is_open() const noexcept -> bool;
    [[nodiscard]] auto negotiated_format() const noexcept -> const std::optional<CaptureFormat>&;

    auto reconnect() -> std::expected<void, Error>;
    auto apply_runtime_profile(const HikRuntimeProfile& profile) -> std::expected<void, Error>;

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
    std::unique_ptr<LatestValue<std::expected<Frame, Error>>> frame_queue_{};
    std::jthread capture_thread_{};
    std::mutex backend_mutex_{};
};

} // namespace rmcs_laser_guidance
