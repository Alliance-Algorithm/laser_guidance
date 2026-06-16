#pragma once

#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <opencv2/core/mat.hpp>

#include "config.hpp"
#include "laser_guidance/runtime.hpp"

namespace rmcs_laser_guidance {

class FifoControlServer {
public:
    explicit FifoControlServer(std::filesystem::path fifo_path = "/tmp/laser_cmd");
    ~FifoControlServer();

    FifoControlServer(const FifoControlServer&) = delete;
    auto operator=(const FifoControlServer&) -> FifoControlServer& = delete;

    auto start() -> std::expected<void, std::string>;
    auto stop() -> void;
    [[nodiscard]] auto poll_command() -> std::optional<RuntimeCommand>;
    [[nodiscard]] auto last_error() const -> const std::string&;
    [[nodiscard]] auto fifo_path() const -> const std::filesystem::path&;

    static auto parse_command(std::string_view text) -> std::expected<RuntimeCommand, std::string>;

private:
    auto consume_buffered_command() -> std::optional<RuntimeCommand>;

    std::filesystem::path fifo_path_;
    int fd_ = -1;
    std::string last_error_{};
    std::string read_buffer_{};
};

class UdpTelemetryPublisher {
public:
    explicit UdpTelemetryPublisher(UdpConfig config);
    ~UdpTelemetryPublisher();

    auto publish(const RuntimeSnapshot& snapshot) -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class ZmqTelemetryPublisher {
public:
    explicit ZmqTelemetryPublisher(ZmqConfig config);
    ~ZmqTelemetryPublisher();

    auto publish(const RuntimeSnapshot& snapshot) -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class ShmFramePublisher {
public:
    ShmFramePublisher();
    ~ShmFramePublisher();

    auto start(int width, int height) -> bool;
    auto publish(const cv::Mat& frame) -> void;
    auto stop() -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class RtpFramePublisher {
public:
    explicit RtpFramePublisher(RtpConfig config);
    ~RtpFramePublisher();

    auto start(int width, int height, float framerate) -> bool;
    auto publish(const cv::Mat& frame) -> void;
    auto publish(cv::Mat&& frame) -> void;
    auto stop() -> void;
    [[nodiscard]] auto is_active() const -> bool;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rmcs_laser_guidance
