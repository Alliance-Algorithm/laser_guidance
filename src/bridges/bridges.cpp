#include "laser_guidance/bridges.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <sstream>
#include <utility>
#include <vector>

#include "streaming/rtp_streamer.hpp"
#include "streaming/udp_sender.hpp"
#include "streaming/video_shm.hpp"
#include "streaming/zmq_sender.hpp"

namespace rmcs_laser_guidance {
namespace {

auto trim_copy(std::string value) -> std::string {
    const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    if (begin >= end) {
        return {};
    }
    return std::string(begin, end);
}

auto lower_copy(std::string value) -> std::string {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

auto to_model_candidates(const DetectionBatch& batch) -> std::vector<ModelCandidate> {
    std::vector<ModelCandidate> candidates;
    candidates.reserve(batch.detections.size());
    for (const auto& detection : batch.detections) {
        candidates.push_back(
            ModelCandidate{
                .score = detection.score,
                .class_id = detection.class_id,
                .bbox = detection.bbox,
                .center = detection.center,
            });
    }
    return candidates;
}

auto to_target_observation(const DetectionBatch& batch) -> TargetObservation {
    return TargetObservation{
        .detected = batch.detected,
        .center = batch.selected_center,
        .candidates = to_model_candidates(batch),
    };
}

} // namespace

FifoControlServer::FifoControlServer(std::filesystem::path fifo_path)
    : fifo_path_(std::move(fifo_path)) {}

FifoControlServer::~FifoControlServer() { stop(); }

auto FifoControlServer::start() -> std::expected<void, Error> {
    stop();
    ::mkfifo(fifo_path_.c_str(), 0666);
    fd_ = ::open(fifo_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        const auto err = make_error(
            ErrorKind::device, "failed to open FIFO: " + fifo_path_.string());
        last_error_ = format_error(err);
        return std::unexpected(err);
    }
    last_error_.clear();
    return {};
}

auto FifoControlServer::stop() -> void {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (!fifo_path_.empty()) {
        ::unlink(fifo_path_.c_str());
    }
    read_buffer_.clear();
}

auto FifoControlServer::poll_command() -> std::optional<RuntimeCommand> {
    if (fd_ < 0) {
        return std::nullopt;
    }

    if (auto buffered = consume_buffered_command(); buffered.has_value()) {
        return buffered;
    }

    char buffer[256];
    const auto bytes_read = ::read(fd_, buffer, sizeof(buffer) - 1);
    if (bytes_read <= 0) {
        return std::nullopt;
    }

    buffer[bytes_read] = '\0';
    read_buffer_.append(buffer, static_cast<std::size_t>(bytes_read));

    return consume_buffered_command();
}

auto FifoControlServer::last_error() const -> const std::string& { return last_error_; }

auto FifoControlServer::fifo_path() const -> const std::filesystem::path& { return fifo_path_; }

auto FifoControlServer::consume_buffered_command() -> std::optional<RuntimeCommand> {
    while (true) {
        const auto newline_pos = read_buffer_.find('\n');
        if (newline_pos == std::string::npos) {
            return std::nullopt;
        }

        auto command_text = read_buffer_.substr(0, newline_pos);
        read_buffer_.erase(0, newline_pos + 1);
        auto parsed = parse_command(command_text);
        if (!parsed) {
            last_error_ = format_error(parsed.error());
            continue;
        }
        last_error_.clear();
        return *parsed;
    }
}

auto FifoControlServer::parse_command(std::string_view text)
    -> std::expected<RuntimeCommand, Error> {
    const auto normalized = lower_copy(trim_copy(std::string(text)));
    if (normalized.empty()) {
        return std::unexpected(make_error(ErrorKind::config, "empty FIFO command"));
    }
    if (normalized == "quit") {
        return runtime_command::shutdown();
    }
    if (normalized == "stream on") {
        return runtime_command::set_streaming(true);
    }
    if (normalized == "stream off") {
        return runtime_command::set_streaming(false);
    }
    if (normalized == "record on") {
        return runtime_command::set_recording(true);
    }
    if (normalized == "record off") {
        return runtime_command::set_recording(false);
    }
    if (normalized == "enemy red") {
        return runtime_command::set_enemy_color(EnemyColor::red);
    }
    if (normalized == "enemy blue") {
        return runtime_command::set_enemy_color(EnemyColor::blue);
    }
    if (normalized == "enemy auto") {
        return runtime_command::set_enemy_color(EnemyColor::auto_select);
    }
    if (normalized == "backend onnx") {
        return runtime_command::set_backend(RuntimeBackend::onnx);
    }
    if (normalized == "backend tensorrt") {
        return runtime_command::set_backend(RuntimeBackend::tensorrt);
    }
    if (normalized == "ekf on") {
        return runtime_command::set_ekf(true);
    }
    if (normalized == "ekf off") {
        return runtime_command::set_ekf(false);
    }
    if (normalized.starts_with("offset")) {
        const auto rest = trim_copy(std::string(normalized).substr(6));
        if (rest.empty()) {
            return std::unexpected(make_error(ErrorKind::config, "offset requires a value"));
        }
        std::vector<float> values;
        std::stringstream stream(rest);
        std::string token;
        while (stream >> token) {
            try {
                std::size_t consumed = 0;
                values.push_back(std::stof(token, &consumed));
                if (consumed != token.size()) {
                    return std::unexpected(
                        make_error(ErrorKind::config, "invalid offset value: " + token));
                }
            } catch (const std::exception&) {
                return std::unexpected(
                    make_error(ErrorKind::config, "invalid offset value: " + token));
            }
        }
        if (values.size() == 1) {
            return runtime_command::set_offset(values[0], 0.0F);
        }
        if (values.size() == 2) {
            return runtime_command::set_offset(values[0], values[1]);
        }
        return std::unexpected(
            make_error(ErrorKind::config, "offset expects <x_deg> [y_deg]"));
    }
    return std::unexpected(
        make_error(ErrorKind::config, "unsupported FIFO command: " + normalized));
}

struct UdpTelemetryPublisher::Impl {
    explicit Impl(UdpConfig config)
        : sender(std::move(config)) {}

    UdpSender sender;
};

UdpTelemetryPublisher::UdpTelemetryPublisher(UdpConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

UdpTelemetryPublisher::~UdpTelemetryPublisher() = default;

auto UdpTelemetryPublisher::publish(const RuntimeSnapshot& snapshot) -> void {
    impl_->sender.send(to_target_observation(snapshot.detection));
}

struct ZmqTelemetryPublisher::Impl {
    explicit Impl(ZmqConfig config)
        : sender(std::move(config)) {}

    ZmqSender sender;
};

ZmqTelemetryPublisher::ZmqTelemetryPublisher(ZmqConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

ZmqTelemetryPublisher::~ZmqTelemetryPublisher() = default;

auto ZmqTelemetryPublisher::publish(const RuntimeSnapshot& snapshot) -> void {
    impl_->sender.send(to_target_observation(snapshot.detection));
}

struct ShmFramePublisher::Impl {
    VideoShmProducer producer;
    bool started = false;
};

ShmFramePublisher::ShmFramePublisher() = default;

auto ShmFramePublisher::start(const int width, const int height) -> bool {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    impl_->started = impl_->producer.open(width, height);
    return impl_->started;
}

ShmFramePublisher::~ShmFramePublisher() = default;

auto ShmFramePublisher::publish(const cv::Mat& frame) -> void {
    if (!impl_ || !impl_->started || frame.empty()) {
        return;
    }
    if (frame.isContinuous()) {
        impl_->producer.push_frame(frame.data, frame.cols, frame.rows);
        return;
    }
    cv::Mat contiguous;
    frame.copyTo(contiguous);
    impl_->producer.push_frame(contiguous.data, contiguous.cols, contiguous.rows);
}

auto ShmFramePublisher::stop() -> void {
    if (!impl_) {
        return;
    }
    impl_->producer.close();
    impl_->started = false;
}

struct RtpFramePublisher::Impl {
    explicit Impl(RtpConfig config)
        : streamer(std::move(config)) {}

    RtpStreamer streamer;
};

RtpFramePublisher::RtpFramePublisher(RtpConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RtpFramePublisher::~RtpFramePublisher() = default;

auto RtpFramePublisher::start(const int width, const int height, const float framerate) -> bool {
    return impl_->streamer.start(width, height, framerate);
}

auto RtpFramePublisher::publish(const cv::Mat& frame) -> void { impl_->streamer.push(frame); }

auto RtpFramePublisher::publish(cv::Mat&& frame) -> void { impl_->streamer.push(std::move(frame)); }

auto RtpFramePublisher::stop() -> void { impl_->streamer.stop(); }

auto RtpFramePublisher::is_active() const -> bool { return impl_->streamer.is_active(); }

} // namespace rmcs_laser_guidance
