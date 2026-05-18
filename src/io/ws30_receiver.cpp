#include "io/ws30_receiver.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <format>
#include <netinet/in.h>
#include <print>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace rmcs_laser_guidance {
namespace {

constexpr std::size_t kPointsPerPacket = 120;
constexpr std::size_t kPacketsPerFrame = 450;

auto make_timeout() -> timeval {
    return timeval { .tv_sec = 0, .tv_usec = 200000 };
}

auto convert_coord_mm(std::int16_t raw) -> float {
    return static_cast<float>(raw) / 1000.0F * 2.0F;
}

} // namespace

Ws30Receiver::Ws30Receiver(Ws30Config config)
    : config_(std::move(config)) {
    if (!config_.enabled) return;

    points_sock_ = setup_socket(config_.points_port);
    scan_sock_ = setup_socket(config_.scan_port);
    if (config_.receive_imu) {
        imu_sock_ = setup_socket(config_.imu_port);
    }
    if (points_sock_ < 0 || scan_sock_ < 0 || (config_.receive_imu && imu_sock_ < 0)) {
        return;
    }

    const auto total_points = static_cast<std::size_t>(config_.grid_cols * config_.grid_rows);
    frame_a_.resize(total_points);
    frame_b_.resize(total_points);
    latest_frame_.points.resize(total_points);

    initialized_ = true;
    point_thread_ = std::thread([this] { point_loop(); });
    if (config_.receive_imu) {
        imu_thread_ = std::thread([this] { imu_loop(); });
    }
    std::println("WS30: receiver ready {}:{}", config_.host, config_.points_port);
}

Ws30Receiver::~Ws30Receiver() {
    stop_ = true;
    if (point_thread_.joinable()) point_thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();
    if (points_sock_ >= 0) close(points_sock_);
    if (imu_sock_ >= 0) close(imu_sock_);
    if (scan_sock_ >= 0) close(scan_sock_);
}

auto Ws30Receiver::latest_frame() const -> std::optional<LidarFrame> {
    std::scoped_lock lock(frame_mutex_);
    if (latest_frame_.points.empty() || latest_frame_.timestamp_ns == 0) return std::nullopt;
    return latest_frame_;
}

auto Ws30Receiver::latest_imu_timestamp_ns() const -> std::uint64_t {
    return latest_imu_timestamp_ns_;
}

auto Ws30Receiver::last_error() const -> std::string {
    std::scoped_lock lock(error_mutex_);
    return last_error_;
}

auto Ws30Receiver::setup_socket(int port) -> int {
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::scoped_lock lock(error_mutex_);
        last_error_ = std::format("WS30: socket creation failed for port {}", port);
        return -1;
    }

    const auto timeout = make_timeout();
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(0);
    if (bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        std::scoped_lock lock(error_mutex_);
        last_error_ = std::format("WS30: bind failed for port {} ({})", port, std::strerror(errno));
        close(sock);
        return -1;
    }

    return sock;
}

auto Ws30Receiver::make_remote_addr(int port) const -> sockaddr_in {
    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, config_.host.c_str(), &remote.sin_addr) != 1) {
        throw std::runtime_error(std::format("WS30: invalid host '{}'", config_.host));
    }
    return remote;
}

auto Ws30Receiver::send_handshake(int sock, int, const char* payload) const -> bool {
    Packet packet{};
    std::snprintf(packet.data, sizeof(packet.data), "%s", payload);
    const auto remote = make_remote_addr(
        std::strcmp(payload, "hello,imu") == 0 ? config_.imu_port
        : (std::strcmp(payload, "hello,points") == 0 ? config_.points_port : config_.scan_port));
    const auto sent = sendto(sock, &packet, sizeof(packet), MSG_DONTWAIT,
                             reinterpret_cast<const sockaddr*>(&remote), sizeof(remote));
    return sent == static_cast<ssize_t>(sizeof(packet));
}

auto Ws30Receiver::point_loop() -> void {
    auto last_handshake = std::chrono::steady_clock::now() - std::chrono::milliseconds(config_.handshake_interval_ms);
    PointsPacket packet{};

    while (!stop_) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_handshake >= std::chrono::milliseconds(config_.handshake_interval_ms)) {
            (void)send_handshake(points_sock_, config_.points_port, "hello,points");
            last_handshake = now;
        }

        const auto received = recv(points_sock_, &packet, sizeof(packet), 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::scoped_lock lock(error_mutex_);
                last_error_ = std::format("WS30: point recv failed ({})", std::strerror(errno));
            }
            continue;
        }
        if (static_cast<std::size_t>(received) != sizeof(packet)) continue;
        if (packet.data_type[0] != 0x5A || packet.data_type[1] != 0xA5) continue;
        ingest_points_packet(packet);
    }
}

auto Ws30Receiver::imu_loop() -> void {
    auto last_handshake = std::chrono::steady_clock::now() - std::chrono::milliseconds(config_.handshake_interval_ms);
    ImuPacket packet{};

    while (!stop_) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_handshake >= std::chrono::milliseconds(config_.handshake_interval_ms)) {
            (void)send_handshake(imu_sock_, config_.imu_port, "hello,imu");
            last_handshake = now;
        }

        const auto received = recv(imu_sock_, &packet, sizeof(packet), 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::scoped_lock lock(error_mutex_);
                last_error_ = std::format("WS30: imu recv failed ({})", std::strerror(errno));
            }
            continue;
        }
        if (static_cast<std::size_t>(received) != sizeof(packet)) continue;
        if (packet.data_type[0] != 0x1A || packet.data_type[1] != 0xA1) continue;
        latest_imu_timestamp_ns_ = packet.timestamp_ns;
    }
}

auto Ws30Receiver::ingest_points_packet(const PointsPacket& packet) -> void {
    auto& write_frame = writing_a_ ? frame_a_ : frame_b_;
    if (packet.label == 0x00) {
        packet_count_ = 0;
    }

    const auto base = static_cast<std::size_t>(packet_count_) * kPointsPerPacket;
    if (base + kPointsPerPacket > write_frame.size()) {
        packet_count_ = 0;
        return;
    }

    for (std::size_t i = 0; i < kPointsPerPacket; ++i) {
        LidarPoint point;
        point.row = static_cast<std::int32_t>(packet.row[i]);
        point.col = static_cast<std::int32_t>(packet.col[i]);
        point.intensity = static_cast<float>(packet.intensity[i]);
        point.x_mm = convert_coord_mm(packet.point_x[i]);
        point.y_mm = convert_coord_mm(packet.point_y[i]);
        point.z_mm = convert_coord_mm(packet.point_z[i]);
        write_frame[base + i] = point;
    }

    ++packet_count_;
    if (packet.label == 0x02 || packet_count_ >= static_cast<int>(kPacketsPerFrame)) {
        swap_frame(packet.timestamp_us * 1000ULL);
        writing_a_ = !writing_a_;
        packet_count_ = 0;
    }
}

auto Ws30Receiver::swap_frame(std::uint64_t timestamp_ns) -> void {
    std::scoped_lock lock(frame_mutex_);
    latest_frame_.points = writing_a_ ? frame_a_ : frame_b_;
    latest_frame_.timestamp_ns = timestamp_ns;
}

} // namespace rmcs_laser_guidance
