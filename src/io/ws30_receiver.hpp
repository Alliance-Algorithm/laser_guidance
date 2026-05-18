#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <netinet/in.h>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

class Ws30Receiver {
public:
    explicit Ws30Receiver(Ws30Config config);
    ~Ws30Receiver();

    Ws30Receiver(const Ws30Receiver&) = delete;
    auto operator=(const Ws30Receiver&) -> Ws30Receiver& = delete;

    [[nodiscard]] auto is_initialized() const noexcept -> bool { return initialized_; }
    [[nodiscard]] auto latest_frame() const -> std::optional<LidarFrame>;
    [[nodiscard]] auto latest_imu_timestamp_ns() const -> std::uint64_t;
    [[nodiscard]] auto last_error() const -> std::string;

private:
    struct Packet {
        char data[256];
    };

    struct PointsPacket {
        std::uint8_t data_type[2];
        std::uint64_t timestamp_us;
        std::uint8_t label;
        std::uint16_t row[120];
        std::uint16_t col[120];
        std::uint8_t intensity[120];
        std::int16_t point_x[120];
        std::int16_t point_y[120];
        std::int16_t point_z[120];
    } __attribute__((packed));

    struct ImuPacket {
        std::uint8_t data_type[2];
        std::uint64_t timestamp_ns;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
    } __attribute__((packed));

    auto make_remote_addr(int port) const -> sockaddr_in;
    auto setup_socket(int port) -> int;
    auto send_handshake(int sock, int port, const char* payload) const -> bool;
    auto point_loop() -> void;
    auto imu_loop() -> void;
    auto ingest_points_packet(const PointsPacket& packet) -> void;
    auto swap_frame(std::uint64_t timestamp_ns) -> void;

    Ws30Config config_;
    int points_sock_ = -1;
    int imu_sock_ = -1;
    int scan_sock_ = -1;
    bool initialized_ = false;
    bool stop_ = false;
    mutable std::mutex frame_mutex_;
    mutable std::mutex error_mutex_;
    LidarFrame latest_frame_{};
    std::vector<LidarPoint> frame_a_{};
    std::vector<LidarPoint> frame_b_{};
    bool writing_a_ = true;
    int packet_count_ = 0;
    std::uint64_t latest_imu_timestamp_ns_ = 0;
    std::string last_error_{};
    std::thread point_thread_;
    std::thread imu_thread_;
};

} // namespace rmcs_laser_guidance
