#include <chrono>
#include <cstring>
#include <print>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "config.hpp"
#include "io/ws30_receiver.hpp"
#include "test_utils.hpp"

namespace {

using namespace rmcs_laser_guidance;
using namespace rmcs_laser_guidance::tests;

constexpr int kPointsPort = 21101;
constexpr int kImuPort = 21102;
constexpr int kScanPort = 21103;

#pragma pack(push)
#pragma pack(1)
struct VendorPointsPacket {
    std::uint8_t data_type[2];
    std::uint64_t timestamp_us;
    std::uint8_t label;
    std::uint16_t row[120];
    std::uint16_t col[120];
    std::uint8_t intensity[120];
    std::int16_t point_x[120];
    std::int16_t point_y[120];
    std::int16_t point_z[120];
};
#pragma pack(pop)

class FakeWs30Server {
public:
    FakeWs30Server()
        : sock_(socket(AF_INET, SOCK_DGRAM, 0)) {
        require(sock_ >= 0, "failed to create fake WS30 socket");
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(kPointsPort);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        require(bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0,
            "failed to bind fake WS30 points socket");
        worker_ = std::thread([this] { run(); });
    }

    ~FakeWs30Server() {
        stop_ = true;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(kPointsPort);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        char wake = 0;
        sendto(sock_, &wake, sizeof(wake), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        if (worker_.joinable()) worker_.join();
        close(sock_);
    }

private:
    void run() {
        while (!stop_) {
            sockaddr_in client{};
            socklen_t client_len = sizeof(client);
            char buf[256]{};
            const auto received = recvfrom(sock_, buf, sizeof(buf), 0,
                                           reinterpret_cast<sockaddr*>(&client), &client_len);
            if (received <= 0) continue;
            if (std::strcmp(buf, "hello,points") != 0) continue;
            for (int packet_idx = 0; packet_idx < 450; ++packet_idx) {
                VendorPointsPacket packet{};
                packet.data_type[0] = 0x5A;
                packet.data_type[1] = 0xA5;
                packet.timestamp_us = 1234567;
                packet.label = packet_idx == 0 ? 0x00 : (packet_idx == 449 ? 0x02 : 0x01);
                for (int i = 0; i < 120; ++i) {
                    const int global_idx = packet_idx * 120 + i;
                    packet.row[i] = static_cast<std::uint16_t>(global_idx / 360);
                    packet.col[i] = static_cast<std::uint16_t>(global_idx % 360);
                    packet.intensity[i] = static_cast<std::uint8_t>(global_idx % 255);
                    packet.point_x[i] = static_cast<std::int16_t>(1000 + i);
                    packet.point_y[i] = static_cast<std::int16_t>(2000 + i);
                    packet.point_z[i] = static_cast<std::int16_t>(3000 + i);
                }
                const auto sent = sendto(sock_, &packet, sizeof(packet), 0,
                                         reinterpret_cast<sockaddr*>(&client), client_len);
                require(sent == static_cast<ssize_t>(sizeof(packet)), "fake WS30 send failed");
            }
        }
    }

    int sock_ = -1;
    bool stop_ = false;
    std::thread worker_;
};

void test_reassembles_complete_frame() {
    FakeWs30Server server;

    Ws30Config cfg;
    cfg.enabled = true;
    cfg.host = "127.0.0.1";
    cfg.points_port = kPointsPort;
    cfg.imu_port = kImuPort;
    cfg.scan_port = kScanPort;
    cfg.handshake_interval_ms = 20;
    cfg.receive_imu = false;

    Ws30Receiver receiver(cfg);
    require(receiver.is_initialized(), "receiver should initialize on localhost");

    std::optional<LidarFrame> frame;
    for (int attempt = 0; attempt < 120; ++attempt) {
        frame = receiver.latest_frame();
        if (frame.has_value()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    require(frame.has_value(), "receiver should publish a lidar frame");
    require(frame->points.size() == 54000, "frame should contain 54000 points");
    require(frame->timestamp_ns == 1234567000ULL, "frame timestamp mismatch");
    require_near(frame->points[0].x_mm, 2.0F, 1e-3F, "first point x mismatch");
    require_near(frame->points[0].y_mm, 4.0F, 1e-3F, "first point y mismatch");
    require_near(frame->points[0].z_mm, 6.0F, 1e-3F, "first point z mismatch");
    require(frame->points[0].row == 0, "first point row mismatch");
    require(frame->points[0].col == 0, "first point col mismatch");
    require(frame->points[53999].row == 149, "last point row mismatch");
    require(frame->points[53999].col == 359, "last point col mismatch");
}

} // namespace

int main() {
    std::println("ws30_receiver_test:");
    test_reassembles_complete_frame();
    std::println("PASSED");
    return 0;
}
