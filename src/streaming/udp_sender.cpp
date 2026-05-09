#include "streaming/udp_sender.hpp"

#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <print>
#include <sys/socket.h>
#include <unistd.h>

namespace rmcs_laser_guidance {

UdpSender::UdpSender(UdpConfig config)
    : config_(std::move(config)) {
    if (!config_.enabled) return;
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        std::println(stderr, "UDP: failed to create socket");
        return;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(config_.port));
    inet_pton(AF_INET, config_.host.c_str(), &addr.sin_addr);
    if (connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::println(stderr, "UDP: failed to connect");
    }
    std::println("UDP sender: {}:{}", config_.host, config_.port);
}

UdpSender::~UdpSender() {
    if (sock_ >= 0) close(sock_);
}

auto UdpSender::send(const TargetObservation& observation) -> void {
    if (sock_ < 0) return;

    std::uint8_t buf[4096];
    std::size_t offset = 0;

    buf[offset++] = 0x47;
    buf[offset++] = 0x4C;
    buf[offset++] = 1;
    buf[offset++] = seq_++;

    auto now = std::chrono::steady_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
    std::memcpy(buf + offset, &us, 8); offset += 8;

    std::size_t len_offset = offset;
    offset += 4;

    std::size_t payload_start = offset;
    buf[offset++] = observation.detected ? static_cast<std::uint8_t>(1) : static_cast<std::uint8_t>(0);

    float cx = observation.center.x;
    float cy = observation.center.y;
    std::memcpy(buf + offset, &cx, 4); offset += 4;
    std::memcpy(buf + offset, &cy, 4); offset += 4;
    std::memcpy(buf + offset, &observation.brightness, 4); offset += 4;

    std::uint32_t contour_count = static_cast<std::uint32_t>(observation.contour.size());
    std::memcpy(buf + offset, &contour_count, 4); offset += 4;
    for (const auto& p : observation.contour) {
        float px = p.x, py = p.y;
        std::memcpy(buf + offset, &px, 4); offset += 4;
        std::memcpy(buf + offset, &py, 4); offset += 4;
    }

    std::uint32_t cand_count = static_cast<std::uint32_t>(observation.candidates.size());
    std::memcpy(buf + offset, &cand_count, 4); offset += 4;
    for (const auto& c : observation.candidates) {
        std::memcpy(buf + offset, &c.score, 4); offset += 4;
        std::int32_t cid = c.class_id;
        std::memcpy(buf + offset, &cid, 4); offset += 4;
        float bx = c.bbox.x, by = c.bbox.y, bw = c.bbox.width, bh = c.bbox.height;
        std::memcpy(buf + offset, &bx, 4); offset += 4;
        std::memcpy(buf + offset, &by, 4); offset += 4;
        std::memcpy(buf + offset, &bw, 4); offset += 4;
        std::memcpy(buf + offset, &bh, 4); offset += 4;
        float ccx = c.center.x, ccy = c.center.y;
        std::memcpy(buf + offset, &ccx, 4); offset += 4;
        std::memcpy(buf + offset, &ccy, 4); offset += 4;
    }

    std::uint32_t payload_len = static_cast<std::uint32_t>(offset - payload_start);
    std::memcpy(buf + len_offset, &payload_len, 4);

    ::send(sock_, buf, offset, MSG_DONTWAIT);
}

}
