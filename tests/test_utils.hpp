#pragma once

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rmcs_laser_guidance::tests {

inline auto default_config_path() -> std::filesystem::path {
    return "config/default.yaml";
}

inline auto default_sample_replay_path() -> std::filesystem::path {
    return "test_data/sample_images";
}

inline auto default_video_session_root() -> std::filesystem::path {
    return "videos";
}

inline auto make_temp_path(std::string_view stem) -> std::filesystem::path {
    const auto unique =
        std::to_string(std::filesystem::file_time_type::clock::now().time_since_epoch().count());
    return std::filesystem::temp_directory_path() / (std::string(stem) + "_" + unique + ".yaml");
}

inline auto make_temp_dir(std::string_view stem) -> std::filesystem::path {
    const auto unique =
        std::to_string(std::filesystem::file_time_type::clock::now().time_since_epoch().count());
    const auto path = std::filesystem::temp_directory_path() / (std::string(stem) + "_" + unique);
    std::filesystem::create_directories(path);
    return path;
}

inline auto find_free_udp_port() -> std::uint16_t {
    const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        throw std::runtime_error("failed to create UDP socket");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(0);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(sock);
        throw std::runtime_error("failed to bind UDP socket");
    }

    sockaddr_in bound{};
    socklen_t len = sizeof(bound);
    if (::getsockname(sock, reinterpret_cast<sockaddr*>(&bound), &len) != 0) {
        ::close(sock);
        throw std::runtime_error("failed to read UDP socket name");
    }

    const auto port = ntohs(bound.sin_port);
    ::close(sock);
    return port;
}

inline auto write_text_file(const std::filesystem::path& path, std::string_view content) -> void {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());

    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("failed to open temp file for writing");
    out << content;
}

inline auto read_text_file(const std::filesystem::path& path) -> std::string {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("failed to open file for reading");

    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
}

inline auto require(const bool condition, std::string_view message) -> void {
    if (!condition)
        throw std::runtime_error(std::string(message));
}

inline auto require_contains(
    const std::string& actual, std::string_view expected_fragment, std::string_view label) -> void {
    if (actual.contains(expected_fragment))
        return;

    std::ostringstream oss;
    oss << label << " expected to contain '" << expected_fragment << "', got '" << actual << "'";
    throw std::runtime_error(oss.str());
}

inline auto require_near(
    const float actual, const float expected, const float tolerance, std::string_view label)
    -> void {
    if (std::fabs(actual - expected) <= tolerance)
        return;

    std::ostringstream oss;
    oss << label << " expected " << expected << " +/- " << tolerance << ", got " << actual;
    throw std::runtime_error(oss.str());
}

} // namespace rmcs_laser_guidance::tests
