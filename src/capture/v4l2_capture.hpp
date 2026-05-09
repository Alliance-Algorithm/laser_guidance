#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include <opencv2/videoio.hpp>

#include "config.hpp"
#include "types.hpp"

namespace rmcs_laser_guidance {

struct V4l2NegotiatedFormat {
    std::filesystem::path device_path{};
    int width = 0;
    int height = 0;
    double framerate = 0.0;
    std::string fourcc{};
};

class V4l2Capture {
public:
    explicit V4l2Capture(V4l2Config config);
    ~V4l2Capture() noexcept;

    auto open() -> std::expected<V4l2NegotiatedFormat, std::string>;
    auto read_frame() -> std::expected<Frame, std::string>;
    auto close() noexcept -> void;

    [[nodiscard]] auto is_open() const noexcept -> bool;
    [[nodiscard]] auto negotiated_format() const noexcept -> const V4l2NegotiatedFormat&;

private:
    V4l2Config config_;
    cv::VideoCapture capture_;
    V4l2NegotiatedFormat negotiated_{};
};

auto fourcc_string_from_int(int fourcc) -> std::string;

} // namespace rmcs_laser_guidance
