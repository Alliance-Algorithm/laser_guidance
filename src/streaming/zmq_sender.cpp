#include "streaming/zmq_sender.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <print>

#include <zmq.hpp>

namespace rmcs_laser_guidance {
namespace {

constexpr int kLaserCmdId = 0x2003;

auto finite_value(const float value) -> float {
    return std::isfinite(value) ? value : 0.0F;
}

auto append_json_float(std::ostringstream& out, const float value) -> void {
    out << std::setprecision(std::numeric_limits<float>::max_digits10) << finite_value(value);
}

auto append_json_point(std::ostringstream& out, const cv::Point2f& point) -> void {
    out << '[';
    append_json_float(out, point.x);
    out << ',';
    append_json_float(out, point.y);
    out << ']';
}

auto append_json_bbox(std::ostringstream& out, const cv::Rect2f& bbox) -> void {
    out << '[';
    append_json_float(out, bbox.x);
    out << ',';
    append_json_float(out, bbox.y);
    out << ',';
    append_json_float(out, bbox.width);
    out << ',';
    append_json_float(out, bbox.height);
    out << ']';
}

}

struct ZmqSender::Impl {
    zmq::context_t ctx{1};
    zmq::socket_t pub{ctx, zmq::socket_type::pub};
    bool enabled = false;
};

ZmqSender::ZmqSender(ZmqConfig config)
    : impl_(std::make_unique<Impl>()) {
    if (!config.enabled)
        return;
    impl_->pub.bind(std::format("tcp://*:{}", config.port));
    impl_->enabled = true;
    std::println("ZMQ sender: tcp://*:{}", config.port);
}

ZmqSender::~ZmqSender() = default;

auto serialize_laser_json(const TargetObservation& observation) -> std::string {
    std::ostringstream out;
    out << '{';
    out << "\"cmd_id\":" << kLaserCmdId;
    out << ",\"detected\":" << (observation.detected ? "true" : "false");
    out << ",\"center\":";
    append_json_point(out, observation.center);
    out << ",\"brightness\":";
    append_json_float(out, observation.brightness);
    out << ",\"contour\":[";
    for (std::size_t index = 0; index < observation.contour.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        append_json_point(out, observation.contour[index]);
    }
    out << ']';
    out << ",\"candidates\":[";
    for (std::size_t index = 0; index < observation.candidates.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        const auto& candidate = observation.candidates[index];
        out << '{';
        out << "\"score\":";
        append_json_float(out, candidate.score);
        out << ",\"class_id\":" << candidate.class_id;
        out << ",\"bbox\":";
        append_json_bbox(out, candidate.bbox);
        out << ",\"center\":";
        append_json_point(out, candidate.center);
        out << '}';
    }
    out << ']';
    out << '}';
    return out.str();
}

auto ZmqSender::send(const TargetObservation& observation) -> void {
    if (!impl_->enabled)
        return;

    const std::string payload = serialize_laser_json(observation);
    impl_->pub.send(zmq::buffer(payload), zmq::send_flags::dontwait);
}

} // namespace rmcs_laser_guidance
