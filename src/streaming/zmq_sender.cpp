#include "streaming/zmq_sender.hpp"

#include <cmath>
#include <limits>
#include <print>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

namespace rmcs_laser_guidance {
namespace {

constexpr int kLaserCmdId = 0x2003;

auto finite_value(const float value) -> float {
    return std::isfinite(value) ? value : 0.0F;
}

}

struct ZmqSender::Impl {
    zmq::context_t ctx{1};
    zmq::socket_t pub{ctx, zmq::socket_type::pub};
    bool enabled = false;
};

// ⚠️ 禁止启用（ZmqConfig 注释）：端口与 SDR bridge(:5555)/radar_bridge(:5556)
// 冲突，且 egui 端未消费该数据。请勿将 yaml 的 zmq.enabled 设为 true。
ZmqSender::ZmqSender(ZmqConfig config)
    : impl_(std::make_unique<Impl>()) {
    if (!config.enabled)
        return;
    impl_->pub.bind(std::format("tcp://*:{}", config.port));
    impl_->enabled = true;
    std::println(stderr, "ZMQ sender: tcp://*:{}", config.port);
}

ZmqSender::~ZmqSender() = default;

auto serialize_laser_json(const TargetObservation& observation) -> std::string {
    nlohmann::json j;

    j["cmd_id"] = kLaserCmdId;
    j["detected"] = observation.detected;
    j["center"] = {finite_value(observation.center.x), finite_value(observation.center.y)};
    j["brightness"] = finite_value(observation.brightness);

    auto& contour_arr = j["contour"] = nlohmann::json::array();
    for (const auto& pt : observation.contour) {
        contour_arr.push_back({finite_value(pt.x), finite_value(pt.y)});
    }

    auto& candidates_arr = j["candidates"] = nlohmann::json::array();
    for (const auto& candidate : observation.candidates) {
        nlohmann::json c;
        c["score"] = finite_value(candidate.score);
        c["class_id"] = candidate.class_id;
        c["bbox"] = {finite_value(candidate.bbox.x), finite_value(candidate.bbox.y),
                     finite_value(candidate.bbox.width), finite_value(candidate.bbox.height)};
        c["center"] = {finite_value(candidate.center.x), finite_value(candidate.center.y)};
        candidates_arr.push_back(std::move(c));
    }

    return j.dump();
}

auto ZmqSender::send(const TargetObservation& observation) -> void {
    if (!impl_->enabled)
        return;

    const std::string payload = serialize_laser_json(observation);
    impl_->pub.send(zmq::buffer(payload), zmq::send_flags::dontwait);
}

} // namespace rmcs_laser_guidance
