#include "runtime/referee_link.hpp"

#include <algorithm>
#include <nlohmann/json.hpp>

namespace rmcs_laser_guidance::runtime_internal {
namespace {

constexpr std::uint16_t kGameStateCmdId = 0x0001;
constexpr std::uint16_t kMarkCmdId = 0x020C;

// radar-egui 以 0/1 整数下发布尔位，nlohmann 的 get<bool>() 只接受 JSON true/false
auto to_bool(const nlohmann::json& value) -> bool {
    if (value.is_boolean())
        return value.get<bool>();
    return value.is_number() && value.get<int>() != 0;
}

} // namespace

auto parse_game_state_json(std::string_view json) -> std::optional<GameStateSample> {
    try {
        const auto parsed = nlohmann::json::parse(json);
        if (parsed.at("cmd_id").get<std::uint16_t>() != kGameStateCmdId)
            return std::nullopt;
        GameStateSample sample;
        sample.game_type = parsed.at("game_type").get<std::uint8_t>();
        sample.game_progress = parsed.at("game_progress").get<std::uint8_t>();
        sample.stage_remain_time = parsed.at("stage_remain_time").get<std::uint16_t>();
        sample.sync_timestamp = parsed.at("sync_timestamp").get<std::uint64_t>();
        if (sample.game_progress > 5)
            return std::nullopt;
        return sample;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

auto parse_mark_json(std::string_view json) -> std::optional<MarkSample> {
    try {
        const auto parsed = nlohmann::json::parse(json);
        if (parsed.at("cmd_id").get<std::uint16_t>() != kMarkCmdId)
            return std::nullopt;
        MarkSample sample;
        sample.opponent_aerial_targeted = to_bool(parsed.at("opponent_aerial_targeted"));
        sample.opponent_aerial_countered = to_bool(parsed.at("opponent_aerial_countered"));
        sample.opponent_aerial_marked = to_bool(parsed.at("opponent_aerial_marked"));
        return sample;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

RefereeWindow::RefereeWindow(int match_duration_s)
    : match_duration_s_(std::max(1, match_duration_s)) {}

void RefereeWindow::update(std::uint8_t game_progress, std::int64_t now_ns) {
    signal_available_ = true;
    if (game_progress == 5) {
        in_window_ = false;
        start_ns_ = 0;
        return;
    }
    if (in_window_) {
        if (now_ns - start_ns_ >= static_cast<std::int64_t>(match_duration_s_) * 1'000'000'000) {
            in_window_ = false;
            start_ns_ = 0;
        }
        return;
    }
    if (game_progress == 4) {
        in_window_ = true;
        start_ns_ = now_ns;
        match_started_pending_ = true;
    }
}

auto RefereeWindow::consume_match_started() -> bool {
    const bool pending = match_started_pending_;
    match_started_pending_ = false;
    return pending;
}

auto RefereeWindow::match_elapsed_s(std::int64_t now_ns) const -> std::int64_t {
    if (!in_window_)
        return -1;
    return std::max<std::int64_t>(0, (now_ns - start_ns_) / 1'000'000'000);
}

} // namespace rmcs_laser_guidance::runtime_internal
