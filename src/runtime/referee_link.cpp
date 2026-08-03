#include "runtime/referee_link.hpp"

#include <algorithm>
#include <chrono>
#include <nlohmann/json.hpp>
#include <utility>
#include <zmq.hpp>

#include "laser_guidance/runtime.hpp"

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

void RefereeWindow::update(
    std::uint8_t game_progress, std::uint16_t stage_remain_time, std::int64_t now_ns) {
    signal_available_ = true;
    if (game_progress == 5) {
        in_window_ = false;
        start_ns_ = 0;
        match_started_pending_ = false;
        timed_out_ = false;
        return;
    }
    if (in_window_) {
        // 裁判剩余时间归零 → 局结束。stage_remain_time 在技术暂停时冻结，
        // 比本地墙钟可靠；本地 match_duration_s 只作断流兜底（expire_if_over）。
        if (stage_remain_time == 0) {
            in_window_ = false;
            start_ns_ = 0;
            timed_out_ = true;
        }
        return;
    }
    if (game_progress == 4 && !timed_out_) {
        in_window_ = true;
        start_ns_ = now_ns;
        match_started_pending_ = true;
        official_counter_count_ = 0;
    }
}

void RefereeWindow::note_official_counter() {
    if (official_counter_count_ < 5)
        ++official_counter_count_;
}

void RefereeWindow::expire_if_over(std::int64_t now_ns) {
    if (!in_window_)
        return;
    if (now_ns - start_ns_ >= static_cast<std::int64_t>(match_duration_s_) * 1'000'000'000) {
        in_window_ = false;
        start_ns_ = 0;
        timed_out_ = true;
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

namespace {

constexpr std::int64_t kNsPerSecond = 1'000'000'000;

auto now_ns() -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               rmcs_laser_guidance::Clock::now().time_since_epoch()).count();
}

} // namespace

struct RefereeLink::Impl {
    explicit Impl(RefereeConfig config)
        : config(std::move(config))
        , window(config.match_duration_s) {}

    RefereeConfig config;
    zmq::context_t ctx{1};
    std::unique_ptr<zmq::socket_t> sub;
    RefereeWindow window;
    GameStateSample game_state;
    MarkSample mark;
    bool last_countered = false;
    bool countered_pending = false;
    std::uint64_t parse_errors = 0;
    std::optional<rmcs_laser_guidance::Clock::time_point> last_message_time;
};
RefereeLink::RefereeLink(RefereeConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
RefereeLink::~RefereeLink() = default;

auto RefereeLink::poll() -> void {
    if (!impl_->config.enabled)
        return;
    // 断流兜底只在无合法消息流时生效（超过 signal_timeout_s）：有实时信号时
    // 比赛窗口由裁判 stage_remain_time 判定，本地 match_duration_s 兜底会误关
    // 技术暂停中的比赛窗口。
    const bool stream_alive = impl_->last_message_time.has_value()
        && std::chrono::duration<double>(
               rmcs_laser_guidance::Clock::now() - *impl_->last_message_time).count()
               <= static_cast<double>(impl_->config.signal_timeout_s);
    if (!stream_alive)
        impl_->window.expire_if_over(now_ns());
    if (!impl_->sub) {
        impl_->sub = std::make_unique<zmq::socket_t>(impl_->ctx, zmq::socket_type::sub);
        impl_->sub->connect(impl_->config.zmq_address);
        impl_->sub->set(zmq::sockopt::subscribe, "");
    }
    while (true) {
        zmq::message_t message;
        if (!impl_->sub->recv(message, zmq::recv_flags::dontwait))
            break;
        const std::string_view text(static_cast<const char*>(message.data()), message.size());
        if (const auto state = parse_game_state_json(text); state.has_value()) {
            impl_->game_state = *state;
            impl_->window.update(state->game_progress, state->stage_remain_time, now_ns());
            impl_->last_message_time = rmcs_laser_guidance::Clock::now();
        } else if (const auto mark = parse_mark_json(text); mark.has_value()) {
            impl_->mark = *mark;
            if (mark->opponent_aerial_countered && !impl_->last_countered) {
                impl_->countered_pending = true;
                // 官方反制次数：不被本地锁定期门控，官方报一次计一次
                impl_->window.note_official_counter();
            }
            impl_->last_countered = mark->opponent_aerial_countered;
            impl_->last_message_time = rmcs_laser_guidance::Clock::now();
        } else {
            ++impl_->parse_errors;
        }
    }
}

auto RefereeLink::consume_match_started() -> bool { return impl_->window.consume_match_started(); }
auto RefereeLink::in_window() const -> bool { return impl_->window.in_window(); }
auto RefereeLink::official_counter_count() const -> int {
    return impl_->window.official_counter_count();
}
auto RefereeLink::consume_countered_edge() -> bool {
    const bool pending = impl_->countered_pending;
    impl_->countered_pending = false;
    return pending;
}

auto RefereeLink::snapshot() const -> rmcs_laser_guidance::RefereeSnapshot {
    rmcs_laser_guidance::RefereeSnapshot snap;
    snap.signal_available = impl_->window.signal_available();
    snap.game_progress = impl_->game_state.game_progress;
    snap.game_type = impl_->game_state.game_type;
    snap.match_elapsed_s = impl_->window.match_elapsed_s(now_ns());
    snap.stage_remain_time = impl_->game_state.stage_remain_time;
    snap.official_aerial_targeted = impl_->mark.opponent_aerial_targeted;
    snap.official_aerial_countered = impl_->mark.opponent_aerial_countered;
    if (impl_->last_message_time.has_value()) {
        const auto age = std::chrono::duration<double>(
            rmcs_laser_guidance::Clock::now() - *impl_->last_message_time).count();
        snap.last_message_age_s = age;
        // 看门狗：断流只告警，不中断任何执行（断流续导语义）
        snap.signal_stale = age > static_cast<double>(impl_->config.signal_timeout_s);
    }
    snap.parse_errors = impl_->parse_errors;
    return snap;
}

} // namespace rmcs_laser_guidance::runtime_internal
