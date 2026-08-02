#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>

#include "config.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct GameStateSample {
    std::uint8_t game_type = 0;
    std::uint8_t game_progress = 0;
    std::uint16_t stage_remain_time = 0;
    std::uint64_t sync_timestamp = 0;
};

struct MarkSample {
    bool opponent_aerial_targeted = false;
    bool opponent_aerial_countered = false;
    bool opponent_aerial_marked = false;
};

// radar-egui 0x0001 JSON（与 radar_bridge 收到的同一格式）
auto parse_game_state_json(std::string_view json) -> std::optional<GameStateSample>;
// radar-egui 0x020C JSON
auto parse_mark_json(std::string_view json) -> std::optional<MarkSample>;

// 比赛窗口状态机（纯逻辑，now_ns 为本地 steady 时钟纳秒，测试可注入）
class RefereeWindow {
public:
    explicit RefereeWindow(int match_duration_s = 420);

    void update(std::uint8_t game_progress, std::int64_t now_ns);
    /// 每帧调用：窗口内且本地计时已超过 match_duration_s 时强制退出（断流场景也生效）
    void expire_if_over(std::int64_t now_ns);
    [[nodiscard]] auto signal_available() const -> bool { return signal_available_; }
    [[nodiscard]] auto in_window() const -> bool { return in_window_; }
    // 无信号不门控（旧行为）；有信号时仅在比赛窗口内放行
    [[nodiscard]] auto allowed() const -> bool { return !signal_available_ || in_window_; }
    [[nodiscard]] auto consume_match_started() -> bool;
    [[nodiscard]] auto match_elapsed_s(std::int64_t now_ns) const -> std::int64_t;

private:
    int match_duration_s_{};
    bool signal_available_ = false;
    bool in_window_ = false;
    std::int64_t start_ns_ = 0;
    bool match_started_pending_ = false;
    // 420s 硬超时后窗口终结，须收到 progress 5 才允许 re-arm
    bool timed_out_ = false;
};

} // namespace rmcs_laser_guidance::runtime_internal

namespace rmcs_laser_guidance {
struct RefereeSnapshot;
} // namespace rmcs_laser_guidance

namespace rmcs_laser_guidance::runtime_internal {

// RefereeLink 内部持有 zmq 对象，通过 Impl PIMPL 隔离，公共头不暴露 libzmq 类型
class RefereeLink {
public:
    explicit RefereeLink(RefereeConfig config);
    ~RefereeLink();
    auto poll() -> void;
    [[nodiscard]] auto guidance_allowed() const -> bool;
    [[nodiscard]] auto hit_progress_allowed() const -> bool;
    [[nodiscard]] auto consume_match_started() -> bool;
    [[nodiscard]] auto consume_countered_edge() -> bool;
    [[nodiscard]] auto snapshot() const -> RefereeSnapshot;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rmcs_laser_guidance::runtime_internal
