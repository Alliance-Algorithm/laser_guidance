#include <print>
#include <stdexcept>

#include "laser_guidance/runtime.hpp"
#include "runtime/referee_link.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::runtime_internal;
        using rmcs_laser_guidance::tests::require;

        // ---- JSON 解析 ----
        {
            const auto sample = parse_game_state_json(
                R"({"cmd_id":1,"game_type":1,"game_progress":4,"stage_remain_time":300,"sync_timestamp":12345})");
            require(sample.has_value(), "valid game state parses");
            require(sample->game_type == 1, "game_type parsed");
            require(sample->game_progress == 4, "game_progress parsed");
            require(sample->stage_remain_time == 300, "stage_remain_time parsed");
            require(sample->sync_timestamp == 12345, "sync_timestamp parsed");
        }
        {
            require(!parse_game_state_json(R"({"cmd_id":999,"game_progress":4})").has_value(),
                "wrong cmd_id rejected");
            require(!parse_game_state_json("not json").has_value(), "garbage rejected");
            require(!parse_game_state_json(R"({"cmd_id":1,"game_progress":9})").has_value(),
                "invalid game_progress rejected");
        }
        {
            const auto mark = parse_mark_json(
                R"({"cmd_id":524,"opponent_aerial_targeted":1,"opponent_aerial_countered":1,"opponent_aerial_marked":0})");
            require(mark.has_value(), "valid mark parses");
            require(mark->opponent_aerial_targeted && mark->opponent_aerial_countered,
                "targeted/countered parsed");
            require(!mark->opponent_aerial_marked, "marked parsed");
        }

        // ---- 状态机：无信号 ----
        {
            RefereeWindow w;
            require(!w.in_window(), "no signal not in window");
            require(w.signal_available() == false, "no signal flag");
            require(w.match_elapsed_s(1'000'000'000) == -1, "no elapsed before start");
            w.update(1, 0, 5'000'000'000'000);  // 准备阶段
            require(!w.in_window(), "prep phase not in window");
            require(w.signal_available(), "signal now available");
        }

        // ---- 4 进入 + 边沿 + 计时 ----
        {
            RefereeWindow w;
            w.update(1, 0, 1'000'000'000);
            w.update(2, 0, 2'000'000'000);
            w.update(3, 0, 3'000'000'000);
            require(!w.consume_match_started(), "no edge before match");
            w.update(4, 300, 4'000'000'000);
            require(w.in_window(), "match enters window");
            require(w.consume_match_started(), "match_started edge fired");
            require(!w.consume_match_started(), "edge consumed once");
            require(w.match_elapsed_s(14'000'000'000) == 10, "elapsed = local clock diff");
        }

        // ---- 5 退出 + re-arm ----
        {
            RefereeWindow w;
            w.update(4, 300, 4'000'000'000);
            w.consume_match_started();
            w.update(5, 0, 424'000'000'000);
            require(!w.in_window(), "settle exits window");
            require(w.match_elapsed_s(425'000'000'000) == -1, "elapsed reset");
            w.update(1, 0, 430'000'000'000);
            w.update(4, 300, 500'000'000'000);
            require(w.in_window(), "next round re-arms");
            require(w.consume_match_started(), "re-arm fires edge again");
        }

        // ---- 比赛窗口以裁判 stage_remain_time 为权威（技术暂停时本地墙钟会误判）----
        {
            RefereeWindow w;
            w.update(4, 300, 4'000'000'000);
            w.consume_match_started();
            // 暂停冻结：裁判仍报剩余 100s，但本地墙钟已过 420s
            w.update(4, 100, 430'000'000'000);   // 墙钟 426s > 420
            require(w.in_window(), "live signal stays in window past 420s wall clock");
            require(w.match_elapsed_s(430'000'000'000) == 426, "elapsed still tracks local clock");
            // 裁判剩余时间归零 → 权威关窗
            w.update(4, 0, 431'000'000'000);
            require(!w.in_window(), "stage_remain_time==0 exits window");
            require(!w.consume_match_started(), "no edge on authoritative close");
            // 关窗是终态的：收到 5 之前 4 不再重开
            w.update(4, 100, 432'000'000'000);
            require(!w.in_window(), "authoritative close is terminal until 5");
            w.update(5, 0, 433'000'000'000);
            w.update(4, 300, 500'000'000'000);
            require(w.in_window(), "5 clears, next 4 re-arms");
            require(w.consume_match_started(), "re-arm after 5 fires edge");
        }

        // ---- 断流兜底：update 不再按墙钟关窗，只有 expire_if_over 兜底 ----
        {
            RefereeWindow w;
            w.update(4, 300, 4'000'000'000);
            w.consume_match_started();
            // 断流后仅本地时钟推进（消息陈旧，stage_remain_time 不再可信）
            w.update(4, 300, 500'000'000'000);   // 墙钟 496s > 420
            require(w.in_window(), "update alone never wall-clock expires");
            // 断流兜底由 expire_if_over 承担
            w.expire_if_over(501'000'000'000);
            require(!w.in_window(), "no-signal fallback expires at 420s wall clock");
            w.expire_if_over(502'000'000'000);
            require(!w.in_window(), "expired window stays closed");
            w.update(5, 0, 503'000'000'000);
            w.update(4, 300, 600'000'000'000);
            require(w.in_window(), "5 clears timeout, next 4 re-arms");
            require(w.consume_match_started(), "re-arm after expiry fires edge");
        }

        // ---- 断流且无任何消息：仅 expire_if_over 按本地时钟到期 ----
        {
            RefereeWindow w;
            w.update(4, 300, 4'000'000'000);     // 窗口开启，start=4s
            w.consume_match_started();
            // 此后不再调用 update()，模拟发布方死亡
            w.expire_if_over(423'000'000'000);   // 419s
            require(w.in_window(), "no-message window still open at 419s");
            require(w.match_elapsed_s(423'000'000'000) == 419, "elapsed tracks local clock");
            w.expire_if_over(424'100'000'000);   // 420.1s
            require(!w.in_window(), "no-message window expires at 420s");
            require(!w.consume_match_started(), "no match_started edge on expiry");
            w.expire_if_over(500'000'000'000);
            require(!w.in_window(), "expired window stays closed on repeated calls");
            w.update(5, 0, 501'000'000'000);   // 5 清除 timed_out_
            w.update(4, 300, 600'000'000'000);
            require(w.in_window(), "5 clears timeout, next 4 re-arms after expiry");
            require(w.consume_match_started(), "re-arm after expiry fires edge");
        }

        // ---- 官方反制计数（0x020C 边沿，每局归零，上限 5，不丢边沿）----
        {
            RefereeWindow w;
            require(w.official_counter_count() == 0, "initial official count 0");
            w.note_official_counter();
            w.note_official_counter();
            require(w.official_counter_count() == 2, "edges accumulate");
            for (int i = 0; i < 6; ++i)
                w.note_official_counter();
            require(w.official_counter_count() == 5, "capped at 5 (per-round max locks)");
        }

        // ---- 每局开局（progress 4）归零官方计数 ----
        {
            RefereeWindow w;
            w.note_official_counter();
            w.note_official_counter();
            w.update(4, 300, 4'000'000'000);   // 窗口开启 = 新一局
            require(w.official_counter_count() == 0, "match start resets official count");
            w.note_official_counter();
            w.update(5, 0, 424'000'000'000);    // 首局结束
            w.update(4, 300, 500'000'000'000);  // 第二局
            require(w.official_counter_count() == 0, "second round starts fresh");
        }

        // ---- 窗口外计数不触发（边沿本身由 RefereeLink 判断，这里验证计数独立）----
        {
            RefereeWindow w;
            w.update(1, 0, 1'000'000'000);      // 准备阶段
            w.note_official_counter();
            require(w.official_counter_count() == 1, "count tracks edges regardless of phase");
        }

        // ---- RefereeLink enabled=false 空转不抛异常 ----
        {
            rmcs_laser_guidance::RefereeConfig cfg;
            cfg.enabled = false;
            RefereeLink link(cfg);
            link.poll();
            require(!link.snapshot().signal_available, "disabled link no signal");
            require(!link.consume_match_started(), "no edges when disabled");
            require(!link.consume_countered_edge(), "no countered edge when disabled");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "referee_link_test failed: {}", e.what());
        return 1;
    }
}
