#include <print>
#include <stdexcept>

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

        // ---- 状态机：无信号不门控 ----
        {
            RefereeWindow w;
            require(w.allowed(), "no signal -> ungated (old behavior)");
            require(w.signal_available() == false, "no signal flag");
            require(w.match_elapsed_s(1'000'000'000) == -1, "no elapsed before start");
            w.update(1, 5'000'000'000'000);  // 准备阶段
            require(w.allowed() == false, "prep phase gated");
            require(w.signal_available(), "signal now available");
        }

        // ---- 4 进入 + 边沿 + 计时 ----
        {
            RefereeWindow w;
            w.update(1, 1'000'000'000);
            w.update(2, 2'000'000'000);
            w.update(3, 3'000'000'000);
            require(!w.consume_match_started(), "no edge before match");
            w.update(4, 4'000'000'000);
            require(w.allowed(), "match -> allowed");
            require(w.consume_match_started(), "match_started edge fired");
            require(!w.consume_match_started(), "edge consumed once");
            require(w.match_elapsed_s(14'000'000'000) == 10, "elapsed = local clock diff");
        }

        // ---- 5 退出 + re-arm ----
        {
            RefereeWindow w;
            w.update(4, 4'000'000'000);
            w.consume_match_started();
            w.update(5, 424'000'000'000);
            require(!w.allowed(), "settle exits window");
            require(w.match_elapsed_s(425'000'000'000) == -1, "elapsed reset");
            w.update(1, 430'000'000'000);
            w.update(4, 500'000'000'000);
            require(w.allowed(), "next round re-arms");
            require(w.consume_match_started(), "re-arm fires edge again");
        }

        // ---- 420s 硬窗口超时 ----
        {
            RefereeWindow w;
            w.update(4, 4'000'000'000);
            w.consume_match_started();
            w.update(4, 423'000'000'000);   // 419s
            require(w.allowed(), "within window at 419s");
            w.update(4, 424'100'000'000);   // 420.1s
            require(!w.allowed(), "exits at 420s despite progress still 4");
        }

        // ---- 断流：窗口判定继续由本地时钟驱动 ----
        {
            RefereeWindow w;
            w.update(4, 4'000'000'000);
            w.consume_match_started();
            w.update(4, 10'000'000'000);    // 6s
            w.update(4, 200'000'000'000);   // 196s（模拟断流后仅本地时钟推进）
            require(w.allowed(), "stale signal keeps window");
            w.update(4, 500'000'000'000);   // 496s > 420
            require(!w.allowed(), "stale signal still expires at 420s");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "referee_link_test failed: {}", e.what());
        return 1;
    }
}
