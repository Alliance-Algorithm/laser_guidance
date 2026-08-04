#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <print>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

namespace {

using namespace std::chrono_literals;

struct Options {
    std::string bind = "tcp://*:5561";
    int interval_ms = 1000;
    int prep_s = 300;        // progress 1
    int selfcheck_s = 15;    // progress 2
    int countdown_s = 5;     // progress 3
    int match_s = 420;       // progress 4
    int settle_s = 10;       // progress 5
    std::vector<int> counter_at_s{60};   // 比赛开始后第几秒触发 0x020C 反制（逗号分隔多个）
    int counter_duration_s = 45;
    bool loop = false;
    bool show_help = false;
};

auto print_help() -> void {
    std::println(
        "usage: tool_referee_sim [--bind tcp://*:5561] [--interval-ms 1000] "
        "[--prep-s 300] [--selfcheck-s 15] [--countdown-s 5] [--match-s 420] "
        "[--settle-s 10] [--counter-at-s 60,130,200] [--counter-duration-s 5] [--loop]");
}

auto parse_counter_times(const char* value) -> std::vector<int> {
    std::vector<int> times;
    std::string_view view(value);
    while (!view.empty()) {
        const auto comma = view.find(',');
        const auto piece = view.substr(0, comma);
        times.push_back(std::atoi(std::string(piece).c_str()));
        if (comma == std::string_view::npos)
            break;
        view.remove_prefix(comma + 1);
    }
    if (times.empty())
        times.push_back(60);
    return times;
}

auto parse_options(int argc, char** argv) -> Options {
    Options opt;
    const auto need_value = [&](int& i, const char* flag) -> std::string {
        if (i + 1 >= argc) {
            std::println(stderr, "error: {} requires a value", flag);
            std::exit(1);
        }
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--bind") opt.bind = need_value(i, "--bind");
        else if (arg == "--interval-ms") opt.interval_ms = std::atoi(need_value(i, "--interval-ms").c_str());
        else if (arg == "--prep-s") opt.prep_s = std::atoi(need_value(i, "--prep-s").c_str());
        else if (arg == "--selfcheck-s") opt.selfcheck_s = std::atoi(need_value(i, "--selfcheck-s").c_str());
        else if (arg == "--countdown-s") opt.countdown_s = std::atoi(need_value(i, "--countdown-s").c_str());
        else if (arg == "--match-s") opt.match_s = std::atoi(need_value(i, "--match-s").c_str());
        else if (arg == "--settle-s") opt.settle_s = std::atoi(need_value(i, "--settle-s").c_str());
        else if (arg == "--counter-at-s") opt.counter_at_s = parse_counter_times(need_value(i, "--counter-at-s").c_str());
        else if (arg == "--counter-duration-s") opt.counter_duration_s = std::atoi(need_value(i, "--counter-duration-s").c_str());
        else if (arg == "--loop") opt.loop = true;
        else if (arg == "--help" || arg == "-h") opt.show_help = true;
        else {
            std::println(stderr, "error: unknown option {}", arg);
            std::exit(1);
        }
    }
    return opt;
}

auto game_state_json(int progress, int remain_s) -> std::string {
    nlohmann::json j;
    j["cmd_id"] = 0x0001;
    j["game_type"] = 1;
    j["game_progress"] = progress;
    j["stage_remain_time"] = remain_s;
    j["sync_timestamp"] = 0;
    return j.dump();
}

auto mark_json(bool targeted, bool countered) -> std::string {
    nlohmann::json j;
    j["cmd_id"] = 0x020C;
    j["opponent_hero_vulnerable"] = false;
    j["opponent_engineer_vulnerable"] = false;
    j["opponent_infantry_3_vulnerable"] = false;
    j["opponent_infantry_4_vulnerable"] = false;
    j["opponent_aerial_marked"] = false;
    j["opponent_sentry_vulnerable"] = false;
    j["ally_hero_marked"] = false;
    j["ally_engineer_marked"] = false;
    j["ally_infantry_3_marked"] = false;
    j["ally_infantry_4_marked"] = false;
    j["ally_aerial_marked"] = false;
    j["ally_sentry_marked"] = false;
    j["opponent_aerial_targeted"] = targeted;
    j["opponent_aerial_countered"] = countered;
    j["ally_aerial_targeted"] = false;
    j["ally_aerial_countered"] = false;
    return j.dump();
}

} // namespace

int main(int argc, char** argv) {
    const auto opt = parse_options(argc, argv);
    if (opt.show_help) {
        print_help();
        return 0;
    }

    zmq::context_t ctx(1);
    zmq::socket_t pub(ctx, zmq::socket_type::pub);
    pub.bind(opt.bind);
    std::println(stderr, "referee_sim: PUB on {}", opt.bind);

    const auto publish = [&](const std::string& payload) { pub.send(zmq::buffer(payload), zmq::send_flags::dontwait); };

    const auto phases = std::vector<std::pair<int, int>>{
        {1, opt.prep_s}, {2, opt.selfcheck_s}, {3, opt.countdown_s}, {4, opt.match_s}, {5, opt.settle_s}};

    do {
        for (const auto& [progress, duration_s] : phases) {
            for (int s = 0; s < duration_s; ++s) {
                const int remain = progress == 4 ? opt.match_s - s : duration_s - s;
                publish(game_state_json(progress, remain));
                if (progress == 4) {
                    const bool in_counter = std::ranges::any_of(
                        opt.counter_at_s,
                        [&](const int t) { return s >= t && s < t + opt.counter_duration_s; });
                    publish(mark_json(in_counter, in_counter));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(opt.interval_ms));
            }
        }
    } while (opt.loop);

    return 0;
}
