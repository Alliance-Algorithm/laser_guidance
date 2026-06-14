#include <filesystem>
#include <print>
#include <thread>

#include "config.hpp"
#include "laser_guidance/bridges.hpp"
#include "laser_guidance/runtime.hpp"
#include "laser_guidance/support.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1) {
        return rmcs_laser_guidance::resolve_config_path(argv[1]);
    }
    return rmcs_laser_guidance::resolve_config_path();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path = resolve_config_path(argc, argv);
        const auto config = rmcs_laser_guidance::load_config(config_path);

        rmcs_laser_guidance::CompetitionRuntime runtime(
            config, {.profile = rmcs_laser_guidance::CompetitionProfile::preview});
        if (auto start_result = runtime.start(); !start_result) {
            std::println(stderr, "preview runtime start failed: {}", start_result.error());
            return 1;
        }

        rmcs_laser_guidance::FifoControlServer fifo;
        if (auto fifo_result = fifo.start(); fifo_result) {
            std::println("FIFO ready: {}", fifo.fifo_path().string());
        }

        while (true) {
            if (auto command = fifo.poll_command(); command.has_value()) {
                if (auto result = runtime.submit_command(*command); !result) {
                    std::println(stderr, "command rejected: {}", result.error());
                }
            }

            const auto snapshot = runtime.snapshot();
            if (!snapshot.status.running || snapshot.status.stop_requested) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        runtime.stop();
        runtime.join();
        fifo.stop();
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_preview failed: {}", e.what());
        return 1;
    }
}
