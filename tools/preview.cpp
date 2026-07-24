

#include <filesystem>
#include <cstdlib>
#include <csignal>
#include <print>
#include <thread>

#include "config.hpp"
#include "laser_guidance/bridges.hpp"
#include "laser_guidance/error.hpp"
#include "laser_guidance/runtime.hpp"
#include "laser_guidance/support.hpp"

namespace {

volatile sig_atomic_t g_stop_requested{0};

auto handle_signal(int) -> void { g_stop_requested = 1; }

auto install_signal_handlers() -> void {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
}

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1) {
        return rmcs_laser_guidance::resolve_config_path(argv[1]);
    }
    return rmcs_laser_guidance::resolve_config_path();
}

} // namespace

int main(int argc, char** argv) {
    try {
        install_signal_handlers();

        const auto config_path = resolve_config_path(argc, argv);
        const auto config = rmcs_laser_guidance::load_config(config_path);

        rmcs_laser_guidance::CompetitionRuntime runtime(
            config, {.profile = rmcs_laser_guidance::CompetitionProfile::preview});

        rmcs_laser_guidance::FifoControlServer fifo;
        const bool fifo_started = fifo.start().has_value();
        if (fifo_started) {
            std::println("FIFO ready: {}", fifo.fifo_path().string());
        }

        std::thread command_thread([&] {
            while (g_stop_requested == 0) {
                if (fifo_started) {
                    if (auto command = fifo.poll_command(); command.has_value()) {
                        if (auto result = runtime.submit_command(*command); !result) {
                            std::println(stderr, "command rejected: {}", format_error(result.error()));
                        }
                    } else if (!fifo.last_error().empty()) {
                        std::println(stderr, "FIFO parse error: {}", fifo.last_error());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            runtime.stop();
        });

        if (auto run_result = runtime.run(); !run_result) {
            std::println(stderr, "preview runtime start failed: {}", format_error(run_result.error()));
            g_stop_requested = true;
            if (command_thread.joinable()) {
                command_thread.join();
            }
            fifo.stop();
            return 1;
        }

        g_stop_requested = true;
        if (command_thread.joinable()) {
            command_thread.join();
        }
        fifo.stop();
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_preview failed: {}", e.what());
        return 1;
    }
}
