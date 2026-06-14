#include <atomic>
#include <csignal>
#include <filesystem>
#include <print>
#include <thread>

#include "config.hpp"
#include "laser_guidance/bridges.hpp"
#include "laser_guidance/runtime.hpp"
#include "laser_guidance/support.hpp"

namespace {

std::atomic_bool g_stop_requested{false};

auto handle_signal(int) -> void { g_stop_requested.store(true); }

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

        rmcs_laser_guidance::RecordSessionOptions record_options;
        try {
            record_options = rmcs_laser_guidance::load_record_session_options(config_path);
        } catch (const std::exception& e) {
            std::println(stderr, "record options unavailable: {}", e.what());
        }

        rmcs_laser_guidance::CompetitionRuntime runtime(config, {.record_options = record_options});
        if (auto start_result = runtime.start(); !start_result) {
            std::println(stderr, "competition runtime start failed: {}", start_result.error());
            return 1;
        }

        rmcs_laser_guidance::FifoControlServer fifo;
        if (auto fifo_result = fifo.start(); !fifo_result) {
            std::println(stderr, "FIFO start failed: {}", fifo_result.error());
            runtime.stop();
            runtime.join();
            return 1;
        }
        std::println("FIFO ready: {}", fifo.fifo_path().string());

        while (!g_stop_requested.load()) {
            if (auto command = fifo.poll_command(); command.has_value()) {
                if (auto result = runtime.submit_command(*command); !result) {
                    std::println(stderr, "command rejected: {}", result.error());
                }
            } else if (!fifo.last_error().empty()) {
                std::println(stderr, "FIFO parse error: {}", fifo.last_error());
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
        std::println(stderr, "tool_competition failed: {}", e.what());
        return 1;
    }
}
