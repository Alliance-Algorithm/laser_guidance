#include <fcntl.h>
#include <print>
#include <stdexcept>
#include <unistd.h>
#include <string_view>

#include "laser_guidance/bridges.hpp"
#include "laser_guidance/runtime.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::EnemyColor;
        using rmcs_laser_guidance::FifoControlServer;
        using rmcs_laser_guidance::RuntimeBackend;
        using rmcs_laser_guidance::RuntimeCommandType;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_contains;
        using rmcs_laser_guidance::tests::make_temp_dir;

        {
            const auto command = FifoControlServer::parse_command("stream on\n");
            require(command.has_value(), "stream on should parse");
            require(command->type == RuntimeCommandType::set_streaming, "stream on type mismatch");
            require(command->enabled, "stream on enabled mismatch");
        }

        {
            const auto command = FifoControlServer::parse_command("record off");
            require(command.has_value(), "record off should parse");
            require(command->type == RuntimeCommandType::set_recording, "record off type mismatch");
            require(!command->enabled, "record off enabled mismatch");
        }

        {
            const auto command = FifoControlServer::parse_command("enemy blue");
            require(command.has_value(), "enemy blue should parse");
            require(
                command->type == RuntimeCommandType::set_enemy_color,
                "enemy blue type mismatch");
            require(command->enemy_color == EnemyColor::blue, "enemy blue value mismatch");
        }

        {
            const auto command = FifoControlServer::parse_command("backend tensorrt");
            require(command.has_value(), "backend tensorrt should parse");
            require(
                command->type == RuntimeCommandType::set_backend,
                "backend tensorrt type mismatch");
            require(
                command->backend == RuntimeBackend::tensorrt,
                "backend tensorrt value mismatch");
        }

        {
            const auto command = FifoControlServer::parse_command("ekf off");
            require(command.has_value(), "ekf off should parse");
            require(command->type == RuntimeCommandType::set_ekf, "ekf off type mismatch");
            require(!command->enabled, "ekf off enabled mismatch");
        }

        {
            const auto command = FifoControlServer::parse_command("quit");
            require(command.has_value(), "quit should parse");
            require(command->type == RuntimeCommandType::shutdown, "quit type mismatch");
        }

        {
            const auto command = FifoControlServer::parse_command("backend invalid");
            require(!command.has_value(), "invalid backend should fail");
            require_contains(command.error(), "unsupported FIFO command", "invalid backend error");
        }

        {
            const auto command = FifoControlServer::parse_command("   ");
            require(!command.has_value(), "blank command should fail");
            require_contains(command.error(), "empty FIFO command", "blank command error");
        }

        {
            const auto fifo_dir = make_temp_dir("fifo_control");
            const auto fifo_path = fifo_dir / "laser_cmd";
            FifoControlServer fifo(fifo_path);
            const auto start_result = fifo.start();
            require(start_result.has_value(), "fifo should start");

            const int write_fd = ::open(fifo_path.c_str(), O_WRONLY | O_NONBLOCK);
            require(write_fd >= 0, "write side should open");

            const std::string payload = "stream on\nekf off\n";
            require(::write(write_fd, payload.data(), payload.size()) == static_cast<ssize_t>(payload.size()),
                "multi-line write should succeed");

            auto command = fifo.poll_command();
            require(command.has_value(), "first multi-line command should parse");
            require(command->type == RuntimeCommandType::set_streaming, "first multi-line command mismatch");
            command = fifo.poll_command();
            require(command.has_value(), "second multi-line command should parse");
            require(command->type == RuntimeCommandType::set_ekf, "second multi-line command mismatch");
            require(!command->enabled, "second multi-line command payload mismatch");

            const std::string partial1 = "backend ten";
            require(::write(write_fd, partial1.data(), partial1.size()) == static_cast<ssize_t>(partial1.size()),
                "partial write 1 should succeed");
            require(!fifo.poll_command().has_value(), "partial line should not parse early");

            const std::string partial2 = "sorrt\n";
            require(::write(write_fd, partial2.data(), partial2.size()) == static_cast<ssize_t>(partial2.size()),
                "partial write 2 should succeed");
            command = fifo.poll_command();
            require(command.has_value(), "completed partial line should parse");
            require(command->backend == RuntimeBackend::tensorrt, "partial command value mismatch");

            const std::string recover = "backend nope\nenemy red\n";
            require(::write(write_fd, recover.data(), recover.size()) == static_cast<ssize_t>(recover.size()),
                "recovery write should succeed");
            command = fifo.poll_command();
            require(command.has_value(), "parser should recover after invalid line");
            require(command->type == RuntimeCommandType::set_enemy_color, "recovery command type mismatch");
            require(command->enemy_color == EnemyColor::red, "recovery command value mismatch");
            require(fifo.last_error().empty(), "successful recovery should clear last error");

            ::close(write_fd);
            fifo.stop();
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "fifo_control_test failed: {}", e.what());
        return 1;
    }
}
