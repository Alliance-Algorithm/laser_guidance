#include <fcntl.h>
#include <print>
#include <stdexcept>
#include <unistd.h>
#include <string_view>
#include <variant>

#include "laser_guidance/bridges.hpp"
#include "laser_guidance/runtime.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::CmdSetBackend;
        using rmcs_laser_guidance::CmdSetEkf;
        using rmcs_laser_guidance::CmdSetEnemyColor;
        using rmcs_laser_guidance::CmdSetOffset;
        using rmcs_laser_guidance::CmdSetRecording;
        using rmcs_laser_guidance::CmdSetStreaming;
        using rmcs_laser_guidance::CmdShutdown;
        using rmcs_laser_guidance::EnemyColor;
        using rmcs_laser_guidance::ErrorKind;
        using rmcs_laser_guidance::FifoControlServer;
        using rmcs_laser_guidance::RuntimeBackend;
        using rmcs_laser_guidance::RuntimeCommand;
        using rmcs_laser_guidance::tests::make_temp_dir;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_contains;
        using rmcs_laser_guidance::tests::require_near;

        // stream on
        {
            const auto command = FifoControlServer::parse_command("stream on\n");
            require(command.has_value(), "stream on should parse");
            const auto* streaming = std::get_if<CmdSetStreaming>(&*command);
            require(streaming != nullptr, "stream on type mismatch");
            require(streaming->enabled, "stream on enabled mismatch");
        }

        // record off
        {
            const auto command = FifoControlServer::parse_command("record off");
            require(command.has_value(), "record off should parse");
            const auto* recording = std::get_if<CmdSetRecording>(&*command);
            require(recording != nullptr, "record off type mismatch");
            require(!recording->enabled, "record off enabled mismatch");
        }

        // enemy blue
        {
            const auto command = FifoControlServer::parse_command("enemy blue");
            require(command.has_value(), "enemy blue should parse");
            const auto* enemy = std::get_if<CmdSetEnemyColor>(&*command);
            require(enemy != nullptr, "enemy blue type mismatch");
            require(enemy->enemy_color == EnemyColor::blue, "enemy blue value mismatch");
        }

        // backend tensorrt
        {
            const auto command = FifoControlServer::parse_command("backend tensorrt");
            require(command.has_value(), "backend tensorrt should parse");
            const auto* backend = std::get_if<CmdSetBackend>(&*command);
            require(backend != nullptr, "backend tensorrt type mismatch");
            require(backend->backend == RuntimeBackend::tensorrt, "backend tensorrt value mismatch");
        }

        // ekf off
        {
            const auto command = FifoControlServer::parse_command("ekf off");
            require(command.has_value(), "ekf off should parse");
            const auto* ekf = std::get_if<CmdSetEkf>(&*command);
            require(ekf != nullptr, "ekf off type mismatch");
            require(!ekf->enabled, "ekf off enabled mismatch");
        }

        // offset 0.5 -0.3
        {
            const auto command = FifoControlServer::parse_command("offset 0.5 -0.3");
            require(command.has_value(), "offset should parse");
            const auto* offset = std::get_if<CmdSetOffset>(&*command);
            require(offset != nullptr, "offset type mismatch");
            require_near(offset->x_deg, 0.5F, 1e-4F, "offset x value");
            require_near(offset->y_deg, -0.3F, 1e-4F, "offset y value");
        }

        // offset with single value keeps y
        {
            const auto command = FifoControlServer::parse_command("offset 0.2");
            require(command.has_value(), "offset single should parse");
            const auto* offset = std::get_if<CmdSetOffset>(&*command);
            require(offset != nullptr, "offset single type mismatch");
            require_near(offset->x_deg, 0.2F, 1e-4F, "offset single x value");
            require_near(offset->y_deg, 0.0F, 1e-4F, "offset single y default");
        }

        // invalid offset
        {
            const auto command = FifoControlServer::parse_command("offset abc");
            require(!command.has_value(), "invalid offset should fail");
        }

        // quit
        {
            const auto command = FifoControlServer::parse_command("quit");
            require(command.has_value(), "quit should parse");
            require(std::holds_alternative<CmdShutdown>(*command), "quit type mismatch");
        }

        // invalid backend
        {
            const auto command = FifoControlServer::parse_command("backend invalid");
            require(!command.has_value(), "invalid backend should fail");
            require(command.error().kind == ErrorKind::config,
                "invalid backend should be config error");
            require_contains(command.error().message, "unsupported FIFO command",
                "invalid backend error");
        }

        // blank command
        {
            const auto command = FifoControlServer::parse_command("   ");
            require(!command.has_value(), "blank command should fail");
            require(command.error().kind == ErrorKind::config,
                "blank should be config error");
            require_contains(command.error().message, "empty FIFO command",
                "blank command error");
        }

        // multi-line, partial, and recovery via FIFO
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
            require(std::holds_alternative<CmdSetStreaming>(*command),
                "first multi-line command should be set_streaming");

            command = fifo.poll_command();
            require(command.has_value(), "second multi-line command should parse");
            {
                const auto* ekf = std::get_if<CmdSetEkf>(&*command);
                require(ekf != nullptr, "second multi-line command type mismatch");
                require(!ekf->enabled, "second multi-line command payload mismatch");
            }

            const std::string partial1 = "backend ten";
            require(::write(write_fd, partial1.data(), partial1.size()) == static_cast<ssize_t>(partial1.size()),
                "partial write 1 should succeed");
            require(!fifo.poll_command().has_value(), "partial line should not parse early");

            const std::string partial2 = "sorrt\n";
            require(::write(write_fd, partial2.data(), partial2.size()) == static_cast<ssize_t>(partial2.size()),
                "partial write 2 should succeed");
            command = fifo.poll_command();
            require(command.has_value(), "completed partial line should parse");
            {
                const auto* backend = std::get_if<CmdSetBackend>(&*command);
                require(backend != nullptr, "partial command type mismatch");
                require(backend->backend == RuntimeBackend::tensorrt, "partial command value mismatch");
            }

            const std::string recover = "backend nope\nenemy red\n";
            require(::write(write_fd, recover.data(), recover.size()) == static_cast<ssize_t>(recover.size()),
                "recovery write should succeed");
            command = fifo.poll_command();
            require(command.has_value(), "parser should recover after invalid line");
            {
                const auto* enemy = std::get_if<CmdSetEnemyColor>(&*command);
                require(enemy != nullptr, "recovery command type mismatch");
                require(enemy->enemy_color == EnemyColor::red, "recovery command value mismatch");
            }
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
