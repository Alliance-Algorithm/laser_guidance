#include "laser_guidance/runtime.hpp"

namespace rmcs_laser_guidance::runtime_command {

auto set_streaming(const bool enabled) -> RuntimeCommand {
    return CmdSetStreaming{.enabled = enabled};
}
auto set_recording(const bool enabled) -> RuntimeCommand {
    return CmdSetRecording{.enabled = enabled};
}
auto set_enemy_color(const EnemyColor color) -> RuntimeCommand {
    return CmdSetEnemyColor{.enemy_color = color};
}
auto set_backend(const RuntimeBackend backend) -> RuntimeCommand {
    return CmdSetBackend{.backend = backend};
}
auto set_ekf(const bool enabled) -> RuntimeCommand {
    return CmdSetEkf{.enabled = enabled};
}
auto shutdown() -> RuntimeCommand { return CmdShutdown{}; }

} // namespace rmcs_laser_guidance::runtime_command
