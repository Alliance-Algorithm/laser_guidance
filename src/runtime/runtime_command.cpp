#include "laser_guidance/runtime.hpp"

namespace rmcs_laser_guidance {

auto RuntimeCommand::set_streaming(const bool enabled) -> RuntimeCommand {
    return RuntimeCommand{
        .type = RuntimeCommandType::set_streaming,
        .enabled = enabled,
    };
}

auto RuntimeCommand::set_recording(const bool enabled) -> RuntimeCommand {
    return RuntimeCommand{
        .type = RuntimeCommandType::set_recording,
        .enabled = enabled,
    };
}

auto RuntimeCommand::set_enemy_color(const EnemyColor color) -> RuntimeCommand {
    return RuntimeCommand{
        .type = RuntimeCommandType::set_enemy_color,
        .enemy_color = color,
    };
}

auto RuntimeCommand::set_backend(const RuntimeBackend backend) -> RuntimeCommand {
    return RuntimeCommand{
        .type = RuntimeCommandType::set_backend,
        .backend = backend,
    };
}

auto RuntimeCommand::set_ekf(const bool enabled) -> RuntimeCommand {
    return RuntimeCommand{
        .type = RuntimeCommandType::set_ekf,
        .enabled = enabled,
    };
}

auto RuntimeCommand::shutdown() -> RuntimeCommand {
    return RuntimeCommand{
        .type = RuntimeCommandType::shutdown,
    };
}

} // namespace rmcs_laser_guidance
