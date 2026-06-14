#include <filesystem>
#include <print>

#include "config.hpp"
#include "laser_guidance/support.hpp"
#include "runtime/guidance_ops_app.hpp"

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
        auto config = rmcs_laser_guidance::load_config(config_path);

        rmcs_laser_guidance::runtime_internal::GuidanceOpsApp app(std::move(config));
        if (auto run_result = app.run(); !run_result) {
            std::println(stderr, "guidance app failed: {}", run_result.error());
            return 1;
        }
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_guidance failed: {}", e.what());
        return 1;
    }
}
