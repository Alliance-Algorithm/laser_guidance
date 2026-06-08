#include <filesystem>
#include <print>

#include "config.hpp"
#include "laser_guidance/support.hpp"
#include "runtime/guidance_tool_runtime.hpp"

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
        rmcs_laser_guidance::runtime_internal::GuidanceToolRuntime runtime(std::move(config));
        return runtime.run();
    } catch (const std::exception& e) {
        std::println(stderr, "tool_guidance failed: {}", e.what());
        return 1;
    }
}
