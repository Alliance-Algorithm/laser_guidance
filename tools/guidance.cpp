#include <cstdlib>
#include <print>

#include "capture/hik_backend.hpp"
#include "config.hpp"
#include "laser_guidance/support.hpp"
#include "runtime/guidance_ops_app.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1 && argv[1][0] != '-') {
        return rmcs_laser_guidance::resolve_config_path(argv[1]);
    }
    return rmcs_laser_guidance::resolve_config_path();
}

auto apply_hik_profile_option(
    rmcs_laser_guidance::Config& config, const int argc, char** argv) -> bool {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string_view(argv[i]) != "--hik-profile")
            continue;
        const std::string_view value(argv[i + 1]);
        if (value == "lit") {
            config.hik.startup_profile_kind = rmcs_laser_guidance::HikProfileKind::lit;
            return true;
        }
        if (value == "unlit") {
            config.hik.startup_profile_kind = rmcs_laser_guidance::HikProfileKind::unlit;
            return true;
        }
        std::println(stderr, "tool_guidance: invalid --hik-profile '{}' (expected lit|unlit)", value);
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path = resolve_config_path(argc, argv);
        auto config = rmcs_laser_guidance::load_config(config_path);

        if (!apply_hik_profile_option(config, argc, argv))
            return 1;
        if (config.capture_backend != rmcs_laser_guidance::CaptureBackendKind::hikcamera) {
            std::println(stderr, "tool_guidance: --hik-profile requires capture_backend: hikcamera");
            return 1;
        }
        const auto startup_profile = rmcs_laser_guidance::select_startup_profile(
            config.hik, config.hik.startup_profile_kind);
        if (!startup_profile) {
            std::println(stderr, "tool_guidance: {}", format_error(startup_profile.error()));
            return 1;
        }
        std::println(
            stderr, "tool_guidance: startup profile={} exposure_us={} fps={} wb={}",
            config.hik.startup_profile_kind == rmcs_laser_guidance::HikProfileKind::unlit
                ? "unlit"
                : "lit",
            startup_profile->exposure_us, startup_profile->framerate,
            startup_profile->set_white_balance ? "manual" : "auto");

        rmcs_laser_guidance::runtime_internal::GuidanceOpsApp app(std::move(config));
        if (auto run_result = app.run(); !run_result) {
            std::println(stderr, "guidance app failed: {}", format_error(run_result.error()));
            return 1;
        }
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_guidance failed: {}", e.what());
        return 1;
    }
}
