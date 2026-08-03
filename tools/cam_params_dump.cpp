// Debug helper: read the Hik camera's current runtime parameter state.
//
// Usage:
//   tool_cam_params_dump [device_id]      # default device_id is empty (first camera)
//   # needs the Hik MVS runtime env, e.g.:
//   source .script/hik-mvs-env.sh (bash) or
//   export LD_LIBRARY_PATH=vendor/hikcamera/src/sdk/lib:$LD_LIBRARY_PATH
//
// Background (2026-08-03): the Hik camera keeps its last parameter state across
// sessions. A session that changed parameters outside this repo (e.g. the MVS
// client: exposure/gain/white-balance/gamma) leaves them persisted on the
// camera; HikBackend only resets exposure/gain/framerate/white-balance at open
// and never resets gamma/contrast/saturation/etc. So the values printed here
// while NO daemon is running are leftovers from the previous session — compare
// them against config/hik.yaml before blaming the daemon. While a daemon is
// running the camera is exclusively held and this tool gets "Access denied".
#include <cstdio>
#include <hikcamera/capturer.hpp>
#include <hikcamera/parameters.hpp>
#include <string>

using namespace hikcamera;

template <typename Tag>
void dump(const char* name, hikcamera::Camera& cam) {
    auto v = cam.parameter<Tag>().get();
    if (v)
        std::printf("%-22s = %d\n", name, static_cast<int>(*v));
    else
        std::printf("%-22s = GET FAILED: %s\n", name, v.error().c_str());
}

int main(int argc, char** argv) {
    const std::string device_id = argc > 1 ? argv[1] : "";
    hikcamera::Config config;
    config.device_id = device_id;
    hikcamera::Camera cam;
    cam.configure(config);
    auto conn = cam.connect();
    if (!conn) {
        std::printf("camera connect failed: %s\n", conn.error().c_str());
        return 1;
    }
    auto& c = cam;
    dump<hikcamera::param::exposure_time_us>("exposure_us", c);
    dump<hikcamera::param::exposure_auto>("exposure_auto", c);
    dump<hikcamera::param::gain>("gain", c);
    dump<hikcamera::param::gain_auto>("gain_auto", c);
    dump<hikcamera::param::gamma>("gamma", c);
    dump<hikcamera::param::white_balance_auto>("wb_auto", c);
    dump<hikcamera::param::white_balance_ratio_red>("wb_ratio_red", c);
    dump<hikcamera::param::white_balance_ratio_green>("wb_ratio_green", c);
    dump<hikcamera::param::white_balance_ratio_blue>("wb_ratio_blue", c);
    dump<hikcamera::param::frame_rate_enabled>("fps_enabled", c);
    dump<hikcamera::param::frame_rate_fps>("fps", c);
    dump<hikcamera::param::trigger_mode>("trigger_mode", c);
    dump<hikcamera::param::reverse_x>("reverse_x", c);
    dump<hikcamera::param::reverse_y>("reverse_y", c);
    return 0;
}
