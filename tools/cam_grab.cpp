// Debug helper: set Hik camera parameters and grab a single frame to disk.
//
// Usage:
//   tool_cam_grab [device_id] [--exposure US] [--gain G] [--gamma GAMMA]
//                 [--wb-r R --wb-g G --wb-b B] [--out path.png]
//   # default: exposure 20000us, gain 16.9, AWB (no --wb-*), gamma untouched
//   # --wb-* given => white balance auto off + fixed ratios (e.g. stage-3
//   # training capture: --wb-r 1450 --wb-g 900 --wb-b 2300)
//   # needs the Hik MVS runtime env, e.g.:
//   export LD_LIBRARY_PATH=vendor/hikcamera/src/sdk/lib:$LD_LIBRARY_PATH
//
// Background (2026-08-03): used to reproduce camera-parameter drift. The Hik
// camera persists parameters across sessions (MVS client leftovers, e.g.
// exposure 2000us / gain 16 / fixed WB 1231-1024-3193 were found idle), and
// HikBackend does not reset gamma/contrast/saturation. Grab frames under
// different gamma/WB and feed them to tool_detect_dump to isolate model
// detection regressions from camera state. Field finding: with stage-3 params
// (20000us / gain 16.9 / AWB) colorless (class 3) detection works ONLY in a
// dark environment (frame mean <~40); bright scenes give 0 candidates.
#include <cstdio>
#include <hikcamera/capturer.hpp>
#include <hikcamera/parameters.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>

using namespace hikcamera;

int main(int argc, char** argv) {
    const std::string device_id = argc > 1 ? argv[1] : "";
    float gamma = -1.0F;
    float exposure_us = 20000.0F;
    float gain = 16.9F;
    float fps = 0.0F;
    int wb_r = -1, wb_g = -1, wb_b = -1;
    const char* out_path = "/tmp/opencode/grab_frame.png";
    for (int i = 2; i + 1 < argc; i += 2) {
        const std::string key = argv[i];
        if (key == "--gamma") gamma = std::stof(argv[i + 1]);
        else if (key == "--exposure") exposure_us = std::stof(argv[i + 1]);
        else if (key == "--gain") gain = std::stof(argv[i + 1]);
        else if (key == "--fps") fps = std::stof(argv[i + 1]);
        else if (key == "--wb") { wb_r = std::stoi(argv[i + 1]); wb_g = wb_r; wb_b = wb_r; }
        else if (key == "--wb-r") wb_r = std::stoi(argv[i + 1]);
        else if (key == "--wb-g") wb_g = std::stoi(argv[i + 1]);
        else if (key == "--wb-b") wb_b = std::stoi(argv[i + 1]);
        else if (key == "--out") out_path = argv[i + 1];
    }

    Config config;
    config.device_id = device_id;
    Camera cam;
    cam.configure(config);
    auto conn = cam.connect();
    if (!conn) {
        std::printf("connect failed: %s\n", conn.error().c_str());
        return 1;
    }

    auto apply = [](auto p, auto v, const char* name) {
        auto r = p.set(v);
        if (!r) std::printf("set %s failed: %s\n", name, r.error().c_str());
        else std::printf("set %s = ok\n", name);
    };
    apply(cam.parameter<param::exposure_auto>(), auto_mode::off, "exposure_auto");
    apply(cam.parameter<param::gain_auto>(), auto_mode::off, "gain_auto");
    apply(cam.parameter<param::exposure_time_us>(), exposure_us, "exposure_us");
    apply(cam.parameter<param::gain>(), gain, "gain");
    if (wb_r >= 0 || wb_g >= 0 || wb_b >= 0) {
        apply(cam.parameter<param::white_balance_auto>(), auto_mode::off, "wb_auto(off)");
        if (wb_r >= 0) apply(cam.parameter<param::white_balance_ratio_red>(), static_cast<float>(wb_r), "wb_ratio_red");
        if (wb_g >= 0) apply(cam.parameter<param::white_balance_ratio_green>(), static_cast<float>(wb_g), "wb_ratio_green");
        if (wb_b >= 0) apply(cam.parameter<param::white_balance_ratio_blue>(), static_cast<float>(wb_b), "wb_ratio_blue");
    } else {
        apply(cam.parameter<param::white_balance_auto>(), auto_mode::continuous, "wb_auto(AWB)");
    }
    if (gamma >= 0.0F) apply(cam.parameter<param::gamma>(), gamma, "gamma");

    for (auto& n : {"exposure_us", "gain", "gamma"}) {
        (void)n;
    }
    auto ex = cam.parameter<param::exposure_time_us>().get();
    auto gn = cam.parameter<param::gain>().get();
    auto gm = cam.parameter<param::gamma>().get();
    auto wa = cam.parameter<param::white_balance_auto>().get();
    std::printf("now: exposure=%s gain=%s gamma=%s wb_auto=%s\n",
        ex ? std::to_string(*ex).c_str() : "?", gn ? std::to_string(*gn).c_str() : "?",
        gm ? std::to_string(*gm).c_str() : "?", wa ? std::to_string(static_cast<int>(*wa)).c_str() : "?");

    auto img = cam.read_image_with_timestamp();
    if (!img) {
        std::printf("grab failed: %s\n", img.error().c_str());
        return 1;
    }
    if (!cv::imwrite(out_path, img->mat)) {
        std::printf("write failed: %s\n", out_path);
        return 1;
    }
    std::printf("frame saved: %s (%dx%d)\n", out_path, img->mat.cols, img->mat.rows);
    return 0;
}
