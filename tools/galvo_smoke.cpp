#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <limits>
#include <print>
#include <string>
#include <string_view>
#include <thread>

#include "io/ft4222_spi.hpp"

namespace {

using namespace std::chrono_literals;

enum class WiringMode {
    Differential,
    SingleEnded,
};

enum class Target {
    Sequence,
    Center,
    XPositive,
    XNegative,
    YPositive,
    YNegative,
    XSineSweep,
    YSineSweep,
    XYSinePattern,
};

struct Options {
    uint8_t x_plus{0};   // A
    uint8_t x_minus{2};  // C
    uint8_t y_plus{1};   // B
    uint8_t y_minus{3};  // D
    double diff_voltage{0.5};
    int hold_ms{1200};
    double sweep_start_hz{0.5};
    double sweep_end_hz{5.0};
    int sweep_duration_ms{5000};
    int sample_hz{100};
    double curve_cycles{1.0};
    bool keep_last{false};
    bool show_help{false};
    WiringMode wiring{WiringMode::Differential};
    Target target{Target::Sequence};
};

auto print_help() -> void {
    std::println(
        "usage: tool_galvo_smoke [--target sequence|center|xp|xn|yp|yn|xsine|ysine|xysine] [--wiring differential|single-ended]\n"
        "                        [--diff-voltage V] [--hold-ms N] [--keep-last]\n"
        "                        [--sweep-start-hz F0] [--sweep-end-hz F1] [--sweep-duration-ms N] [--sample-hz N] [--curve-cycles N]\n"
        "                        [--x-plus A-H] [--x-minus A-H] [--y-plus A-H] [--y-minus A-H]\n"
        "\n"
        "Default behavior:\n"
        "  Runs a low-risk galvo command sequence using DAC8568 outputs:\n"
        "  center -> X+ -> center -> X- -> center -> Y+ -> center -> Y- -> center\n"
        "\n"
        "Recommended default channel mapping:\n"
        "  X+ = A, X- = C, Y+ = B, Y- = D\n"
        "\n"
        "Options:\n"
        "  --target ...      sequence | center | xp | xn | yp | yn | xsine | ysine | xysine\n"
        "  --wiring ...      differential | single-ended (default: differential)\n"
        "  --diff-voltage V  Per-leg command voltage in range [-5, 5] (default: 0.5)\n"
        "  --hold-ms N       Delay between steps in milliseconds (default: 1200)\n"
        "  --sweep-start-hz  Start frequency for xsine/ysine (default: 0.5)\n"
        "  --sweep-end-hz    End frequency for xsine/ysine (default: 5.0)\n"
        "  --sweep-duration-ms N  Sweep duration in milliseconds (default: 5000)\n"
        "  --sample-hz N     Command update rate for xsine/ysine (default: 100)\n"
        "  --curve-cycles N  Sine cycles across one X traversal for xysine (default: 1.0)\n"
        "  --keep-last       Keep the last target instead of restoring center\n"
        "  --x-plus A-H      DAC channel wired to galvo X+ / X\n"
        "  --x-minus A-H     DAC channel wired to galvo X- (unused in single-ended mode)\n"
        "  --y-plus A-H      DAC channel wired to galvo Y+ / Y\n"
        "  --y-minus A-H     DAC channel wired to galvo Y- (unused in single-ended mode)\n"
        "  --help            Show this message\n"
        "\n"
        "Connection guidance:\n"
        "  Differential mode: DAC A->X+, DAC C->X-, DAC B->Y+, DAC D->Y-, DAC GND->signal GND\n"
        "  Single-ended mode: DAC A->X, DAC B->Y, DAC GND->signal GND\n"
        "\n"
        "Power guidance:\n"
        "  The galvo driver must use its own +/-15V supply. Do NOT power it from the DAC board.\n"
        "\n"
        "Safety:\n"
        "  In differential mode, the driver sees 2x this voltage between + and -. Start with the default 0.5V.\n"
        "  For an effective \u00b13V sine sweep, use --target xsine --diff-voltage 1.5\n"
        "  For an effective \u00b15V XY sine image, use --target xysine --diff-voltage 2.5"
    );
}

auto channel_name(uint8_t channel) -> char {
    return static_cast<char>('A' + channel);
}

auto parse_channel(std::string_view text) -> std::expected<uint8_t, std::string> {
    if (text.size() != 1)
        return std::unexpected("channel must be a single letter A-H");
    const char ch = static_cast<char>(std::toupper(static_cast<unsigned char>(text[0])));
    if (ch < 'A' || ch > 'H')
        return std::unexpected("channel must be in range A-H");
    return static_cast<uint8_t>(ch - 'A');
}

auto parse_int(std::string_view text, const char* name) -> std::expected<int, std::string> {
    std::string value{text};
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (!end || *end != '\0')
        return std::unexpected(std::string{name} + " must be an integer");
    if (parsed < 0)
        return std::unexpected(std::string{name} + " must be >= 0");
    if (parsed > std::numeric_limits<int>::max())
        return std::unexpected(std::string{name} + " is too large");
    return static_cast<int>(parsed);
}

auto parse_double(std::string_view text, const char* name) -> std::expected<double, std::string> {
    std::string value{text};
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (!end || *end != '\0')
        return std::unexpected(std::string{name} + " must be a number");
    if (!std::isfinite(parsed))
        return std::unexpected(std::string{name} + " must be finite");
    return parsed;
}

auto parse_wiring(std::string_view text) -> std::expected<WiringMode, std::string> {
    if (text == "differential")
        return WiringMode::Differential;
    if (text == "single-ended")
        return WiringMode::SingleEnded;
    return std::unexpected("wiring must be differential or single-ended");
}

auto parse_target(std::string_view text) -> std::expected<Target, std::string> {
    if (text == "sequence")
        return Target::Sequence;
    if (text == "center")
        return Target::Center;
    if (text == "xp")
        return Target::XPositive;
    if (text == "xn")
        return Target::XNegative;
    if (text == "yp")
        return Target::YPositive;
    if (text == "yn")
        return Target::YNegative;
    if (text == "xsine")
        return Target::XSineSweep;
    if (text == "ysine")
        return Target::YSineSweep;
    if (text == "xysine")
        return Target::XYSinePattern;
    return std::unexpected("target must be sequence, center, xp, xn, yp, yn, xsine, ysine, or xysine");
}

auto validate_unique_channels(const Options& options) -> std::expected<void, std::string> {
    std::array<uint8_t, 4> channels{options.x_plus, options.x_minus, options.y_plus, options.y_minus};
    if (options.wiring == WiringMode::SingleEnded)
        channels[1] = channels[3] = 0xFF;

    for (std::size_t i = 0; i < channels.size(); ++i) {
        if (channels[i] == 0xFF)
            continue;
        for (std::size_t j = i + 1; j < channels.size(); ++j) {
            if (channels[j] == 0xFF)
                continue;
            if (channels[i] == channels[j])
                return std::unexpected("DAC channel assignments must be unique for all used outputs");
        }
    }
    return {};
}

auto parse_options(int argc, char** argv) -> std::expected<Options, std::string> {
    Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--help") {
            options.show_help = true;
            continue;
        }
        if (arg == "--keep-last") {
            options.keep_last = true;
            continue;
        }
        if (arg == "--wiring") {
            if (i + 1 >= argc)
                return std::unexpected("--wiring requires a value");
            auto parsed = parse_wiring(argv[++i]);
            if (!parsed)
                return std::unexpected(parsed.error());
            options.wiring = *parsed;
            continue;
        }
        if (arg == "--target") {
            if (i + 1 >= argc)
                return std::unexpected("--target requires a value");
            auto parsed = parse_target(argv[++i]);
            if (!parsed)
                return std::unexpected(parsed.error());
            options.target = *parsed;
            continue;
        }
        if (arg == "--diff-voltage") {
            if (i + 1 >= argc)
                return std::unexpected("--diff-voltage requires a value");
            auto parsed = parse_double(argv[++i], "diff-voltage");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.diff_voltage = *parsed;
            continue;
        }
        if (arg == "--hold-ms") {
            if (i + 1 >= argc)
                return std::unexpected("--hold-ms requires a value");
            auto parsed = parse_int(argv[++i], "hold-ms");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.hold_ms = *parsed;
            continue;
        }
        if (arg == "--sweep-start-hz") {
            if (i + 1 >= argc)
                return std::unexpected("--sweep-start-hz requires a value");
            auto parsed = parse_double(argv[++i], "sweep-start-hz");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.sweep_start_hz = *parsed;
            continue;
        }
        if (arg == "--sweep-end-hz") {
            if (i + 1 >= argc)
                return std::unexpected("--sweep-end-hz requires a value");
            auto parsed = parse_double(argv[++i], "sweep-end-hz");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.sweep_end_hz = *parsed;
            continue;
        }
        if (arg == "--sweep-duration-ms") {
            if (i + 1 >= argc)
                return std::unexpected("--sweep-duration-ms requires a value");
            auto parsed = parse_int(argv[++i], "sweep-duration-ms");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.sweep_duration_ms = *parsed;
            continue;
        }
        if (arg == "--sample-hz") {
            if (i + 1 >= argc)
                return std::unexpected("--sample-hz requires a value");
            auto parsed = parse_int(argv[++i], "sample-hz");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.sample_hz = *parsed;
            continue;
        }
        if (arg == "--curve-cycles") {
            if (i + 1 >= argc)
                return std::unexpected("--curve-cycles requires a value");
            auto parsed = parse_double(argv[++i], "curve-cycles");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.curve_cycles = *parsed;
            continue;
        }
        if (arg == "--x-plus") {
            if (i + 1 >= argc)
                return std::unexpected("--x-plus requires a value");
            auto parsed = parse_channel(argv[++i]);
            if (!parsed)
                return std::unexpected(parsed.error());
            options.x_plus = *parsed;
            continue;
        }
        if (arg == "--x-minus") {
            if (i + 1 >= argc)
                return std::unexpected("--x-minus requires a value");
            auto parsed = parse_channel(argv[++i]);
            if (!parsed)
                return std::unexpected(parsed.error());
            options.x_minus = *parsed;
            continue;
        }
        if (arg == "--y-plus") {
            if (i + 1 >= argc)
                return std::unexpected("--y-plus requires a value");
            auto parsed = parse_channel(argv[++i]);
            if (!parsed)
                return std::unexpected(parsed.error());
            options.y_plus = *parsed;
            continue;
        }
        if (arg == "--y-minus") {
            if (i + 1 >= argc)
                return std::unexpected("--y-minus requires a value");
            auto parsed = parse_channel(argv[++i]);
            if (!parsed)
                return std::unexpected(parsed.error());
            options.y_minus = *parsed;
            continue;
        }
        return std::unexpected("unknown argument: " + std::string{arg});
    }

    if (std::abs(options.diff_voltage) > 5.0)
        return std::unexpected("diff-voltage must be within [-5.0, 5.0] for a safe first test");
    if (options.sample_hz <= 0)
        return std::unexpected("sample-hz must be > 0");
    if (options.sweep_duration_ms <= 0)
        return std::unexpected("sweep-duration-ms must be > 0");
    if (options.sweep_start_hz < 0.0 || options.sweep_end_hz < 0.0)
        return std::unexpected("sweep frequencies must be >= 0");
    if (options.curve_cycles <= 0.0)
        return std::unexpected("curve-cycles must be > 0");
    if (auto valid = validate_unique_channels(options); !valid)
        return std::unexpected(valid.error());

    return options;
}

auto clamp_dac_voltage(double voltage) -> double {
    return std::clamp(voltage, -10.0, 10.0);
}

auto voltage_to_code(double voltage) -> uint16_t {
    const double clipped = clamp_dac_voltage(voltage);
    const double code = ((clipped + 10.0) / 20.0) * static_cast<double>(std::numeric_limits<uint16_t>::max());
    return static_cast<uint16_t>(std::lround(code));
}

auto build_payload(uint8_t control, uint8_t address, uint16_t data, uint8_t feature = 0) -> uint32_t {
    return (static_cast<uint32_t>(control) << 24)
         | (static_cast<uint32_t>(address) << 20)
         | (static_cast<uint32_t>(data) << 4)
         | static_cast<uint32_t>(feature);
}

auto payload_bytes(uint32_t payload) -> std::array<uint8_t, 4> {
    return {
        static_cast<uint8_t>((payload >> 24) & 0xFF),
        static_cast<uint8_t>((payload >> 16) & 0xFF),
        static_cast<uint8_t>((payload >> 8) & 0xFF),
        static_cast<uint8_t>(payload & 0xFF),
    };
}

auto write_payload(rmcs_laser_guidance::Ft4222Spi& spi, uint32_t payload, std::string_view label, bool verbose = true)
    -> std::expected<void, std::string> {

    const auto bytes = payload_bytes(payload);
    if (verbose) {
        std::println(
            "{}: payload=0x{:08X} bytes=[0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}]",
            label,
            payload,
            bytes[0], bytes[1], bytes[2], bytes[3]);
    }
    return spi.write(bytes.data(), static_cast<uint16_t>(bytes.size()));
}

auto write_internal_reference_enable(rmcs_laser_guidance::Ft4222Spi& spi)
    -> std::expected<void, std::string> {
    constexpr uint32_t kEnableInternalReference = 0x08000001u;
    return write_payload(spi, kEnableInternalReference, "enable_internal_reference");
}

auto write_voltage(rmcs_laser_guidance::Ft4222Spi& spi, uint8_t channel, double voltage, std::string_view label, bool verbose = true)
    -> std::expected<void, std::string> {

    const double clipped = clamp_dac_voltage(voltage);
    const uint16_t code = voltage_to_code(clipped);
    const uint32_t payload = build_payload(0x03, channel, code, 0x00);

    if (verbose) {
        std::println(
            "{}: channel={} target_voltage={:.3f}V code=0x{:04X}",
            label,
            channel_name(channel),
            clipped,
            code);
    }

    return write_payload(spi, payload, label, verbose);
}

auto maybe_sleep(int hold_ms) -> void {
    if (hold_ms <= 0)
        return;
    std::this_thread::sleep_for(std::chrono::milliseconds{hold_ms});
}

auto write_center(rmcs_laser_guidance::Ft4222Spi& spi, const Options& options)
    -> std::expected<void, std::string> {

    if (auto r = write_voltage(spi, options.x_plus, 0.0, "center_x_plus"); !r)
        return r;
    if (options.wiring == WiringMode::Differential) {
        if (auto r = write_voltage(spi, options.x_minus, 0.0, "center_x_minus"); !r)
            return r;
    }
    if (auto r = write_voltage(spi, options.y_plus, 0.0, "center_y_plus"); !r)
        return r;
    if (options.wiring == WiringMode::Differential) {
        if (auto r = write_voltage(spi, options.y_minus, 0.0, "center_y_minus"); !r)
            return r;
    }
    return {};
}

auto write_axis_state(
    rmcs_laser_guidance::Ft4222Spi& spi,
    const Options& options,
    double x_diff,
    double y_diff,
    std::string_view label,
    bool verbose = true) -> std::expected<void, std::string> {

    const double x_pos = options.wiring == WiringMode::Differential ? x_diff : x_diff;
    const double x_neg = options.wiring == WiringMode::Differential ? -x_diff : 0.0;
    const double y_pos = options.wiring == WiringMode::Differential ? y_diff : y_diff;
    const double y_neg = options.wiring == WiringMode::Differential ? -y_diff : 0.0;

    const double x_seen = options.wiring == WiringMode::Differential ? (x_pos - x_neg) : x_pos;
    const double y_seen = options.wiring == WiringMode::Differential ? (y_pos - y_neg) : y_pos;

    if (verbose) {
        std::println(
            "{}: galvo effective command X={:.3f}V Y={:.3f}V",
            label,
            x_seen,
            y_seen);
    }

    if (auto r = write_voltage(spi, options.x_plus, x_pos, "write_x_plus", verbose); !r)
        return r;
    if (options.wiring == WiringMode::Differential) {
        if (auto r = write_voltage(spi, options.x_minus, x_neg, "write_x_minus", verbose); !r)
            return r;
    }
    if (auto r = write_voltage(spi, options.y_plus, y_pos, "write_y_plus", verbose); !r)
        return r;
    if (options.wiring == WiringMode::Differential) {
        if (auto r = write_voltage(spi, options.y_minus, y_neg, "write_y_minus", verbose); !r)
            return r;
    }
    return {};
}

auto run_sine_sweep(
    rmcs_laser_guidance::Ft4222Spi& spi,
    const Options& options,
    bool x_axis) -> std::expected<void, std::string> {

    const double duration_s = static_cast<double>(options.sweep_duration_ms) / 1000.0;
    const double f0 = options.sweep_start_hz;
    const double f1 = options.sweep_end_hz;
    const double k = (f1 - f0) / duration_s;
    const auto sample_period = std::chrono::duration<double>(1.0 / static_cast<double>(options.sample_hz));
    const auto start = std::chrono::steady_clock::now();
    auto next_tick = start;

    std::println(
        "starting {} sine sweep: per-leg peak={:.3f}V effective peak={:.3f}V start_hz={:.3f} end_hz={:.3f} duration={}ms sample_hz={}",
        x_axis ? "X-axis" : "Y-axis",
        options.diff_voltage,
        options.wiring == WiringMode::Differential ? options.diff_voltage * 2.0 : options.diff_voltage,
        f0,
        f1,
        options.sweep_duration_ms,
        options.sample_hz);

    std::size_t step = 0;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        if (elapsed_s > duration_s)
            break;

        const double phase = 2.0 * 3.14159265358979323846 * (f0 * elapsed_s + 0.5 * k * elapsed_s * elapsed_s);
        const double value = options.diff_voltage * std::sin(phase);
        const double x = x_axis ? value : 0.0;
        const double y = x_axis ? 0.0 : value;

        if (auto r = write_axis_state(spi, options, x, y, x_axis ? "xsine" : "ysine", false); !r)
            return r;

        if (step % static_cast<std::size_t>(std::max(1, options.sample_hz / 2)) == 0) {
            const double freq = f0 + k * elapsed_s;
            const double effective = options.wiring == WiringMode::Differential ? value * 2.0 : value;
            std::println(
                "sweep progress {:.0f}% freq={:.3f}Hz effective_{}={:.3f}V",
                (elapsed_s / duration_s) * 100.0,
                freq,
                x_axis ? 'X' : 'Y',
                effective);
        }

        ++step;
        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(sample_period);
        std::this_thread::sleep_until(next_tick);
    }

    return {};
}

auto run_xy_sine_pattern(
    rmcs_laser_guidance::Ft4222Spi& spi,
    const Options& options) -> std::expected<void, std::string> {

    const double duration_s = static_cast<double>(options.sweep_duration_ms) / 1000.0;
    const double f0 = options.sweep_start_hz;
    const double f1 = options.sweep_end_hz;
    const double k = (f1 - f0) / duration_s;
    const auto sample_period = std::chrono::duration<double>(1.0 / static_cast<double>(options.sample_hz));
    const auto start = std::chrono::steady_clock::now();
    auto next_tick = start;

    std::println(
        "starting XY sine pattern: per-leg peak={:.3f}V effective peak={:.3f}V cycles={} start_hz={:.3f} end_hz={:.3f} duration={}ms sample_hz={}",
        options.diff_voltage,
        options.wiring == WiringMode::Differential ? options.diff_voltage * 2.0 : options.diff_voltage,
        options.curve_cycles,
        f0,
        f1,
        options.sweep_duration_ms,
        options.sample_hz);

    std::size_t step = 0;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        if (elapsed_s > duration_s)
            break;

        const double cycles = f0 * elapsed_s + 0.5 * k * elapsed_s * elapsed_s;
        const double frac = cycles - std::floor(cycles);
        const double x_norm = 1.0 - 4.0 * std::abs(frac - 0.5);
        const double s = (x_norm + 1.0) * 0.5;
        const double y_norm = std::sin(2.0 * 3.14159265358979323846 * options.curve_cycles * s);
        const double x = options.diff_voltage * x_norm;
        const double y = options.diff_voltage * y_norm;

        if (auto r = write_axis_state(spi, options, x, y, "xysine", false); !r)
            return r;

        if (step % static_cast<std::size_t>(std::max(1, options.sample_hz / 2)) == 0) {
            const double freq = f0 + k * elapsed_s;
            const double effective_x = options.wiring == WiringMode::Differential ? x * 2.0 : x;
            const double effective_y = options.wiring == WiringMode::Differential ? y * 2.0 : y;
            std::println(
                "pattern progress {:.0f}% freq={:.3f}Hz effective_X={:.3f}V effective_Y={:.3f}V",
                (elapsed_s / duration_s) * 100.0,
                freq,
                effective_x,
                effective_y);
        }

        ++step;
        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(sample_period);
        std::this_thread::sleep_until(next_tick);
    }

    return {};
}

auto run_sequence(rmcs_laser_guidance::Ft4222Spi& spi, const Options& options)
    -> std::expected<void, std::string> {

    const std::array<std::pair<std::string_view, std::pair<double, double>>, 9> steps{{
        {"center_1", {0.0, 0.0}},
        {"x_positive", {options.diff_voltage, 0.0}},
        {"center_2", {0.0, 0.0}},
        {"x_negative", {-options.diff_voltage, 0.0}},
        {"center_3", {0.0, 0.0}},
        {"y_positive", {0.0, options.diff_voltage}},
        {"center_4", {0.0, 0.0}},
        {"y_negative", {0.0, -options.diff_voltage}},
        {"center_5", {0.0, 0.0}},
    }};

    for (std::size_t i = 0; i < steps.size(); ++i) {
        const auto& [label, xy] = steps[i];
        std::println("step {}/{}: {}", i + 1, steps.size(), label);
        if (auto r = write_axis_state(spi, options, xy.first, xy.second, label); !r)
            return r;

        const bool is_last = i + 1 == steps.size();
        if (!is_last)
            maybe_sleep(options.hold_ms);
    }
    return {};
}

auto run_target(rmcs_laser_guidance::Ft4222Spi& spi, const Options& options)
    -> std::expected<void, std::string> {

    switch (options.target) {
    case Target::Center:
        return write_center(spi, options);
    case Target::XPositive:
        return write_axis_state(spi, options, options.diff_voltage, 0.0, "x_positive");
    case Target::XNegative:
        return write_axis_state(spi, options, -options.diff_voltage, 0.0, "x_negative");
    case Target::YPositive:
        return write_axis_state(spi, options, 0.0, options.diff_voltage, "y_positive");
    case Target::YNegative:
        return write_axis_state(spi, options, 0.0, -options.diff_voltage, "y_negative");
    case Target::XSineSweep:
        return run_sine_sweep(spi, options, true);
    case Target::YSineSweep:
        return run_sine_sweep(spi, options, false);
    case Target::XYSinePattern:
        return run_xy_sine_pattern(spi, options);
    case Target::Sequence:
        return run_sequence(spi, options);
    }
    return std::unexpected("unsupported target");
}

} // namespace

int main(int argc, char** argv) {
    const auto options = parse_options(argc, argv);
    if (!options) {
        std::println(stderr, "tool_galvo_smoke: {}", options.error());
        print_help();
        return 1;
    }
    if (options->show_help) {
        print_help();
        return 0;
    }

    try {
        auto spi = rmcs_laser_guidance::Ft4222Spi::open({
            .sys_clock = rmcs_laser_guidance::Ft4222SysClock::k60MHz,
            .clock_div = rmcs_laser_guidance::Ft4222SpiDiv::kDiv64,
            .cpol = rmcs_laser_guidance::Ft4222Cpol::kIdleLow,
            .cpha = rmcs_laser_guidance::Ft4222Cpha::kTrailing,
            .cs_active = rmcs_laser_guidance::Ft4222CsActive::kLow,
            .cs_channel = 0,
        });
        if (!spi) {
            std::println(stderr, "tool_galvo_smoke: failed to open FT4222: {}", spi.error());
            return 1;
        }

        std::println(
            "opened FT4222 on CS0 with estimated SCLK {} Hz",
            spi->negotiated_clock_hz());
        std::println(
            "galvo test wiring={} diff_voltage={:.3f}V x+={} x-={} y+={} y-={}",
            options->wiring == WiringMode::Differential ? "differential" : "single-ended",
            options->diff_voltage,
            channel_name(options->x_plus),
            channel_name(options->x_minus),
            channel_name(options->y_plus),
            channel_name(options->y_minus));
        std::println(
            "ensure the galvo driver uses its own +/-15V supply and shares signal ground with the DAC board");

        if (auto r = write_internal_reference_enable(*spi); !r) {
            std::println(stderr, "tool_galvo_smoke: {}", r.error());
            return 1;
        }

        if (auto r = run_target(*spi, *options); !r) {
            std::println(stderr, "tool_galvo_smoke: {}", r.error());
            return 1;
        }

        if (!options->keep_last && options->target != Target::Sequence) {
            maybe_sleep(options->hold_ms);
            if (auto r = write_center(*spi, *options); !r) {
                std::println(stderr, "tool_galvo_smoke: failed to restore center: {}", r.error());
                return 1;
            }
            std::println("restored galvo command outputs to center");
        }

        if (options->keep_last) {
            std::println("kept the requested galvo command output active; restore center manually when done");
        }

        std::println("tool_galvo_smoke: galvo signal command path completed successfully");
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_galvo_smoke failed: {}", e.what());
        return 1;
    }
}
