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

struct Options {
    uint8_t channel{0};
    int hold_ms{1500};
    bool show_help{false};
    bool keep_last{false};
    bool has_single_voltage{false};
    double single_voltage{0.0};
};

auto print_help() -> void {
    std::println(
        "usage: tool_dac8568_smoke [--channel A-H] [--hold-ms N] [--voltage V] [--keep-last]\n"
        "\n"
        "Default behavior:\n"
        "  Runs a safe communication smoke sequence on one channel: 0V -> +1V -> -1V -> 0V\n"
        "\n"
        "Options:\n"
        "  --channel A-H   Select DAC output channel (default: A)\n"
        "  --hold-ms N     Delay between steps in milliseconds (default: 1500)\n"
        "  --voltage V     Write a single voltage in range [-10, 10] volts\n"
        "  --keep-last     Keep the final output instead of restoring 0V\n"
        "  --help          Show this message\n"
        "\n"
        "Notes:\n"
        "  - Requires FT4222 USB-to-SPI hardware and a DAC8568 +/-10V board.\n"
        "  - DAC board must have its own 5V supply and share GND with FT4222.\n"
        "  - This tool validates the write path; use a multimeter on the selected channel to confirm analog output."
    );
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
        if (arg == "--channel") {
            if (i + 1 >= argc)
                return std::unexpected("--channel requires a value");
            auto parsed = parse_channel(argv[++i]);
            if (!parsed)
                return std::unexpected(parsed.error());
            options.channel = *parsed;
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
        if (arg == "--voltage") {
            if (i + 1 >= argc)
                return std::unexpected("--voltage requires a value");
            auto parsed = parse_double(argv[++i], "voltage");
            if (!parsed)
                return std::unexpected(parsed.error());
            options.has_single_voltage = true;
            options.single_voltage = *parsed;
            continue;
        }
        return std::unexpected("unknown argument: " + std::string{arg});
    }

    return options;
}

auto channel_name(uint8_t channel) -> char {
    return static_cast<char>('A' + channel);
}

auto clamp_voltage(double voltage) -> double {
    return std::clamp(voltage, -10.0, 10.0);
}

auto voltage_to_code(double voltage) -> uint16_t {
    const double clipped = clamp_voltage(voltage);
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

auto write_payload(rmcs_laser_guidance::Ft4222Spi& spi, uint32_t payload, std::string_view label)
    -> std::expected<void, std::string> {

    const auto bytes = payload_bytes(payload);
    std::println(
        "{}: payload=0x{:08X} bytes=[0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}]",
        label,
        payload,
        bytes[0], bytes[1], bytes[2], bytes[3]);
    return spi.write(bytes.data(), static_cast<uint16_t>(bytes.size()));
}

auto write_internal_reference_enable(rmcs_laser_guidance::Ft4222Spi& spi)
    -> std::expected<void, std::string> {
    constexpr uint32_t kEnableInternalReference = 0x08000001u;
    return write_payload(spi, kEnableInternalReference, "enable_internal_reference");
}

auto write_voltage(rmcs_laser_guidance::Ft4222Spi& spi, uint8_t channel, double voltage)
    -> std::expected<void, std::string> {

    const double clipped = clamp_voltage(voltage);
    const uint16_t code = voltage_to_code(clipped);
    const uint32_t payload = build_payload(
        0x03,
        channel,
        code,
        0x00);

    std::println(
        "channel={} target_voltage={:.3f}V code=0x{:04X}",
        channel_name(channel),
        clipped,
        code);

    return write_payload(spi, payload, "write_voltage");
}

auto maybe_sleep(int hold_ms) -> void {
    if (hold_ms <= 0)
        return;
    std::this_thread::sleep_for(std::chrono::milliseconds{hold_ms});
}

auto run_single_voltage(rmcs_laser_guidance::Ft4222Spi& spi, const Options& options)
    -> std::expected<void, std::string> {

    if (auto result = write_voltage(spi, options.channel, options.single_voltage); !result)
        return result;

    if (options.keep_last) {
        std::println(
            "kept channel {} at {:.3f}V; measure it now with a multimeter and reset manually when done",
            channel_name(options.channel),
            clamp_voltage(options.single_voltage));
        return {};
    }

    maybe_sleep(options.hold_ms);
    if (auto result = write_voltage(spi, options.channel, 0.0); !result)
        return result;
    std::println("restored channel {} to 0.000V", channel_name(options.channel));
    return {};
}

auto run_sequence(rmcs_laser_guidance::Ft4222Spi& spi, const Options& options)
    -> std::expected<void, std::string> {

    const std::array<double, 4> sequence{0.0, 1.0, -1.0, 0.0};
    for (std::size_t i = 0; i < sequence.size(); ++i) {
        const double voltage = sequence[i];
        std::println(
            "step {}/{}: set channel {} to {:.3f}V and observe the output",
            i + 1,
            sequence.size(),
            channel_name(options.channel),
            voltage);

        if (auto result = write_voltage(spi, options.channel, voltage); !result)
            return result;

        const bool is_last = i + 1 == sequence.size();
        if (!is_last)
            maybe_sleep(options.hold_ms);
    }

    std::println(
        "sequence finished; expected multimeter readings followed 0V -> +1V -> -1V -> 0V on channel {}",
        channel_name(options.channel));
    return {};
}

} // namespace

int main(int argc, char** argv) {
    const auto options = parse_options(argc, argv);
    if (!options) {
        std::println(stderr, "tool_dac8568_smoke: {}", options.error());
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
            std::println(stderr, "tool_dac8568_smoke: failed to open FT4222: {}", spi.error());
            return 1;
        }

        std::println(
            "opened FT4222 on CS{} with estimated SCLK {} Hz",
            0,
            spi->negotiated_clock_hz());
        std::println(
            "selected channel {} on DAC8568; ensure the DAC board has independent 5V power and shared GND",
            channel_name(options->channel));

        if (auto result = write_internal_reference_enable(*spi); !result) {
            std::println(stderr, "tool_dac8568_smoke: {}", result.error());
            return 1;
        }

        const auto result = options->has_single_voltage
            ? run_single_voltage(*spi, *options)
            : run_sequence(*spi, *options);
        if (!result) {
            std::println(stderr, "tool_dac8568_smoke: {}", result.error());
            return 1;
        }

        std::println("tool_dac8568_smoke: communication write path completed successfully");
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_dac8568_smoke failed: {}", e.what());
        return 1;
    }
}
