#pragma once

#include <cstdint>
#include <expected>
#include <string>

namespace rmcs_laser_guidance {

enum class Ft4222SysClock : uint8_t {
    k60MHz = 0,
    k24MHz = 1,
    k48MHz = 2,
    k80MHz = 3,
};

enum class Ft4222SpiDiv : uint8_t {
    kDiv2   = 1,
    kDiv4   = 2,
    kDiv8   = 3,
    kDiv16  = 4,
    kDiv32  = 5,
    kDiv64  = 6,
    kDiv128 = 7,
    kDiv256 = 8,
    kDiv512 = 9,
};

enum class Ft4222Cpol : uint8_t {
    kIdleLow  = 0,
    kIdleHigh = 1,
};

enum class Ft4222Cpha : uint8_t {
    kLeading  = 0,
    kTrailing = 1,
};

enum class Ft4222CsActive : uint8_t {
    kLow  = 0,
    kHigh = 1,
};

struct Ft4222Config {
    Ft4222SysClock sys_clock{Ft4222SysClock::k60MHz};
    Ft4222SpiDiv   clock_div{Ft4222SpiDiv::kDiv64};
    Ft4222Cpol     cpol{Ft4222Cpol::kIdleLow};
    Ft4222Cpha     cpha{Ft4222Cpha::kLeading};
    Ft4222CsActive cs_active{Ft4222CsActive::kLow};
    uint8_t        cs_channel{0};
};

class Ft4222Spi {
public:
    Ft4222Spi() = delete;
    Ft4222Spi(const Ft4222Spi&) = delete;
    auto operator=(const Ft4222Spi&) -> Ft4222Spi& = delete;
    Ft4222Spi(Ft4222Spi&&) noexcept;
    auto operator=(Ft4222Spi&&) noexcept -> Ft4222Spi&;

    ~Ft4222Spi() noexcept;

    [[nodiscard]] static auto open(Ft4222Config config)
        -> std::expected<Ft4222Spi, std::string>;

    auto write(const uint8_t* data, uint16_t len)
        -> std::expected<void, std::string>;

    auto transfer(const uint8_t* tx_buf, uint16_t tx_len,
                  uint8_t* rx_buf, uint16_t rx_len)
        -> std::expected<void, std::string>;

    auto close() noexcept -> void;

    [[nodiscard]] auto is_open() const noexcept -> bool { return handle_ != nullptr; }

    [[nodiscard]] auto negotiated_clock_hz() const noexcept -> uint32_t;

private:
    Ft4222Spi(void* ft_handle, Ft4222Config cfg) noexcept;

    static constexpr uint32_t kSysClocksHz[4] = {
        60'000'000, 24'000'000, 48'000'000, 80'000'000
    };

    void*  handle_{nullptr};
    Ft4222Config config_{};
};

}  // namespace rmcs_laser_guidance
