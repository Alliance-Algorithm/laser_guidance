#include "io/ft4222_spi.hpp"

#include <cassert>
#include <cstring>
#include <expected>
#include <memory>
#include <string>
#include <utility>

#ifdef WITH_FT4222
#include "io/libft4222.h"
#endif

#ifdef WITH_FT4222

namespace rmcs_laser_guidance {

auto Ft4222Spi::negotiated_clock_hz() const noexcept -> uint32_t {
    return kSysClocksHz[static_cast<uint8_t>(config_.sys_clock)]
         / (1U << static_cast<uint8_t>(config_.clock_div));
}

Ft4222Spi::Ft4222Spi(void* ft_handle, Ft4222Config cfg) noexcept
    : handle_(ft_handle), config_(cfg) {}

Ft4222Spi::Ft4222Spi(Ft4222Spi&& other) noexcept
    : handle_(std::exchange(other.handle_, nullptr))
    , config_(other.config_) {}

auto Ft4222Spi::operator=(Ft4222Spi&& other) noexcept -> Ft4222Spi& {
    if (this != &other) {
        close();
        handle_ = std::exchange(other.handle_, nullptr);
        config_ = other.config_;
    }
    return *this;
}

Ft4222Spi::~Ft4222Spi() noexcept { close(); }

void Ft4222Spi::close() noexcept {
    if (!handle_) return;
    FT4222_UnInitialize(static_cast<FT_HANDLE>(handle_));
    FT_Close(static_cast<FT_HANDLE>(handle_));
    handle_ = nullptr;
}

auto Ft4222Spi::open(Ft4222Config config)
    -> std::expected<Ft4222Spi, std::string> {

    DWORD num_devs = 0;
    FT_STATUS ft_status = FT_CreateDeviceInfoList(&num_devs);
    if (ft_status != FT_OK)
        return std::unexpected("FT_CreateDeviceInfoList failed: " + std::to_string(ft_status));
    if (num_devs == 0)
        return std::unexpected("No FTDI devices found");

    auto dev_list = std::make_unique<FT_DEVICE_LIST_INFO_NODE[]>(num_devs);
    ft_status = FT_GetDeviceInfoList(dev_list.get(), &num_devs);
    if (ft_status != FT_OK)
        return std::unexpected("FT_GetDeviceInfoList failed: " + std::to_string(ft_status));

    FT_HANDLE ft_handle = nullptr;
    bool found = false;

    for (DWORD i = 0; i < num_devs; ++i) {
        if (dev_list[i].Type == FT_DEVICE_4222H_0 ||
            dev_list[i].Type == FT_DEVICE_4222H_1_2 ||
            dev_list[i].Type == FT_DEVICE_4222H_3) {

            ft_status = FT_OpenEx(
                reinterpret_cast<PVOID>(static_cast<uintptr_t>(dev_list[i].LocId)),
                FT_OPEN_BY_LOCATION, &ft_handle);
            if (ft_status != FT_OK)
                continue;

            uint8_t chip_mode = 0;
            FT4222_GetChipMode(ft_handle, &chip_mode);
            uint8_t max_cs = (chip_mode == 0 || chip_mode == 3) ? 1U
                           : (chip_mode == 1)                   ? 3U
                           :                                      4U;

            if (config.cs_channel >= max_cs) {
                FT_Close(ft_handle);
                ft_handle = nullptr;
                continue;
            }

            found = true;
            break;
        }
    }

    if (!found)
        return std::unexpected("FT4222H not found with compatible chip mode");

    FT4222_STATUS ft4222_status;
    ft4222_status = FT4222_SPIMaster_Init(
        ft_handle,
        static_cast<FT4222_SPIMode>(SPI_IO_SINGLE),
        static_cast<FT4222_SPIClock>(config.clock_div),
        static_cast<FT4222_SPICPOL>(config.cpol),
        static_cast<FT4222_SPICPHA>(config.cpha),
        static_cast<uint8_t>(1U << config.cs_channel));

    if (ft4222_status != FT4222_OK) {
        FT_Close(ft_handle);
        return std::unexpected("SPI Master init failed: " + std::to_string(ft4222_status));
    }

    ft4222_status = FT4222_SetClock(
        ft_handle,
        static_cast<FT4222_ClockRate>(config.sys_clock));
    if (ft4222_status != FT4222_OK) {
        FT4222_UnInitialize(ft_handle);
        FT_Close(ft_handle);
        return std::unexpected("FT4222_SetClock failed: " + std::to_string(ft4222_status));
    }

    ft4222_status = FT4222_SPIMaster_SetCS(
        ft_handle,
        static_cast<SPI_ChipSelect>(config.cs_active));
    if (ft4222_status != FT4222_OK) {
        FT4222_UnInitialize(ft_handle);
        FT_Close(ft_handle);
        return std::unexpected("FT4222_SPIMaster_SetCS failed: " + std::to_string(ft4222_status));
    }

    uint16_t max_transfer = 0;
    ft4222_status = FT4222_GetMaxTransferSize(ft_handle, &max_transfer);
    if (ft4222_status != FT4222_OK) {
        FT4222_UnInitialize(ft_handle);
        FT_Close(ft_handle);
        return std::unexpected("FT4222_GetMaxTransferSize failed: " + std::to_string(ft4222_status));
    }

    ft4222_status = FT4222_SPI_SetDrivingStrength(ft_handle, DS_8MA, DS_8MA, DS_8MA);
    if (ft4222_status != FT4222_OK) {
        FT4222_UnInitialize(ft_handle);
        FT_Close(ft_handle);
        return std::unexpected("SetDrivingStrength failed: " + std::to_string(ft4222_status));
    }

    return Ft4222Spi(ft_handle, config);
}

auto Ft4222Spi::write(const uint8_t* data, uint16_t len)
    -> std::expected<void, std::string> {

    uint16_t transferred = 0;
    FT4222_STATUS status = FT4222_SPIMaster_SingleWrite(
        static_cast<FT_HANDLE>(handle_),
        const_cast<uint8_t*>(data),
        len, &transferred, 1);

    if (status != FT4222_OK)
        return std::unexpected("SPI write error: " + std::to_string(status));
    if (transferred != len)
        return std::unexpected("SPI write short: " +
            std::to_string(transferred) + "/" + std::to_string(len));
    return {};
}

auto Ft4222Spi::transfer(const uint8_t* tx_buf, uint16_t tx_len,
                         uint8_t* rx_buf, uint16_t rx_len)
    -> std::expected<void, std::string> {

    const uint16_t total = tx_len + rx_len;
    auto combined_tx = std::make_unique<uint8_t[]>(total);
    auto combined_rx = std::make_unique<uint8_t[]>(total);

    std::memcpy(combined_tx.get(), tx_buf, tx_len);
    std::memset(combined_tx.get() + tx_len, 0xFF, rx_len);

    uint16_t transferred = 0;
    FT4222_STATUS status = FT4222_SPIMaster_SingleReadWrite(
        static_cast<FT_HANDLE>(handle_),
        combined_rx.get(), combined_tx.get(),
        total, &transferred, 1);

    if (status != FT4222_OK)
        return std::unexpected("SPI transfer error: " + std::to_string(status));
    if (transferred != total)
        return std::unexpected("SPI transfer short: " +
            std::to_string(transferred) + "/" + std::to_string(total));

    std::memcpy(rx_buf, combined_rx.get() + tx_len, rx_len);
    return {};
}

}  // namespace rmcs_laser_guidance

#else

namespace rmcs_laser_guidance {

auto Ft4222Spi::negotiated_clock_hz() const noexcept -> uint32_t { return 0; }

Ft4222Spi::Ft4222Spi(void*, Ft4222Config) noexcept : handle_(nullptr) {}

Ft4222Spi::Ft4222Spi(Ft4222Spi&&) noexcept = default;
auto Ft4222Spi::operator=(Ft4222Spi&&) noexcept -> Ft4222Spi& = default;
Ft4222Spi::~Ft4222Spi() noexcept = default;

auto Ft4222Spi::open(Ft4222Config) -> std::expected<Ft4222Spi, std::string> {
    return std::unexpected(std::string("FT4222 support not compiled"));
}

auto Ft4222Spi::write(const uint8_t*, uint16_t) -> std::expected<void, std::string> {
    return std::unexpected(std::string("FT4222 not available"));
}

auto Ft4222Spi::transfer(const uint8_t*, uint16_t, uint8_t*, uint16_t)
    -> std::expected<void, std::string> {
    return std::unexpected(std::string("FT4222 not available"));
}

auto Ft4222Spi::close() noexcept -> void {}

}  // namespace rmcs_laser_guidance

#endif
