#include "io/ft4222_spi.hpp"

#include <cstring>
#include <dlfcn.h>
#include <expected>
#include <memory>
#include <string>
#include <utility>

#include <print>

#include "libft4222.h"

namespace rmcs_laser_guidance {
namespace {

struct Ft4222Api {
    void* library_handle = nullptr;

    decltype(FT_CreateDeviceInfoList)* ft_create_device_info_list = nullptr;
    decltype(FT_GetDeviceInfoList)* ft_get_device_info_list = nullptr;
    decltype(FT_OpenEx)* ft_open_ex = nullptr;
    decltype(FT_Close)* ft_close = nullptr;

    decltype(FT4222_UnInitialize)* ft4222_uninitialize = nullptr;
    decltype(FT4222_GetChipMode)* ft4222_get_chip_mode = nullptr;
    decltype(FT4222_SPIMaster_Init)* ft4222_spi_master_init = nullptr;
    decltype(FT4222_SetClock)* ft4222_set_clock = nullptr;
    decltype(FT4222_SPIMaster_SetCS)* ft4222_spi_master_set_cs = nullptr;
    decltype(FT4222_GetMaxTransferSize)* ft4222_get_max_transfer_size = nullptr;
    decltype(FT4222_SPI_SetDrivingStrength)* ft4222_spi_set_driving_strength = nullptr;
    decltype(FT4222_SPIMaster_SingleWrite)* ft4222_spi_master_single_write = nullptr;
    decltype(FT4222_SPIMaster_SingleReadWrite)* ft4222_spi_master_single_read_write = nullptr;

    ~Ft4222Api() noexcept {
        if (library_handle != nullptr) {
            dlclose(library_handle);
            library_handle = nullptr;
        }
    }

    template <typename Fn>
    auto load_symbol(Fn*& slot, const char* symbol_name) -> bool {
        dlerror();
        slot = reinterpret_cast<Fn*>(dlsym(library_handle, symbol_name));
        if (slot != nullptr)
            return true;

        const char* err = dlerror();
        std::println(stderr, "FT4222: failed to resolve {}: {}", symbol_name,
                     err != nullptr ? err : "unknown error");
        return false;
    }

    auto load() -> bool {
        if (library_handle != nullptr)
            return true;

        library_handle = dlopen("libft4222.so", RTLD_NOW | RTLD_LOCAL);
        if (library_handle == nullptr) {
            const char* err = dlerror();
            std::println(stderr, "FT4222: failed to load libft4222.so: {}",
                         err != nullptr ? err : "unknown error");
            return false;
        }

        if (!load_symbol(ft_create_device_info_list, "FT_CreateDeviceInfoList")
            || !load_symbol(ft_get_device_info_list, "FT_GetDeviceInfoList")
            || !load_symbol(ft_open_ex, "FT_OpenEx")
            || !load_symbol(ft_close, "FT_Close")
            || !load_symbol(ft4222_uninitialize, "FT4222_UnInitialize")
            || !load_symbol(ft4222_get_chip_mode, "FT4222_GetChipMode")
            || !load_symbol(ft4222_spi_master_init, "FT4222_SPIMaster_Init")
            || !load_symbol(ft4222_set_clock, "FT4222_SetClock")
            || !load_symbol(ft4222_spi_master_set_cs, "FT4222_SPIMaster_SetCS")
            || !load_symbol(ft4222_get_max_transfer_size, "FT4222_GetMaxTransferSize")
            || !load_symbol(ft4222_spi_set_driving_strength, "FT4222_SPI_SetDrivingStrength")
            || !load_symbol(ft4222_spi_master_single_write, "FT4222_SPIMaster_SingleWrite")
            || !load_symbol(
                ft4222_spi_master_single_read_write, "FT4222_SPIMaster_SingleReadWrite")) {
            dlclose(library_handle);
            library_handle = nullptr;
            return false;
        }

        return true;
    }

    static auto instance() -> Ft4222Api& {
        static Ft4222Api api{};
        return api;
    }
};

inline auto& ft4222_api() { return Ft4222Api::instance(); }

} // namespace

auto Ft4222Spi::negotiated_clock_hz() const noexcept -> uint32_t {
    return kSysClocksHz[static_cast<uint8_t>(config_.sys_clock)]
         / (1U << static_cast<uint8_t>(config_.clock_div));
}

Ft4222Spi::Ft4222Spi(void* ft_handle, Ft4222Config cfg) noexcept
    : handle_(ft_handle)
    , config_(cfg) {}

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
    if (!handle_)
        return;
    auto& api = ft4222_api();
    api.ft4222_uninitialize(static_cast<FT_HANDLE>(handle_));
    api.ft_close(static_cast<FT_HANDLE>(handle_));
    handle_ = nullptr;
}

auto Ft4222Spi::open(Ft4222Config config) -> std::expected<Ft4222Spi, Error> {
    auto& api = ft4222_api();
    if (!api.load())
        return std::unexpected(make_error(ErrorKind::device, "FT4222: failed to load libft4222.so"));

    DWORD num_devs = 0;
    FT_STATUS ft_status = api.ft_create_device_info_list(&num_devs);
    if (ft_status != FT_OK)
        return std::unexpected(make_error(
            ErrorKind::device, "FT_CreateDeviceInfoList failed: " + std::to_string(ft_status)));
    if (num_devs == 0)
        return std::unexpected(make_error(ErrorKind::device, "No FTDI devices found"));

    auto dev_list = std::make_unique<FT_DEVICE_LIST_INFO_NODE[]>(num_devs);
    ft_status = api.ft_get_device_info_list(dev_list.get(), &num_devs);
    if (ft_status != FT_OK)
        return std::unexpected(make_error(
            ErrorKind::device, "FT_GetDeviceInfoList failed: " + std::to_string(ft_status)));

    FT_HANDLE ft_handle = nullptr;
    bool found = false;

    for (DWORD i = 0; i < num_devs; ++i) {
        if (dev_list[i].Type != FT_DEVICE_4222H_0 && dev_list[i].Type != FT_DEVICE_4222H_1_2
            && dev_list[i].Type != FT_DEVICE_4222H_3) {
            continue;
        }

        ft_status = api.ft_open_ex(
            reinterpret_cast<PVOID>(static_cast<uintptr_t>(dev_list[i].LocId)),
            FT_OPEN_BY_LOCATION, &ft_handle);
        if (ft_status != FT_OK)
            continue;

        uint8_t chip_mode = 0;
        auto chip_mode_status = api.ft4222_get_chip_mode(ft_handle, &chip_mode);
        if (chip_mode_status != FT4222_OK) {
            api.ft_close(ft_handle);
            ft_handle = nullptr;
            continue;
        }

        const uint8_t max_cs = (chip_mode == 0 || chip_mode == 3) ? 1U : (chip_mode == 1) ? 3U : 4U;
        if (config.cs_channel >= max_cs) {
            api.ft_close(ft_handle);
            ft_handle = nullptr;
            continue;
        }

        found = true;
        break;
    }

    if (!found)
        return std::unexpected(make_error(
            ErrorKind::device, "FT4222H not found with compatible chip mode"));

    FT4222_STATUS ft4222_status = api.ft4222_spi_master_init(
        ft_handle, static_cast<FT4222_SPIMode>(SPI_IO_SINGLE),
        static_cast<FT4222_SPIClock>(config.clock_div), static_cast<FT4222_SPICPOL>(config.cpol),
        static_cast<FT4222_SPICPHA>(config.cpha), static_cast<uint8_t>(1U << config.cs_channel));
    if (ft4222_status != FT4222_OK) {
        api.ft_close(ft_handle);
        return std::unexpected(make_error(
            ErrorKind::device, "SPI Master init failed: " + std::to_string(ft4222_status)));
    }

    ft4222_status = api.ft4222_set_clock(ft_handle, static_cast<FT4222_ClockRate>(config.sys_clock));
    if (ft4222_status != FT4222_OK) {
        api.ft4222_uninitialize(ft_handle);
        api.ft_close(ft_handle);
        return std::unexpected(make_error(
            ErrorKind::device, "FT4222_SetClock failed: " + std::to_string(ft4222_status)));
    }

    ft4222_status = api.ft4222_spi_master_set_cs(
        ft_handle, static_cast<SPI_ChipSelect>(config.cs_active));
    if (ft4222_status != FT4222_OK) {
        api.ft4222_uninitialize(ft_handle);
        api.ft_close(ft_handle);
        return std::unexpected(make_error(
            ErrorKind::device, "FT4222_SPIMaster_SetCS failed: " + std::to_string(ft4222_status)));
    }

    uint16_t max_transfer = 0;
    ft4222_status = api.ft4222_get_max_transfer_size(ft_handle, &max_transfer);
    if (ft4222_status != FT4222_OK) {
        api.ft4222_uninitialize(ft_handle);
        api.ft_close(ft_handle);
        return std::unexpected(make_error(
            ErrorKind::device,
            "FT4222_GetMaxTransferSize failed: " + std::to_string(ft4222_status)));
    }

    ft4222_status = api.ft4222_spi_set_driving_strength(ft_handle, DS_8MA, DS_8MA, DS_8MA);
    if (ft4222_status != FT4222_OK) {
        api.ft4222_uninitialize(ft_handle);
        api.ft_close(ft_handle);
        return std::unexpected(make_error(
            ErrorKind::device, "SetDrivingStrength failed: " + std::to_string(ft4222_status)));
    }

    return Ft4222Spi(ft_handle, config);
}

auto Ft4222Spi::write(const uint8_t* data, uint16_t len) -> std::expected<void, Error> {
    auto& api = ft4222_api();
    // FT4222 shares a USB controller with other devices (cameras etc.); a busy
    // controller can delay the transaction past the device timeout and report
    // FAILED_TO_WRITE_DEVICE. Retry briefly before surfacing the error.
    constexpr int kMaxWriteAttempts = 3;
    for (int attempt = 1; attempt <= kMaxWriteAttempts; ++attempt) {
        uint16_t transferred = 0;
        FT4222_STATUS status = api.ft4222_spi_master_single_write(
            static_cast<FT_HANDLE>(handle_), const_cast<uint8_t*>(data), len, &transferred, 1);

        if (status == FT4222_OK && transferred == len)
            return {};

        if (attempt == kMaxWriteAttempts) {
            if (status != FT4222_OK)
                return std::unexpected(make_error(
                    ErrorKind::device, "SPI write error: " + std::to_string(status)));
            return std::unexpected(make_error(
                ErrorKind::device,
                "SPI write short: " + std::to_string(transferred) + "/" + std::to_string(len)));
        }
    }
    __builtin_unreachable();
}

auto Ft4222Spi::transfer(const uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf, uint16_t rx_len)
    -> std::expected<void, Error> {
    auto& api = ft4222_api();

    const uint16_t total = tx_len + rx_len;
    auto combined_tx = std::make_unique<uint8_t[]>(total);
    auto combined_rx = std::make_unique<uint8_t[]>(total);

    std::memcpy(combined_tx.get(), tx_buf, tx_len);
    std::memset(combined_tx.get() + tx_len, 0xFF, rx_len);

    uint16_t transferred = 0;
    FT4222_STATUS status = api.ft4222_spi_master_single_read_write(
        static_cast<FT_HANDLE>(handle_), combined_rx.get(), combined_tx.get(), total, &transferred,
        1);

    if (status != FT4222_OK)
        return std::unexpected(make_error(
            ErrorKind::device, "SPI transfer error: " + std::to_string(status)));
    if (transferred != total)
        return std::unexpected(make_error(
            ErrorKind::device,
            "SPI transfer short: " + std::to_string(transferred) + "/" + std::to_string(total)));

    std::memcpy(rx_buf, combined_rx.get() + tx_len, rx_len);
    return {};
}

} // namespace rmcs_laser_guidance
