// LD_PRELOAD shim: 修复 MVS SDK U3V 传输层（libMvUsb3vTL.so）的控制传输不
// claim 接口问题。
//
// 现象：内核 usbfs 报 "did not claim interface N before use" 并对相机执行
// USB reset，导致相机反复中断（dmesg 中 tool_competitio / hikcamera_ros_d /
// host_sdk_sample 均触发）。
//
// 本 shim 拦截 libusb 调用：对以接口为目标的控制传输，若该接口尚未被本进程
// claim，先自动 claim 再转发；SDK 正式 claim 某接口时，若该接口此前由本 shim
// 自动 claim，先释放归还再转发，保证 SDK 语义不变。
//
// 使用：LD_PRELOAD=liblibusb_claim_shim.so 启动 daemon；不链接 libusb，全部
// 经 dlsym(RTLD_NEXT) 转发到真实实现。
#include <dlfcn.h>
#include <libusb-1.0/libusb.h>

#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "io/libusb_claim_policy.hpp"

namespace {

std::mutex g_mutex;
// 本 shim 自动 claim 的 (handle, interface) 集合
std::unordered_map<libusb_device_handle*, std::unordered_set<int>> g_auto_claimed;

template <typename T>
auto resolve_next(const char* name) -> T {
    return reinterpret_cast<T>(dlsym(RTLD_NEXT, name));
}

using open_fn = int (*)(libusb_device*, libusb_device_handle**);
using open_vidpid_fn = libusb_device_handle* (*)(libusb_context*, std::uint16_t, std::uint16_t);
using claim_fn = int (*)(libusb_device_handle*, int);
using release_fn = int (*)(libusb_device_handle*, int);
using close_fn = void (*)(libusb_device_handle*);
using ctrl_fn = int (*)(libusb_device_handle*, std::uint8_t, std::uint8_t, std::uint16_t,
    std::uint16_t, unsigned char*, std::uint16_t, unsigned int);
using submit_fn = int (*)(libusb_transfer*);

auto real_claim() -> claim_fn { return resolve_next<claim_fn>("libusb_claim_interface"); }
auto real_release() -> release_fn { return resolve_next<release_fn>("libusb_release_interface"); }
auto real_ctrl() -> ctrl_fn { return resolve_next<ctrl_fn>("libusb_control_transfer"); }
auto real_submit() -> submit_fn { return resolve_next<submit_fn>("libusb_submit_transfer"); }
auto real_open() -> open_fn { return resolve_next<open_fn>("libusb_open"); }
auto real_open_vidpid() -> open_vidpid_fn {
    return resolve_next<open_vidpid_fn>("libusb_open_device_with_vid_pid");
}
auto real_claim_interface() -> claim_fn { return resolve_next<claim_fn>("libusb_claim_interface"); }
auto real_release_interface() -> release_fn {
    return resolve_next<release_fn>("libusb_release_interface");
}
auto real_close() -> close_fn { return resolve_next<close_fn>("libusb_close"); }

// 自动 claim 接口（幂等；BUSY 表示已被其它句柄/内核驱动 claim，无需管理）
auto auto_claim(libusb_device_handle* dev, const int ifnum) -> void {
    std::lock_guard lock(g_mutex);
    auto& claimed = g_auto_claimed[dev];
    if (claimed.contains(ifnum))
        return;
    if (real_claim()(dev, ifnum) == LIBUSB_SUCCESS)
        claimed.insert(ifnum);
}

// 控制请求以接口为目标时补 claim
auto ensure_claimed_for_setup(libusb_device_handle* dev, const libusb_control_setup* setup)
    -> void {
    if (const auto target = laser_shim::control_target_interface(
            setup->bmRequestType, setup->wIndex);
        target.has_value()) {
        auto_claim(dev, *target);
    }
}

} // namespace

extern "C" {

int libusb_open(libusb_device* dev, libusb_device_handle** dev_handle) {
    return real_open()(dev, dev_handle);
}

libusb_device_handle* libusb_open_device_with_vid_pid(
    libusb_context* ctx, std::uint16_t vid, std::uint16_t pid) {
    return real_open_vidpid()(ctx, vid, pid);
}

int libusb_claim_interface(libusb_device_handle* dev, int interface_number) {
    {
        // SDK 正式 claim 前，先归还本 shim 自动 claim 的接口，避免 EBUSY
        std::lock_guard lock(g_mutex);
        auto it = g_auto_claimed.find(dev);
        if (it != g_auto_claimed.end() && it->second.erase(interface_number) > 0) {
            (void)real_release()(dev, interface_number);
        }
    }
    return real_claim_interface()(dev, interface_number);
}

int libusb_release_interface(libusb_device_handle* dev, int interface_number) {
    {
        std::lock_guard lock(g_mutex);
        if (auto it = g_auto_claimed.find(dev); it != g_auto_claimed.end()) {
            it->second.erase(interface_number);
        }
    }
    return real_release_interface()(dev, interface_number);
}

void libusb_close(libusb_device_handle* dev) {
    {
        std::lock_guard lock(g_mutex);
        g_auto_claimed.erase(dev);
    }
    real_close()(dev);
}

int libusb_control_transfer(libusb_device_handle* dev_handle, std::uint8_t request_type,
    std::uint8_t b_request, std::uint16_t w_value, std::uint16_t w_index, unsigned char* data,
    std::uint16_t w_length, unsigned int timeout) {
    const libusb_control_setup setup{
        .bmRequestType = request_type,
        .bRequest = b_request,
        .wValue = w_value,
        .wIndex = w_index,
        .wLength = w_length,
    };
    ensure_claimed_for_setup(dev_handle, &setup);
    return real_ctrl()(dev_handle, request_type, b_request, w_value, w_index, data, w_length,
        timeout);
}

int libusb_submit_transfer(libusb_transfer* transfer) {
    if (transfer != nullptr && transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL
        && transfer->buffer != nullptr) {
        ensure_claimed_for_setup(transfer->dev_handle,
            reinterpret_cast<const libusb_control_setup*>(transfer->buffer));
    }
    return real_submit()(transfer);
}

} // extern "C"
