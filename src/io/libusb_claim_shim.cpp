// LD_PRELOAD shim: 修复 MVS SDK U3V 传输层（libMvUsb3vTL.so）不 claim 接口问题。
//
// 现象：内核 usbfs 报 "did not claim interface N before use" 并对相机执行
// USB reset，导致相机反复中断（dmesg 中 tool_competitio / hikcamera_ros_d /
// host_sdk_sample 均触发）。
//
// 根因（实测）：SDK 的 U3V 传输层绕过 libusb，直接以裸 ioctl/syscall 提交
// USBDEVFS_SUBMITURB（含 interface 2 的中断端点 URB）与控制传输，但从未
// claim 对应接口；内核 usbfs 对未 claim 接口的 URB 告警并触发设备 reset。
//
// 本 shim 在 usbfs 边界拦截：
//   - USBDEVFS_SUBMITURB / USBDEVFS_CONTROL / USBDEVFS_BULK：先对该 fd 预
//     claim 全部接口（一次），保证任何 URB 提交前接口已 claim；
//   - USBDEVFS_CLAIMINTERFACE / RELEASEINTERFACE：记录 SDK 的 claim 状态，
//     若撞上本 shim 预 claim 的接口则先归还再转发（保持 SDK 语义不变）；
//   - libusb_* 同步/异步接口同样拦截（SDK 部分路径走 libusb）。
//
// 使用：LD_PRELOAD=liblibusb_claim_shim.so 启动 daemon；不链接 libusb，全部
// 经 dlopen+dlsym 转发到真实实现（RTLD_NEXT 在 SDK 的 RTLD_LOCAL 加载链下
// 不可用）。
#include <dlfcn.h>
#include <libusb-1.0/libusb.h>

#include <linux/usbdevice_fs.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>

#include <cstring>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "io/libusb_claim_policy.hpp"

namespace {

std::mutex g_mutex;
// 本 shim 自动 claim 的 (handle, interface) 集合（libusb 层）
std::unordered_map<libusb_device_handle*, std::unordered_set<int>> g_auto_claimed;
// fd 层已 claim 的 (fd, interface) 集合（含 SDK 自己 claim 的，用于 handover）
std::unordered_map<int, std::unordered_set<int>> g_fd_claimed;
// fd 层已执行"预 claim 全部接口"的设备
std::unordered_set<int> g_fd_fully_claimed;

// 解析真实函数。不用 RTLD_NEXT：SDK 以 RTLD_LOCAL dlopen 传输层，libusb
// 可能不在全局作用域，RTLD_NEXT 会找不到符号。按 SONAME dlopen + dlsym(handle)
// 与作用域无关；若已加载则返回同一实例。
template <typename T>
auto resolve_next(const char* name) -> T {
    static void* handle = [] {
        void* h = dlopen("libusb-1.0.so.0", RTLD_NOW | RTLD_GLOBAL);
        if (h == nullptr)
            h = dlopen("libusb-1.0.so", RTLD_NOW | RTLD_GLOBAL);
        return h;
    }();
    if (handle == nullptr)
        return nullptr;
    return reinterpret_cast<T>(dlsym(handle, name));
}

// 从真实 libusb 取函数，取不到时打印原因（避免调用空指针）
template <typename T>
auto real_symbol(const char* name) -> T {
    const auto fn = resolve_next<T>(name);
    if (fn == nullptr)
        std::fprintf(stderr, "libusb_claim_shim: resolve %s failed: %s\n", name,
            dlerror() != nullptr ? dlerror() : "dlopen failed");
    return fn;
}

using claim_fn = int (*)(libusb_device_handle*, int);
using release_fn = int (*)(libusb_device_handle*, int);
using ctrl_fn = int (*)(libusb_device_handle*, std::uint8_t, std::uint8_t, std::uint16_t,
    std::uint16_t, unsigned char*, std::uint16_t, unsigned int);
using submit_fn = int (*)(libusb_transfer*);
using close_fn = void (*)(libusb_device_handle*);
using ioctl_fn = int (*)(int, unsigned long, void*);
using syscall_fn = long (*)(long, long, long, long, long, long, long);
using posix_close_fn = int (*)(int);

auto real_claim() -> claim_fn { return real_symbol<claim_fn>("libusb_claim_interface"); }
auto real_release() -> release_fn { return real_symbol<release_fn>("libusb_release_interface"); }
auto real_ctrl() -> ctrl_fn { return real_symbol<ctrl_fn>("libusb_control_transfer"); }
auto real_submit() -> submit_fn { return real_symbol<submit_fn>("libusb_submit_transfer"); }
auto real_close() -> close_fn { return real_symbol<close_fn>("libusb_close"); }
auto real_ioctl_fn() -> ioctl_fn { return real_symbol<ioctl_fn>("ioctl"); }
auto real_syscall_fn() -> syscall_fn { return real_symbol<syscall_fn>("syscall"); }
auto real_posix_close() -> posix_close_fn { return real_symbol<posix_close_fn>("close"); }

// ---- fd 层（ioctl/syscall）----

// 预 claim 该 fd 设备的所有接口（0..15，ENOENT 即不存在，忽略其它错误）。
// 只执行一次：SDK 之后无论从 libusb 还是裸 ioctl claim，都会经过
// CLAIMINTERFACE 拦截点完成 handover（先归还本 shim 的 claim）。
auto claim_all_interfaces(const int fd) -> void {
    std::lock_guard lock(g_mutex);
    if (g_fd_fully_claimed.contains(fd))
        return;
    auto& claimed = g_fd_claimed[fd];
    for (int ifnum = 0; ifnum < 16; ++ifnum) {
        int n = ifnum;
        if (real_ioctl_fn()(fd, USBDEVFS_CLAIMINTERFACE, &n) == 0)
            claimed.insert(ifnum);
    }
    g_fd_fully_claimed.insert(fd);
}

// SDK 正式 claim 某接口时，若由本 shim 预 claim，先归还（调用方持锁）
auto handover_if_auto_claimed(const int fd, const int ifnum) -> void {
    auto it = g_fd_claimed.find(fd);
    if (it == g_fd_claimed.end() || !it->second.erase(ifnum))
        return;
    int n = ifnum;
    (void)real_ioctl_fn()(fd, USBDEVFS_RELEASEINTERFACE, &n);
}

// 处理 usbfs ioctl 请求（ioctl 与 syscall 路径共用）
auto handle_usb_ioctl(const int fd, const unsigned long request, void* const arg) -> void {
    if (arg == nullptr)
        return;
    if (request == USBDEVFS_SUBMITURB || request == USBDEVFS_BULK
        || request == USBDEVFS_IOCTL) {
        // URB/BULK 提交：端点归属接口未知，预 claim 全部接口兜底。
        // 内核 usbfs 对未 claim 接口的 URB（含 interface 2 的中断端点）告警。
        claim_all_interfaces(fd);
        return;
    }
    if (request == USBDEVFS_CONTROL) {
        const auto* ctrl = static_cast<const usbdevfs_ctrltransfer*>(arg);
        if ((ctrl->bRequestType & 0x03U) == laser_shim::kInterfaceRecipient)
            claim_all_interfaces(fd);
        return;
    }
    if (request == USBDEVFS_CLAIMINTERFACE) {
        const int ifnum = *static_cast<const int*>(arg);
        std::lock_guard lock(g_mutex);
        handover_if_auto_claimed(fd, ifnum);
        return;
    }
    if (request == USBDEVFS_RELEASEINTERFACE) {
        const int ifnum = *static_cast<const int*>(arg);
        std::lock_guard lock(g_mutex);
        if (auto it = g_fd_claimed.find(fd); it != g_fd_claimed.end()) {
            it->second.erase(ifnum);
        }
        return;
    }
}

// ---- libusb 层 ----

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

// fd 关闭时清空注册表：SDK 会关闭并复用 fd 号（两台相机轮换打开），
// 复用后必须重新预 claim，否则新设备的 URB 会命中未 claim 接口。
int close(int fd) {
    {
        std::lock_guard lock(g_mutex);
        g_fd_claimed.erase(fd);
        g_fd_fully_claimed.erase(fd);
    }
    return real_posix_close()(fd);
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
    return real_claim()(dev, interface_number);
}

int libusb_release_interface(libusb_device_handle* dev, int interface_number) {
    {
        std::lock_guard lock(g_mutex);
        if (auto it = g_auto_claimed.find(dev); it != g_auto_claimed.end()) {
            it->second.erase(interface_number);
        }
    }
    return real_release()(dev, interface_number);
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
    if (transfer != nullptr)
    if (transfer != nullptr && transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL
        && transfer->buffer != nullptr) {
        ensure_claimed_for_setup(transfer->dev_handle,
            reinterpret_cast<const libusb_control_setup*>(transfer->buffer));
    }
    return real_submit()(transfer);
}

int ioctl(int fd, unsigned long request, ...) {
    va_list ap;
    va_start(ap, request);
    void* arg = va_arg(ap, void*);
    va_end(ap);

    handle_usb_ioctl(fd, request, arg);
    return real_ioctl_fn()(fd, request, arg);
}

// SDK 可能绕过 libc 的 ioctl 直接用 syscall(SYS_ioctl, ...)；固定 6 参原型与
// varargs 调用在 x86-64 SysV ABI 下寄存器传参一致，可安全转发。
long syscall(long number, long a1, long a2, long a3, long a4, long a5, long a6) {
    if (number == SYS_ioctl) {
        handle_usb_ioctl(static_cast<int>(a1), static_cast<unsigned long>(a2),
            reinterpret_cast<void*>(a3));
    }
    return real_syscall_fn()(number, a1, a2, a3, a4, a5, a6);
}

} // extern "C"
