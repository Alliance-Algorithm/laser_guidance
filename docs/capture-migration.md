# Capture 层抽象接入方案

## 目标

将 Capture 层的后端差异完全封装在模块内部，使所有外部 consumer 只依赖 `ICaptureDevice` + `CaptureFormat` 抽象，不再感知 V4L2 或 HikCamera 的具体类型。接入第三个后端时只需在 Capture 层内部加一个类 + 工厂里加一个 `case`，不改动任何 consumer。

## 模块化原则

```
改之前（高耦合）：
    V4l2Capture / V4l2NegotiatedFormat
        泄漏到 →
    ├── RuntimeBase（runtime_base.hpp + .cpp）
    ├── RuntimeOutputController（runtime_base.hpp + .cpp）
    ├── GuidanceToolRuntime（guidance_tool_runtime.hpp + .cpp）
    ├── tool_record（tools/record.cpp）
    └── tool_calibrate（tools/calibrate.cpp）

改之后（高内聚低耦合）：
    Capture 层对外只暴露：
        ICaptureDevice         ← 接口
        CaptureFormat          ← 统一格式
        create_capture_device  ← 工厂
    
    Capture 层内部消化：
        V4l2Capture / V4l2NegotiatedFormat  ← 外部不可见
        HikCameraCaptureDevice / hikcamera::Camera
    
    5 个 consumer 全部改为依赖 ICaptureDevice
```

## V4L2 类型泄漏面分析

当前 `V4l2Capture` 和 `V4l2NegotiatedFormat` 被 **5 个模块/类** 直接引用：

| 模块 | 泄漏点 | 行号 |
|------|--------|------|
| `RuntimeBase` | 成员 `V4l2Capture capture_` | `runtime_base.hpp:136` |
| | 成员 `std::optional<V4l2NegotiatedFormat> negotiated_format_` | `runtime_base.hpp:149` |
| `RuntimeOutputController` | `start_streaming(const V4l2NegotiatedFormat&)` | `runtime_base.hpp:58` |
| | `apply_requests(..., const std::optional<V4l2NegotiatedFormat>&)` | `runtime_base.hpp:61` |
| | `begin_recording(..., const V4l2NegotiatedFormat&)` | `runtime_base.hpp:70` |
| `GuidanceToolRuntime` | 成员 `V4l2Capture capture_` | `guidance_tool_runtime.hpp:84` |
| `tool_record` | `V4l2Capture capture(...)` + `open()` 返回 `V4l2NegotiatedFormat` | `tools/record.cpp:124-146` |
| | `print_mode(..., const V4l2NegotiatedFormat&)` | `tools/record.cpp:60-70` |
| `tool_calibrate` | `V4l2Capture capture(...)` + `open()` + `negotiated_format()` | `tools/calibrate.cpp:505-515` |
| `make_capture_snapshot` | 参数 `const V4l2NegotiatedFormat&` | `runtime_support.hpp:26` |

**这些都不是 "consumer 需要适配 Capture 层的新接口"——而是 "Capture 层的内部类型早已泄漏到 consumer 中，现在要收回来"。改动是在 consumer 侧修正类型声明，不是适配新 API。**

---

## 涉及文件

| 文件 | 改动类型 | 说明 |
|------|---------|------|
| **Capture 层** | | |
| `include/config.hpp` | 修改 | `Config` 增加 `capture_backend` + `HikCameraConfig`；`record_session_v4l2_config` 改为通用版本 |
| `src/capture/capture_device.hpp` | 修改 | `CaptureFormat` 增加录制字段（`device_path`, `pixel_encoding`） |
| `src/capture/capture_device_factory.hpp` | **新建** | 简单工厂：`switch(capture_backend)` 创建 `ICaptureDevice` |
| `src/capture/v4l2_capture_device.cpp` | 修改 | `to_capture_format` 填充新字段 |
| `src/capture/hik_camera_capture_device.cpp` | 修改 | `to_capture_format` 填充新字段 |
| **Runtime 层** | | |
| `src/runtime/runtime_base.hpp` | 修改 | `V4l2Capture` → `std::unique_ptr<ICaptureDevice>`；签名泛化 |
| `src/runtime/runtime_base.cpp` | 修改 | 工厂初始化、方法实现适配 `CaptureFormat` |
| `src/runtime/runtime_support.hpp` | 修改 | `make_capture_snapshot` 新增 `CaptureFormat` 重载 |
| `src/runtime/runtime_support.cpp` | 修改 | 实现新重载 |
| `src/runtime/guidance_tool_runtime.hpp` | 修改 | `V4l2Capture` → `std::unique_ptr<ICaptureDevice>` |
| `src/runtime/guidance_tool_runtime.cpp` | 修改 | `.` → `->`；include 变更 |
| **工具层** | | |
| `tools/record.cpp` | 修改 | `V4l2Capture` → 工厂获取；`V4l2NegotiatedFormat` → `CaptureFormat` |
| `tools/calibrate.cpp` | 修改 | 同上 |
| **录制支持** | | |
| `src/vision/training_data.hpp` | 无需修改 | `VideoSessionMetadata` 保持不变（`device_path`/`fourcc` 字段由 `CaptureFormat` 映射填充） |
| `src/runtime/support.cpp` | 修改 | `record_session_v4l2_config` 泛化 |

---

## 步骤

### Step 1：Config 扩展

`include/config.hpp` — 在 `Config` 中增加 backend 选择：

```cpp
// 新增枚举（与 capture_device.hpp 中的 CaptureBackendKind 一致）
enum class CaptureBackendKind : int { v4l2 = 0, hikcamera = 1 };

struct HikCameraConfig {
    std::string device_id{};
    unsigned int timeout_ms = 2000;
    float exposure_us = 2000.0F;
    float framerate = 80.0F;
    float gain = 16.9807F;
    bool invert_image = false;
    bool software_sync = false;
    bool trigger_mode = false;
    bool fixed_framerate = true;
};

struct Config {
    CaptureBackendKind capture_backend = CaptureBackendKind::v4l2;  // 新增
    V4l2Config v4l2{};
    HikCameraConfig hik{};                                          // 新增
    // DebugConfig, RuntimeConfig, InferenceConfig ... 保持不变
};
```

YAML 对应：

```yaml
capture_backend: v4l2          # v4l2 | hikcamera
v4l2:
  device_path: /dev/video0
  ...
hik:
  device_id: ""
  exposure_us: 2000.0
  ...
```

### Step 2：CaptureFormat 扩展

`src/capture/capture_device.hpp` — 增加录制所需字段：

```cpp
struct CaptureFormat {
    CaptureBackendKind backend = CaptureBackendKind::v4l2;
    std::string device_id{};         // 已有：设备标识
    int width = 0;                   // 已有
    int height = 0;                  // 已有
    double framerate = 0.0;          // 已有
    std::string format_name{};       // 已有：像素格式名

    // 新增 —— 录制/推流需要，替代 V4l2NegotiatedFormat
    std::string device_path{};       // 设备路径或标识（V4L2: /dev/video0; Hik: device_id）
    std::string pixel_encoding{};    // 编码格式（V4L2: fourcc 如 "MJPG"; Hik: "BGR8"）
};
```

同步更新两个 `to_capture_format()` 函数填充新字段：

```cpp
// v4l2_capture_device.cpp
auto to_capture_format(const V4l2NegotiatedFormat& format) -> CaptureFormat {
    return CaptureFormat{
        .backend = CaptureBackendKind::v4l2,
        .device_id = format.device_path.string(),
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .format_name = format.fourcc,
        .device_path = format.device_path.string(),   // 新增
        .pixel_encoding = format.fourcc,               // 新增
    };
}

// hik_camera_capture_device.cpp
auto to_capture_format(
    const hikcamera::DeviceInfo& device_info, const hikcamera::StreamFormat& format)
    -> CaptureFormat {
    return CaptureFormat{
        .backend = CaptureBackendKind::hikcamera,
        .device_id = device_info.device_id,
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .format_name = format.pixel_format_name,
        .device_path = device_info.device_id,          // 新增：HikCamera 无 /dev 路径，用 device_id
        .pixel_encoding = "BGR8",                       // 新增：HikCamera 统一输出 BGR8
    };
}
```

### Step 3：工厂方法

新建 `src/capture/capture_device_factory.hpp`：

```cpp
#pragma once

#include <memory>
#include <stdexcept>

#include "capture/capture_device.hpp"
#include "capture/v4l2_capture_device.hpp"
#include "capture/hik_camera_capture_device.hpp"
#include "config.hpp"

namespace rmcs_laser_guidance {

inline auto create_capture_device(const Config& config) -> std::unique_ptr<ICaptureDevice> {
    switch (config.capture_backend) {
    case CaptureBackendKind::v4l2:
        return std::make_unique<V4l2CaptureDevice>(config.v4l2);
    case CaptureBackendKind::hikcamera:
#ifdef WITH_HIKCAMERA
        return std::make_unique<HikCameraCaptureDevice>(
            HikCameraCaptureConfig{
                .device_id = config.hik.device_id,
                .timeout_ms = config.hik.timeout_ms,
                .exposure_us = config.hik.exposure_us,
                .framerate = config.hik.framerate,
                .gain = config.hik.gain,
                .invert_image = config.hik.invert_image,
                .software_sync = config.hik.software_sync,
                .trigger_mode = config.hik.trigger_mode,
                .fixed_framerate = config.hik.fixed_framerate,
            });
#else
        throw std::runtime_error("HikCamera backend requested but WITH_HIKCAMERA=OFF");
#endif
    }
    return nullptr;
}

} // namespace rmcs_laser_guidance
```

### Step 4：Consumer 逐个迁移

所有 consumer 的改动模式完全一致：**include 替换 + 成员类型替换 + `.` → `->`**。逻辑不改一行。

#### 4a. RuntimeBase + RuntimeOutputController

**`runtime_base.hpp`**：

```diff
-#include "capture/v4l2_capture.hpp"
+#include "capture/capture_device.hpp"

-    V4l2Capture capture_;
+    std::unique_ptr<ICaptureDevice> capture_;

-    std::optional<V4l2NegotiatedFormat> negotiated_format_;
+    std::optional<CaptureFormat> negotiated_format_;

-    auto start_streaming(const V4l2NegotiatedFormat& format) -> bool;
+    auto start_streaming(const CaptureFormat& format) -> bool;

-    auto apply_requests(
-        bool streaming_requested, bool recording_requested,
-        const RecordSessionOptions& record_options,
-        const std::optional<V4l2NegotiatedFormat>& negotiated_format) -> void;
+    auto apply_requests(
+        bool streaming_requested, bool recording_requested,
+        const RecordSessionOptions& record_options,
+        const std::optional<CaptureFormat>& negotiated_format) -> void;

-    auto begin_recording(const RecordSessionOptions&, const V4l2NegotiatedFormat&) -> void;
+    auto begin_recording(const RecordSessionOptions&, const CaptureFormat&) -> void;
```

**`runtime_base.cpp`**：

```diff
-    , capture_(config_.v4l2)
+    , capture_(create_capture_device(config_))

-    const auto open_result = capture_.open();
+    const auto open_result = capture_->open();

-    capture_.close();
+    capture_->close();

-    auto frame = capture_.read_frame();
+    auto frame = capture_->read_frame();
```

`RuntimeOutputController::begin_recording` 内部构建 `VideoSessionMetadata`：

```diff
     VideoSessionMetadata{
-        .device_path = format.device_path,
+        .device_path = format.device_path,     // CaptureFormat 已有此字段
         .width = format.width,
         .height = format.height,
         .framerate = format.framerate,
-        .fourcc = format.fourcc,
+        .fourcc = format.pixel_encoding,       // 用 CaptureFormat 通用字段
     };
```

#### 4b. GuidanceToolRuntime

**`guidance_tool_runtime.hpp`**：

```diff
-#include "capture/v4l2_capture.hpp"
+#include "capture/capture_device.hpp"

-    V4l2Capture capture_;
+    std::unique_ptr<ICaptureDevice> capture_;
```

**`guidance_tool_runtime.cpp`**：
构造时 `capture_ = create_capture_device(config_)`；`capture_.open()` → `capture_->open()`；`capture_.read_frame()` → `capture_->read_frame()`；`capture_.close()` → `capture_->close()`。

#### 4c. tool_record

**`tools/record.cpp`** `print_mode` 函数：

```diff
-auto print_mode(
-    const rmcs_laser_guidance::V4l2Config& requested,
-    const rmcs_laser_guidance::V4l2NegotiatedFormat& actual) -> void {
+auto print_mode(
+    const rmcs_laser_guidance::V4l2Config& requested,
+    const rmcs_laser_guidance::CaptureFormat& actual) -> void {
     std::println(
         "requested device={} mode={}x{}@{} format={}", requested.device_path.string(),
         requested.width, requested.height, requested.framerate,
         rmcs_laser_guidance::pixel_format_name(requested.pixel_format));
     std::println(
         "actual    device={} mode={}x{}@{} format={}", actual.device_path,
-        actual.width, actual.height, actual.framerate, actual.fourcc);
+        actual.width, actual.height, actual.framerate, actual.pixel_encoding);
 }
```

主函数：

```diff
-    const auto record_v4l2_config =
-        rmcs_laser_guidance::record_session_v4l2_config(config.v4l2);
-    ...
-    rmcs_laser_guidance::V4l2Capture capture(record_v4l2_config);
-    const auto open_result = capture.open();
+    // 录制时强制 YUYV 格式（仅 V4L2 需要）
+    auto record_config = config;
+    record_config.v4l2.pixel_format = rmcs_laser_guidance::V4l2PixelFormat::yuyv;
+    auto capture = rmcs_laser_guidance::create_capture_device(record_config);
+    const auto open_result = capture->open();
     if (!open_result) { ... }

-    print_mode(record_v4l2_config, *open_result);
+    print_mode(record_config.v4l2, *open_result);

     // VideoSessionMetadata 构建中：
-    .device_path = open_result->device_path,
+    .device_path = open_result->device_path,
     .width = open_result->width,
     .height = open_result->height,
     .framerate = fps,
-    .fourcc = open_result->fourcc,
+    .fourcc = open_result->pixel_encoding,

-    auto frame = capture.read_frame();
+    auto frame = capture->read_frame();

-    capture.close();
+    capture->close();
```

注：`tool_record` 原先的 `record_session_v4l2_config` 辅助函数（强制 YUYV 格式）逻辑移到 main 内直接修改 config，或者在 `support.cpp` 中提供通用版本。

#### 4d. tool_calibrate

**`tools/calibrate.cpp`**：

```diff
-    rmcs_laser_guidance::V4l2Capture capture(config.v4l2);
-    const auto open_result = capture.open();
+    auto capture = rmcs_laser_guidance::create_capture_device(config);
+    const auto open_result = capture->open();
     if (!open_result) { ... }

-    const auto& negotiated = capture.negotiated_format();
+    const auto& negotiated = *capture->negotiated_format();
     std::println(
         "Camera opened: {} ({}x{} @ {} fps {})", negotiated.device_path,
-        negotiated.width, negotiated.height, negotiated.framerate, negotiated.fourcc);
+        negotiated.width, negotiated.height, negotiated.framerate, negotiated.pixel_encoding);

-    auto frame = capture.read_frame();
+    auto frame = capture->read_frame();
```

### Step 5：make_capture_snapshot 追加重载

**`runtime_support.hpp`**：

```cpp
auto make_capture_snapshot(const CaptureFormat& format) -> CaptureFormatSnapshot;
```

**`runtime_support.cpp`**：

```cpp
auto make_capture_snapshot(const CaptureFormat& format) -> CaptureFormatSnapshot {
    return CaptureFormatSnapshot{
        .device_path = format.device_path,
        .width = format.width,
        .height = format.height,
        .framerate = format.framerate,
        .fourcc = format.pixel_encoding,
    };
}
```

旧的 `make_capture_snapshot(const V4l2NegotiatedFormat&)` 重载保留到所有 consumer 迁移完成后再移除。

---

## 执行顺序

```
Commit 1（Capture 层内部，不破坏编译）
  Step 1: Config 扩展
  Step 2: CaptureFormat 扩展 + 两个 to_capture_format 更新
  Step 3: 新建工厂 capture_device_factory.hpp

Commit 2（Consumer 迁移，一起改）
  Step 4a: RuntimeBase + RuntimeOutputController
  Step 4b: GuidanceToolRuntime
  Step 4c: tool_record
  Step 4d: tool_calibrate
  Step 5: make_capture_snapshot 新重载

Commit 3（清理，可选）
  移除旧的 make_capture_snapshot(V4l2NegotiatedFormat) 重载
  移除 No Consumer 引用的 V4L2 include
```

Step 4a-4d + Step 5 需要同一 commit —— `CaptureFormat` 字段调整和 consumer 的类型替换互为依赖。

## 验证清单

- [ ] `cmake --build build --parallel` 通过（WITH_HIKCAMERA=ON 和 OFF 两种配置）
- [ ] `ctest --test-dir build --output-on-failure` 通过
- [ ] `tool_preview` 以 V4L2 后端运行，取图/推流正常
- [ ] `tool_competition` 以 V4L2 后端运行，全链路正常
- [ ] `tool_guidance` 以 V4L2 后端运行，EKF/calib/CSV 正常
- [ ] `tool_record` 以 V4L2 后端运行，录制/元数据正常
- [ ] `tool_calibrate` 以 V4L2 后端运行，标定正常
- [ ]（需硬件）以 HikCamera 后端运行 `tool_preview`，取图正常

## 改造后加第三个后端的流程

1. 实现 `XXXCaptureDevice : ICaptureDevice`
2. 工厂加 `case CaptureBackendKind::xxx: return std::make_unique<XXXCaptureDevice>(...)`
3. `Config` 加 `XXXConfig` 字段
4. **5 个 consumer 一行不改**

## 暂不纳入改造

- HikCamera 运行时的参数热调接口（`Camera::parameter<Tag>()` 暂不暴露到 `HikCameraCaptureDevice` public 接口，后续单独设计）
- `VideoSessionRecorder` 的 `fourcc` 字段保持向后兼容（V4L2 用真实值，HikCamera 填 `"BGR8"`）
