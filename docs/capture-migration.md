# Capture 层收口说明

本文档描述 capture 层的当前形态。旧的 `ICaptureDevice` / factory 迁移方案已经结束，不再是现行设计。

## 当前设计

capture 层对外只保留一个具体入口：

```text
CaptureDevice
-> V4l2 backend
-> Hik backend
```

`CaptureDevice` 负责：

- 按 `Config.capture_backend` 选择 backend
- 打开设备并返回统一 `CaptureFormat`
- 读取 `Frame`
- 关闭设备

## 设计边界

- `V4l2Capture` 仍是低层实现细节。
- Hik SDK 仍隔离在独立 backend 实现中。
- runtime、tools、tests 不再通过虚接口或工厂选择 capture backend。

## 为什么这样收口

- 运行时只有两个稳定后端，`variant` 比虚接口更直接。
- 设备选择只发生在启动阶段，不需要长期多态。
- `CaptureFormat` 足够承载录制、推流和 snapshot 所需的统一信息。

## 新增 backend 时的规则

- 先判断是否真的需要第三个 backend。
- 如果需要，优先在 `CaptureDevice` 内增加一个新的 backend state。
- 只有当第三方 SDK 污染 public 头或编译边界时，才新增局部 wrapper。
- 不要恢复 `ICaptureDevice` 或独立 factory。

## 已删除的旧结构

- `ICaptureDevice`
- `capture_device_factory`
- `V4l2CaptureDevice`
- `HikCameraCaptureDevice`
