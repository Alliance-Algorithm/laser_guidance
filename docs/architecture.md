# 架构

## 当前目标

使用相机视觉引导激光振镜，使激光命中移动目标。

当前系统只保留一条主视觉链路：

`CaptureDevice -> PerceptionRunner -> TargetTrack -> GuidanceSession -> RuntimeOutputs`

不再包含 ws30 / lidar 接入。

## Runtime 结构

- `CompetitionRuntime`
  - 唯一 public runtime。
  - 只保留 `main` / `preview` profile。
- `ControlLoop`
  - 主循环骨架，直接持有具体组件，不再通过 builder / controller / hooks 装配。
- `RuntimeCommand`
  - 运行时 typed 控制面。
- `RuntimeSnapshot`
  - 运行时 typed 观测面。
- `RuntimeOutputs`
  - 统一管理 RTP、SHM、UDP telemetry、录制。
- `GuidanceSession`
  - 封装 FT4222、`AimSolver`、`GalvoExecutor`、`ScanController`。
- `GuidanceOpsApp`
  - `tool_guidance` 的独立应用循环，用于标定和记录。

## 数据流

```text
CaptureDevice
-> Frame
-> PerceptionRunner
-> DetectionBatch
-> TargetTrack
-> GuidanceSession (optional)
-> AimOutput + telemetry
-> RuntimeOutputs / overlay / RuntimeSnapshot
```

## 调试桥接

`RosBridge` (可选，`WITH_ROS2_BRIDGE`) 将 `RuntimeSnapshot` 发布为 ROS2 topic：

- `/laser_guidance/detections` → `MarkerArray`（检测框）
- `/laser_guidance/tracks` → `MarkerArray`（跟踪轨迹+速度箭头）
- `/laser_guidance/aim_point` → `Marker`（瞄准方向）
- `/laser_guidance/hit_progress` → `Float64`（命中进度）

通过 Foxglove Studio（WebSocket `ws://localhost:8765`）或 rviz2 可视化。

## 采集层

`CaptureDevice` 在内部按配置选择 backend：

```text
CaptureDevice
-> V4l2Capture
-> Hik camera backend
```

外部只依赖 `CaptureFormat`，不再接触 backend 细节。

Hik backend 继续被视为主仓库内置能力：`CaptureDevice` 直接依赖 `hikcamera`，但 Hik SDK 的运行时环境注入和构建接入仍由主仓库处理，不向 `CompetitionRuntime` 暴露 `hikcamera::param::*` 等子模块类型。

## 推理层

`PerceptionRunner` 负责：

- 持有 ONNX / TensorRT 推理实例
- 按 `RuntimeCommand` 切换 active backend
- 做敌方颜色过滤
- 维护 EKF 跟踪输入

## 引导层

`GuidanceSession` 负责：

- 打开 FT4222H
- 构建 `AimSolver`
- 构建 `GalvoExecutor`
- 在矩形扫描模式下启用 `ScanController`

`tool_guidance` 的标定键盘、CSV 记录、purple hit edge 记录都在 `GuidanceOpsApp` 内部处理。

## 配置边界

当前有效配置包括：

- `capture_backend`
- `v4l2` / `hik`
- `runtime`
- `inference`
- `streaming`
- `udp`
- `ekf`
- `guidance`

已删除：

- `ws30`
- `guidance.depth_source`
- `guidance.lidar_*`

## 运行入口

- `tool_competition`
  - `CompetitionRuntime(profile=main)`
  - FIFO 控制
  - 允许录制
- `tool_preview`
  - `CompetitionRuntime(profile=preview)`
  - FIFO 控制
  - 不允许录制
- `tool_guidance`
  - `GuidanceOpsApp`
  - 不走 `CompetitionRuntime`

## 构建脚本

- `.script/build` — CMake 构建（`--ros2` 启用 ROS2 桥接）
- `.script/clean` — 清理 `build/`
- `.script/docker-build` — Docker 镜像构建（`--push` 推送）
- `.script/foxglove` — 启动 Foxglove WebSocket 桥接

## 约束

- 入口只负责启动和退出，不承载业务编排。
- 新控制项先扩 `RuntimeCommand` / `RuntimeSnapshot`。
- 新输出先扩 `RuntimeOutputs`，不要回塞主循环。
- 新引导能力优先扩 `GuidanceSession` / `AimSolver` / `GalvoExecutor`。
- `WITH_ROS2_BRIDGE` 是唯一的可选编译选项；`ros_bridge.cpp` 始终编译（无 ROS2 时为 no-op）。
