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
- 统一管理 RTP、SHM、UDP telemetry、ZMQ Laser JSON、录制。
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

`RosBridge` 将 `RuntimeSnapshot` 发布为 ROS2 topic：

- `/laser_guidance/detections` → `MarkerArray`（检测框）
- `/laser_guidance/tracks` → `MarkerArray`（跟踪轨迹+速度箭头）
- `/laser_guidance/aim_point` → `Marker`（瞄准方向）
- `/laser_guidance/hit_progress` → `Float64`（命中进度）

通过 Foxglove Studio（WebSocket `ws://localhost:8765`）或 rviz2 可视化。

`HitProgress` 按 RoboMaster 2026 空中机器人反制规则本地估算“被瞄准进度”：P 限制在 `[0,100]`，未照射时以 `0.5/s` 衰减并重置连续计数；连续照射时每满 `0.1s` 增加 `0.6*n`。第一次反制前 `P0=50`，之后 `P0=100`；每局最多 5 次锁定，难度序列为 `1,2,2,3,3`。

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

- 通过 `Ft4222Spi` 动态加载 `libft4222.so`（不再为进程级硬依赖）
- 构建 `AimSolver`
- 构建 `GalvoExecutor`
- 在矩形扫描模式下启用 `ScanController`

`Ft4222Spi` 的运行时加载策略：
- runtime（`GuidanceSession`）默认使用高速 SPI 时钟（`k60MHz / kDiv2` ≈ 30 MHz）
- smoke 工具（`tool_dac8568_smoke`、`tool_galvo_smoke`）默认使用保守低速（`kDiv64`）
- 缺 `libft4222.so` 或缺板卡时，guidance 初始化失败但不阻塞主流程

`tool_guidance` 的标定键盘、CSV 记录、purple hit edge 记录都在 `GuidanceOpsApp` 内部处理。

旋转外参标定采用以下边界：

- `tool_guidance` 新建几何标定文件时写入七列表头，并记录 `theta_x/y`、原始像素、正深度、`depth_source` 和 `depth_sigma_mm`。
- 自动深度记录标记为 `bbox`；手持测距数据由操作者标记为 `rangefinder`。
- `tool_calib_solve` 固定配置中的机械平移和镜距，外参标定期间强制 angle offset 为零，只在 Ceres `QuaternionManifold` 上优化相机到振镜旋转。
- 最终残差使用与运行时一致的近场模型 `p_G = R_GC * (p_C - t)`；Wahba 只作诊断或显式备用初值。
- 四列无深度和历史五列格式不再作为有效外参标定输入。

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

- `build-laser` — CMake 构建
- `clean-laser` — 清理 `build/`
- `docker-build-laser` — Docker 镜像构建（`--push` 推送）
- `foxglove-laser` — 启动 Foxglove WebSocket 桥接

## 约束

- 入口只负责启动和退出，不承载业务编排。
- 新控制项先扩 `RuntimeCommand` / `RuntimeSnapshot`。
- 新输出先扩 `RuntimeOutputs`，不要回塞主循环。
- 新引导能力优先扩 `GuidanceSession` / `AimSolver` / `GalvoExecutor`。
- ROS2 bridge 为强制依赖（Docker 内置），ROS2 类型通过 PIMPL 隔离在 `ros_bridge.cpp` 内部；`RosBridge` 构造时自行调用 `rclcpp::init()` 若尚未初始化。
- FT4222 为主入口工具级运行时依赖，不进 `CompetitionRuntime` 直接链接；指导层通过 `dlopen` 装载。
- inference backend 遵循"首选 + 必要时降级"策略，ONNX/TensorRT 不再同时无条件初始化。
- streaming encoder 在无 CUDA 设备时自动回退至 `libx264`。
