# 仓库指南

## 项目定位

`rmcs_laser_guidance` 的目标是用视觉引导激光振镜命中移动目标。

当前已落地的能力有：

- `V4L2/UVC` 取图
- Hik 工业相机取图（通过 `CaptureDevice` 内部 backend 选择）
- 原始视频会话录制与可选离线抽帧导出
- YAML 配置加载
- `CompetitionRuntime` + `CompetitionProfile(main / preview)`
- 独立的 `tool_guidance` / `GuidanceOpsApp`
- ROS2 bridge 为强制编译依赖（Docker 内置），`ros_bridge.cpp` 始终以完整 ROS2 实现编译；ROS2 类型通过 PIMPL 隔离在 `.cpp` 内部，不泄露到公共头文件。
- `build-laser` (构建)、`clean-laser` (清理)、`docker-build-laser` (镜像)、`foxglove-laser` (桥接) 为统一构建入口，容器内任意路径可调用。
- 调试 overlay 与回放样本
- 自动测试与工具入口
- ONNX / TensorRT 推理后端切换
- FT4222H USB-to-SPI 振镜控制
- Direct voltage 视觉到电压映射
- `HitProgress`、EKF 跟踪、敌方颜色过滤
- `RefereeLink` 订阅裁判系统 ZMQ（0x0001/0x020C），门控引导与 HitProgress；无信号时退化为不门控；`tools/referee_sim` 提供本地 mock。

当前明确不包含：

- ROS 控制总线
- ws30 / lidar 接入
- 通用 planner / solver 框架

当前特殊约束：

- ROS2 bridge 为强制编译依赖（Docker 内置），`RosBridge` 构造时自行调用 `rclcpp::init()` 若尚未初始化。
- FT4222 `libft4222.so` 由 `src/io/ft4222_spi.cpp` 通过 `dlopen` 动态加载，缺库或缺板卡时只影响 guidance，不阻塞主流程。
- `tools/dac8568_smoke` / `tools/galvo_smoke` 保留硬失败语义。
- 推理后端初始化遵循"先首选择后降级"策略，不再同时无条件构造 ONNX 和 TensorRT。
- `PerceptionRunner::degraded()` 反映后端实际可用性。
- `HitProgress` 按 RoboMaster 2026 空中机器人反制规则计算 5 次锁定与 1/2/3 难度阶段。
- 推流 encoder 在无 CUDA 设备时自动从 `h264_nvenc` 回退到 `libx264`。
- 旋转外参标定由工具级 `laser_guidance_calibration` 承担：固定机械平移与镜距，只接受七列有深度记录，并通过 Ceres `QuaternionManifold` 优化 `R_GC`；Wahba 只用于诊断或备用初值。
- 运行时日志写入 `logs/<timestamp>/laser_daemon.log`（stream）和 `logs/<timestamp>/laser_competition.log`（competition）。

## 目录职责

- `include/`
  - 稳定 public API，只放对外头文件。
- `include/laser_guidance/`
  - 稳定 facade：`support.hpp`、`runtime.hpp`、`bridges.hpp`。
- `src/capture/`
  - 视频采集实现：`CaptureDevice`、`V4l2Capture`，以及可选 Hik 后端。
- `src/vision/`
  - 视觉与推理：`ModelInfer`、`ModelRuntime`、`ModelAdapter`、`TensorRTEngine`、`TrainingData`。
- `src/guidance/`
  - 引导模块：`AimSolver`、`GalvoExecutor`、`ScanController`、`VoltageMapper`、`GalvoDriver`、`CameraProjection`。
- `src/runtime/`
  - runtime 核心：`ControlLoop`、`PerceptionRunner`、`GuidanceSession`、`RuntimeOutputs`、`OverlayRenderer`、`GuidanceOpsApp`。
- `src/bridges/`
  - FIFO / RTP / SHM / UDP bridge。
- `src/io/`
  - 硬件 I/O：`Ft4222Spi`。
- `tools/`
  - 可执行入口；`competition.cpp`、`preview.cpp`、`guidance.cpp` 只做启动和参数解析。
- `tests/`
  - 自动化验证，按模块分子目录。

## 当前边界

- `CompetitionRuntime` 只服务比赛 / 预览两种 profile。
- `tool_guidance` 不再通过 `CompetitionRuntime` 复用主 runtime。
- Hik typed parameter API 只保留在 `vendor/hikcamera` 与 `CaptureDevice` 内部，不进入 `CompetitionRuntime` public API。
- Hik SDK 的 RPATH、运行时环境注入和 mode 选择属于主仓库集成层，不继续下沉到子模块业务逻辑。
- runtime 内部不再保留 `guidance_ops`、`ControlLoopBuilder`、`GuidanceController`、`ModeHooks`、`OutputBundle`、`SnapshotAssembler`、`InferenceFacade`、`ICaptureDevice`。
- `RuntimeCommand` / `RuntimeSnapshot` 是 runtime 的 typed 控制面和观测面。
- 新增控制项优先改 typed command，再考虑是否映射到 FIFO。
- 新增输出优先走 `RuntimeSnapshot`，不要在主循环里直接拼协议细节。
- `TargetTrack` 与 `RuntimeSnapshot` 必须保持值语义安全。

## 迁移规则

- 新增比赛 / 预览能力时，先改 `runtime.hpp` 和 `ControlLoop`。
- 新增引导执行能力时，优先扩 `GuidanceSession`、`AimSolver`、`GalvoExecutor`、`ScanController`。
- 新增工具级校准或记录逻辑时，放到独立 app runner，不要再塞回共享 runtime。
- 仓库结构变化时，至少同步更新根目录 `AGENTS.md`、`README.md`、`docs/architecture.md`、`docs/runtime_operations.md`。
