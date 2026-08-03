# Agent Notes

本文件是仓库根目录的稳定入口，详细说明见 [docs/AGENTS.md](docs/AGENTS.md)。

当前运行时状态：

- 唯一 public runtime 是 `CompetitionRuntime`
- `CompetitionProfile` 只保留 `main` / `preview`
- `tool_guidance` 走独立的 `runtime_internal::GuidanceOpsApp`
- `ControlLoop` 直接组合 `CaptureDevice`、`PerceptionRunner`、可选 `GuidanceSession`、`RuntimeOutputs`
- capture 后端选择收口在 `CaptureDevice` 内部
- Hik typed parameter API 只保留在 `vendor/hikcamera` 与 `CaptureDevice` 内部边界，不暴露到 `CompetitionRuntime`
- Hik SDK 运行时环境注入只由上层脚本处理，不下沉到子模块
- ws30 / lidar 接入已删除
- ROS2 bridge 为强制编译依赖（Docker 内置），`ros_bridge.cpp` 始终以完整 ROS2 实现编译；ROS2 类型通过 PIMPL 隔离在 `.cpp` 内部，不泄露到公共头文件；`RosBridge` 构造时自行调用 `rclcpp::init()` 若尚未初始化。
- FT4222 为主入口工具级运行时依赖，`src/io/ft4222_spi.cpp` 通过 `dlopen` 动态加载 `libft4222.so`；缺库或缺板卡时只影响 guidance，不影响主流程；`tools/dac8568_smoke` / `tools/galvo_smoke` 保留硬失败语义。
- MVS SDK U3V 控制传输不 claim 接口会触发内核 reset（dmesg `did not claim interface`）；`src/io/libusb_claim_shim.cpp` 为 LD_PRELOAD 拦截层（`build/liblibusb_claim_shim.so`），由 `.script/host-runtime-env.sh` 注入 daemon 进程，自动补 claim；缺失时不影响主流程。
- 推理后端初始化采用"先首选择后降级"策略，`PerceptionRunner::degraded()` 反映后端实际可用性；ONNX/TensorRT 初始化不再同时无条件构造。
- `HitProgress` 按 RoboMaster 2026 空中机器人反制规则计算 5 次锁定与 1/2/3 难度阶段。
- `RefereeLink` 订阅裁判系统 ZMQ（0x0001/0x020C），全程引导；比赛窗口以裁判 `stage_remain_time` 为权威（技术暂停时停表），断流超过 `signal_timeout_s` 才退化本地 `match_duration_s` 兜底；game_progress 用于每局 HitProgress 重置与 0x020C 锁定校核，无信号/赛外退化纯本地计算；`tools/referee_sim` 提供本地 mock；比赛窗口内以官方反制成功次数（0x020C 边沿计数）权威同步阶段，离线退回本地。
- 推流 encoder 在无 CUDA 设备时自动从 `h264_nvenc` 回退到 `libx264`。
- 旋转外参标定固定机械平移与镜距，只接受七列有深度记录，并通过 Ceres `QuaternionManifold` 优化 `R_GC`；Wahba 只用于诊断或备用初值。
- `build-laser` (构建)、`clean-laser` (清理)、`docker-build-laser` (镜像)、`foxglove-laser` (桥接) 为统一构建入口，容器内任意路径可调用。
- 运行时日志写入 `logs/<timestamp>/laser_daemon.log` / `laser_competition.log`。

当仓库结构、架构边界或阶段约束变化时，请同步更新：

- [AGENTS.md](AGENTS.md)
- [docs/AGENTS.md](docs/AGENTS.md)
- [README.md](README.md)
- [docs/architecture.md](docs/architecture.md)
- [docs/runtime_operations.md](docs/runtime_operations.md)
