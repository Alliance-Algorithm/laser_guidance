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
- `WITH_ROS2_BRIDGE` 是唯一的可选编译选项，用于 Foxglove/rviz 调试；始终编译 `ros_bridge.cpp`（无 ROS2 时为 no-op）。
- `.script/build` (构建)、`.script/clean` (清理)、`.script/docker-build` (镜像)、`.script/foxglove` (桥接) 为统一构建入口。

当仓库结构、架构边界或阶段约束变化时，请同步更新：

- [AGENTS.md](AGENTS.md)
- [docs/AGENTS.md](docs/AGENTS.md)
- [README.md](README.md)
- [docs/architecture.md](docs/architecture.md)
- [docs/runtime_operations.md](docs/runtime_operations.md)
