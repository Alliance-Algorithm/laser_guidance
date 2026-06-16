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
- ROS2 bridge 为强制编译依赖（Docker 内置），`ros_bridge.cpp` 始终以完整 ROS2 实现编译；ROS2 类型通过 PIMPL 隔离在 `.cpp` 内部，不泄露到公共头文件。
- `.script/build` (构建)、`.script/clean` (清理)、`.script/docker-build` (镜像)、`.script/foxglove` (桥接) 为统一构建入口。

当仓库结构、架构边界或阶段约束变化时，请同步更新：

- [AGENTS.md](AGENTS.md)
- [docs/AGENTS.md](docs/AGENTS.md)
- [README.md](README.md)
- [docs/architecture.md](docs/architecture.md)
- [docs/runtime_operations.md](docs/runtime_operations.md)
