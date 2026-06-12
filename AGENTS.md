# Agent Notes

本文件是仓库根目录的稳定入口，详细说明见 [docs/AGENTS.md](/home/yukikaze/Documents/workspace/laser_guidance/docs/AGENTS.md)。

当前运行时状态：

- 唯一 public runtime 是 `CompetitionRuntime`
- `CompetitionProfile` 只保留 `main` / `preview`
- `tool_guidance` 走独立的 `runtime_internal::GuidanceOpsApp`
- `ControlLoop` 直接组合 `CaptureDevice`、`PerceptionRunner`、可选 `GuidanceSession`、`RuntimeOutputs`
- capture 后端选择收口在 `CaptureDevice` 内部
- ws30 / lidar 接入已删除

当仓库结构、架构边界或阶段约束变化时，请同步更新：

- [AGENTS.md](/home/yukikaze/Documents/workspace/laser_guidance/AGENTS.md)
- [docs/AGENTS.md](/home/yukikaze/Documents/workspace/laser_guidance/docs/AGENTS.md)
- [README.md](/home/yukikaze/Documents/workspace/laser_guidance/README.md)
- [docs/architecture.md](/home/yukikaze/Documents/workspace/laser_guidance/docs/architecture.md)
- [docs/runtime_operations.md](/home/yukikaze/Documents/workspace/laser_guidance/docs/runtime_operations.md)
