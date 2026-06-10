# Agent Notes

本文件保留在仓库根目录，作为 agent / tooling 的稳定入口。

详细仓库指南已迁移到：

- [docs/AGENTS.md](/home/yukikaze/Documents/workspace/laser_guidance/docs/AGENTS.md)

当前 Phase 1 采集层补充：

- `src/capture/` 已新增 internal 通用采集接口 `ICaptureDevice`、`CaptureFormat`
- `V4l2Capture` 保持原状，新增 `V4l2CaptureDevice` adapter
- 已新增 `HikCameraCaptureDevice`，但主 runtime / tool 链路暂未切换到 Hik
- Hik SDK 运行库默认直接来自仓库内 `vendor/hikcamera/src/sdk/lib`

当仓库结构、架构边界或阶段约束发生变化时，请至少同步更新：

- [AGENTS.md](/home/yukikaze/Documents/workspace/laser_guidance/AGENTS.md)
- [docs/AGENTS.md](/home/yukikaze/Documents/workspace/laser_guidance/docs/AGENTS.md)
- [README.md](/home/yukikaze/Documents/workspace/laser_guidance/README.md)
- [docs/architecture.md](/home/yukikaze/Documents/workspace/laser_guidance/docs/architecture.md)
