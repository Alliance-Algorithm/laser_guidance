# Runtime Operations

## 运行约定

- `CompetitionRuntime.start()` 仍然是同步 readiness barrier。
- `main` / `preview` 共用同一套 `ControlLoop`，只是在 profile 上区分输出能力。
- `tool_guidance` 使用独立 `GuidanceOpsApp`。
- 运行时 backend 只在已构建可用的后端之间切换。

## Runtime Config

```yaml
runtime:
  max_input_age_ms: 25
  max_observation_age_ms: 35
  max_infer_fps: 60
  warmup_frames: 30
  engine_path: models/target_fp16_1x3x640x640.engine
  hit_confirm_frames: 3
  hit_release_frames: 5
  debug_enabled: false
  debug_max_fps: 30
  record_enabled: false
  record_queue_size: 16
```

说明：

- `record_enabled` 只在 `main` profile 下生效。
- `streaming`、`recording`、`enemy`、`backend`、`ekf` 都通过 `RuntimeCommand` 动态控制。
- `guidance.depth_source`、`guidance.lidar_*`、`ws30` 已删除。

## Build

```bash
cmake -S . -B build/default -DCMAKE_BUILD_TYPE=Release
cmake --build build/default --parallel
ctest --test-dir build/default --output-on-failure
```

所有后端均为强制依赖，不再需要 CMake option 开关。Hik 子模块默认 `HIKCAMERA_SDK_MODE=AUTO`，优先使用 vendored SDK，缺失时回退系统 MVS。可显式 `HIKCAMERA_SDK_MODE=system|vendor`，系统模式下用 `-DMVS_SDK_ROOT` 指定路径。

## Operational Notes

- `ControlLoop` 负责 capture、perception、guidance、overlay、outputs 和 snapshot。
- `RuntimeOutputs` 负责 RTP、SHM、UDP telemetry、recording。
- `GuidanceSession` 负责 FT4222、`AimSolver`、`GalvoExecutor`、`ScanController`。
- `tool_guidance` 的校准记录和 hit edge 记录都在独立 app 内完成。

## Verification

- backend 切换只发生在可用后端之间。
- `main` profile 允许录制，`preview` profile 不允许录制。
- FIFO 多行、半行、非法命令后可以恢复。
- `RuntimeSnapshot` 保持值语义安全。
- `tool_guidance` 仍能完成标定和 CSV 落盘。
