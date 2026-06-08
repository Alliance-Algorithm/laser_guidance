# Runtime Operations

## Runtime Assumptions

- `CompetitionRuntime.start()` / `PreviewRuntime.start()` 仍然是同步 readiness barrier。
- 推理后端在启动时一次性探测，运行时只切换已可用 backend。
- `InferenceFacade` 是 active backend 的唯一事实源。
- shutdown 顺序固定为：
  - `stop_requested = true`
  - `frame_queue_.shutdown()`
  - inference worker 退出并 join
  - `InferenceFacade::stop()`
  - guidance recenter / recorder / RTP / SHM / capture 收尾
  - snapshot/status 收敛为 stopped

## Runtime Config

当前有效 runtime 配置项：

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

- 当前没有 `runtime.mode` 配置项。
- TensorRT engine 路径来自 `runtime.engine_path`。
- recording / streaming 运行时开关通过 `RuntimeCommand` 和 FIFO bridge 动态控制。

## Build Commands

### Default Build

```bash
cmake -S . -B build/default -DCMAKE_BUILD_TYPE=Release
cmake --build build/default --parallel
ctest --test-dir build/default --output-on-failure
```

### ONNX Runtime Build

```bash
cmake -S . -B build/onnx -DCMAKE_BUILD_TYPE=Release \
  -DWITH_ONNXRUNTIME=ON
cmake --build build/onnx --parallel
```

### TensorRT Build

```bash
cmake -S . -B build/tensorrt -DCMAKE_BUILD_TYPE=Release \
  -DWITH_ONNXRUNTIME=ON \
  -DWITH_TENSORRT=ON
cmake --build build/tensorrt --parallel
```

### FT4222 Toggle

```bash
cmake -S . -B build/no-ft4222 -DCMAKE_BUILD_TYPE=Release \
  -DWITH_FT4222=OFF
cmake --build build/no-ft4222 --parallel
```

## TensorRT Engine Build

```bash
trtexec \
  --onnx=models/target.onnx \
  --saveEngine=models/target_fp16_1x3x640x640.engine \
  --fp16 \
  --optShapes=images:1x3x640x640 \
  --skipInference
```

## Operational Notes

- Debug 和 recording 都必须是 lossy 的，不能阻塞 capture -> infer 主路径。
- `tool_competition` / `tool_preview` 只处理 runtime 生命周期和 FIFO bridge。
- `tool_guidance` 现由 internal `GuidanceToolRuntime` 承接：
  - 异步推理
  - WASD 校准步进
  - geometry/direct-voltage CSV 记录
  - purple confirmed hit 边沿记录
- UDP telemetry 继续输出兼容 `TargetObservation` 语义，但数据来源统一走 `RuntimeSnapshot`.

## Verification Focus

- backend 切换是否只在可用 backend 间发生
- stop 后 `active_backend_name` / `backend_uses_tensorrt` 是否正确清空
- FIFO 多行、半行、非法命令后恢复是否正常
- `tool_guidance` 校准记录和 purple hit 边沿记录是否稳定
