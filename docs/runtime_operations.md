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
- `HitProgress` 使用 RoboMaster 2026 空中机器人反制规则：未照射按 `0.5/s` 衰减，连续照射每 `0.1s` 增加 `0.6*n`，最多 5 次锁定。

## Build

```bash
cmake -S . -B build/default -DCMAKE_BUILD_TYPE=Release
cmake --build build/default --parallel
ctest --test-dir build/default --output-on-failure
```

Hik 子模块默认 `HIKCAMERA_SDK_MODE=AUTO`，优先使用 vendored SDK，缺失时回退系统 MVS。可显式 `HIKCAMERA_SDK_MODE=system|vendor`，系统模式下用 `-DMVS_SDK_ROOT` 指定路径。

## Operational Notes

- `ControlLoop` 负责 capture、perception、guidance、overlay、outputs 和 snapshot。
- `RuntimeOutputs` 负责 RTP、SHM、UDP telemetry、ZMQ Laser JSON (`cmd_id=0x2003`)、recording。
- `GuidanceSession` 负责 FT4222、`AimSolver`、`GalvoExecutor`、`ScanController`。
- FT4222 为主入口工具级运行时依赖；缺库或缺板卡时只影响 guidance，主流程继续。
- `tool_guidance` 的校准记录和 hit edge 记录都在独立 app 内完成。

### 旋转外参标定

1. 相机必须使用与 `camera_calib.yaml` 相同的分辨率、ROI 和翻转方式。
2. 确认配置中的 `t_x_mm/t_y_mm/t_z_mm` 是机械测量值；标定期间 angle offset 按零处理。
3. 使用 `tool_guidance` 采集覆盖视场和多个距离的记录，默认写入 `test_data/calib/rotation_calib_records.csv`。新文件格式为：

```text
theta_x_deg,theta_y_deg,pixel_x,pixel_y,depth_mm,depth_source,depth_sigma_mm
```

4. 自动框深度使用 `bbox`；手持测距仪的斜距使用 `rangefinder`，并填写包含手持起点误差的标准差。
5. 求解命令示例：

```bash
./build/tool_calib_solve records.csv config/calib.yaml config/camera_calib.yaml \
  --image-width 2448 --image-height 2048
```

6. 工具固定平移和镜距，只优化旋转；四列无深度和历史五列 CSV 会被拒绝。
7. 用未参与拟合的点验证 RMS、P95 和最大角误差后，再复制高精度 `r_x/r_y/r_z`。
8. 锁定旋转后，使用独立中心靶从零开始调整 `angle_offset_x/y_deg`。

### 推理后端降级
- `PerceptionRunner::initialize_backends()` 按"先首选择后降级"策略初始化；ONNX/TensorRT 不再同时无条件构造。
- `PerceptionRunner::degraded()` 返回 true 时（无可用后端或推理未启用），主循环跳过推理，不退出进程。
- TensorRT 引擎加载前检查 CUDA 设备可用性；无 CUDA 设备时跳过 TensorRT 初始化。

### RTP 推流
- encoder 在无 CUDA 设备时自动从 `h264_nvenc` 回退到 `libx264`。
- `RtpStreamer::stop()` 为幂等调用。

### ROS2 桥接
- `RosBridge` 构造时自行调用 `rclcpp::init()` 若尚未初始化。
- 桥接初始化失败时主流程继续运行，不退出。

### 日志
- 运行时日志由启动脚本重定向 `stderr` 写入仓库根目录下的时间戳目录：`logs/<timestamp>/laser_daemon.log`（stream）、`logs/<timestamp>/laser_competition.log`（competition）。

## Verification

- backend 切换只发生在可用后端之间。
- `main` profile 允许录制，`preview` profile 不允许录制。
- FIFO 多行、半行、非法命令后可以恢复。
- `RuntimeSnapshot` 保持值语义安全。
- `tool_guidance` 能写入七列深度标定 CSV，`tool_calib_solve` 能完成固定平移的 SO(3) 旋转优化。
