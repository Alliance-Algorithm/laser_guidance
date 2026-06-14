# laser_guidance

`laser_guidance` 是一个激光视觉引导系统，当前覆盖：

- `V4L2/UVC` 采集
- Hik 工业相机采集
- 原始视频录制与离线抽帧导出
- YAML 配置加载
- `CompetitionRuntime` 的 `main` / `preview` profile
- 独立 `tool_guidance` 引导标定入口
- ONNX / TensorRT 推理后端切换
- FT4222H USB-to-SPI 振镜控制
- Direct voltage 视觉到电压映射
- 调试 overlay、回放样本与自动测试

当前不包含：

- ws30 / lidar 接入
- ROS 控制总线
- 通用 planner / planner-like 抽象

## Runtime

- `CompetitionRuntime`
  - 唯一 public runtime facade。
  - 只保留 `main` / `preview` 两种 profile。
- `ControlLoop`
  - 直接组合 `CaptureDevice`、`PerceptionRunner`、可选 `GuidanceSession`、`RuntimeOutputs`。
- `CaptureDevice`
  - 内部按配置选择 `v4l2` 或 `hikcamera` backend。
- `PerceptionRunner`
  - 管理 ONNX / TensorRT 推理实例和 runtime backend 切换。
- `GuidanceSession`
  - 封装 FT4222、`AimSolver`、`GalvoExecutor`、`ScanController`。
- `RuntimeOutputs`
  - 管理 RTP、共享内存、UDP telemetry、录制。
- `tool_guidance`
  - 独立的 `GuidanceOpsApp`，用于校准和记录，不再走 `CompetitionRuntime`。

`tool_competition` 和 `tool_preview` 通过 FIFO `/tmp/laser_cmd` 接收运行时命令；`tool_guidance` 不使用这条链路。

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

可选后端：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DWITH_ONNXRUNTIME=ON \
  -DWITH_TENSORRT=ON \
  -DWITH_FT4222=ON \
  -DWITH_HIKCAMERA=ON
```

## Tools

```bash
./build/tool_competition
./build/tool_preview
./build/tool_guidance
./build/tool_record
./build/tool_model_infer
./build/tool_calibrate
./build/tool_transcode
./build/tool_export
./build/tool_dac8568_smoke
./build/tool_galvo_smoke
./build/tool_calib_solve
```

## Config

- `config/default.yaml`
  - 默认运行配置。
- `config/capture_hik.yaml`
  - Hik 工业相机示例配置。
- `config/geometry_run.yaml`
  - 比赛/运行配置示例。

## Repo Layout

```text
include/              public API
src/capture/          capture backend
src/vision/           inference and adapters
src/guidance/         solver and galvo control
src/runtime/          runtime core
src/bridges/          FIFO / RTP / SHM / UDP bridges
tools/                entrypoints
tests/                automated tests
config/               yaml configs
models/               onnx / engine files
test_data/            sample data
```
