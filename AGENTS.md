# 仓库指南

## 项目定位
`rmcs_laser_guidance` 的目标是：

- 为二维云台搭载激光
- 视觉引导云台
- 使激光实时命中移动无人机搭载的特征靶

当前阶段仍然只是视觉最小骨架，已经落地的能力有：

- `V4L2/UVC` 取图
- 原始视频会话录制与可选离线抽帧导出
- `Config` / `Frame` / `TargetObservation` / `Pipeline`
- 内部 `Detector` / `ModelInfer` / `ModelRuntime` / `ModelAdapter` / `TensorRTEngine` / `TrainingData` / `DebugRenderer` / `Replay` / `V4l2Capture` / `RtpStreamer` / `UdpSender` / `EkfTracker` / `HitProgress` / `Ft4222Spi` / `GuidancePipeline` / `VoltageMapper` / `GalvoDriver` / `CameraProjection` / `DepthEstimator`
- 自动测试与工具运行入口
- 比赛模式统一守护进程 `tool_competition`（预览+引导+录制融合）
- ONNX + TensorRT 双推理后端，运行时热切换

## 目录职责
- `package.xml`
  - ROS 2 / ament 包声明。
- `CMakeLists.txt`
  - 顶层构建入口，支持 standalone CMake 与 ament 双模式。
- `config/default.yaml`
  - 默认运行配置。
- `config/capture_red_20m.yaml`
  - 推荐的原始视频采集配置。
- `models/`
  - 放置 `.onnx` 模型文件。
- `include/`
  - 扁平 public API，仅放稳定对外头文件。
- `src/core/`
  - 核心模块：`Config`、`Pipeline`、`FrameFormat`、`DebugRenderer`、`Replay`。
- `src/vision/`
  - 视觉/推理模块：`Detector`、`ModelInfer`、`ModelRuntime`、`ModelAdapter`、`TensorRTEngine`、`TrainingData`。
- `src/capture/`
  - 视频采集模块：`V4l2Capture`。
- `src/streaming/`
  - 网络推流模块：`RtpStreamer`、`UdpSender`、`VideoShm`。
- `src/tracking/`
  - 跟踪/状态模块：`EkfTracker`、`HitState`、`HitProgress`、`FreshnessQueue`、`RuntimeMetrics`。
- `src/io/`
  - 硬件 I/O 模块：`Ft4222Spi`、LibFT4222 头文件。
- `tools/`
  - 可执行工具入口（原 `examples/`）。
- `tests/`
  - 自动化验证，按模块分子目录（`core/`、`vision/`、`tracking/`），文件名统一为 `*_test.cpp`。
- `test_data/`
  - 样本回放与固定测试资源。
- `docs/`
  - 补充文档。
- `.script/`
  - 便捷 Shell 脚本：`set-config`（选配置）、`scan-camera`（扫描采集卡）、`preview`（imshow 预览）、`stream`（RTP 推流+ffplay）、`stop`（停止）。
  - 架构：`make preview` 启动常驻进程（FIFO `/tmp/laser_cmd`），后续命令通过管道发送，运行时开关推流无需重启。

## 当前公开与内部接口
public：

- `include/config.hpp`
- `include/types.hpp`
- `include/pipeline.hpp`
- `include/laser_guidance.hpp`

internal：

- `src/core/frame_format.hpp`
- `src/core/debug_renderer.hpp`
- `src/core/replay.hpp`
- `src/vision/detector.hpp`
- `src/vision/model_runtime.hpp`
- `src/vision/model_adapter.hpp`
- `src/vision/model_infer.hpp`
- `src/vision/training_data.hpp`
- `src/capture/v4l2_capture.hpp`
- `src/streaming/rtp_streamer.hpp`
- `src/streaming/udp_sender.hpp`
- `src/streaming/video_shm.hpp`
- `src/tracking/ekf_tracker.hpp`
- `src/tracking/hit_state.hpp`
- `src/tracking/freshness_queue.hpp`
- `src/tracking/runtime_metrics.hpp`
- `src/io/ft4222_spi.hpp`

设计边界：

- `Pipeline` 是唯一对外视觉入口。
- `Pipeline` 通过 `Details` 收束内部依赖，并在构造时选择视觉后端。
- `Frame` / `TargetObservation` 仍然公开 `OpenCV` 类型。
- `tools/` 与白盒 tests 可以直接使用 `src/` 下的内部模块头，但这些头不算 public API。

## 精要框架流图

### 当前核心链路
```text
config/direct_voltage_run.yaml (比赛) / config/default.yaml
-> load_config()
-> Config

V4l2Capture / synthetic frame / replay frame
-> Frame {image, timestamp}
-> ModelInfer (ONNX or TensorRT, dual-backend hot-switch)
-> ModelRuntime + ModelAdapter (ONNX) / TensorRTEngine
-> TargetObservation + candidates
-> enemy_color filter (keep enemy class + purple)
-> EkfTracker (optional, can bypass for raw detection)
-> GuidancePipeline::process_ekf_guided()
-> VoltageMapper (poly3) -> GalvoDriver (FT4222 SPI -> DAC8568 -> galvo)
-> HitProgress (lock-progress, 3-stage P0)
-> RtpStreamer (RTP + ffplay) / UdpSender / VideoShmProducer
-> VideoSessionRecorder (optional, FIFO toggle)
```

### 比赛模式入口
```text
tool_competition
-> V4l2Capture.open()
-> ModelInfer (ONNX + TensorRT dual load)
-> Ft4222Spi.open() -> GuidancePipeline
-> read_frame() -> async infer -> EKF/raw -> guidance -> overlay
-> RtpStreamer.push() + UdpSender + VideoShm + Recording
-> FIFO control: stream/record/enemy/backend/ekf/quit
```

### 当前运行入口
```text
tool_competition
-> V4l2Capture.open() + ModelInfer + Ft4222Spi + GuidancePipeline
-> async infer -> EKF/raw -> guidance -> RTP + UDP + SHM + Recording
-> FIFO /tmp/laser_cmd: stream/record/enemy/backend/ekf/quit

tool_preview
-> V4l2Capture.open()
-> read_frame()
-> Pipeline.process()
-> overlay + preview

tool_capture
-> V4l2Capture.open()
-> read_frame()
-> ReplayRecorder.record_frame()
-> manifest.csv + png frames

tool_record
-> V4l2Capture.open()
-> read_frame()
-> VideoSessionRecorder.record_frame()
-> raw.mp4 + session.yaml + notes.txt
-> direct upload to external platform or optional export_training_frames

tool_transcode
-> load session.yaml
-> locate raw.mp4
-> ffmpeg transcode to H.264/avc1 in place

tool_export
-> load session.yaml
-> open raw.mp4
-> export images/train|val|test + export manifest (optional fallback)

tool_replay
-> load_replay_dataset()
-> load frame png
-> Pipeline.process()
-> stdout + optional preview

*_test.cpp
-> load config / synthesize frame / load sample data
-> call one focused module
-> assert expected behavior
```

### 当前构建关系
```text
rmcs_laser_guidance_core
-> config / detector / model_runtime / model_adapter / tensorrt_engine / training_data
   / renderer / replay / v4l2 / rtp_streamer / udp_sender / video_shm
   / ekf_tracker / hit_state / hit_progress / freshness_queue / runtime_metrics
   / ft4222_spi / guidance_pipeline / voltage_mapper / galvo_driver / galvo_kinematics
   / camera_projection / depth_estimator / pipeline

tool_*
-> rmcs_laser_guidance_core
-> OpenCV

*_test
-> rmcs_laser_guidance_core
-> OpenCV
```

### 当前独立运行链路
```text
V4l2Source / VideoSource / ReplaySource
-> Frame
-> Detector
-> TargetObservation
-> EkfTracker (optional)
-> GuidancePipeline / VoltageMapper
-> FT4222H -> DAC8568 -> galvo
-> RTP / UDP / SHM / Recording
```

## 方向约束

当前仓库按 standalone 视觉引导工具维护，不再为 RMCS / ROS 控制链预留 bridge 设计。

如果后续新增能力，应优先延续现有独立入口与配置驱动模式，而不是先抽象外部总线接口。

## 当前边界
当前阶段的完成标准是：

- 相机稳定运行
- pipeline 能稳定处理正常图像和异常输入
- 自动测试与人工入口都存在
- 核心逻辑不依赖 ROS 控制链

当前模型接入约束补充：

- ONNX Runtime 只能作为可选构建接入，默认构建仍应可在没有 ONNX Runtime 的机器上通过。
- `model` 后端当前允许在启用 ONNX Runtime 时接入 YOLO26 端到端 ONNX（输出契约 `[1,300,6]`）；3 class（purple=0, red=1, blue=2）。TensorRT engine 可通过可选构建接入。输出契约不匹配时明确报错并打印输入输出元数据。
- 本仓库当前只负责模型接入和数据集生成，不负责本地训练逻辑。

以下内容不应在这个阶段偷偷引入：

- `/tf`
- `HardSyncSnapshot`
- `tracker`
- `solver`
- `planner`
- 控制指令
- `fire_control`

如果未来结构发生变化，至少同步更新：

- `README.md`
- `AGENTS.md`
- `docs/architecture.md`
