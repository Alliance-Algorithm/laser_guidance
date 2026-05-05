# 仓库指南

## 项目定位
`rmcs_laser_guidance` 的目标是：

- 为二维云台搭载激光笔
- 视觉引导云台
- 使激光实时命中移动无人机搭载的特征靶

当前阶段仍然只是视觉最小骨架，已经落地的能力只有：

- `V4L2/UVC` 取图
- 原始视频会话录制与可选离线抽帧导出
- `Config` / `Frame` / `TargetObservation` / `Pipeline`
- 内部 `Detector` / `ModelInfer` / `ModelRuntime` / `ModelAdapter` / `RedTargetRefiner` / `TrainingData` / `DebugRenderer` / `Replay` / `V4l2Capture`
- 自动测试与人工运行入口

当前明确**不是**闭环控制系统，不包含：

- `tracker`
- `solver`
- `/tf`
- `/gimbal/*`
- `fire_control`
- `pluginlib`
- `rmcs_executor`

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
- `src/`
  - 公开接口实现。
- `src/internal/`
  - 不安装的内部实现头。
- `examples/`
  - 人工运行入口。
- `tests/`
  - 自动化验证，文件名统一为 `*_test.cpp`。
- `test_data/`
  - 样本回放与固定测试资源。
- `docs/`
  - 补充文档。

## 当前公开与内部接口
public：

- `include/config.hpp`
- `include/types.hpp`
- `include/pipeline.hpp`
- `include/rmcs_laser_guidance.hpp`

internal：

- `src/internal/frame_format.hpp`
- `src/internal/detector.hpp`
- `src/internal/debug_renderer.hpp`
- `src/internal/replay.hpp`
- `src/internal/model_runtime.hpp`
- `src/internal/model_adapter.hpp`
- `src/internal/model_infer.hpp`
- `src/internal/red_target_refiner.hpp`
- `src/internal/training_data.hpp`
- `src/internal/v4l2_capture.hpp`

设计边界：

- `Pipeline` 是唯一对外视觉入口。
- `Pipeline` 通过 `Details` 收束内部依赖，并在构造时选择视觉后端。
- `Frame` / `TargetObservation` 仍然公开 `OpenCV` 类型。
- examples 与白盒 tests 可以使用 `src/internal/`，但这些头不算 public API。

## 精要框架流图

### 当前核心链路
```text
config/default.yaml
-> load_config()
-> Config

V4l2Capture / synthetic frame / replay frame
-> Frame {image, timestamp}
-> Pipeline
-> selected backend
-> Detector / ModelInfer
-> ModelRuntime + ModelAdapter (when backend=model)
-> TargetObservation {detected, center, contour, brightness}
-> DebugRenderer / stdout / replay output
```

### 当前运行入口
```text
example_v4l2_preview
-> V4l2Capture.open()
-> read_frame()
-> Pipeline.process()
-> overlay + preview

example_v4l2_capture
-> V4l2Capture.open()
-> read_frame()
-> ReplayRecorder.record_frame()
-> manifest.csv + png frames

example_v4l2_record_session
-> V4l2Capture.open()
-> read_frame()
-> VideoSessionRecorder.record_frame()
-> raw.mp4 + session.yaml + notes.txt
-> direct upload to external platform or optional export_training_frames

example_transcode_recorded_session
-> load session.yaml
-> locate raw.mp4
-> ffmpeg transcode to H.264/avc1 in place

example_export_training_frames
-> load session.yaml
-> open raw.mp4
-> export images/train|val|test + export manifest (optional fallback)

example_replay_preview
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
-> config / detector / model_runtime / model_adapter / red_target_refiner / training_data / renderer / replay / v4l2 / pipeline

example_*
-> rmcs_laser_guidance_core
-> OpenCV

*_test
-> rmcs_laser_guidance_core
-> OpenCV
```

### 目标演进链路
```text
V4l2Source / VideoSource / ReplaySource
-> Frame
-> Detector
-> TargetObservation
-> Tracker
-> TargetState
-> Solver / GuidanceLogic
-> GuidanceResult
-> LaserGuidanceBridge
-> /gimbal/laser_guidance/*
-> gimbal controller
```

## 未来接入约束
推荐命名空间：

```text
/gimbal/laser_guidance/*
```

不要直接复用：

```text
/gimbal/auto_aim/*
```

未来 bridge 的职责只应包括：

- 参数读取
- 输入采集
- 结果发布

bridge 不应承载算法实现本身。

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
