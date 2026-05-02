# 架构

## 当前目标

`rmcs_laser_guidance` 当前只解决视觉链路最小可运行问题：

- 相机是否能稳定出图
- 图像是否能被统一封装
- 检测入口是否已经固定
- 调试输出是否能独立于 RMCS 运行

它当前不是控制模块，因此不包含：

- 姿态同步
- 目标跟踪
- 弹道或开火时机
- `/gimbal/*` 输出

## 当前数据流

```text
V4l2Capture
-> read_frame()
-> Frame
-> Pipeline::process()
-> selected backend
-> Detector / ModelInfer
-> TargetObservation
-> draw_debug_overlay() / stdout
```

数据集生成链路当前是：

```text
V4l2Capture
-> VideoSessionRecorder
-> raw.mp4 + session.yaml + notes.txt
-> Ultralytics Platform (recommended)
or
-> example_transcode_recorded_session
-> ffmpeg in-place transcode to H.264/avc1
-> Ultralytics Platform
or
-> export_training_frames()
-> images/train|val|test + export_manifest.csv
-> external annotation / external training platform
```

其中：

- `Frame` 是图像和时间戳的统一载体
- `TargetObservation` 是最小视觉结果
- `Detector` 负责最小亮点检测
- `ModelInfer` 负责模型推理接缝，并组合 `ModelRuntime` 与 `ModelAdapter`
- `ModelRuntime` 负责 ONNX Runtime session（可选）或 TensorRT engine（可选）；输入输出元数据读取和实际推理执行
- `ModelAdapter` 负责把具体模型契约映射到仓库内部结果；当前契约是 3 class YOLO（Red=0, Blue=1, Purple=2），Purple 命中由 `HitStateMachine` 单独判定
- `RedTargetRefiner` 负责对红色 ROI 做灯条几何精修，给后续模型 ROI 后处理预留接缝
- `DebugRenderer` 负责最小调试绘制
- `Pipeline` 组合“已选视觉后端”与 `DebugRenderer`
- `V4l2Capture` 负责从 `/dev/videoN` 读取 UVC 图像

## 模块职责

### Config

负责把 `default.yaml` 解析成强类型结构，避免入口程序散落 YAML 解析逻辑。

当前还承载视觉后端选择配置：

- `inference.backend`
  - 当前支持 `bright_spot` 与 `model`
- `inference.model_path`
  - 指向 `.onnx` 模型文件；只有启用 ONNX Runtime 构建时才会实际加载

`model` 后端还能在构建时选择额外的 TensorRT 支持，但它和 ONNX Runtime 一样都只是可选运行时；TensorRT engine 需要离线预先生成，运行时不会现场构建。

### Frame

统一图像输入格式，后续不管是：

- `V4L2/UVC`
- 视频文件
- RMCS bridge

都先变成 `Frame` 再进入 pipeline。

### Pipeline

当前是最小视觉主入口，职责只有：

- 接受 `Frame`
- 返回 `TargetObservation`
- 在构造时选择视觉后端
- 转调 `Detector` 或 `ModelInfer`
- 转调 `DebugRenderer`

当前 `model` 路径的约束是：

- 未启用 ONNX Runtime 时，明确报错
- 未启用 TensorRT 时，TensorRT engine 路径不会被选中
- `model_path` 为空或文件不存在时，明确报错
- 模型加载成功后会执行预处理、推理和契约识别
- 当前优先支持 YOLOv5 原始输出、单张量 NMS 输出和 split-NMS 输出
- 输出契约未适配时，明确报错并保留输入输出元数据

### Freshness Runtime Primitives

运行时链路按“最新帧优先”设计，核心原语包括：

- `LatestValue<T>` 队列
  - 只保留最新值，旧值可被新值覆盖
  - 适合 Capture→Worker、debug、record 这类不允许积压的链路
- `StaleFramePolicy`
  - 定义帧是否因过期被丢弃
  - 新鲜度优先，过旧帧直接 drop，不允许排队等待
- `RuntimeMetrics`
  - 记录输入年龄、推理耗时、丢帧计数、覆盖计数等运行时观测值
- `HitStateMachine`
  - 负责 Purple HIT 状态判定
  - 通过连续帧迟滞确认/释放，避免 Purple 检测抖动导致状态频繁翻转
- `MockRuntime`
  - 用于无硬件、无模型或纯软件测试场景
  - 提供与真实 runtime 对齐的最小接口，便于验证 freshness 逻辑与状态机行为

### Replay

- `ReplayRecorder`
  - 把 `Frame` 录成 `PNG + manifest.csv`
- `load_replay_dataset`
  - 从样本或录帧目录回放 `Frame`

### Training Data

- `VideoSessionRecorder`
  - 把 live 相机流录成 `raw.mp4`（默认 `H.264/avc1`），并在 flush 时写 `session.yaml` 和 `notes.txt`
- `transcode_video_to_h264_in_place`
  - 用 `ffmpeg` 把已有 `raw.mp4` 原地转成 `H.264/avc1`
- `example_v4l2_record_session`
  - 把 live 相机录成原始视频会话，并同时写 `session.yaml`
- `example_transcode_recorded_session`
  - 对单个已录会话做原地 H.264 转码
- `export_training_frames`
  - 把单个视频会话离线抽成待标注图片；当前作为备用链路保留
- `write_export_manifest`
  - 为单次导出写出时间戳、split、blur score 等元数据

### Model Contract

当前 `model` 后端按 3 类目标检测契约工作：

- `0 = Red`
- `1 = Blue`
- `2 = Purple`

运行时会根据 `match_color` 做颜色过滤：

- `match_color=red`
  - 接受 `Blue` 与 `Purple`
  - 拒绝 `Red`
- `match_color=blue`
  - 接受 `Red` 与 `Purple`
  - 拒绝 `Blue`

其中 `Purple` 不是单独的控制支路，而是由 `HitStateMachine` 基于连续帧迟滞判定的 HIT 状态输入。

### Examples

- `example_v4l2_preview`
  - 真机入口
- `example_v4l2_capture`
  - 录帧入口
- `example_v4l2_record_session`
  - 原始视频会话录制入口
- `example_transcode_recorded_session`
  - 已录视频原地转码入口
- `example_export_training_frames`
  - 离线抽帧导出入口
- `example_offline_smoke`
  - 纯软件入口
- `example_replay_preview`
  - 回放入口
- `example_detector_benchmark`
  - 离线 benchmark
- `example_model_infer`
  - 打印 ONNX 模型路径、输入输出元数据和当前失败原因

Examples 只负责运行流程，不负责视觉算法本身。

当前推荐的数据集生成工作流详见：

- `docs/dataset_collection.md`

## 当前输出

当前输出只有两类：

- 结构化结果：`TargetObservation`
- 调试结果：图像窗口 / 标准输出 / 回放目录 / 视频会话目录 / 导出 manifest

模型后端在当前阶段还会额外暴露内部调试元数据：

- 输入输出 tensor 名称
- 输入输出 tensor shape
- 输入输出 tensor element type

运行时还会暴露 freshness 相关元数据：

- 输入帧年龄
- stale drop 计数
- 队列覆盖计数
- HIT 状态机当前状态

视频会话与可选导出链路当前还会保留：

- 视频会话元数据
- 原始视频路径
- 导出帧时间戳
- 导出帧 blur score

这意味着当前仓库的“外部协议”还没有形成。真正的 RMCS 内部总线接口要到第二阶段才会定义。

## 当前非目标

以下内容不应该在第一阶段偷偷引入：

- `/tf`
- `HardSyncSnapshot`
- `rmcs_executor::Component`
- `pluginlib`
- 规划器
- 控制指令
- `fire_control`

这些内容一旦进入，就会把视觉最小骨架变成“半个控制系统”，偏离当前目标。
