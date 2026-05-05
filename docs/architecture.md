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
-> draw_debug_overlay()
-> RtpStreamer.push()        ← 可选 RTP 推流
-> cv::imshow()              ← 可选本地预览
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
- `ModelAdapter` 负责把 YOLO26 端到端输出 `[1,300,6]` 映射为内部 `ModelCandidate`；3 class（purple=0, red=1, blue=2），无需 NMS
- `EkfTracker` 负责对检测中心做常加速度 EKF 平滑/预测，丢帧时保持状态估计（当前为 standalone 模块，尚未接入主链路）
- `DebugRenderer` 负责调试 overlay：含候选框、类别、置信度、准星；无 candidates 时回退为 contour + center
- `Pipeline` 组合"已选视觉后端"与 `DebugRenderer`
- `RtpStreamer` 负责将 overlay 后的画面通过 RTP 推流输出（ffmpeg 子进程），供 VLC 等外部播放器接收
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

串流输出配置：

- `streaming.enabled`
  - 启用/禁用 RTP 推流
- `streaming.host`
  - 接收端 IP 地址
- `streaming.port`
  - RTP 端口
- `streaming.sdp_path`
  - 自动生成的 SDP 描述文件路径

RTP 推流基于系统 ffmpeg，不依赖 ROS，纯 C++ standalone 构建即可使用。

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
- 当前支持 YOLO26 端到端 ONNX 输出（`[1,300,6]`）；契约不匹配时明确报错并保留输入输出元数据
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

### EkfTracker

- `EkfTracker`
  - 常加速度模型 EKF，6 维状态 `[x, y, vx, vy, ax, ay]`
  - 观测为 `TargetObservation.center` 位置
  - 支持 predict-only（丢帧时状态传播）和 update（检测帧校正）
  - `max_missed_frames` 判定目标丢失后自动重置
  - 当前为 standalone internal 模块，尚未接入 Pipeline 主链路

### RTP Streaming

- `RtpStreamer`
  - 将 overlay 后的 BGR 帧通过 ffmpeg 子进程编码为 H.264/RTP 推流
  - 零 ROS 依赖，纯 C++ standalone 构建即可使用
  - 自动生成 SDP 文件供 VLC 等 RTP 播放器接收
  - 与 `cv::imshow` 本地预览并行工作，互不阻塞

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

YOLO26 端到端推理输出 `[1, 300, 6]`，每行为 `[x1, y1, x2, y2, confidence, class_id]`。

3 class：
- `0 = Purple`
- `1 = Red`
- `2 = Blue`

取 confidence 最高且 > threshold 的候选作为最终检测结果。

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
