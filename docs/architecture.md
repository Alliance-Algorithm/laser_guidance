# 架构

## 当前目标

使用相机视觉引导激光振镜，使激光实时命中移动无人机搭载的特征靶。
控制链路：Host PC → USB-to-SPI Bridge → SPI-to-DAC 驱动板 → 模拟电压 → 振镜 X/Y 角度。

`rmcs_laser_guidance` 负责视觉链路：

- 相机稳定出图
- 图像统一封装
- 检测 + EKF 跟踪
- 深度估计 + 3D 反投影 + 振镜角度解算
- 振镜 SPI 实时控制
- 视频输出（共享内存 + RTP 推流）
- 控制指令收发（FIFO + UDP）

## 数据输出分离

控制指令 (高频/小数据, 延迟敏感) 与 视频流 (大数据, 带宽敏感) 走不同通道：

| 通道 | 数据 | 协议 | 路径 |
|---|---|---|---|---|
| 控制 | 振镜角度指令 | SPI | FT4222H → DAC8568 → 振镜驱动 |
| 控制 | 检测结果、EKF 状态 | UDP | `127.0.0.1:5001` |
| 控制 | 运行时命令 | FIFO | `/tmp/laser_cmd` |
| 视频 | BGR 原始帧 | 共享内存 | `/laser_frame` (shm_open) |
| 视频 | H.264 编码流 | RTP | `127.0.0.1:5004` (可选) |

## Runtime 分层

当前运行架构不再由 `tool_competition` / `tool_preview` 各自拼装主循环，而是拆成 runtime facade + bridge；`tool_guidance` 仍保留独立兼容 runtime：

- runtime
  - `CompetitionRuntime`
  - `PreviewRuntime`
  - `RuntimeCommand`
  - `RuntimeSnapshot`
- bridges
  - `FifoControlServer`
  - `UdpTelemetryPublisher`
  - `ShmFramePublisher`
  - `RtpFramePublisher`
- guidance
  - `AimSolver`
  - `GalvoExecutor`
  - `ScanController`
  - `GuidancePipeline` 仅保留给 `GuidanceToolRuntime` 的兼容内核

职责边界：

- `tool_*.cpp` 只做入口，不承载主业务循环
- runtime 只处理 typed command，不解析 FIFO 字符串
- bridge 只做协议转换和数据搬运，不承载算法判断
- `RuntimeSnapshot` 必须可安全复制、缓存和延后消费

## 设计原则

- 先稳定运行时边界，再扩能力
  - 新需求优先判断它属于 runtime、guidance、vision、tracking 还是 bridge
- 先类型化，再协议化
  - 先定义内部 command/snapshot/data model，再决定是否暴露 FIFO/UDP/CLI
- 入口薄，模块厚
  - `tool_*` 只负责启动、参数、退出
  - 主业务循环必须沉到库层
- 兼容保留，但不继续扩张
- `GuidancePipeline`、`TargetObservation`、`rmcs_laser_guidance_core` 是迁移缓冲层
  - 新逻辑默认接 `DetectionBatch` / `TargetTrack` / `AimOutput`
- `TargetTrack` 是 public value type
  - 不允许再暴露依赖临时检测 batch 生命周期的裸指针字段
- bridge 不反向侵入 runtime
  - runtime 不依赖字符串协议和外部 UI 细节
  - bridge 也不把算法策略回写进 runtime

## 后续演进约束

- 如果要加新控制项：
  - 先改 `RuntimeCommand`
  - 再改 runtime 状态机
  - 最后再决定 FIFO/CLI 是否暴露
- 如果要加新调试或遥测：
  - 先改 `RuntimeSnapshot`
  - 再由 bridge 或 UI 消费
- 如果要加新引导策略：
  - 解算逻辑进入 `AimSolver`
  - 硬件下发进入 `GalvoExecutor`
  - 搜索/扫描进入 `ScanController`
- 不允许再新增“工具私有主循环”作为长期形态

## 当前数据流

```text
CompetitionRuntime / PreviewRuntime
-> V4l2Capture
-> Frame
-> LatestValue<Frame> / LatestValue<QueuedFrame>
-> ModelInfer (异步推理线程)
-> DetectionBatch
-> TargetTrack (raw / EKF-guided)
-> AimSolver -> AimOutput
-> draw_candidates() + draw_ekf_state()
-> ShmFramePublisher / RtpFramePublisher / UdpTelemetryPublisher
-> cv::imshow()                           ← 可选本地预览
```

`tool_guidance` 当前仍是独立兼容路径：

```text
tool_guidance
-> GuidanceToolRuntime
-> async infer + EKF + GuidancePipeline(compat core)
-> calib / tracking / CSV / purple hit edge record
```

## 引导数据流 (统一 runtime 路径)

```text
ModelInfer
-> DetectionBatch
-> TargetTrack
-> AimSolver
  -> DepthEstimator / LidarDepthEstimator
  -> CameraProjection
  -> GalvoKinematics / VoltageMapper
  -> AimOutput
-> GalvoExecutor
-> ScanController (rectangle scan only)
-> FT4222H → DAC8568 → 振镜驱动 → 反射镜
```

### Direct Voltage 数据流 (command_model: direct_voltage)

```text
ModelInfer + TargetTrack
-> (center + bbox/area)
-> VoltageMapper (LUT 或 poly3 三阶多项式)
-> 直接预测电压 (Vx, Vy)
-> GalvoExecutor
-> DAC/SPI 写入
```

矩形扫描模式下，扫描线程独占驱动输出，主线程仅更新扫描中心，避免 `driver_mutex_` 锁竞争。
扫描中心使用模型预测电压，扫描范围内部由角度参数换算为电压。

**EKF 策略**：瞄准中心用 EKF 预测位置，测距用当前帧 bbox 尺度。
丢帧时 EKF 短时预测维持，深度沿用最近有效值。EKF lost 时振镜回中。

**误差特征**：当前以固定误差（标定残余、外参粗糙、振镜非线性）为主。
EKF 参数偏向快响应：增大 process_noise_q、减小 measurement_noise_r，
降低预测权重，预测仅覆盖推理-执行延迟（~10-15ms）。

RTP 推流链路：
  main thread → push(latest_frame) → writer thread → fwrite → pipe → ffmpeg (libx264)
  ffplay 先占端口监听，daemon 启动后立即接收（无 late joiner 问题）

共享内存链路：
  main thread → VideoShmProducer.push_frame() → memcpy → /laser_frame
  consumer → shm_open + mmap → 读 ShmHeader → 读 buf_[write_idx]
```

数据集生成链路当前是：

```text
V4l2Capture
-> VideoSessionRecorder
-> raw.mp4 + session.yaml + notes.txt
-> Ultralytics Platform (recommended)
or
-> tool_transcode
-> ffmpeg in-place transcode to H.264/avc1
-> Ultralytics Platform
or
-> export_training_frames()
-> images/train|val|test + export_manifest.csv
-> external annotation / external training platform
```

其中：

- `Frame` 是图像和时间戳的统一载体
- `DetectionBatch` 是模型侧最小输出
- `TargetTrack` 是 runtime 侧跟踪视图
- `AimOutput` 是 guidance 侧执行视图
- `TargetObservation` 保留为兼容输出与 UDP 观测格式
- `ModelInfer` 负责模型推理接缝，并组合 `ModelRuntime`（ONNX）或 `TensorRTEngine`，以及 `ModelAdapter`（输出映射）
- `ModelRuntime` 负责 ONNX Runtime session（可选）或 TensorRT engine（可选）；输入输出元数据读取和实际推理执行
- `ModelAdapter` 负责把 YOLO26 端到端输出 `[1,300,6]` 映射为内部 `ModelCandidate`；3 class（purple=0, red=1, blue=2），无需 NMS
- `EkfTracker` 负责对检测中心做常加速度 EKF 平滑/预测，丢帧时保持状态估计（已接入异步推理线程）
- `DebugRenderer` 负责调试 overlay：含候选框、类别、置信度、准星；无 candidates 时回退为 contour + center
- `RtpFramePublisher` / `RtpStreamer` 负责将 overlay 后的画面通过 RTP 推流输出
- `ShmFramePublisher` / `VideoShmProducer` 负责通过 POSIX 共享内存输出 BGR 原始帧
- `UdpTelemetryPublisher` / `UdpSender` 负责通过 UDP 发送兼容观测格式
- `FifoControlServer` 负责 `/tmp/laser_cmd` -> `RuntimeCommand` 映射
- `V4l2Capture` 负责从 `/dev/videoN` 读取 UVC 图像

## 模块职责

### Config

负责把 `default.yaml` 解析成强类型结构，避免入口程序散落 YAML 解析逻辑。

当前还承载视觉后端选择配置：

- `inference.backend`
  - 当前支持 `model` 与 `tensorrt`；`bright_spot` 枚举值保留作为"禁用推理"标志
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

都先变成 `Frame` 再进入推理链路。

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
  - 纯测试框架，用于无硬件、无模型时验证 freshness 逻辑与 HitStateMachine
  - 当前仅测试使用，比赛链路未接入

### Replay

- `load_replay_dataset`
  - 从样本目录加载离线帧数据集用于测试和模型验证

### EkfTracker

- `EkfTracker`
  - 常加速度模型 EKF，6 维状态 `[x, y, vx, vy, ax, ay]`
  - 观测为检测中心位置
  - 支持 predict-only（丢帧时状态传播）和 update（检测帧校正）
  - `max_missed_frames` 判定目标丢失后自动重置
  - 已接入 `GuidanceToolRuntime` 和统一 runtime 推理线程

### Guidance（引导管线）

- `AimSolver`
  - 负责 depth / projection / kinematics / voltage mapping 解算
  - 不直接写硬件

- `GalvoExecutor`
  - 负责把 `AimOutput` 下发到 `GalvoDriver`
  - 负责回中、校准角度/电压命令

- `ScanController`
  - 独立管理 rectangle scan 线程
  - 主循环只更新扫描中心，不直接占有驱动输出

- `DepthEstimator`
  - 单目测距：`Z = fx × W_physical / pixel_size`
  - 按 `class_id` 查 `target_geometry` 获取靶物理尺寸
  - 取 `max(bbox.width, bbox.height)` 做尺度估计

- `CameraProjection`
  - 针孔反投影：去畸变像素坐标 + 深度 Z → 相机系 3D 点 P_c
  - 加载 `config/camera_calib.yaml`（由 `tool_calibrate` 生成）

- `GalvoKinematics`
  - 相机系 3D 点 → 外参平移 → 振镜系 → 光学角度 θx,θy
  - 带镜间距模型，fov ±30° optical

- `GalvoDriver`
  - 角度→电压线性映射 → DAC8568 码值 → SPI 写入
  - 支持差分/单端接线
  - 封装 DAC 内部参考使能、回中、角度下发

- `GuidancePipeline`
  - 兼容入口，当前主要由 `GuidanceToolRuntime` 内部复用
  - 提供 `process_ekf_guided(ekf_center, candidate, io_depth)` 接口
  - EKF 中心瞄准 + bbox 测距，丢帧时深度保持
  - 标定模式：`calib_mode=true` 时锁死振镜角度，通过 `estimate_depth` + `project_to_camera` 输出靶 3D 坐标
  - 扫描模式：`scan_mode=rectangle` 时启动独立线程持续 raster 扫描，主线程仅更新中心

### Runtime / Bridge

- `CompetitionRuntime`
  - 比赛守护进程统一入口
  - 负责 capture / infer / EKF / guidance / RTP / SHM / UDP / recording 生命周期

- `PreviewRuntime`
  - 复用同一套异步推理和输出管线，但不要求 guidance 就绪

- `RuntimeCommand`
  - 固定控制集合：`set_streaming` / `set_recording` / `set_enemy_color`
  - `set_backend` / `set_ekf` / `shutdown`

- `RuntimeSnapshot`
  - 汇总状态、检测、track、aim、recording root、active backend
  - backend 字段直接从 `InferenceFacade` 的当前 active runner 推导，不在 runtime 内维护第二份 backend 状态

- `FifoControlServer`
  - 兼容旧字符串协议：
    - `stream on/off`
    - `record on/off`
    - `enemy red/blue/auto`
    - `backend onnx/tensorrt`
    - `ekf on/off`
    - `quit`
  - 但它只负责转换，不进入算法主循环

### 为什么保留兼容层

- `GuidancePipeline`
  - `GuidanceToolRuntime` 仍用它承接现有校准、CSV 记录和 purple hit 边沿逻辑，短期保留可降低迁移风险
- `TargetObservation`
  - UDP 和旧测试仍围绕它工作，短期保留可避免一次性改穿所有输出面
- `rmcs_laser_guidance_core`
  - 现有工具和测试还在链接这个聚合 target，保留它可以让分层演进与迁移解耦

兼容层的含义是“暂时保留”，不是“继续扩展”。新能力优先接入分层 target 和 facade。

### Calibration Tools

- `tool_guidance` 标定模式
  - 配置 `calib_mode: true` + `config/calib_guidance.yaml`
  - `GuidanceToolRuntime` 负责 WASD 实时微调、步长缩放和 CSV 落盘
  - geometry 模式：空格键记录当前 `(θx, θy, P_c)` 到 `geometry_calib_records.csv`
  - direct_voltage 模式：空格键记录当前 `(center, bbox, Vx, Vy)` 到 `voltage_records.csv`

- `tool_calib_solve`
  - 读取 `geometry_calib_records.csv`，坐标下降法优化旋转外参
  - 平移量固定（物理测量值），只优化 `r_x/y/z_deg`
  - 输出优化后的旋转角和残差

- `purple confirmed` 命中样本
  - `tool_guidance` 正常跟踪模式下，若最高置信候选类别为 `purple`
  且 `HitStateMachine` 边沿进入 `Confirmed`，自动追加一条
    `(theta_x, theta_y, P_c_x, P_c_y, P_c_z)` 到 `geometry_hit_calib_records.csv`
  - 文件格式与 `geometry_calib_records.csv` 完全一致，可直接复用 `tool_calib_solve`

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
- `tool_record`
  - 把 live 相机录成原始视频会话，并同时写 `session.yaml`
- `tool_transcode`
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

- `tool_preview`
  - 本地预览 + 推流入口
- `tool_record`
  - 原始视频会话录制入口
- `tool_transcode`
  - 已录视频原地转码入口
- `tool_export`
  - 离线抽帧导出入口
- `tool_model_infer`
  - 打印 ONNX 模型元数据与推理结果

这些 `tool_*` 入口只负责运行流程，不负责视觉算法本身。

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

这意味着当前仓库的“外部协议”仍然很薄：当前主要面对本地配置、UDP 调试输出、RTP/SHM 视频输出和硬件 I/O，而不是更大的机器人中间件总线。

## 当前非目标

以下内容不应该在第一阶段偷偷引入：

- `/tf`
- `HardSyncSnapshot`
- ROS executor / plugin 运行时
- `pluginlib`
- 规划器
- 控制指令
- `fire_control`

这些内容一旦进入，就会把视觉最小骨架变成“半个控制系统”，偏离当前目标。
