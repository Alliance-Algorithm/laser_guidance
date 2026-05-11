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

## 当前数据流

```text
V4l2Capture
-> read_frame()
-> Frame
-> ModelInfer (异步推理线程)
-> EkfTracker (CA-EKF 平滑/预测)
-> draw_candidates() + draw_ekf_state()   ← 框/标签/准星 + EKF 绿点/速度箭头
-> VideoShmProducer.push_frame()          ← 共享内存 BGR 帧 (零拷贝)
-> RtpStreamer.push()                     ← 可选 RTP 推流 (ffmpeg)
-> UdpSender.send()                       ← UDP 观测 + EKF 状态
-> cv::imshow()                           ← 可选本地预览 (关窗即停)
```

## 引导数据流 (tool_guidance)

```text
ModelInfer
-> TargetObservation (center + bbox + class_id)
-> EkfTracker (CA-EKF, 像素空间平滑/预测)
-> EKF position (aim_center)
        ↓
DepthEstimator (bbox 尺度 → 深度 Z)
        ↓ Z + aim_center
CameraProjection (像素 + Z → 相机 3D 点 P_c)
        ↓ P_c
GalvoKinematics (外参平移 → 振镜系 P_g → 角度 θx,θy)
        ↓ θx,θy
GalvoDriver (角度→电压→DAC码值→SPI写入)
        ↓ SPI
FT4222H → DAC8568 → 振镜驱动 → 反射镜
```

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
- `EkfTracker` 负责对检测中心做常加速度 EKF 平滑/预测，丢帧时保持状态估计（已接入异步推理线程）
- `DebugRenderer` 负责调试 overlay：含候选框、类别、置信度、准星；无 candidates 时回退为 contour + center
- `Pipeline` 组合"已选视觉后端"与 `DebugRenderer`
- `RtpStreamer` 负责将 overlay 后的画面通过 RTP 推流输出（ffmpeg 子进程），供 VLC 等外部播放器接收
- `VideoShmProducer` 负责通过 POSIX 共享内存（`/laser_frame`）输出 BGR 原始帧，双缓冲 + 原子序号，供 UI 零拷贝读取
- `UdpSender` 负责通过 UDP 发送 TargetObservation 和 EKF 状态（检测结果、跟踪位置、速度等）
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
  - 已接入 `tool_guidance`：瞄准用 EKF 预测中心，测距用当前 bbox 尺度

### Guidance（引导管线）

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
  - 编排 depth → projection → kinematics → driver
  - 提供 `process_ekf_guided(ekf_center, candidate, io_depth)` 接口
  - EKF 中心瞄准 + bbox 测距，丢帧时深度保持
  - 标定模式：`calib_mode=true` 时锁死振镜角度，通过 `estimate_depth` + `project_to_camera` 输出靶 3D 坐标
  - 扫描模式：`scan_mode=rectangle` 时启动独立线程持续 raster 扫描，主线程仅更新中心

### Calibration Tools

- `tool_guidance` 标定模式
  - 配置 `calib_mode: true` + `config/calib_guidance.yaml`
  - WASD 实时微调振镜角度（步长 0.1°）
  - 空格键记录当前 `(θx, θy, P_c)` 到 `calib_records.csv`

- `tool_calib_solve`
  - 读取 `calib_records.csv`，坐标下降法优化旋转外参
  - 平移量固定（物理测量值），只优化 `r_x/y/z_deg`
  - 输出优化后的旋转角和残差

- `purple confirmed` 命中样本
  - `tool_guidance` 正常跟踪模式下，若最高置信候选类别为 `purple`
    且 `HitStateMachine` 边沿进入 `Confirmed`，自动追加一条
    `(theta_x, theta_y, P_c_x, P_c_y, P_c_z)` 到 `hit_calib_records.csv`
  - 文件格式与 `calib_records.csv` 完全一致，可直接复用 `tool_calib_solve`

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
