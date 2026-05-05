# laser_guidance

`laser_guidance` 是一个最小激光视觉引导骨架，当前阶段只覆盖：

- `V4L2/UVC` 采集卡取图
- 原始视频会话录制与可选离线抽帧导出
- YAML 配置加载
- `Pipeline` 统一视觉入口与可切换后端骨架
- 最小亮点检测 `Detector`
- 红色目标 ROI 精修骨架
- 调试 overlay 与回放样本
- 自动测试与人工运行示例

当前阶段**不包含**：

- RMCS 控制接入
- `/gimbal/*` 接口
- `HardSyncSnapshot`
- 发弹时机或 `fire_control`

## Build

仓库根目录直接构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

配套测试：

```bash
ctest --test-dir build --output-on-failure
```

如果你在带 Jazzy 的 ROS 工作区里，也可以继续使用：

```bash
build-rmcs --packages-select rmcs_laser_guidance
```

或：

```bash
cd rmcs_ws
colcon build --packages-select rmcs_laser_guidance --symlink-install --merge-install
```

说明：

- 当机器上没有 `/opt/ros/jazzy` 和 `ament_cmake` 时，ROS/`colcon` 路径不可用，但根目录直接 `cmake` 仍应可用。
- 当机器上有完整 Jazzy 环境时，两种构建方式都支持。
- 如果要启用 ONNX Runtime 骨架，额外传：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DRMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON \
  -DONNXRUNTIME_ROOT=/abs/path/to/onnxruntime
```

## Auto Tests

自动验证目标位于 `tests/*_test.cpp`，可通过：

```bash
ctest --output-on-failure
```

在对应构建目录中执行全部自动测试。Standalone 构建对应 `build/`，ROS 工作区构建对应 `build/rmcs_laser_guidance/`。

## Quick Scripts

```bash
make set-config      # 选择配置文件（交互式，选一次全局生效）
make scan-camera     # 扫描可用采集卡
make preview         # 本地预览（cv::imshow 窗口，不推流）
make stream          # RTP 推流（自动打开 ffplay 窗口，关掉即退出）
make stop            # 停止后台守护进程
```

`make stream` 无需手动开 ffplay，自动后台启动守护进程并前台显示推流画面。

守护进程运行时，`make stream` 秒开推流（通过 `/tmp/laser_cmd` FIFO 发命令，无需重启）。

外部程序也可控制：
```bash
echo "stream on"  > /tmp/laser_cmd
echo "stream off" > /tmp/laser_cmd
echo "quit"       > /tmp/laser_cmd
```

## Examples

离线冒烟：

```bash
ros2 run rmcs_laser_guidance example_offline_smoke
```

V4L2 预览：

```bash
ros2 run rmcs_laser_guidance example_v4l2_preview
```

也可以显式传配置路径：

```bash
ros2 run rmcs_laser_guidance example_v4l2_preview /abs/path/to/default.yaml
```

V4L2 采图录帧：

```bash
ros2 run rmcs_laser_guidance example_v4l2_capture
```

V4L2 原始视频会话录制：

```bash
ros2 run rmcs_laser_guidance example_v4l2_record_session
```

不显式传输出目录时，默认保存到仓库根目录下的 `videos/`。
录制入口会在运行时强制把 `v4l2.pixel_format` 覆盖为 `yuyv`；`preview` 仍按配置文件读取。
`capture_red_20m.yaml` 现在还包含 `record.output_root`、`record.duration_seconds` 和场景标签，所以只传配置文件就能录制。

也可以显式传录制根目录和时长：

```bash
ros2 run rmcs_laser_guidance example_v4l2_record_session \
  /abs/path/to/config/capture_red_20m.yaml \
  /abs/path/to/session_root \
  30 \
  indoor_lab \
  plain_wall \
  20m \
  red
```

回放预览：

```bash
ros2 run rmcs_laser_guidance example_replay_preview
```

检测 benchmark：

```bash
ros2 run rmcs_laser_guidance example_detector_benchmark
```

模型推理占位入口：

```bash
ros2 run rmcs_laser_guidance example_model_infer
```

也可以显式覆盖模型路径：

```bash
ros2 run rmcs_laser_guidance example_model_infer \
  /abs/path/to/default.yaml \
  /abs/path/to/replay_dir \
  /abs/path/to/model.onnx
```

把已录制 session 的 `raw.mp4` 原地转为 `H.264/avc1`：

```bash
ros2 run rmcs_laser_guidance example_transcode_recorded_session \
  /abs/path/to/videos/<session_id>
```

离线抽帧生成待标注数据集（可选备用）：

```bash
ros2 run rmcs_laser_guidance example_export_training_frames \
  /abs/path/to/session_dir \
  /abs/path/to/dataset_root \
  train \
  200
```

## Notes

- 默认配置文件位于 `config/default.yaml`
- 推荐视频采集配置位于 `config/capture_red_20m.yaml`
  默认使用 UGREEN 采集卡直读节点 `/dev/video2`；如果你的机器编号不同，修改 `v4l2.device_path`
  录制参数也可放在 `record.*` 中；现有 CLI 位置参数仍可覆盖这些值
- 默认样本回放位于 `test_data/sample_images`
- 默认原始视频会话目录位于仓库根目录 `videos/`
- `models/` 用于放置 `.onnx`、`.pt`、`.engine` 模型文件
- public 头文件平铺在 `include/`，实现细节头收束在 `src/internal/`
- 当前默认 live 输入为 UGREEN 采集卡直读节点 `v4l2.device_path=/dev/video2`
- 当前默认 live 模式为 `1920x1080 @ 60 FPS`，优先 `mjpeg`
- RTP 视频推流通过 `streaming` 配置段启用，基于系统 ffmpeg（零 ROS 依赖），播放端用 VLC 打开 SDP
- EKF 跟踪器（`EkfTracker`）已实现为 standalone 模块，待接入主链路
- 训练数据链路当前推荐“先录原始视频会话，再直接上传外部平台”；离线抽帧只作为本地待标注备用链路
- `raw.mp4 + session.yaml + notes.txt` 用于保留完整采集会话，并对接外部标注/训练平台
  `.mp4` 现在默认写为 `H.264/avc1`
- 抽帧导出会生成 `images/train|val|test` 和按会话区分的 `export_manifest.csv`
- 当前检测逻辑仍然是极简亮点检测实现，用于把工程链路跑通
- `Pipeline` 通过 `inference.backend` 在 `bright_spot` 和 `model` 占位后端间切换
- `inference.model_path` 指向模型文件；启用 ONNX Runtime 需 `-DRMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON -DONNXRUNTIME_ROOT=/usr`
- `model` 后端当前已具备 ONNX Runtime + YOLO26 端到端推理能力，输出契约 `[1, 300, 6]` 已验证通过
- 当前优先支持单类检测 ONNX 输出；契约不匹配时报告输入输出元数据和失败原因
- 本仓库当前不负责本地训练；训练应在外部平台完成，仓库只负责数据集生成和模型接入
- 后续如果要接 RMCS，再单独增加 bridge 和控制接口
- `.script/` 提供便捷脚本：`set-config.sh`（选择配置）、`scan-camera.sh`（扫描采集卡）、`preview.sh`（本地预览）、`stream.sh`（RTP 推流）、`record.sh`（录制）

## Docs

- `plan.md`
- `docs/README.md`
- `docs/architecture.md`
- `docs/dataset_collection.md`
- `docs/development.md`
- `docs/future_rmcs_integration.md`
