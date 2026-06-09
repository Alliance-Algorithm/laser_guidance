# laser_guidance

`laser_guidance` 是一个激光视觉引导系统，当前阶段覆盖：

- `V4L2/UVC` 采集卡取图
- 原始视频会话录制与可选离线抽帧导出
- YAML 配置加载
- `CompetitionRuntime` / `PreviewRuntime` 统一运行时入口
- 调试 overlay 与回放样本
- 自动测试与工具入口
- ONNX / TensorRT GPU 推理，双后端运行时热切换
- FT4222H USB-to-SPI 振镜控制
- Direct voltage 视觉→电压多项式映射（Poly3，支持增量标定训练）
- 比赛模式统一守护进程 (`tool_competition`)
- 空中机器人被瞄准进度计算 (`HitProgress`，RoboMaster 2026 §5.6.3)
- EKF 跟踪（可运行时关闭，切换原始检测直驱）
- 敌方颜色过滤（red/blue/auto）

当前阶段**不包含**：

- ROS 控制总线 / `/gimbal/*` 接口
- 通用 `solver` / `planner`
- `fire_control`

## Runtime 架构

当前运行时已经从“每个 `tool_*.cpp` 自己拼主循环”收敛为“稳定模块 + runtime facade + bridge”；其中统一 runtime 当前覆盖 `tool_competition` / `tool_preview`，`tool_guidance` 仍走兼容 runtime：

- `CompetitionRuntime`
  - 比赛模式运行时，统一管理 capture / infer / EKF / guidance / RTP / SHM / UDP / recording
- `PreviewRuntime`
  - 预览模式运行时，复用同一套生命周期和异步推理骨架
- `RuntimeCommand`
  - 类型化控制面：`set_streaming` / `set_recording` / `set_enemy_color` / `set_backend` / `set_ekf` / `shutdown`
- `RuntimeSnapshot`
  - 统一导出当前运行状态、检测结果、track、aim 输出和 recording 路径
- `FifoControlServer`
  - 只负责 `string -> RuntimeCommand` 协议转换，FIFO 不再是核心接口
- `UdpTelemetryPublisher` / `ShmFramePublisher` / `RtpFramePublisher`
  - 只做 bridge 和数据搬运，不承载算法判断
- `GuidanceToolRuntime`
  - internal `tool_guidance` 兼容 runtime，独立管理异步推理、校准按键、CSV 记录与 purple hit 边沿记录

`tools/competition.cpp` 和 `tools/preview.cpp` 现在只负责：

- 解析配置路径
- 创建 runtime
- 绑定 FIFO bridge
- 等待退出

### 架构原则

- `tool_*` 不承载业务编排
  - 新功能优先进 runtime / guidance / bridge，而不是继续堆回工具入口
- runtime facade 负责生命周期
  - capture、infer、EKF、guidance、stream、record、shutdown 都由 runtime 统一管理
- bridge 负责协议，不负责判断
  - FIFO、UDP、RTP、SHM 只做协议适配和搬运，不做算法决策
- typed command 优先
  - 运行时控制先定义 `RuntimeCommand`，再决定是否暴露为 FIFO / CLI / 其他输入面
- 输出快照统一
  - 状态观测优先走 `RuntimeSnapshot`，避免每个入口各自拼状态结构
- `RuntimeSnapshot` / `TargetTrack` 必须保持值语义安全
  - public snapshot 不允许暴露依赖临时 batch 生命周期的裸指针
- 兼容路径逐步收缩
  - `GuidancePipeline`、`TargetObservation`、`rmcs_laser_guidance_core` 当前保留兼容，但新代码默认接 runtime facade 和分层 target

### 后续迁移规则

- 新增比赛/预览控制项时：
  - 先加 `RuntimeCommand` 和 `RuntimeSnapshot`
  - 再决定是否映射到 FIFO 文本协议
- 新增输出通道时：
  - 先建 bridge，再由 runtime 挂接
  - 不在算法主循环里直接写协议代码
- 新增 guidance 策略时：
  - 解算放 `AimSolver`
  - 硬件执行放 `GalvoExecutor`
  - 扫描/搜索放 `ScanController`
- `tool_guidance` 入口只保留参数解析与生命周期调用；其内部仍是独立兼容 runtime
- 只有兼容旧实现细节时，才继续触碰 `GuidancePipeline`

## Build

```bash
# 基础构建
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

启用可选特性：

```bash
# ONNX Runtime + TensorRT + FT4222 (推荐比赛配置)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DWITH_ONNXRUNTIME=ON \
  -DWITH_TENSORRT=ON \
  -DWITH_FT4222=ON

# 仅 ONNX Runtime
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DWITH_ONNXRUNTIME=ON

# 禁用 FT4222H
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DWITH_FT4222=OFF
```

## Tests

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Tools

构建后在 `build/` 下直接运行：

```bash
./build/tool_competition             # 比赛模式（守护进程，TensorRT+EKF+振镜+推流+录制）
./build/tool_preview                 # 本地预览（RTP推流+SHM+FIFO）
./build/tool_guidance                # 激光引导（模型+EKF+振镜，支持标定模式）
./build/tool_record                  # 原始视频会话录制
./build/tool_model_infer             # 模型推理
./build/tool_calibrate               # 相机标定
./build/tool_dac8568_smoke           # FT4222 -> DAC8568 硬件冒烟
./build/tool_galvo_smoke             # DAC8568 -> 振镜信号冒烟
./build/tool_calib_solve             # 外参旋转求解
./build/tool_export                  # 离线抽帧导出
./build/tool_transcode               # 已录视频转码
```

大部分工具接受可选的配置文件路径：

```bash
./build/tool_preview config/capture_red_20m.yaml
```

录制工具支持显式参数覆盖：

```bash
./build/tool_record \
  config/capture_red_20m.yaml \
  ./videos/my_session \
  30 \
  indoor_lab plain_wall 20m red
```

不显式传输出目录时，默认保存到 `videos/`。

模型推理覆盖模型路径：

```bash
./build/tool_model_infer \
  config/default.yaml \
  test_data/sample_images \
  models/my_model.onnx
```

离线抽帧导出：

```bash
./build/tool_export /path/to/session_dir /path/to/dataset_root train 200
```

转码已录 session：

```bash
./build/tool_transcode /path/to/videos/<session_id>
```

## Quick Scripts

```bash
make set-config      # 选择配置文件（交互式，选一次全局生效）
make scan-camera     # 扫描可用采集卡
make competition     # 比赛模式（守护进程 + RTP推流 + ffplay）
make stream          # RTP 推流（自动打开 ffplay 窗口）
make preview         # 本地预览（cv::imshow 窗口，不推流）
make stop            # 停止后台守护进程
make record          # 视频录制
```

守护进程运行时，通过 `/tmp/laser_cmd` FIFO 控制：

```bash
echo "stream on"       > /tmp/laser_cmd   # 推流开关
echo "record on"       > /tmp/laser_cmd   # 录制开关
echo "enemy red"       > /tmp/laser_cmd   # 敌方颜色过滤
echo "backend tensorrt" > /tmp/laser_cmd  # 推理后端切换
echo "ekf off"         > /tmp/laser_cmd   # EKF 直驱模式
echo "quit"            > /tmp/laser_cmd   # 优雅退出
```

FIFO 协议仍兼容旧命令词，但运行时内部已经改成 typed command，不再由业务主循环直接解析字符串。

## 项目结构

```
laser_guidance/
├── include/               # 公开 API
│   ├── config.hpp         # YAML 配置加载
│   ├── types.hpp          # Frame, TargetObservation
│   └── laser_guidance/    # 稳定 facade: support/runtime/bridges
├── src/
│   ├── core/              # 核心模块
│   ├── vision/            # 检测/推理模块
│   ├── capture/           # 视频采集 (V4L2)
│   ├── streaming/         # 网络推流 (RTP/UDP/SHM)
│   ├── tracking/          # EKF 跟踪
│   ├── guidance/          # AimSolver / GalvoExecutor / ScanController / GuidancePipeline(兼容)
│   ├── runtime/           # RuntimeBase / InferenceFacade / GuidanceToolRuntime / runtime facade
│   ├── bridges/           # FIFO / RTP / SHM / UDP bridge
│   └── io/                # 硬件 I/O (FT4222H 等)
├── tools/                 # 可执行工具
├── tests/                 # 自动化测试
│   ├── core/
│   ├── runtime/
│   ├── vision/
│   └── tracking/
├── config/                # YAML 配置文件
├── models/                # .onnx / .engine 模型文件
└── test_data/             # 样本回放资源
```

## Build Targets

当前 CMake 已拆为分层 target：

- `laser_guidance_support`
- `laser_guidance_vision`
- `laser_guidance_tracking`
- `laser_guidance_guidance`
- `laser_guidance_bridges`
- `laser_guidance_runtime`

兼容层 `rmcs_laser_guidance_core` 仍保留，用于平滑迁移现有工具和测试。

建议新增代码直接依赖分层 target，而不是继续把所有依赖都挂在兼容聚合 target 上。

## C++ API 示例

```cpp
#include "config.hpp"
#include "laser_guidance/runtime.hpp"

using namespace rmcs_laser_guidance;

auto config = load_config("config/default.yaml");
PreviewRuntime runtime(config);

if (auto started = runtime.start(); !started) {
    throw std::runtime_error(started.error());
}

auto snapshot = runtime.snapshot();
if (snapshot.detection.detected) {
    const auto center = snapshot.detection.selected_center;
    printf("target at (%.1f, %.1f)\n", center.x, center.y);
}

runtime.stop();
runtime.join();
```

FT4222H / DAC8568 最小写入示例：

```cpp
#include "io/ft4222_spi.hpp"

#include <array>

auto spi = Ft4222Spi::open(Ft4222Config{
    .sys_clock = Ft4222SysClock::k60MHz,
    .clock_div = Ft4222SpiDiv::kDiv64,   // SCK ≈ 937 kHz
    .cs_channel = 0                       // SS0O
});

if (spi) {
    std::array<uint8_t, 4> enable_ref{0x08, 0x00, 0x00, 0x01};
    spi->write(enable_ref.data(), static_cast<uint16_t>(enable_ref.size()));
}
```

## Notes

- 默认配置 `config/default.yaml`，采集配置推荐 `config/capture_red_20m.yaml`
- 采集卡设备路径通过 `v4l2.device_path` 配置；`make scan-camera` 可扫描可用设备
- 默认采集模式 `1920x1080 @ 60 FPS`，优先 `mjpeg` 编码
- 默认回放样本位于 `test_data/sample_images`
- 原始视频会话目录默认为 `videos/`
- `models/` 放置 `.onnx`、`.engine`、电压映射模型文件（`vision_voltage_poly_v*.yaml`、`vision_voltage_lut*.yaml`）
- `test_data/calib/` 放置标定 CSV（voltage_records, geometry_calib_records 等）
- 运行时 facade 通过 `CompetitionRuntime` / `PreviewRuntime` 暴露统一生命周期和控制接口
- `RuntimeCommand` 负责运行时热切换：推流、录制、敌方颜色、ONNX/TensorRT、EKF 开关、优雅退出
- `tool_competition`、`tool_preview` 不再承载主业务循环，只做 runtime 启动入口
- `tool_guidance` 入口已压薄，但内部仍保留兼容 runtime 主循环
- TensorRT 需 `-DWITH_TENSORRT=ON` 且预先生成 `.engine` 文件
- 模型后端支持 YOLO26 端到端推理，输出 `[1,300,6]`（3 class：purple/red/blue）
- 本仓库不负责本地训练；训练应在外部平台完成，仓库负责数据集生成和模型接入
- EKF 跟踪器 (`EkfTracker`) 为 standalone 模块，常加速度 6 维状态，支持 lookahead 超前预测
- Direct voltage 视觉→电压映射：`config/direct_voltage_run.yaml`（比赛配置）、`config/direct_voltage_calib.yaml`（标定采集）。训练脚本 `scripts/train_voltage_poly.py`，模型版本 `models/vision_voltage_poly_v*.yaml`
- 比赛模式 (`tool_competition`) 融合预览+引导+录制，守护进程启动，支持 RTP 推流（可配码率 `streaming.bitrate`）、视频内录（`record.output_root`）、FIFO 运行时控制
- 双推理后端：ONNX + TensorRT，启动时同时加载，运行时通过 FIFO 或配置 `inference.backend` 切换
- guidance 已按职责分出 `AimSolver`、`GalvoExecutor`、`ScanController`；`GuidancePipeline` 仅保留给 `GuidanceToolRuntime` 的兼容内核和旧实现细节
- 敌方颜色过滤：`inference.enemy_color` (red/blue/auto)，仅处理敌方颜色+紫色候选框
- EKF 可运行时关闭 (`ekf.enabled: false` 或 `ekf off`)，切换原始检测直驱振镜
- 被瞄准进度 (`HitProgress`)：规则手册 §5.6.3，3 阶段 P0 阈值，45s 锁定，overlay 进度条
- 训练数据链路推荐「先录原始视频会话，再直接上传外部平台」；离线抽帧为备用链路
- 录制输出 `raw.mp4 + session.yaml + notes.txt`，默认 H.264/avc1 编码
- RTP 推流：`make stream` 后台 daemon + ffplay 窗口，关闭即停。`streaming` 配置段控制，默认端口 5004
- `.script/` 提供便捷脚本：`set-config`、`scan-camera`、`preview`、`stream`、`stop`

## Docker

### 快速开始

```bash
# 1. 拉取镜像
docker pull yukikaze2233/laser-guidance:latest     # 统一镜像（4.6 GB，ONNX + TensorRT）

# 2. 启动比赛模式
docker compose up -d

# 3. 运行时控制（通过 FIFO）
echo "stream on"  > /tmp/laser_cmd        # 开启推流
echo "record on"  > /tmp/laser_cmd        # 开启录制
echo "enemy red"  > /tmp/laser_cmd        # 敌方颜色
echo "backend tensorrt" > /tmp/laser_cmd  # 切换推理后端
echo "quit"       > /tmp/laser_cmd        # 优雅退出

# 4. 停止
docker compose down
```

### 镜像说明

| 镜像 | 大小 | 内含 |
|---|---|---|
| `yukikaze2233/laser-guidance:latest` | 4.6 GB | ONNX Runtime 1.26.0 + TensorRT 10.13 / CUDA 13.0 / cuDNN 9.12 + OpenCV + ffmpeg + FT4222 |

> 一个镜像同时支持 ONNX Runtime 和 TensorRT 双后端，通过 `config/direct_voltage_run.yaml` 中的 `inference.backend` 或 FIFO `backend` 命令运行时切换。无需 GPU 环境也能正常运行 ONNX 模式。

### 构建（可选，已提供预构建镜像）

```bash
# 需要 NGC 访问权限（docker login nvcr.io）以提取 TensorRT/CUDA 库
docker build -t yukikaze2233/laser-guidance:latest . --target runtime

# 开发镜像（含 cmake / gdb）
docker build -t yukikaze2233/laser-guidance:latest . --target develop
```

### Compose Profiles

```bash
# 比赛模式（默认，headless，自动重启，GPU enabled）
docker compose up -d

# 比赛模式 + 宿主 ffplay 推流预览
docker compose --profile stream up

# 交互式开发环境（含 GUI 穿透 + OpenCode 复用）
docker compose run --rm dev

# 指定摄像头设备
LASER_CAMERA_DEVICE=/dev/video0 docker compose up -d
```

### 容器运行时绑定

| 资源 | 为什么 |
|---|---|
| `/dev:/dev` | 摄像头 V4L2、FT4222 USB 设备、GPU 设备 |
| `--privileged` | FT4222 D2XX 用户态驱动（libusb）需要 USB 控制权 |
| `--network=host` | RTP/UDP 推流使用宿主机网络栈 |
| `--ipc=host` | `/laser_frame` POSIX 共享内存 |
| `/tmp:/tmp` | FIFO `/tmp/laser_cmd`、SDP `/tmp/laser_guidance.sdp` |
| `--gpus all` | NVIDIA Container Toolkit GPU 透传 |
| X11/Wayland | 仅 `dev` profile，支持 `cv::imshow` |
| OpenCode 配置 | 仅 `dev` profile，复用宿主机 opencode 状态 |

### dev profile 与宿主机 OpenCode 复用

`dev` profile 会自动挂载宿主机的 OpenCode 配置目录，与 RMCS / librmcs 方式一致：

- `${HOME}/.config/opencode` → `/root/.config/opencode`
- `${HOME}/.local/share/opencode` → `/root/.local/share/opencode`
- `${HOME}/.agents/skills` → `/root/.agents/skills`（只读）

容器内直接使用宿主机已有的 OpenCode 配置与 skills，无需重新初始化。

### 推送到 Docker Hub

```bash
docker push yukikaze2233/laser-guidance:latest
```

## Docs

- `plan.md`
- `docs/architecture.md`
- `docs/hardware_ft4222_dac8568.md`
- `docs/dataset_collection.md`
- `docs/development.md`
