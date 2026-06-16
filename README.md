# laser_guidance

[![Docker Publish](https://github.com/Yukikaze2233/laser_guidance/actions/workflows/docker-publish.yml/badge.svg)](https://github.com/Yukikaze2233/laser_guidance/actions/workflows/docker-publish.yml)
[![ghcr.io](https://img.shields.io/badge/ghcr.io-yukikaze2233%2Flaser--guidance-blue)](https://github.com/Yukikaze2233/laser_guidance/pkgs/container/laser-guidance)

视觉引导激光振镜命中移动目标的实时系统。

## Features

- **多后端采集** — V4L2/UVC 与 Hik 工业相机，由 `CaptureDevice` 统一 dispatch
- **ONNX / TensorRT 推理** — 运行时切换，敌方颜色过滤，EKF 目标跟踪
- **几何引导** — 相机标定 + 外参解算 → FT4222H USB-to-SPI 振镜控制
- **硬件容错** — 相机或 FT4222 缺失时记录错误并继续运行，硬件恢复后自动重连
- **ZMQ / UDP telemetry** — 二进制协议（0x47 0x4C magic），双通道并行发布
- **RTP 推流 + 录制** — h264_nvenc 编码，原始视频会话录制与离线抽帧导出
- **调试桥接** — RosBridge → Foxglove Studio / rviz2 可视化检测框、跟踪轨迹、瞄准点
- **FIFO 运行时控制** — `/tmp/laser_cmd` 接收 `stream on/off`、`record on/off`、`enemy red/blue` 等命令
- **独立标定入口** — `tool_guidance` 用于相机标定与命中记录，不经过竞赛 runtime

## Quick Start

```bash
docker pull ghcr.io/yukikaze2233/laser-guidance:latest

# 比赛模式（后台守护）
docker compose up -d

# 比赛 + ffplay 拉流
docker compose --profile stream up

# 交互 shell（开发/调试）
docker compose run --rm shell

# Foxglove 可视化 → 浏览器打开 ws://localhost:8765
docker compose run --rm shell foxglove-laser
```

## Image Tags

| Tag | 说明 |
|------|------|
| `latest` | main 分支最新构建 |
| `sha-XXXXXX` | 指定 commit |
| `vX.Y.Z` | 发行版本 |

push main 或打 tag 时由 GitHub Actions 自动构建推送至 ghcr.io。

## Architecture

### 视觉链路

```
CaptureDevice → PerceptionRunner → TargetTrack → GuidanceSession → RuntimeOutputs
                                                       │
                                                RosBridge → Foxglove/rviz2
```

### 设计原则

**后端统一调度。** `CaptureDevice` 内部按配置选择 `v4l2` 或 `hikcamera` backend，外部只接触 `CaptureFormat`，不感知硬件细节。

**硬件容错。** 相机或 FT4222H 初始化失败只记录错误，不退出进程。相机断开后自动进入重连循环（1s 间隔），恢复后重新打开流、重建 GuidanceSession。

**依赖隐藏。** 重型依赖（ROS2、hikcamera SDK、ZMQ）通过 PIMPL 隔离在 `.cpp` 内部，公共头文件不泄露第三方类型，减少增量编译时间。

**Typed 控制面。** `RuntimeCommand` / `RuntimeSnapshot` 是运行时唯一的 typed 控制面和观测面。新增能力优先扩 typed 接口，再考虑映射到 FIFO 协议。

### 组件

| 组件 | 职责 |
|------|------|
| `CaptureDevice` | 采集后端选择：v4l2 / hikcamera |
| `PerceptionRunner` | ONNX / TensorRT 推理 + 敌方颜色过滤 + EKF 跟踪 |
| `GuidanceSession` | FT4222H SPI 振镜控制 + 几何解算 + 瞄准执行 |
| `RuntimeOutputs` | RTP 推流、SHM 共享内存、UDP/ZMQ telemetry、录制 |
| `RosBridge` | RuntimeSnapshot → ROS2 topic（MarkerArray / Marker / Float64） |
| `CompetitionRuntime` | main / preview profile，FIFO 命令入口 |
| `GuidanceOpsApp` | 独立标定应用（tool_guidance），不经过竞赛 runtime |

## Build

### 容器内构建（推荐）

```bash
build-laser                 # CMake 配置 + 编译
clean-laser                 # 清理 build/
docker-build-laser --push   # 构建并推送镜像
foxglove-laser              # 启动 Foxglove bridge
```

容器内任意路径可用（`.script/` 已加入 `$PATH`）。

### 本地编译

所有依赖均为强制：

| 依赖 | 说明 |
|------|------|
| ONNX Runtime | `ONNXRUNTIME_ROOT` 或系统路径 |
| TensorRT + CUDA | `libnvinfer`、`libnvonnxparser`、`libnvinfer_plugin` |
| FT4222H | `libft4222.so` → `vendor/ft4222/lib/` 或系统库路径 |
| Hik MVS SDK | submodule 自带 vendored SDK，或 `MVS_SDK_ROOT` |
| ROS2 Jazzy | rclcpp、visualization_msgs、std_msgs |
| ZMQ | libzmq3-dev、cppzmq-dev |

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

### CMake 变量

| 变量 | 说明 |
|------|------|
| `ONNXRUNTIME_ROOT` | ONNX Runtime 安装路径 |
| `CUDA_LIBRARY` | `libcudart.so` 路径 |
| `CUDA_RT_LIBRARY` | `libcuda.so` stub 路径 |
| `HIKCAMERA_SDK_MODE` | `AUTO` / `vendor` / `system` |
| `MVS_SDK_ROOT` | system 模式下的 MVS SDK 路径 |

## Tools

```bash
./build/tool_competition      # 比赛模式
./build/tool_preview          # 预览模式
./build/tool_guidance         # 标定工具
./build/tool_record           # 录制工具
./build/tool_calibrate        # 相机标定
./build/tool_model_infer      # 模型推理测试
./build/tool_calib_solve      # 外参解算
./build/tool_export           # 录像抽帧导出
./build/tool_transcode        # 视频转码
```

## Config

| 文件 | 用途 |
|------|------|
| `config/default.yaml` | 默认运行配置（v4l2） |
| `config/capture_hik.yaml` | Hik 工业相机示例 |
| `config/geometry_run.yaml` | geometry 命令模型比赛配置（v4l2） |
| `config/hik_competition.yaml` | Hik 相机 + geometry 比赛配置 |

## Repo Layout

```text
include/              public API
src/capture/          采集后端（v4l2 / hikcamera）
src/vision/           推理与模型适配
src/guidance/         几何解算与振镜控制
src/runtime/          运行时核心
src/bridges/          FIFO / RTP / SHM / UDP / ZMQ / ROS2
src/io/               硬件 I/O（FT4222H SPI）
tools/                可执行入口
tests/                自动化测试
config/               YAML 配置
models/               ONNX / TensorRT 模型
vendor/               第三方库（hikcamera SDK、FT4222H）
```
