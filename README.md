# laser_guidance

[![Docker Publish](https://github.com/Yukikaze2233/laser_guidance/actions/workflows/docker-publish.yml/badge.svg)](https://github.com/Yukikaze2233/laser_guidance/actions/workflows/docker-publish.yml)
[![ghcr.io](https://img.shields.io/badge/ghcr.io-yukikaze2233%2Flaser--guidance-blue)](https://github.com/Yukikaze2233/laser_guidance/pkgs/container/laser-guidance)

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

## Quick Start

```bash
docker pull ghcr.io/yukikaze2233/laser-guidance:latest

# 比赛模式
docker compose up -d

# 比赛 + ffplay 推流
docker compose --profile stream up

# 交互 shell
docker compose run --rm shell
```

## 镜像标签

| 标签 | 说明 |
|------|------|
| `latest` | main 分支最新构建 |
| `sha-XXXXXX` | 指定 commit 构建 |
| `vX.Y.Z` | 发行版本 |

由 GitHub Actions 在 push main 或打 tag 时自动构建推送。

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
  - 管理 RTP、共享内存、UDP/ZMQ telemetry、录制。
- `tool_guidance`
  - 独立的 `GuidanceOpsApp`，用于校准和记录，不再走 `CompetitionRuntime`。

`tool_competition` 和 `tool_preview` 通过 FIFO `/tmp/laser_cmd` 接收运行时命令；`tool_guidance` 不使用这条链路。

## Build

### Docker（推荐）

ghcr.io: [`yukikaze2233/laser-guidance`](https://github.com/Yukikaze2233/laser_guidance/pkgs/container/laser-guidance)

```bash
docker pull ghcr.io/yukikaze2233/laser-guidance:latest
```

启动：

```bash
docker compose up -d                        # 比赛模式
docker compose --profile stream up          # 比赛 + ffplay
docker compose run --rm shell               # 交互 shell
```

Foxglove 可视化（连接 `ws://localhost:8765`）：

```bash
docker compose run --rm shell foxglove-laser
```

### 宿主机依赖

所有后端均为强制依赖，本地编译需要以下 SDK：

| 依赖 | 说明 |
|------|------|
| **ONNX Runtime** | 设置 `ONNXRUNTIME_ROOT` 环境变量或安装到系统路径 |
| **TensorRT + CUDA** | CUDA Toolkit、TensorRT SDK（`libnvinfer`、`libnvonnxparser`、`libnvinfer_plugin`） |
| **FT4222H** | `libft4222.so` 放 `vendor/ft4222/lib/`、`/opt/libft4222/` 或系统库路径 |
| **Hik MVS SDK** | submodule 自带 vendor SDK，或 `MVS_SDK_ROOT` 指向系统安装路径 |
| **ROS2 Jazzy** | rclcpp、visualization_msgs、std_msgs |
| **ZMQ** | libzmq3-dev、cppzmq-dev |

Docker 镜像已内置所有依赖，推荐使用 Docker 构建和运行。

### 本地编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Docker 构建：

```bash
docker build -t laser-guidance .
```

### CMake 变量

| 变量 | 说明 |
|------|------|
| `ONNXRUNTIME_ROOT` | ONNX Runtime 安装路径 |
| `CUDA_LIBRARY` | `libcudart.so` 路径（默认搜 `/opt/cuda/lib64`） |
| `CUDA_RT_LIBRARY` | `libcuda.so` stub 路径（默认搜 `/opt/cuda/lib64/stubs`） |
| `HIKCAMERA_SDK_MODE` | `AUTO`（默认）/ `vendor` / `system` |
| `MVS_SDK_ROOT` | system 模式下的 MVS SDK 路径 |

### 构建脚本

容器内任意路径可调用：

```bash
build-laser                 # CMake 配置 + 编译
clean-laser                 # 清理 build/
docker-build-laser          # Docker 镜像构建（--push 推送）
foxglove-laser              # 启动 Foxglove WebSocket 桥接
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

- `config/default.yaml` — 默认运行配置
- `config/capture_hik.yaml` — Hik 工业相机示例配置
- `config/geometry_run.yaml` — 比赛/运行配置示例
- `config/hik_competition.yaml` — Hik 相机比赛配置

## Repo Layout

```text
include/              public API
src/capture/          capture backend
src/vision/           inference and adapters
src/guidance/         solver and galvo control
src/runtime/          runtime core
src/bridges/          FIFO / RTP / SHM / UDP / ZMQ bridges
tools/                entrypoints
tests/                automated tests
config/               yaml configs
models/               onnx / engine files
test_data/            sample data
```
