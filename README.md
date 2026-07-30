# laser_guidance

[![Docker Publish](https://github.com/Yukikaze2233/laser_guidance/actions/workflows/docker-publish.yml/badge.svg)](https://github.com/Yukikaze2233/laser_guidance/actions/workflows/docker-publish.yml)
[![ghcr.io](https://img.shields.io/badge/ghcr.io-yukikaze2233%2Flaser--guidance-blue)](https://github.com/Yukikaze2233/laser_guidance/pkgs/container/laser-guidance)

视觉引导激光振镜命中移动目标的实时系统。

## Features

- **多后端采集** — V4L2/UVC 与 Hik 工业相机，由 `CaptureDevice` 统一 dispatch
- **ONNX / TensorRT 推理** — 运行时切换，敌方颜色过滤，EKF 目标跟踪
- **几何引导** — 相机内参 + 固定机械平移 + SO(3) 旋转外参，将带深度目标点转换为振镜角 → FT4222H USB-to-SPI 振镜控制
- **Ceres 旋转标定** — 使用带来源和不确定度的深度记录、完整近场镜距模型和四元数流形优化旋转；平移固定为机械测量值
- **空中机器人反制进度** — `HitProgress` 按 2026 规则计算 0.1s 阶梯增量、5 次锁定与 1/2/3 难度阶段
- **硬件容错** — 相机或 FT4222 缺失时记录错误并继续运行；FT4222 作为 guidance 级运行时依赖，硬件恢复后自动重连
- **ZMQ / UDP telemetry** — UDP 维持二进制 telemetry；ZMQ 发布 `cmd_id=0x2003` 的 Laser JSON 到 `tcp://*:5555`
- **RTP 推流 + 录制** — h264_nvenc 编码，原始视频会话录制与离线抽帧导出
- **调试桥接** — RosBridge → Foxglove Studio / rviz2 可视化检测框、跟踪轨迹、瞄准点
- **FIFO 运行时控制** — `/tmp/laser_cmd` 接收 `stream on/off`、`record on/off`、`enemy red/blue` 等命令
- **独立标定入口** — `tool_guidance` 用于相机标定与命中记录，不经过竞赛 runtime

## Quick Start

### 日常开发（宿主机）

```bash
source /opt/ros/jazzy/setup.bash   # 仅 cmake 编译需要
build-laser                         # CMake 配置 + 编译
make preview                        # 预览模式
make stream                         # 推流模式
foxglove-laser                      # Foxglove 可视化（自动进容器）
```

### 部署（Docker）

```bash
docker pull ghcr.io/yukikaze2233/laser-guidance:latest

# 比赛模式（后台守护）
docker compose up -d

# 比赛 + ffplay 拉流
docker compose --profile stream up

# 进入容器 shell
laser shell
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
| `GuidanceSession` | FT4222H SPI 振镜控制 + 几何解算（固定平移、四元数旋转、镜距模型）+ 瞄准执行 |
| `RuntimeOutputs` | RTP 推流、SHM 共享内存、UDP/ZMQ telemetry、录制 |
| `RosBridge` | RuntimeSnapshot → ROS2 topic（MarkerArray / Marker / Float64） |
| `CompetitionRuntime` | main / preview profile，FIFO 命令入口 |
| `GuidanceOpsApp` | 独立标定应用（tool_guidance），不经过竞赛 runtime |

## Build

### 宿主机构建（推荐）

```bash
build-laser                 # CMake 配置 + 编译
clean-laser                 # 清理 build/
make preview                # 预览
make stream                 # 推流
foxglove-laser              # Foxglove bridge（自动进容器）
```

`.script/` 脚本首次运行时会自动创建 `.runtime-compat/lib/` 兼容软链接（处理 ROS2 tarball 安装的 soname 差异），无需手动操作。

### 容器内构建

```bash
docker-build-laser --push   # 构建并推送镜像
```

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
| Eigen3 | 四元数旋转、向量和坐标变换 |
| Ceres Solver | 固定平移下的相机-振镜 SO(3) 旋转标定 |

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
./build/tool_calib_solve      # 深度感知旋转外参解算
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
src/guidance/         几何解算（固定平移、四元数旋转、深度估计、振镜控制）
src/runtime/          运行时核心
src/bridges/          FIFO / RTP / SHM / UDP / ZMQ / ROS2
src/io/               硬件 I/O（FT4222H SPI）
tools/                可执行入口
tests/                自动化测试
config/               YAML 配置
models/               ONNX / TensorRT 模型
vendor/               第三方库（hikcamera SDK、FT4222H）
```
