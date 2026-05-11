# laser_guidance

`laser_guidance` 是一个最小激光视觉引导骨架，当前阶段覆盖：

- `V4L2/UVC` 采集卡取图
- 原始视频会话录制与可选离线抽帧导出
- YAML 配置加载
- `Pipeline` 统一视觉入口与可切换后端骨架
- 调试 overlay 与回放样本
- 自动测试与工具入口
- ONNX/TensorRT 模型推理
- FT4222H USB-to-SPI 振镜控制

当前阶段**不包含**：

- 控制链路闭环
- `/gimbal/*` 接口
- 相机与激光振镜的坐标系转换

## Build

```bash
# 基础构建
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

启用可选特性：

```bash
# ONNX Runtime 模型推理
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DRMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime

# TensorRT GPU 推理
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DRMCS_LASER_GUIDANCE_WITH_TENSORRT=ON

# FT4222H 振镜控制（默认开启，需 libft4222.so 在项目根目录）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DRMCS_LASER_GUIDANCE_WITH_FT4222=OFF   # 禁用
```

## Tests

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Tools

构建后在 `build/` 下直接运行：

```bash
./build/tool_preview                  # 本地预览
./build/tool_capture                  # 录帧 (PNG + manifest)
./build/tool_record                   # 原始视频会话录制
./build/tool_replay                   # 回放预览
./build/tool_model_infer              # 模型推理
./build/tool_detector_benchmark       # 检测 benchmark
./build/tool_calibrate                # 相机标定
./build/tool_dac8568_smoke            # FT4222 -> DAC8568 硬件冒烟
./build/tool_galvo_smoke              # DAC8568 -> 振镜信号冒烟
./build/tool_guidance                 # 激光引导（模型+EKF+振镜）
./build/tool_calib_solve              # 外参旋转求解（从 calib_records.csv）
./build/tool_export                   # 离线抽帧导出
./build/tool_transcode                # 已录视频转码
./build/tool_smoke                    # 离线冒烟
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
make preview         # 本地预览（cv::imshow 窗口，不推流）
make stream          # RTP 推流（自动打开 ffplay 窗口，关掉即退出）
make stop            # 停止后台守护进程
```

守护进程运行时，`make stream` 秒开推流（通过 `/tmp/laser_cmd` FIFO 发命令，无需重启）。外部程序也可通过 FIFO 控制：

```bash
echo "stream on"  > /tmp/laser_cmd
echo "stream off" > /tmp/laser_cmd
echo "quit"       > /tmp/laser_cmd
```

## 项目结构

```
laser_guidance/
├── include/               # 公开 API
│   ├── config.hpp         # YAML 配置加载
│   ├── types.hpp          # Frame, TargetObservation
│   ├── pipeline.hpp       # 统一视觉入口
│   └── laser_guidance.hpp
├── src/
│   ├── core/              # 核心模块
│   ├── vision/            # 检测/推理模块
│   ├── capture/           # 视频采集 (V4L2)
│   ├── streaming/         # 网络推流 (RTP/UDP/SHM)
│   ├── tracking/          # EKF 跟踪
│   └── io/                # 硬件 I/O (FT4222H 等)
├── tools/                 # 可执行工具
├── tests/                 # 自动化测试
│   ├── core/
│   ├── vision/
│   └── tracking/
├── config/                # YAML 配置文件
├── models/                # .onnx / .engine 模型文件
└── test_data/             # 样本回放资源
```

## C++ API 示例

```cpp
#include "config.hpp"
#include "pipeline.hpp"
#include "types.hpp"

#include "core/frame_format.hpp"
#include "capture/v4l2_capture.hpp"

using namespace rmcs_laser_guidance;

// 加载配置
auto config = load_config("config/default.yaml");

// 打开相机
V4l2Capture capture(config.v4l2);
auto negotiated = capture.open();
if (!negotiated) { /* 处理错误 */ }

// 创建 Pipeline
Pipeline pipeline(config);

// 主循环
while (true) {
    auto frame = capture.read_frame();
    if (!frame) continue;

    auto obs = pipeline.process(*frame);
    if (obs && obs->detected) {
        // 检测到目标
        printf("target at (%.1f, %.1f)\n", obs->center.x, obs->center.y);
    }
}
```

FT4222H 振镜控制：

```cpp
#include "io/ft4222_spi.hpp"

auto spi = Ft4222Spi::open(Ft4222Config{
    .sys_clock = Ft4222SysClock::k60MHz,
    .clock_div = Ft4222SpiDiv::kDiv64,   // SCK ≈ 937 kHz
    .cs_channel = 0                       // SS0O
});

if (spi) {
    uint8_t dac_cmd[2] = {0x30, 0xFF};   // 12-bit DAC value
    spi->write(dac_cmd, 2);
}
```

## Notes

- 默认配置 `config/default.yaml`，采集配置推荐 `config/capture_red_20m.yaml`
- 采集卡设备路径通过 `v4l2.device_path` 配置；`make scan-camera` 可扫描可用设备
- 默认采集模式 `1920x1080 @ 60 FPS`，优先 `mjpeg` 编码
- 默认回放样本位于 `test_data/sample_images`
- 原始视频会话目录默认为 `videos/`
- `models/` 放置 `.onnx`、`.engine` 模型文件
- `Pipeline` 通过 `inference.backend` 在 `bright_spot` / `model`(ONNX) / `tensorrt`(GPU) 间切换
- TensorRT 需 `-DRMCS_LASER_GUIDANCE_WITH_TENSORRT=ON` 且预先生成 `.engine` 文件
- 模型后端支持 YOLO26 端到端推理，输出 `[1,300,6]`（3 class：purple/red/blue）
- 本仓库不负责本地训练；训练应在外部平台完成，仓库负责数据集生成和模型接入
- EKF 跟踪器 (`EkfTracker`) 为 standalone 模块，常加速度 6 维状态
- 训练数据链路推荐「先录原始视频会话，再直接上传外部平台」；离线抽帧为备用链路
- 录制输出 `raw.mp4 + session.yaml + notes.txt`，默认 H.264/avc1 编码
- RTP 推流：`make stream` 后台 daemon + ffplay 窗口，关闭即停。`streaming` 配置段控制，默认端口 5004
- `.script/` 提供便捷脚本：`set-config`、`scan-camera`、`preview`、`stream`、`stop`

## Docs

- `plan.md`
- `docs/architecture.md`
- `docs/hardware_ft4222_dac8568.md`
- `docs/dataset_collection.md`
- `docs/development.md`
- `docs/future_rmcs_integration.md`
