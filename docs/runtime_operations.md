# Runtime Operations

## 运行约定

- `CompetitionRuntime.start()` 仍然是同步 readiness barrier。
- `main` / `preview` 共用同一套 `ControlLoop`，只是在 profile 上区分输出能力。
- `tool_guidance` 使用独立 `GuidanceOpsApp`。
- 运行时 backend 只在已构建可用的后端之间切换。

## Runtime Config

```yaml
runtime:
  max_input_age_ms: 25
  max_observation_age_ms: 35
  max_infer_fps: 60
  warmup_frames: 30
  engine_path: models/target_fp16_1x3x640x640.engine
  hit_confirm_frames: 3
  hit_release_frames: 5
  debug_enabled: false
  debug_max_fps: 30
  record_enabled: false
  record_queue_size: 16
```

说明：

- `record_enabled` 只在 `main` profile 下生效。
- `streaming`、`recording`、`enemy`、`backend`、`ekf` 都通过 `RuntimeCommand` 动态控制。
- `guidance.depth_source`、`guidance.lidar_*`、`ws30` 已删除。
- `HitProgress` 使用 RoboMaster 2026 空中机器人反制规则：未照射按 `0.5/s` 衰减，连续照射每 `0.1s` 增加 `0.6*n`，最多 5 次锁定。
- 第三阶段（难度 3，第 4/5 次锁定前）lit→unlit 相机切换经 `hik.profile_switch_delay_s`（默认 5s）去抖，防止计数抖动来回切参数；`game_progress==4` 每局开启时立即回 lit。

```yaml
referee:
  enabled: true
  zmq_address: "tcp://127.0.0.1:5561"
  match_duration_s: 420
  signal_timeout_s: 5
```

说明：

- `enabled: false` 或 ZMQ 无人发布时行为与旧版完全一致（始终引导、HitProgress 照常累计）。
- `zmq_address` 为 radar-egui 转发裁判系统的 SUB 地址；`RefereeLink` 只解析 `0x0001`（game_status）与 `0x020C`（radar_mark_data）。
- `match_duration_s` 是断流兜底时长（本地时钟计时）：`game_progress==4` 进入比赛窗口；有实时信号时窗口以裁判 `stage_remain_time` 为权威（技术暂停时裁判停表，本地墙钟不适用），`stage_remain_time==0` 或收到 `==5` 退出；断流（超过 `signal_timeout_s` 无合法消息）时退化为本地 `match_duration_s` 兜底，兜底超时后须收到 `==5` 才允许下一局 re-arm。
- `signal_timeout_s` 是断流看门狗阈值：超过该秒数未收到合法消息时 overlay 显示 `[STALE]` 告警；引导永不门控，断流只告警、不改变任何执行。
- 进入比赛窗口时 `HitProgress` 自动 `reset()`（防赛前紫色污染）；`0x020C` 官方反制置位边沿会校核本地锁定次数。
- 比赛窗口内以**官方反制成功次数**（0x020C 上升沿计数，每局 `match_started` 归零、上限 5）权威同步阶段（难度/P0/锁定），官方边沿不被本地锁定期门控；离线或窗口外退回本地紫色累计（`HitProgress::sync_official_counter`）。

## Build

```bash
cmake -S . -B build/default -DCMAKE_BUILD_TYPE=Release
cmake --build build/default --parallel
ctest --test-dir build/default --output-on-failure
```

Hik 子模块默认 `HIKCAMERA_SDK_MODE=AUTO`，优先使用 vendored SDK，缺失时回退系统 MVS。可显式 `HIKCAMERA_SDK_MODE=system|vendor`，系统模式下用 `-DMVS_SDK_ROOT` 指定路径。

MVS SDK 的 U3V 控制传输不 claim 接口会触发内核 USB reset（dmesg `did not claim interface N before use`，相机反复中断）。`.script/host-runtime-env.sh` 会向 daemon 进程注入 `LD_PRELOAD=build/liblibusb_claim_shim.so`（`src/io/libusb_claim_shim.cpp`）自动补 claim；手动裸跑 `./build/tool_competition` 时不注入。

真机验证结论（2026-08-04，MV-CS050-10UC / MV-CS200-10UC）：

- 每次 MVS SDK open 必触发一次内核 reset（10/10 轮压力测试确定触发），SDK 自愈、功能无损；两台相机、SDK 4.6.x/4.8.x 行为一致。
- dmesg 时序为 `reset` → `LPM disabled` → `did not claim`：reset 会清空 usbfs 的 claim 状态，`did not claim` 是 reset 的**症状**而非原因；纯 open/close 设备 fd 不触发任何事件。
- 现场反复 reset 均锚定在应用 open 时刻（daemon 重启 / 雷达驱动启动 / MVS 示例），无运行中自发 reset 证据；比赛单次启动只 reset 一次，后续稳定。
- 因此该现象按 SDK 固有行为接受，无需修复；shim 的价值是消除正常运行期的未 claim 告警。

## Operational Notes

- `ControlLoop` 负责 capture、perception、guidance、overlay、outputs 和 snapshot。
- `RuntimeOutputs` 负责 RTP、SHM、UDP telemetry、ZMQ Laser JSON (`cmd_id=0x2003`)、recording。
- `GuidanceSession` 负责 FT4222、`AimSolver`、`GalvoExecutor`、`ScanController`。
- FT4222 为主入口工具级运行时依赖；缺库或缺板卡时只影响 guidance，主流程继续。
- `tool_guidance` 的校准记录和 hit edge 记录都在独立 app 内完成。

### 旋转外参标定

1. 相机必须使用与 `camera_calib.yaml` 相同的分辨率、ROI 和翻转方式。
2. 确认配置中的 `t_x_mm/t_y_mm/t_z_mm` 是机械测量值；标定期间 angle offset 按零处理。
3. 使用 `tool_guidance` 采集覆盖视场和多个距离的记录，默认写入 `test_data/calib/rotation_calib_records.csv`。新文件格式为：

```text
theta_x_deg,theta_y_deg,pixel_x,pixel_y,depth_mm,depth_source,depth_sigma_mm
```

4. 自动框深度使用 `bbox`；手持测距仪的斜距使用 `rangefinder`，并填写包含手持起点误差的标准差。
5. 求解命令示例：

```bash
./build/tool_calib_solve records.csv config/calib.yaml config/camera_calib.yaml \
  --image-width 2448 --image-height 2048
```

6. 工具固定平移和镜距，只优化旋转；四列无深度和历史五列 CSV 会被拒绝。
7. 用未参与拟合的点验证 RMS、P95 和最大角误差后，再复制高精度 `r_x/r_y/r_z`。
8. 锁定旋转后，使用独立中心靶从零开始调整 `angle_offset_x/y_deg`。

最近一次有效标定（2026-08-02，485 条记录，残差 rms=0.41° p95=0.85° max=0.91°）：

```yaml
# config/hik.yaml guidance:
t_x_mm: 85.5
t_y_mm: 15.0
t_z_mm: 20.0
r_x_deg: -0.750758059984
r_y_deg: -0.477254806404
r_z_deg: -0.277933218604
mirror_separation_mm: 15.0
angle_offset_x_deg: -0.45   # 中心靶微调，待与新外参联调后归零
angle_offset_y_deg: -0.05
```

### 推理后端降级
- `PerceptionRunner::initialize_backends()` 按"先首选择后降级"策略初始化；ONNX/TensorRT 不再同时无条件构造。
- `PerceptionRunner::degraded()` 返回 true 时（无可用后端或推理未启用），主循环跳过推理，不退出进程。
- TensorRT 引擎加载前检查 CUDA 设备可用性；无 CUDA 设备时跳过 TensorRT 初始化。

### RTP 推流
- encoder 在无 CUDA 设备时自动从 `h264_nvenc` 回退到 `libx264`。
- `RtpStreamer::stop()` 为幂等调用。

### ROS2 桥接
- `RosBridge` 构造时自行调用 `rclcpp::init()` 若尚未初始化。
- 桥接初始化失败时主流程继续运行，不退出。

### 日志
- 运行时日志由启动脚本重定向 `stderr` 写入仓库根目录下的时间戳目录：`logs/<timestamp>/laser_daemon.log`（stream）、`logs/<timestamp>/laser_competition.log`（competition）。

## Verification

- backend 切换只发生在可用后端之间。
- `main` profile 允许录制，`preview` profile 不允许录制。
- FIFO 多行、半行、非法命令后可以恢复。
- `RuntimeSnapshot` 保持值语义安全。
- `tool_guidance` 能写入七列深度标定 CSV，`tool_calib_solve` 能完成固定平移的 SO(3) 旋转优化。

### 裁判信号本地验证（tool_referee_sim）

```bash
# 终端 1：本地 mock 裁判（默认完整流程：准备→自检→倒计时→比赛→结算，比赛 420s）
./build/tool_referee_sim --help    # 查看全部参数
./build/tool_referee_sim --prep-s 10 --selfcheck-s 3 --countdown-s 2 --match-s 60
                                    # 缩短各阶段时长便于快速进入比赛窗口

# 终端 2：跑 competition，观察 overlay REF 行
./build/tool_competition
```

overlay REF 行含义（引导永不门控）：

- `REF: no signal (local calc)` — 从未收到合法消息，纯本地计算；
- `REF: pg=1 t=-1s` — 有信号但非比赛窗口（仅显示，引导与 HitProgress 不受影响）；
- `REF: pg=4 t=12s` — 比赛窗口内，HitProgress 已按 match_started 重置并累计；
- `[STALE]` — 断流超过 `signal_timeout_s`（只告警，续导不熄灯）；
- `[CT]` — `0x020C` 官方反制置位中。

加 `--loop` 可循环播放全流程反复验证。
