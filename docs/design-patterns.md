# 设计模式与架构评估

本文档从设计模式视角说明 `laser_guidance` 项目的架构现状，不重复 [architecture.md](./architecture.md) 的模块职责说明，而是聚焦于模式应用、原则遵循度和当前结构性问题。

## 已落地的设计模式

### 1. 策略模式 (Strategy) — 扩展点最成熟

三个独立扩展维度，各自闭环：

| 位置 | 抽象入口 | 策略 A | 策略 B | 选择依据 |
|------|---------|--------|--------|---------|
| `AimSolver::solve()` | 解算策略 | `solve_geometry()` | `solve_direct_voltage()` | `GuidanceConfig.command_model` |
| `InferenceFacade` | 推理后端 | ONNX Runtime runner | TensorRT runner | `RuntimeCommand.set_backend()` |
| `AimSolver::estimate_depth()` | 深度估计 | 单目 bbox 尺度估计 | 激光雷达点云聚类 | `GuidanceConfig.depth_source` |

每个策略封装在自己的方法或类中，新增策略只需加一个实现 + `switch` 分支，不触动调用方。**这是项目中开闭原则贯彻最好的一层。**

### 2. 模板方法模式 (Template Method)

```
RuntimeBase::run()                  ← 主循环骨架（不可覆写）
├── capture_.read_frame()
├── inference_.infer()
├── tracker_.process()
├── solver_.solve()
├── executor_.apply()
├── output_controller_.publish()
└── after_frame_processed()        ← 子类钩子
        ├── CompetitionRuntimeAdapter: cv::imshow + key polling
        └── PreviewRuntimeAdapter:    cv::imshow + key polling
```

`tools/competition.cpp` 只有 ~30 行——创建 runtime、绑定 FIFO、等待退出。业务编排全部在 `RuntimeBase` 内。这落实了 README 中声明的 "tool_* 不承载业务编排"。

### 3. 适配器模式 (Adapter)

```
ICaptureDevice              ← 统一接口（5 个方法）
├── V4l2CaptureDevice       ← 将 V4l2Capture 适配到 ICaptureDevice
└── HikCameraCaptureDevice  ← 将 hikcamera::Camera 适配到 ICaptureDevice
```

两个适配器都做同一件事：把异构底层实现翻译成 `CaptureFormat` + `Frame`，语义一致，里氏替换成立。

**当前状态**：适配器已写好但 RuntimeBase 未接入（见第四节问题 1）。

### 4. 门面模式 (Facade)

- **`InferenceFacade`**：封装 ONNX/TensorRT 双后端加载、惰性初始化、线程安全、运行时热切换
- **`RuntimeBase`**：封装 capture → infer → track → solve → execute → publish 完整管线
- **`CompetitionRuntime` / `PreviewRuntime`**：对外只暴露 4 个方法（`start/stop/submit_command/snapshot`），外部完全不知道内部 20+ 个组件

### 5. 桥接模式 (Bridge)

```
bridges 层 — 只做协议适配和数据搬运，不承载算法判断
├── FifoControlServer       ← 字符串 ↔ RuntimeCommand
├── UdpTelemetryPublisher   ← RuntimeSnapshot → JSON/UDP
├── ShmFramePublisher       ← cv::Mat → 共享内存
└── RtpFramePublisher       ← cv::Mat → RTP 流
```

bridge 层的设计守则：**不反向侵入 runtime，不把算法策略回写进 runtime**。这是一个达到设计意图的桥接模式——抽象（数据通道）和实现（具体协议）分离。

### 6. 工厂方法 (Factory Method)

```cpp
// InferenceFacade — 可注入推理 runner 工厂
using InferRunnerFactory = std::function<std::unique_ptr<IInferRunner>(const InferenceConfig&)>;

// Ft4222Spi — 静态工厂（构造私有，RAII 保证）
static auto Ft4222Spi::open(Ft4222Config) -> std::expected<Ft4222Spi, std::string>;
```

`InferRunnerFactory` 的设计值得注意：默认工厂创建 ONNX/TensorRT runner，测试可注入 mock——依赖注入和工厂模式的组合。

**Capture 层目前缺少对应的工厂**。见第四节问题 2。

### 7. Pimpl (编译防火墙)

以下类均使用 `std::unique_ptr<Impl>` 或 `std::unique_ptr<Details>` 隐藏实现：

- `FifoControlServer`, `UdpTelemetryPublisher`, `ShmFramePublisher`, `RtpFramePublisher`
- `CompetitionRuntime`, `PreviewRuntime`
- `EkfTracker`, `ModelInfer`
- `hikcamera::Camera`

`hikcamera` 模块进一步通过 `.impl.hpp` 把 MVS SDK 的 62 个 `MV_CC_*` 调用全部隔离在私有实现文件中，公共头文件不暴露任何 SDK 类型。

### 8. 依赖注入 (Dependency Injection)

```cpp
GalvoExecutor(const Config& config, Ft4222Spi& spi);
ScanController(const GuidanceConfig& config, GalvoExecutor& executor);
RuntimeOutputController(RtpFramePublisher&, ShmFramePublisher&,
    std::unique_ptr<VideoSessionRecorder>&, std::chrono::steady_clock::time_point&);
```

模块不自己 `new` 依赖，不依赖全局变量。对单元测试友好——可替换为 fake/stub。

---

## SOLID 原则评估

| 原则 | 状态 | 说明 |
|------|------|------|
| **S** 单一职责 | ✅ | 9 个 `src/` 模块各管一个领域；bridge 只做协议不做决策 |
| **O** 开闭原则 | ⚠️ | Guidance/Inference 层好（加策略不改调用方）；**Capture 层在 RuntimeBase 处断裂** |
| **L** 里氏替换 | ✅ | `ICaptureDevice` 两个实现可互换，`IInferRunner` 两个实现可互换 |
| **I** 接口隔离 | ✅ | `ICaptureDevice` 5 个方法，`IInferRunner` 3 个方法——最小接口 |
| **D** 依赖倒置 | ⚠️ | RuntimeBase 依赖具体类 `V4l2Capture` 而非抽象 `ICaptureDevice` |

---

## 当前结构性问题

### 问题 1：V4L2 类型泄漏到运行时层

`RuntimeBase` 和 `RuntimeOutputController` 的多个方法签名绑定 V4L2 特定类型：

```
RuntimeBase 成员:
    V4l2Capture capture_;                           ← 具体类型
    std::optional<V4l2NegotiatedFormat> negotiated_format_;

RuntimeOutputController 方法:
    start_streaming(const V4l2NegotiatedFormat&)
    apply_requests(..., const std::optional<V4l2NegotiatedFormat>&)
    begin_recording(const RecordSessionOptions&, const V4l2NegotiatedFormat&)

make_capture_snapshot(const V4l2NegotiatedFormat&)  ← 仅有 V4L2 重载

VideoSessionMetadata:
    std::filesystem::path device_path{};             ← V4L2 路径思维
    std::string fourcc{};                            ← V4L2 四字符编码
```

后果：HikCamera 设备没有 `/dev/videoX` 路径，没有 fourcc 编码（输出 BGR8 标准 `cv::Mat`），一旦接入会使录制/推流/快照链路断裂。

### 问题 2：Capture 层无工厂

`RuntimeBase` 在构造函数中直接：

```cpp
capture_(config_.v4l2)
```

没有根据 `CaptureBackendKind` 选择创建 `V4l2CaptureDevice` 还是 `HikCameraCaptureDevice` 的工厂方法。对比 `InferenceFacade` 中的 `InferRunnerFactory` 设计，capture 层缺少对应的抽象。

### 问题 3：配置不统一

```cpp
Config::v4l2     → V4l2Config（10 个字段）
HikCameraCaptureConfig               （9 个字段，独立定义）
```

没有统一的 `CaptureConfig`（variant 或 discriminated union），工厂方法无法从单一配置源创建不同类型的设备。

### 问题 4：RuntimeBase 规模偏大

20+ 成员变量、20+ 方法、11 行构造初始化列表。虽然职责仍是"编排"，但 `draw_results_overlay`、`process_guidance_step`、`publish_snapshot` 等方法可以在独立的 `OverlayRenderer`、`GuidancePipeline`、`SnapshotPublisher` 中。当前是中等规模的 God Object，尚未失控但趋势需要关注。

---

## 改进路线图

### 高优先级 — Capture 层抽象接入

- [ ] `RuntimeBase` 将 `V4l2Capture` 替换为 `std::unique_ptr<ICaptureDevice>`
- [ ] `negotiated_format_` 从 `V4l2NegotiatedFormat` 改为 `std::optional<CaptureFormat>`
- [ ] `RuntimeOutputController` 签名泛化为 `CaptureFormat`
- [ ] `make_capture_snapshot()` 提供 `CaptureFormat` 重载
- [ ] 新增 capture 工厂方法，根据配置创建 V4L2 或 HikCamera 设备
- [ ] `VideoSessionMetadata` 添加通用字段替代 `device_path`/`fourcc`

具体方案见 [capture-migration.md](./capture-migration.md)。

### 中优先级

- [ ] `RuntimeBase` 拆分渲染职责到独立 `OverlayRenderer`
- [ ] 统一 `CaptureConfig` 类型（`V4l2Config` 与 `HikCameraCaptureConfig` 合并为 variant）

### 低优先级

- [ ] `GuidanceToolRuntime` 迁移到 `ICaptureDevice` 抽象（当前仍硬编码 V4L2）
- [ ] `VideoSessionRecorder` 解耦 `V4l2NegotiatedFormat` 依赖

---

## 架构优点总结

以下方面是当前架构中值得肯定的设计决策：

1. **策略模式驱动解算扩展** — 加新 command_model、inference backend 或 depth source 只需加策略实现 + switch 分支
2. **Pimpl 隔离外部依赖** — MVS SDK、ffmpeg、FT4222H 库全部隐藏在实现文件内，公共头文件干净
3. **静态工厂 + RAII** — `Ft4222Spi::open()` 返回 `std::expected`，资源生命周期由析构函数保证
4. **可注入工厂** — `InferRunnerFactory` 让测试能替换推理后端，生产/测试切换零代码改动
5. **Bridge 单向依赖** — bridge 知道 runtime 类型（`RuntimeCommand`、`RuntimeSnapshot`），runtime 不知道 bridge 存在
6. **值语义 snapshot** — `RuntimeSnapshot` 是可复制、可缓存的值类型，不依赖临时 batch 生命周期
