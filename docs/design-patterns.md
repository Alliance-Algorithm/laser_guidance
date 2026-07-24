# 设计模式与边界

本文档补充 [architecture.md](./architecture.md)，只描述当前仍然保留的设计模式和边界。

## 当前保留的模式

### 1. Facade

- `CompetitionRuntime`
  - 唯一 public runtime facade。
  - 对外只暴露 `start()`、`stop()`、`join()`、`submit_command()`、`snapshot()`。

### 2. 具体组合

- `ControlLoop`
  - 直接持有 `CaptureDevice`、`PerceptionRunner`、可选 `GuidanceSession`、`RuntimeOutputs`。
- `GuidanceOpsApp`
  - 独立持有 capture / perception / guidance / overlay，不再通过共享 runtime 注入差异。

当前 runtime 内部不再通过 builder、hook、controller、factory 叠抽象层。

### 3. Value Types

以下类型保持值语义：

- `RuntimeCommand`
  - Closed sum type: `std::variant<CmdSetStreaming, CmdSetRecording, CmdSetEnemyColor, CmdSetBackend, CmdSetEkf, CmdShutdown>`.
  - Factories in `runtime_command::` namespace.
  - Exhaustive handling via `std::visit` in `ControlLoop::submit_command`.
- `RuntimeSnapshot`
- `DetectionBatch`
- `TargetTrack`
- `AimInput`
- `AimOutput`

控制面和观测面优先通过这些值类型传播，而不是让 bridge 或 tool 持有内部对象引用。

### 4. Error Domain

- `ErrorKind` — classified error categories: `config`, `device`, `unavailable`, `timeout`, `internal`.
- `Error` — struct pairing `ErrorKind` with a human-readable `message`.
- `format_error()` — produces `"[kind] message"` for logging.
- Public control path (`CompetitionRuntime::start/run/submit_command`), capture (`CaptureDevice::open/reconnect`), guidance (`GuidanceSession::create_*`), and FT4222 (`Ft4222Spi::open/write/transfer`) all return `std::expected<T, Error>`.
- Retry policy: `device` errors keep existing reconnect loops; `config` errors fail start or reject commands; `unavailable` degrades the feature; `internal` logs loudly and prefers fail-closed.

### 5. Backend Selection

- `CaptureDevice`
  - Selects backend via `std::unique_ptr<CaptureBackend>`, where `CaptureBackend` is a thin virtual base with 5-6 pure methods.
  - Concrete implementations: `V4l2Backend` and `HikBackend` (latter in `src/capture/hik_backend.hpp`).

这类"少量后端、启动时选定"的差异，优先用具体类型 + thin virtual interface，避免大型虚接口森林。`CaptureBackend` 保持最小契约（open / read_frame / close / is_open / reconnect）。

### 6. Bridge

bridge 层只做协议转换和搬运：

- `FifoControlServer`
- `UdpTelemetryPublisher`
- `ShmFramePublisher`
- `RtpFramePublisher`

它们不承载算法判断，也不反向侵入 runtime。

### 7. 外部依赖隔离

对 SDK / 外部实现细节仍保留 concrete wrapper 或 Pimpl：

- Hik SDK
- 若干 bridge / infer 细节实现

> `CompetitionRuntime` 本身是 public facade（见 §1），其 Pimpl 只是为了隐藏内部依赖，不属于"外部依赖隔离"范畴。

保留这层隔离的目的是控制编译依赖和第三方头泄漏，不是为了在业务层继续加接口层级。

## 已移除的模式

以下模式已从 runtime 主链路删除：

- `GuidanceController`
- `ModeHooks`
- `IInferRunner` / `InferRunnerFactory`
- `ModelAdapter` 虚接口
- `ICaptureDevice` / capture factory
- `ControlLoopBuilder`
- `OutputBundleHooks`
- ws30 / lidar 分支

## 当前原则

- public 面只保留稳定 facade 和 value types。
- runtime 内部优先具体组合，不优先抽象。
- 新增控制项先扩 `RuntimeCommand` / `RuntimeSnapshot`。
- 失败分类通过 `Error` / `ErrorKind` 驱动重试与拒绝策略，替代裸 `std::string`。
- 新增硬件或协议隔离点时，只在第三方依赖边界保留 wrapper。
