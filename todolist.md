# 重构 TodoList

## 目标

把当前仓库从“工具入口各自拼装运行时”的形态，进一步收敛为“稳定分层模块 + 统一 runtime + 可插拔 bridge + 清晰兼容层”的结构。

这次重构的核心目标不是改算法，而是把边界、职责和迁移路径固定下来：

- `tool_*` 只做入口
- runtime 统一管理生命周期
- bridge 统一管理协议和数据搬运
- guidance 统一管理解算、执行、扫描
- public API 只暴露稳定 facade
- 兼容层只保留，不继续扩张

## 重构原则

- 先收边界，再扩能力
  - 优先修职责混乱、跨层依赖、协议侵入，不优先改检测/跟踪/引导策略
- 先类型化，再协议化
  - 先定义内部数据结构和 command，再决定是否暴露给 FIFO / CLI / UDP
- 入口薄，模块厚
  - 主循环、线程、状态切换必须沉到库层
- 兼容层保留，但不继续长胖
  - `GuidancePipeline`、`TargetObservation`、`rmcs_laser_guidance_core` 仅作迁移缓冲
- 尽量行为等价
  - 默认不主动改用户可见行为，优先保证当前比赛/预览/录制链路继续可用

## 当前状态

已经完成的基础重构：

- [x] 新增稳定 facade
  - `include/laser_guidance/support.hpp`
  - `include/laser_guidance/runtime.hpp`
  - `include/laser_guidance/bridges.hpp`
- [x] 新增 runtime 分层
  - `CompetitionRuntime`
  - `PreviewRuntime`
  - `RuntimeCommand`
  - `RuntimeSnapshot`
- [x] 新增 bridge 分层
  - `FifoControlServer`
  - `UdpTelemetryPublisher`
  - `ShmFramePublisher`
  - `RtpFramePublisher`
- [x] 新增 guidance 分层
  - `AimSolver`
  - `GalvoExecutor`
  - `ScanController`
- [x] `tool_competition` / `tool_preview` 改为薄入口
- [x] FIFO 字符串协议改为 bridge 内部映射 typed command
- [x] CMake 基本拆为多 target
  - `laser_guidance_support`
  - `laser_guidance_vision`
  - `laser_guidance_tracking`
  - `laser_guidance_guidance`
  - `laser_guidance_bridges`
  - `laser_guidance_runtime`
- [x] 文档已提前写入新的架构思想

## 第一阶段：边界收口

### 1. public / internal API 收口

- [ ] 明确 public header 白名单
  - `include/config.hpp`
  - `include/types.hpp`
  - `include/laser_guidance/*.hpp`
- [ ] 避免新的 public header 继续暴露 `src/` 私有类型
- [ ] 补一份“public API 不变更约束”清单

### 2. 工具入口瘦身

- [x] `tool_competition` 改为 runtime facade
- [x] `tool_preview` 改为 runtime facade
- [ ] `tool_guidance` 迁移到新 guidance/runtime 结构
- [ ] 评估 `tool_record` 是否也应复用统一 runtime 录制骨架
- [ ] 清理仍残留在 tools 中的共享逻辑

### 3. bridge 边界固定

- [x] FIFO 解析移入 `FifoControlServer`
- [ ] 明确 UDP 是否只保留兼容观测格式，还是升级为 snapshot/telemetry 格式
- [ ] 明确 SHM / RTP 是否统一接受 overlay 帧还是支持原始帧/调试帧区分

## 第二阶段：runtime 内部继续拆分

### 4. runtime_base 再减重

- [ ] 继续拆 `RuntimeBase`
  - 目标：不要再次长成新的“大一统主循环文件”
- [ ] 提取状态与 snapshot 更新逻辑
  - 如 `runtime_state.*`
- [ ] 提取 frame processing / per-frame orchestration
  - 如 `frame_processor.*` 或 `runtime_policy.*`
- [ ] 提取 recording lifecycle
  - 如 `recording_controller.*`
- [ ] 提取 stream/shm/udp 输出协同逻辑
  - 如 `output_controller.*`

### 5. mode-specific runtime 差异收敛

- [ ] 明确 `CompetitionRuntime` 和 `PreviewRuntime` 的差异点
  - guidance 是否必须
  - recording 默认行为
  - 控制面是否一致
- [ ] 避免 mode-specific 逻辑回流到 `RuntimeBase`

## 第三阶段：数据模型收缩

### 6. 模型输出与运行时数据分层

- [ ] 让新链路默认走
  - `Detection`
  - `DetectionBatch`
  - `TargetTrack`
  - `AimInput`
  - `AimOutput`
- [ ] 停止在新代码里继续扩 `TargetObservation`
- [ ] 逐步把兼容输出适配限制在 bridge / legacy tool 边界

### 7. telemetry / snapshot 一致化

- [ ] 明确 `RuntimeSnapshot` 为统一观测出口
- [ ] 评估是否把 UDP telemetry 直接建立在 snapshot 子集上
- [ ] 评估测试中是否可以用 snapshot 替代部分 ad-hoc 状态观察

## 第四阶段：guidance 迁移完成

### 8. 兼容旧 `GuidancePipeline`

- [ ] 明确 `GuidancePipeline` 的剩余职责
  - 仅供旧工具和兼容路径使用
- [ ] 避免新功能继续加到 `GuidancePipeline`
- [ ] 计划最终把 `tool_guidance` 切到：
  - `AimSolver`
  - `GalvoExecutor`
  - `ScanController`

### 9. scan / direct voltage / geometry 三路统一

- [ ] 把 rectangle scan 策略完全沉到 `ScanController`
- [ ] 把 direct voltage 的新分支逻辑限制在 `AimSolver`
- [ ] 把 executor 和 solver 之间的数据协议固定成 `AimOutput`

## 第五阶段：测试补齐

### 10. runtime 测试

- [ ] `CompetitionRuntime` 生命周期测试
  - `start()`
  - `stop()`
  - 重复 `stop()`
  - `join()`
- [ ] `PreviewRuntime` 生命周期测试
- [ ] `RuntimeCommand` 提交后 snapshot 状态切换测试
- [ ] backend / enemy / ekf / recording / streaming 状态切换测试
- [ ] guidance 初始化失败时降级行为测试

### 11. bridge 测试

- [x] FIFO 文本命令解析测试
- [ ] FIFO 非法命令和多行输入测试
- [ ] telemetry/shm/rtp publisher 的最小冒烟测试

### 12. guidance / chain 测试

- [ ] `DetectionBatch -> TargetTrack -> AimOutput` 单测
- [ ] depth 缺失 / candidate 缺失 / EKF lost 降级策略测试
- [ ] `ScanController` 启停和退出回中测试

## 第六阶段：构建与依赖治理

### 13. target 依赖治理

- [x] 初步拆出 layered targets
- [ ] 检查 target 依赖方向是否还能继续简化
- [ ] 减少不必要的全局 include 暴露
- [ ] 让新测试和新工具尽量直接链接分层 target，而不是总走兼容聚合 target

### 14. 动态库 / 环境治理

- [x] 修 `FT4222` 链接和测试运行时加载问题
- [ ] 评估是否需要把 vendor 动态库处理得更稳
  - 更可靠的 RPATH
  - 安装时行为
  - 坏软链接治理

## 第七阶段：文档与迁移收尾

### 15. 文档同步

- [x] `README.md` 写入新架构思想
- [x] `docs/AGENTS.md` 写入新架构约束
- [x] `docs/architecture.md` 写入 runtime / bridge / guidance 分层
- [ ] 在 `docs/runtime_operations.md` 中补新 runtime 运行/控制说明
- [ ] 在 `docs/development.md` 中补 target 分层开发建议

### 16. 迁移完成标准

- [ ] `tool_competition` / `tool_preview` / `tool_guidance` 都不再承载长期主循环
- [ ] runtime 内部不再解析字符串协议
- [ ] 新功能默认走 facade + layered targets
- [ ] 兼容层只保留，不再继续扩张字段和职责
- [ ] 关键生命周期 / command / FIFO / guidance 降级测试齐备

## 风险与注意事项

- 不要一边重构边偷偷改算法口径
  - 否则很难判断回归是结构问题还是策略问题
- 不要把 bridge 变成第二套 runtime
  - bridge 只做协议和搬运
- 不要把 `RuntimeSnapshot` 做成无边界大对象
  - 只放运行时观测需要的稳定字段
- 不要继续把 shared helper 散落到 `tools/`
  - 优先放 support/runtime/bridge 层

## 建议执行顺序

1. 完成 `tool_guidance` 迁移和 guidance 兼容层收缩
2. 继续拆 `RuntimeBase`
3. 补 runtime 生命周期与 command 测试
4. 再做 snapshot / telemetry 一致化
5. 最后清理兼容层和 include 边界
