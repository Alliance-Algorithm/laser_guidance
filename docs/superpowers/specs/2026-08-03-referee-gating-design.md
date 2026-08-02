# Referee 比赛状态门控与 HitProgress 锁定级校核

日期：2026-08-03
状态：已确认设计，待实现

## 背景与动机

激光引导系统（laser_guidance）目前启动即引导（profile=main + `guidance.enabled` 时），完全不了解比赛状态：

- 比赛未开始时可能就对紫色（Purple HIT）目标照射并本地累计"被瞄准进度"P，污染每局最多 5 次锁定的计数；
- 规则要求激光只在比赛阶段照射（RM2026 §5.6.3：仅在对方空中机器人发起空中支援时上电），赛前/局外照射存在违规风险。

目标：接入裁判系统 `game_progress`（参考 `radar_bridge_node.cpp` 的 ZMQ 订阅方式），用本地计时判定"比赛中"窗口，门控引导与 HitProgress 计算；并利用裁判系统 0x020C 官方反制状态对本地锁定事件做锁定级校核。

## 数据源与事实核对

裁判系统数据链路（2026 通信协议 V2.0.0）：

| 命令 | 内容 | 频率 | 链路 | 现状 |
|---|---|---|---|---|
| 0x0001 | game_status（game_type/game_progress/stage_remain_time/sync_timestamp） | 1Hz | 雷达无线链路 → radar-egui | egui 已转发 ZMQ（cmd_id 0x0001 JSON） |
| 0x020C | radar_mark_data（16 位，含 bit12 对方空中机器人被己方激光瞄准、bit13 处于被反制状态） | 1Hz | 常规链路串口 → radar-egui | egui 已转发 ZMQ（cmd_id 0x020C JSON） |
| 0x0101 | 场地事件（event 8：对方空中支援被反制，参数=剩余可反制次数） | 1Hz | 常规链路串口 → radar-egui | egui **未转发** ZMQ |
| 0x0104 | 裁判警告 | - | - | 与本次无关 |

game_progress 枚举（bit 4-7）：0=未开始，1=准备阶段，2=十五秒裁判系统自检，3=五秒倒计时，4=比赛中，5=比赛结算中。

结论：
- 通过 ZMQ 5558（radar-egui PUB）可拿到 0x0001 与 0x020C，**零新增依赖**（libzmq + nlohmann_json 已是强制依赖）；
- 反制**次数**（0x0101 event 8）egui 未转发，本次不做；改 radar-egui 属于跨仓库改动，明确排除。

## 设计

### 1. 新组件 `RefereeLink`（src/runtime/referee_link.{hpp,cpp}）

职责：ZMQ SUB 轮询 + JSON 解析 + 比赛窗口状态机，为 ControlLoop 提供门控与校核信号。

**ZMQ 层**
- `zmq::socket_type::sub` 连接 `referee.zmq_address`（默认 `tcp://127.0.0.1:5558`），订阅空前缀；
- `poll()` 每帧非阻塞 `recv(dontwait)`，按 `cmd_id` 分派：
  - `0x0001`：解析 game_type / game_progress / stage_remain_time / sync_timestamp → 喂状态机 + 透出；
  - `0x020C`：解析 opponent_aerial_targeted / opponent_aerial_countered / opponent_aerial_marked（其余位不解析）→ 存最新值 + 反制边沿；
  - 其他 cmd_id：忽略；
- JSON 解析失败：忽略该帧并计数（暴露于 snapshot 便于诊断），不中断。

**窗口状态机**（纯函数式，可注入 now_ns 单测，参考 radar_fusion::MatchTimer 的测试模式）

状态：
- `no_signal`：从未收到任何合法消息 → **不门控**（引导/HitProgress 维持旧行为，纯本地计算）；
- `idle`：已收到信号但不在比赛中窗口 → 门控生效（引导关闭、HitProgress 冻结）；
- `running`：比赛中窗口内 → 门控放行。

转移（每收到一条 0x0001 消息时判定）：
- `no_signal` → 收到第一条合法消息 → `idle`（或若 game_progress==4 直接 `running`）；
- `idle` → `game_progress==4` → `running`，记录 `start_ns = now`，产生 match_started 边沿（仅本次消费一次）；
- `running` → `game_progress==5` → `idle`，复位 start_ns；
- `running` → 本地计时 `now - start_ns >= match_duration_s`（默认 420）→ `idle`（硬窗口，不依赖裁判时钟/NTP）。

断流语义（看门狗）：一旦收到过信号，窗口判定完全由本地时钟驱动；**断流保持当前状态续导**——窗口内断流不熄灯、不提前退出，窗口判定继续由本地时钟推进至 `match_duration_s` 或收到新消息；窗口外断流保持不照射。断流（超过 `signal_timeout_s` 未收到合法消息）在 snapshot/overlay/日志中显著告警（`signal_stale` 标志）。窗口退出后的下一局：`game_progress==5 → ... → 4` 再次进入（re-arm），状态机天然覆盖。

相机参数阶段切换（RM2026 §5.6.3 难度表）：反制难度序列 1,2,2,3,3——难度 1/2 用 lit 相机参数，难度 3 用 unlit 参数。难度来源：裁判信号可用时由 0x020C 官方反制状态校核同步（`HitProgress::note_official_countered()` 在官方锁定边沿补计锁定次数，`difficulty()` 随之收敛到官方阶段）；无裁判信号时退化为纯本地 hit 计算。`maybe_switch_hik_profile` 保持读取 `hit_progress_.difficulty()` 不变（该值在有裁判时已被校核同步）。

**API**
```
poll()                                    // ZMQ recv + 状态机推进（使用系统 steady_clock）
guidance_allowed() -> bool                // signal_available && in_window
hit_progress_allowed() -> bool            // 同上
consume_match_started() -> bool           // idle→running 边沿（消费型）
consume_countered_edge() -> bool          // 0x020C bit13 上升沿（消费型）
referee_snapshot() -> RefereeSnapshot     // 透出数据
```

### 2. ControlLoop 集成（run_loop 单线程，无锁）

```cpp
referee_link_.poll();
if (referee_link_.consume_match_started()) hit_progress_.reset();   // 每局开局重置（防赛前紫色污染）
if (referee_link_.consume_countered_edge()) hit_progress_.note_official_countered(); // 官方锁定校核
frame.guidance = guidance->execute(frame.track);   // 全程引导，永不门控
hit_progress_.update(is_purple, is_colorless, dt_s);  // 赛外也用本地计算
```

**全程引导语义（2026-08-03 用户裁决）：** 引导执行（`GuidanceSession::execute`）与 HitProgress 本地计算**永不门控**——无论裁判系统有无信号、是否在比赛窗口内都持续运行。`game_progress` 的用途收窄为：① 每局开局 `match_started` 边沿触发 `HitProgress::reset()`（防赛前紫色污染，每局 5 次锁定全新计数）；② 0x020C 官方反制状态校核锁定计数与难度阶段（裁判可用时难度由官方同步，无裁判时纯本地 hit 计算）；③ 相机 lit/unlit 阶段切换（`maybe_switch_hik_profile` 读 `hit_progress_.difficulty()`）；④ 看门狗断流告警与比赛状态显示（snapshot/overlay/ROS）。

说明：
- GuidanceSession 初始化线程与生命周期保持不变（不反复开关 FT4222）；
- `RefereeWindow` 状态机保留（窗口内/计时/420s 本地超时/终端锁/re-arm），只服务于 `match_started` 边沿、`match_elapsed_s` 显示与校核语义，不再作为引导开关；

### 3. HitProgress 新增方法（src/tracking/hit_progress.{hpp,cpp}）

- `reset()`：p/t/n/p0/stage/difficulty/lock_count/locked/exhausted/hitting/lock_timer 全部归初值（构造默认值）。开局调用，保证每局 5 次锁定全新计数，防赛前紫色污染。
- `note_official_countered()`：0x020C bit13 上升沿触发的官方锁定校核：
  - 若 `locked_` 为 true（本地已计数本次锁定）→ 无操作；
  - 若 `locked_` 为 false 且 `lock_count_ < 5` → 本地漏检：`lock_count_++`、进入 45s 锁定（`locked_=true, lock_timer_=45, p_=0, t_=0, n_=0, hitting_=false`）、`advance_stage()`，并输出告警日志（stderr）；
  - 若 `lock_count_ >= 5` → 忽略（耗尽封顶）。

### 4. 配置（config/default.yaml 新增 referee 节）

```yaml
referee:
  enabled: true          # false = 不轮询、不门控、不校核（完全旧行为）
  zmq_address: "tcp://127.0.0.1:5558"
  match_duration_s: 420
  signal_timeout_s: 5    # 看门狗：超过该秒数未收到合法消息 → signal_stale（断流告警，不改门控状态）
```

`include/config.hpp` 新增 `RefereeConfig` 结构；`src/core/config.cpp` 增加解析。门控天然只在 main profile 生效（preview 不跑 guidance/HitProgress）。

### 5. Snapshot / 遥测透出

`RuntimeSnapshot` 新增字段（结构内嵌或顶层可选字段）：

```cpp
struct RefereeSnapshot {
    bool signal_available = false;
    bool signal_stale = false;        // 超过 signal_timeout_s 未收到合法消息（看门狗告警）
    uint8_t game_progress = 0;
    uint8_t game_type = 0;
    int64_t match_elapsed_s = -1;     // 窗口内已计时；窗口外 -1
    uint16_t stage_remain_time = 0;
    bool official_aerial_targeted = false;
    bool official_aerial_countered = false;
    double last_message_age_s = -1.0; // 距最后一条合法消息的秒数，诊断用
    uint64_t parse_errors = 0;
};
```

- overlay：新增一行状态显示（game_progress / elapsed / STALE 告警 / 官方位）；
- RosBridge 随 snapshot 自动发布；
- ZMQ 遥测（TargetObservation 结构）不扩展（字段为检测数据，语义不符）。

### 6. mock 裁判发布器 tools/referee_sim.cpp

本地端到端验证门控/校核，不依赖真裁判系统：

- 复用 libzmq + nlohmann_json（构建体系内）；
- ZMQ PUB 绑定 `tcp://*:5558`（地址/端口可命令行覆盖）；
- 参数化模拟比赛流程，默认序列：准备(1,300s) → 自检(2,15s) → 倒计时(3,5s) → 比赛中(4,420s) → 结算(5) → 循环或结束；
- 每 1Hz 发 0x0001 JSON（与 egui 格式一致）；0x020C JSON 可命令行触发（模拟 opponent_aerial_countered 置位/清除，验证校核路径）；
- `--help` 说明全部参数。

### 7. 测试

- `tests/runtime/referee_link_test.cpp`：
  - 状态机（注入 now_ns）：无信号不门控；4→running + match_started 边沿；5→idle；420s 超时；断流不提前开门；re-arm；
  - JSON 解析：0x0001 / 0x020C 合法与畸形帧；
  - 反制边沿：countered 0→1 产生一次边沿，连续置位不重复，1→0→1 再次产生。
- `tests/guidance/hit_progress_test.cpp`（追加）：reset() 全复位；note_official_countered() 已锁定跳过 / 漏检补计并锁定 45s / 耗尽忽略。
- 门控决策为纯函数，测试覆盖 guidance_allowed/hit_progress_allowed 三态（no_signal/idle/running）。

### 8. 明确不做（YAGNI）

- 0x0101 场地事件（egui 未转发 ZMQ）；
- 反制**次数**校核（依赖 event 8，需改 radar-egui）；
- stage_remain_time / sync_timestamp 参与门控（仅透出诊断）；
- 局间多轮规则特判（状态机 re-arm 已覆盖）。

## 数据流

```
radar-egui (ZMQ PUB tcp://*:5558)
  ├─ 0x0001 JSON (1Hz) ──┐
  └─ 0x020C JSON (1Hz) ──┤
                         ▼
              RefereeLink (SUB 非阻塞轮询)
                ├─ 窗口状态机（本地 steady_clock）
                ├─ guidance_allowed ──→ ControlLoop: execute() 门控
                ├─ hit_progress_allowed → ControlLoop: update() 门控
                ├─ match_started 边沿 ─→ HitProgress::reset()
                └─ countered 边沿 ─────→ HitProgress::note_official_countered()
```

## 验收标准

1. `referee.enabled: false` 或 ZMQ 无人发布：行为与现状完全一致（始终引导、HitProgress 照常本地计算）；
2. mock 裁判器发布完整流程：全程引导不中断（无 [GATED] 门控显示）；`game_progress==4` 时 HitProgress 重置后开始累计；每局 match_started 边沿重置；420s 或 5 后窗口退出（仅影响计时显示与重置语义，不影响引导）；
3. mock 触发 countered 置位：本地 HitProgress 校核生效（漏检时补计锁定，难度随官方锁定次数推进）；
4. 全部单测通过（referee_link + hit_progress 追加用例）。
