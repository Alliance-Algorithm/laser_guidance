# 第一局前 2 分钟禁止反制（设计）

- 日期：2026-08-03
- 状态：已批准（用户确认方案 A）
- 关联：`referee_link.{hpp,cpp}`、`control_loop.cpp`、`tests/runtime/referee_link_test.cpp`

## 目标

daemon 启动后的第一个比赛窗口（第一局）中，前 120 秒不执行反制——激光不瞄准目标（振镜保持、不扫描、不输出瞄准命令）。2 分钟后恢复，第二局及以后从开局即正常反制。

判定"第一局"以 daemon 启动后见到的第一个 `game_progress==4` 边沿为准；中途重启/中途接入会把当时遇到的第一个窗口视为第一局（用户确认接受）。

## 范围

- **抑制**：`GuidanceSession::execute()` 的瞄准输出（角度/电压命令、扫描）。
- **不抑制**：跟踪、EKF、推理、overlay、推流、HitProgress、相机 lit/unlit 切换。
- HitProgress 无需门控：`is_purple` 是激光照射目标的视觉特征，不照射不会出现紫色，P 自然不涨（用户确认）。

## 行为

| 场景 | 行为 |
|---|---|
| 首局窗口开启（首个 progress-4 边沿） | `counter_delay_active_ = true` |
| 延迟中，`now - start_ns >= 120s` | 解除，恢复瞄准 |
| 第二局 progress-4 边沿 | 不重新激活（`first_round_seen_` 已置位） |
| 窗口关闭（5 / `stage_remain_time==0` / 断流兜底超时） | 清除延迟状态 |
| 无信号 / 赛外 | 窗口从未开启 → 永不激活，行为与现状完全一致 |
| 断流（延迟期内） | 本地 `start_ns` 计时继续走，120s 后照常解除 |

## 设计（方案 A：扩展 RefereeWindow）

`RefereeWindow` 已管理窗口/边沿/计时，且是纯逻辑（`now_ns` 可注入），延迟状态放这里可单测。

### RefereeWindow 新增状态

- `bool first_round_seen_ = false` —— 是否已出现过首个 progress-4 边沿（跨局持久，不随窗口重置）
- `bool counter_delay_active_ = false` —— 当前是否处于第一局延迟

### RefereeWindow 新增逻辑（`update()` 内）

1. `game_progress == 4` 且进入窗口（`match_started_pending_` 置位处）：
   - `if (!first_round_seen_) { first_round_seen_ = true; counter_delay_active_ = true; }`
   - 否则（第二局及以后）`counter_delay_active_ = false`
2. 窗口关闭路径（5 / `stage_remain_time==0` / `expire_if_over`）：`counter_delay_active_ = false`
3. `update()` 与 `expire_if_over()` 内：若 `counter_delay_active_` 且 `now_ns - start_ns_ >= 120s` → 解除

### 新增查询

```cpp
auto counter_delay_active(std::int64_t now_ns) const -> bool;
```

- 常量 `kFirstRoundCounterDelayS = 120`（`referee_link.cpp` 匿名 namespace，硬编码）

### ControlLoop 消费（`control_loop.cpp:403-408`）

```cpp
if (referee_link_.counter_delay_active(now)) {
    frame.guidance = {};  // command_issued=false：振镜保持、不扫描、不推流、ROS 不发
    // 每秒一条 stderr 提示（复用 GUIDE-DIAG 节奏或独立节流）
} else {
    frame.guidance = guidance->execute(frame.track);
}
```

跳过 `execute()` 而不是"调用后丢弃"：`execute()` 内部在 `command_issued` 时直接写 DAC 电压（`guidance_session.cpp:209-216`），延迟期不允许任何写入。

### 不动的部分

- `hit_progress_.reset()`（首局开局仍清零，防紫色污染——虽然延迟期不会紫）
- `note_official_countered()` 校核（延迟期官方不可能发 countered 边沿，因为 P 由我方激光驱动；保持原样）
- `maybe_switch_hik_profile()`（延迟期难度恒为 1 → lit，自动正确）

## 测试（`tests/runtime/referee_link_test.cpp`）

新增用例（注入 `now_ns`，沿用现有风格）：

1. 首个 progress-4 边沿 → `counter_delay_active()` 为 true
2. 延迟中 `now - start >= 120s` → 解除
3. 窗口关闭（5）→ 清除；下一个边沿（第二局）不再激活
4. 断流场景：仅 `expire_if_over` 推进时钟 → 120s 后解除
5. 无信号 → 永不激活

## 验证

- `cmake --build build --target referee_link_test && ./build/referee_link_test`
- `ctest --test-dir build --output-on-failure`（全量 27 项不回归）
- 全量构建 `cmake --build build --parallel` 无错误

## 文档同步

- `docs/runtime_operations.md`（referee 段）与根 `AGENTS.md` / `docs/AGENTS.md` 的 RefereeLink 描述补充"首局前 120s 禁止反制"语义。
