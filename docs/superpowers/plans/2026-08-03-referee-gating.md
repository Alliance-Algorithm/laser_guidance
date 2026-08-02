# Referee 比赛状态门控与 HitProgress 校核 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 接入裁判系统 game_progress（ZMQ 5558），用本地 420s 硬窗口门控引导与 HitProgress，并用 0x020C 官方反制状态对本地锁定做校核。

**Architecture:** 新增 `RefereeLink`（ZMQ SUB 轮询 + 纯状态机）注入 ControlLoop 主循环；`HitProgress` 增加 `reset()`/`note_official_countered()`；snapshot/overlay/ROS 透出；`tools/referee_sim` 提供本地 mock 裁判。无信号时退化为旧行为（不门控）。

**Tech Stack:** C++23, libzmq/cppzmq, nlohmann_json, CMake, 自研测试框架（`tests/test_utils.hpp` 的 `require`）。

## Global Constraints

- 构建入口：`./.script/build-laser`；单测：`ctest --test-dir build -R <name> --output-on-failure`（测试 WORKING_DIRECTORY=repo 根）。
- 代码风格：C++23、`std::print/std::println`、`std::expected`、无注释或仅必要中文注释（沿用现有风格）。
- 依赖仅限已有的 libzmq + nlohmann_json（`laser_guidance_common` 已含）；禁止新增第三方依赖。
- `RefereeLink` 门控语义（spec §2）：从未收到合法消息 → 不门控（旧行为）；`game_progress==4` 进入窗口；`==5` 或本地计时 ≥ `match_duration_s` 退出；断流不改变窗口判定。
- `match_duration_s` 默认 420；`signal_timeout_s` 默认 5（看门狗：断流告警，不改门控状态）；`referee.enabled` 默认 false（结构体默认），`config/default.yaml` 显式 `enabled: true`。
- 所有新增测试文件自动被 CMake GLOB 拾取（tests/*_test.cpp、tools/*.cpp），无需注册；`src/runtime/referee_link.cpp` 必须手动加入 `laser_guidance_runtime_SOURCES` 列表（CMakeLists.txt:221-231）。

---

### Task 1: RefereeConfig 配置层

**Files:**
- Modify: `include/config.hpp`（新增 RefereeConfig + Config::referee）
- Modify: `src/core/config.cpp`（解析 `referee:` YAML 节）
- Modify: `config/default.yaml`（新增 referee 节）
- Test: `tests/core/config_test.cpp`

**Interfaces:**
- Produces: `struct RefereeConfig { bool enabled = false; std::string zmq_address = "tcp://127.0.0.1:5558"; int match_duration_s = 420; };` 与 `Config::referee` 字段。Task 2/4 依赖此结构。

- [ ] **Step 1: 写失败测试（追加到 config_test.cpp main 内，`default_config` 断言之后）**

```cpp
        require(default_config.referee.enabled, "default referee enabled mismatch");
        require(
            default_config.referee.zmq_address == "tcp://127.0.0.1:5558",
            "default referee zmq_address mismatch");
        require(
            default_config.referee.match_duration_s == 420,
            "default referee match_duration_s mismatch");
```

- [ ] **Step 2: 构建并确认测试失败**

Run: `./.script/build-laser && ctest --test-dir build -R config_test --output-on-failure`
Expected: FAIL（`Config` 无 `referee` 字段，编译错误）。

- [ ] **Step 3: 实现**

`include/config.hpp`（放在 `ZmqConfig` 之后）：

```cpp
struct RefereeConfig {
    bool enabled = false;
    std::string zmq_address = "tcp://127.0.0.1:5558";
    int match_duration_s = 420;
};
```

`Config` 结构体加字段：`RefereeConfig referee{};`（`EkfConfig ekf{};` 之后）。

`src/core/config.cpp` 在 zmq 节解析之后追加：

```cpp
    if (const YAML::Node referee_cfg = yaml["referee"]) {
        if (referee_cfg["enabled"])
            config.referee.enabled = referee_cfg["enabled"].as<bool>();
        if (referee_cfg["zmq_address"])
            config.referee.zmq_address = referee_cfg["zmq_address"].as<std::string>();
        if (referee_cfg["match_duration_s"])
            config.referee.match_duration_s = referee_cfg["match_duration_s"].as<int>();
    }
```

`config/default.yaml` 在 `zmq:` 节之后新增：

```yaml
referee:
  enabled: true
  zmq_address: "tcp://127.0.0.1:5558"
  match_duration_s: 420
```

- [ ] **Step 4: 构建并确认测试通过**

Run: `./.script/build-laser && ctest --test-dir build -R config_test --output-on-failure`
Expected: PASS。

- [ ] **Step 5: 提交**

```bash
git add include/config.hpp src/core/config.cpp config/default.yaml tests/core/config_test.cpp
git commit -m "feat(config): referee section for match-state gating"
```

---

### Task 2: RefereeLink 状态机与 JSON 解析（纯逻辑）

**Files:**
- Create: `src/runtime/referee_link.hpp`
- Create: `src/runtime/referee_link.cpp`
- Modify: `CMakeLists.txt:221-231`（`laser_guidance_runtime_SOURCES` 加一行）
- Test: `tests/runtime/referee_link_test.cpp`

**Interfaces:**
- Consumes: `RefereeConfig`（Task 1）。
- Produces（Task 4 使用）：

```cpp
// referee_link.hpp, namespace rmcs_laser_guidance::runtime_internal
struct GameStateSample {
    std::uint8_t game_type = 0;
    std::uint8_t game_progress = 0;
    std::uint16_t stage_remain_time = 0;
    std::uint64_t sync_timestamp = 0;
};
struct MarkSample {
    bool opponent_aerial_targeted = false;
    bool opponent_aerial_countered = false;
    bool opponent_aerial_marked = false;
};
auto parse_game_state_json(std::string_view json) -> std::optional<GameStateSample>;
auto parse_mark_json(std::string_view json) -> std::optional<MarkSample>;

class RefereeWindow {           // 纯状态机，可注入 now_ns
public:
    explicit RefereeWindow(int match_duration_s = 420);
    void update(std::uint8_t game_progress, std::int64_t now_ns);
    [[nodiscard]] auto signal_available() const -> bool;
    [[nodiscard]] auto in_window() const -> bool;
    [[nodiscard]] auto allowed() const -> bool;              // signal_available && in_window
    [[nodiscard]] auto consume_match_started() -> bool;      // idle→running 边沿，消费型
    [[nodiscard]] auto match_elapsed_s(std::int64_t now_ns) const -> std::int64_t; // 窗口外 -1
};
```

- [ ] **Step 1: 写失败测试 `tests/runtime/referee_link_test.cpp`**（状态机 + JSON 解析全用例，代码见下）

```cpp
#include <print>
#include <stdexcept>

#include "runtime/referee_link.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::runtime_internal;
        using rmcs_laser_guidance::tests::require;

        // ---- JSON 解析 ----
        {
            const auto sample = parse_game_state_json(
                R"({"cmd_id":1,"game_type":1,"game_progress":4,"stage_remain_time":300,"sync_timestamp":12345})");
            require(sample.has_value(), "valid game state parses");
            require(sample->game_type == 1, "game_type parsed");
            require(sample->game_progress == 4, "game_progress parsed");
            require(sample->stage_remain_time == 300, "stage_remain_time parsed");
            require(sample->sync_timestamp == 12345, "sync_timestamp parsed");
        }
        {
            require(!parse_game_state_json(R"({"cmd_id":999,"game_progress":4})").has_value(),
                "wrong cmd_id rejected");
            require(!parse_game_state_json("not json").has_value(), "garbage rejected");
            require(!parse_game_state_json(R"({"cmd_id":1,"game_progress":9})").has_value(),
                "invalid game_progress rejected");
        }
        {
            const auto mark = parse_mark_json(
                R"({"cmd_id":524,"opponent_aerial_targeted":1,"opponent_aerial_countered":1,"opponent_aerial_marked":0})");
            require(mark.has_value(), "valid mark parses");
            require(mark->opponent_aerial_targeted && mark->opponent_aerial_countered,
                "targeted/countered parsed");
            require(!mark->opponent_aerial_marked, "marked parsed");
        }

        // ---- 状态机：无信号不门控 ----
        {
            RefereeWindow w;
            require(w.allowed(), "no signal -> ungated (old behavior)");
            require(w.signal_available() == false, "no signal flag");
            require(w.match_elapsed_s(1'000) == -1, "no elapsed before start");
            w.update(1, 5'000'000'000);  // 准备阶段
            require(w.allowed() == false, "prep phase gated");
            require(w.signal_available(), "signal now available");
        }

        // ---- 4 进入 + 边沿 + 计时 ----
        {
            RefereeWindow w;
            w.update(1, 1'000);
            w.update(2, 2'000);
            w.update(3, 3'000);
            require(!w.consume_match_started(), "no edge before match");
            w.update(4, 4'000);
            require(w.allowed(), "match -> allowed");
            require(w.consume_match_started(), "match_started edge fired");
            require(!w.consume_match_started(), "edge consumed once");
            require(w.match_elapsed_s(14'000) == 10, "elapsed = local clock diff");
        }

        // ---- 5 退出 + re-arm ----
        {
            RefereeWindow w;
            w.update(4, 4'000);
            w.consume_match_started();
            w.update(5, 424'000);
            require(!w.allowed(), "settle exits window");
            require(w.match_elapsed_s(425'000) == -1, "elapsed reset");
            w.update(1, 430'000);
            w.update(4, 500'000);
            require(w.allowed(), "next round re-arms");
            require(w.consume_match_started(), "re-arm fires edge again");
        }

        // ---- 420s 硬窗口超时 ----
        {
            RefereeWindow w;
            w.update(4, 4'000);
            w.consume_match_started();
            w.update(4, 423'000);   // 419s
            require(w.allowed(), "within window at 419s");
            w.update(4, 424'100);   // 420.1s
            require(!w.allowed(), "exits at 420s despite progress still 4");
        }

        // ---- 断流：窗口判定继续由本地时钟驱动 ----
        {
            RefereeWindow w;
            w.update(4, 4'000);
            w.consume_match_started();
            w.update(4, 10'000);    // 6s
            w.update(4, 200'000);   // 196s（模拟断流后仅本地时钟推进）
            require(w.allowed(), "stale signal keeps window");
            w.update(4, 500'000);   // 496s > 420
            require(!w.allowed(), "stale signal still expires at 420s");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "referee_link_test failed: {}", e.what());
        return 1;
    }
}
```

- [ ] **Step 2: 构建并确认失败**

Run: `./.script/build-laser && ctest --test-dir build -R referee_link_test --output-on-failure`
Expected: 编译失败（referee_link.hpp 不存在）。

- [ ] **Step 3: 实现 `src/runtime/referee_link.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

#include "config.hpp"

namespace rmcs_laser_guidance::runtime_internal {

struct GameStateSample {
    std::uint8_t game_type = 0;
    std::uint8_t game_progress = 0;
    std::uint16_t stage_remain_time = 0;
    std::uint64_t sync_timestamp = 0;
};

struct MarkSample {
    bool opponent_aerial_targeted = false;
    bool opponent_aerial_countered = false;
    bool opponent_aerial_marked = false;
};

// radar-egui 0x0001 JSON（与 radar_bridge 收到的同一格式）
auto parse_game_state_json(std::string_view json) -> std::optional<GameStateSample>;
// radar-egui 0x020C JSON
auto parse_mark_json(std::string_view json) -> std::optional<MarkSample>;

// 比赛窗口状态机（纯逻辑，now_ns 为本地 steady 时钟纳秒，测试可注入）
class RefereeWindow {
public:
    explicit RefereeWindow(int match_duration_s = 420);

    void update(std::uint8_t game_progress, std::int64_t now_ns);
    [[nodiscard]] auto signal_available() const -> bool { return signal_available_; }
    [[nodiscard]] auto in_window() const -> bool { return in_window_; }
    [[nodiscard]] auto allowed() const -> bool { return signal_available_ && in_window_; }
    [[nodiscard]] auto consume_match_started() -> bool;
    [[nodiscard]] auto match_elapsed_s(std::int64_t now_ns) const -> std::int64_t;

private:
    int match_duration_s_{};
    bool signal_available_ = false;
    bool in_window_ = false;
    std::int64_t start_ns_ = 0;
    bool match_started_pending_ = false;
};

} // namespace rmcs_laser_guidance::runtime_internal
```

- [ ] **Step 4: 实现 `src/runtime/referee_link.cpp`**

```cpp
#include "runtime/referee_link.hpp"

#include <algorithm>
#include <nlohmann/json.hpp>

namespace rmcs_laser_guidance::runtime_internal {
namespace {

constexpr std::uint16_t kGameStateCmdId = 0x0001;
constexpr std::uint16_t kMarkCmdId = 0x020C;

} // namespace

auto parse_game_state_json(std::string_view json) -> std::optional<GameStateSample> {
    try {
        const auto parsed = nlohmann::json::parse(json);
        if (parsed.at("cmd_id").get<std::uint16_t>() != kGameStateCmdId)
            return std::nullopt;
        GameStateSample sample;
        sample.game_type = parsed.at("game_type").get<std::uint8_t>();
        sample.game_progress = parsed.at("game_progress").get<std::uint8_t>();
        sample.stage_remain_time = parsed.at("stage_remain_time").get<std::uint16_t>();
        sample.sync_timestamp = parsed.at("sync_timestamp").get<std::uint64_t>();
        if (sample.game_progress > 5)
            return std::nullopt;
        return sample;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

auto parse_mark_json(std::string_view json) -> std::optional<MarkSample> {
    try {
        const auto parsed = nlohmann::json::parse(json);
        if (parsed.at("cmd_id").get<std::uint16_t>() != kMarkCmdId)
            return std::nullopt;
        MarkSample sample;
        sample.opponent_aerial_targeted =
            parsed.at("opponent_aerial_targeted").get<bool>();
        sample.opponent_aerial_countered =
            parsed.at("opponent_aerial_countered").get<bool>();
        sample.opponent_aerial_marked = parsed.at("opponent_aerial_marked").get<bool>();
        return sample;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

RefereeWindow::RefereeWindow(int match_duration_s)
    : match_duration_s_(std::max(1, match_duration_s)) {}

void RefereeWindow::update(std::uint8_t game_progress, std::int64_t now_ns) {
    signal_available_ = true;
    if (game_progress == 5) {
        in_window_ = false;
        start_ns_ = 0;
        return;
    }
    if (in_window_) {
        if (now_ns - start_ns_ >= static_cast<std::int64_t>(match_duration_s_) * 1'000'000'000) {
            in_window_ = false;
            start_ns_ = 0;
        }
        return;
    }
    if (game_progress == 4) {
        in_window_ = true;
        start_ns_ = now_ns;
        match_started_pending_ = true;
    }
}

auto RefereeWindow::consume_match_started() -> bool {
    const bool pending = match_started_pending_;
    match_started_pending_ = false;
    return pending;
}

auto RefereeWindow::match_elapsed_s(std::int64_t now_ns) const -> std::int64_t {
    if (!in_window_)
        return -1;
    return std::max<std::int64_t>(0, (now_ns - start_ns_) / 1'000'000'000);
}

} // namespace rmcs_laser_guidance::runtime_internal
```

- [ ] **Step 5: CMakeLists.txt 把新源文件加入 runtime 库**

在 `src/runtime/runtime_outputs.cpp` 行后追加：

```cmake
  src/runtime/referee_link.cpp
```

- [ ] **Step 6: 构建并确认测试通过**

Run: `./.script/build-laser && ctest --test-dir build -R referee_link_test --output-on-failure`
Expected: PASS。

- [ ] **Step 7: 提交**

```bash
git add src/runtime/referee_link.hpp src/runtime/referee_link.cpp CMakeLists.txt tests/runtime/referee_link_test.cpp
git commit -m "feat(runtime): referee window state machine and ZMQ JSON parsing"
```

---

### Task 3: RefereeLink ZMQ 轮询封装

**Files:**
- Modify: `src/runtime/referee_link.hpp` / `src/runtime/referee_link.cpp`（加 RefereeLink 类）
- Test: `tests/runtime/referee_link_test.cpp`（追加用例）

**Interfaces:**
- Consumes: Task 1 `RefereeConfig`、Task 2 `RefereeWindow`/parse 函数。
- Produces（Task 4 使用）：

```cpp
class RefereeLink {
public:
    explicit RefereeLink(RefereeConfig config);
    ~RefereeLink();
    auto poll() -> void;                     // 每帧调用；enabled=false 时空转
    [[nodiscard]] auto guidance_allowed() const -> bool;
    [[nodiscard]] auto hit_progress_allowed() const -> bool;
    [[nodiscard]] auto consume_match_started() -> bool;
    [[nodiscard]] auto consume_countered_edge() -> bool;
    [[nodiscard]] auto snapshot() const -> RefereeSnapshot;  // 见 Task 3 Step 1 定义
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
```

- [ ] **Step 1: 追加测试（referee_link_test.cpp 末尾，return 0 之前）**

```cpp
        // ---- RefereeLink enabled=false 空转不抛异常、不门控 ----
        {
            rmcs_laser_guidance::RefereeConfig cfg;
            cfg.enabled = false;
            RefereeLink link(cfg);
            link.poll();
            require(link.guidance_allowed(), "disabled link ungated");
            require(!link.consume_match_started(), "no edges when disabled");
            require(!link.consume_countered_edge(), "no countered edge when disabled");
        }
```

- [ ] **Step 2: 构建确认失败**

Run: `./.script/build-laser && ctest --test-dir build -R referee_link_test --output-on-failure`
Expected: 编译失败（RefereeLink 未定义）。

- [ ] **Step 3: RefereeSnapshot 结构（先于 RefereeLink 定义，供 snapshot() 返回类型使用）**

`include/laser_guidance/runtime.hpp` 在 `RuntimeSnapshot` 之前添加：

```cpp
struct RefereeSnapshot {
    bool signal_available = false;
    bool signal_stale = false;     // 超过 signal_timeout_s 未收到合法消息（看门狗告警，不改门控）
    bool guidance_gated = false;   // true = 引导当前被门控禁止（窗口外）
    std::uint8_t game_progress = 0;
    std::uint8_t game_type = 0;
    std::int64_t match_elapsed_s = -1;
    std::uint16_t stage_remain_time = 0;
    bool official_aerial_targeted = false;
    bool official_aerial_countered = false;
    double last_message_age_s = -1.0;
    std::uint64_t parse_errors = 0;
};
```

`RuntimeSnapshot` 末尾加字段：`RefereeSnapshot referee{};`

- [ ] **Step 4: 实现 RefereeLink 声明（referee_link.hpp 末尾追加）**

```cpp
#include <memory>

// RefereeLink 内部持有 zmq 对象，通过 Impl PIMPL 隔离，公共头不暴露 libzmq 类型
class RefereeLink {
public:
    explicit RefereeLink(RefereeConfig config);
    ~RefereeLink();
    auto poll() -> void;
    [[nodiscard]] auto guidance_allowed() const -> bool;
    [[nodiscard]] auto hit_progress_allowed() const -> bool;
    [[nodiscard]] auto consume_match_started() -> bool;
    [[nodiscard]] auto consume_countered_edge() -> bool;
    [[nodiscard]] auto snapshot() const -> RefereeSnapshot;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
```

（`referee_link.hpp` 需 `#include <memory>` 与前置声明 `namespace rmcs_laser_guidance { struct RefereeSnapshot; }`，置于 `runtime_internal` 命名空间外。）

- [ ] **Step 4a: 看门狗配置字段（Task 1 的配置层扩展，`signal_timeout_s`）**

`include/config.hpp` 的 `RefereeConfig` 追加字段：

```cpp
    int signal_timeout_s = 5;   // 看门狗：超过该秒数未收到合法消息 → signal_stale
```

`src/core/config.cpp` 的 referee 节解析追加：

```cpp
        if (referee_cfg["signal_timeout_s"])
            config.referee.signal_timeout_s = referee_cfg["signal_timeout_s"].as<int>();
```

`config/default.yaml` 的 referee 节追加：

```yaml
  signal_timeout_s: 5
```

`tests/core/config_test.cpp` 追加断言：

```cpp
        require(default_config.referee.signal_timeout_s == 5, "default referee signal_timeout_s mismatch");
```

Run: `./.script/build-laser && ctest --test-dir build -R config_test --output-on-failure`
Expected: PASS。

- [ ] **Step 5: 实现 RefereeLink（referee_link.cpp 末尾追加）**

```cpp
#include <chrono>
#include <utility>
#include <zmq.hpp>

namespace {
constexpr std::int64_t kNsPerSecond = 1'000'000'000;

auto now_ns() -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               rmcs_laser_guidance::Clock::now().time_since_epoch()).count();
}
} // namespace

struct RefereeLink::Impl {
    explicit Impl(RefereeConfig config)
        : config(std::move(config))
        , window(config.match_duration_s) {}

    RefereeConfig config;
    zmq::context_t ctx{1};
    std::unique_ptr<zmq::socket_t> sub;
    RefereeWindow window;
    GameStateSample game_state;
    MarkSample mark;
    bool last_countered = false;
    bool countered_pending = false;
    std::uint64_t parse_errors = 0;
    std::optional<rmcs_laser_guidance::Clock::time_point> last_message_time;
};
RefereeLink::RefereeLink(RefereeConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
RefereeLink::~RefereeLink() = default;

auto RefereeLink::poll() -> void {
    if (!impl_->config.enabled)
        return;
    if (!impl_->sub) {
        impl_->sub = std::make_unique<zmq::socket_t>(impl_->ctx, zmq::socket_type::sub);
        impl_->sub->connect(impl_->config.zmq_address);
        impl_->sub->set(zmq::sockopt::subscribe, "");
    }
    while (true) {
        zmq::message_t message;
        if (!impl_->sub->recv(message, zmq::recv_flags::dontwait))
            break;
        const std::string_view text(static_cast<const char*>(message.data()), message.size());
        if (const auto state = parse_game_state_json(text); state.has_value()) {
            impl_->game_state = *state;
            impl_->window.update(state->game_progress, now_ns());
            impl_->last_message_time = rmcs_laser_guidance::Clock::now();
        } else if (const auto mark = parse_mark_json(text); mark.has_value()) {
            impl_->mark = *mark;
            if (mark->opponent_aerial_countered && !impl_->last_countered)
                impl_->countered_pending = true;
            impl_->last_countered = mark->opponent_aerial_countered;
            impl_->last_message_time = rmcs_laser_guidance::Clock::now();
        } else {
            ++impl_->parse_errors;
        }
    }
}

auto RefereeLink::guidance_allowed() const -> bool { return impl_->window.allowed(); }
auto RefereeLink::hit_progress_allowed() const -> bool { return impl_->window.allowed(); }
auto RefereeLink::consume_match_started() -> bool { return impl_->window.consume_match_started(); }
auto RefereeLink::consume_countered_edge() -> bool {
    const bool pending = impl_->countered_pending;
    impl_->countered_pending = false;
    return pending;
}

auto RefereeLink::snapshot() const -> rmcs_laser_guidance::RefereeSnapshot {
    rmcs_laser_guidance::RefereeSnapshot snap;
    snap.signal_available = impl_->window.signal_available();
    snap.guidance_gated = impl_->window.signal_available() && !impl_->window.in_window();
    snap.game_progress = impl_->game_state.game_progress;
    snap.game_type = impl_->game_state.game_type;
    snap.match_elapsed_s = impl_->window.match_elapsed_s(now_ns());
    snap.stage_remain_time = impl_->game_state.stage_remain_time;
    snap.official_aerial_targeted = impl_->mark.opponent_aerial_targeted;
    snap.official_aerial_countered = impl_->mark.opponent_aerial_countered;
    if (impl_->last_message_time.has_value()) {
        const auto age = std::chrono::duration<double>(
            rmcs_laser_guidance::Clock::now() - *impl_->last_message_time).count();
        snap.last_message_age_s = age;
        // 看门狗：断流只告警，不改门控状态（断流续导语义）
        snap.signal_stale = age > static_cast<double>(impl_->config.signal_timeout_s);
    }
    snap.parse_errors = impl_->parse_errors;
    return snap;
}
```

（`referee_link.cpp` 需要 `#include <optional>`、`#include <string_view>`，`Clock` 取自 `laser_guidance/support.hpp` 或已在现有 include 链中；若 `Clock` 未定义则 `#include "laser_guidance/support.hpp"`。）

- [ ] **Step 6: 构建并确认通过**

Run: `./.script/build-laser && ctest --test-dir build -R referee_link_test --output-on-failure`
Expected: PASS。

- [ ] **Step 7: 提交**

```bash
git add src/runtime/referee_link.hpp src/runtime/referee_link.cpp include/laser_guidance/runtime.hpp tests/runtime/referee_link_test.cpp
git commit -m "feat(runtime): RefereeLink ZMQ polling wrapper with countered edge"
```

---

### Task 4: HitProgress reset 与官方锁定校核

**Files:**
- Modify: `src/tracking/hit_progress.hpp`
- Modify: `src/tracking/hit_progress.cpp`
- Test: `tests/tracking/hit_progress_test.cpp`

**Interfaces:**
- Produces（Task 5 使用）：
  - `void HitProgress::reset();`
  - `auto HitProgress::note_official_countered() -> bool;`（返回 true = 本地漏检、补计一次锁定并进入 45s 锁定）

- [ ] **Step 1: 写失败测试（追加到 hit_progress_test.cpp，return 0 之前）**

```cpp
        {
            HitProgress hp;
            for (int i = 0; i < 20; ++i)
                hp.update(true, kTick);
            hp.update(false, 2.0F);
            require(hp.lock_count() > 0 || hp.progress() > 0.0F, "state dirty before reset");
            hp.reset();
            require(hp.progress() == 0.0F, "reset clears P");
            require(hp.stage() == 0, "reset clears stage");
            require(hp.difficulty() == 1, "reset restores difficulty 1");
            require(hp.p0() == 50.0F, "reset restores P0 50");
            require(hp.lock_count() == 0, "reset clears lock_count");
            require(!hp.is_locked(), "reset clears locked");
            require(!hp.is_exhausted(), "reset clears exhausted");
            require(!hp.is_hitting(), "reset clears hitting");
        }

        {
            HitProgress hp;
            hp.update(true, 45.0F);
            require(hp.is_locked(), "precondition: locked");
            require(!hp.note_official_countered(), "already locked -> no correction");
            require(hp.lock_count() == 1, "lock_count unchanged when already counted");
        }

        {
            HitProgress hp;
            hp.update(true, 0.5F);              // P = 9.0, 未锁定
            require(hp.note_official_countered(), "missed lock -> corrected");
            require(hp.is_locked(), "correction enters locked state");
            require(hp.lock_count() == 1, "correction increments lock_count");
            require(hp.stage() == 1, "correction advances stage");
            require(hp.difficulty() == 2, "correction advances difficulty");
            require(hp.progress() == 0.0F, "correction resets P");
            require_near(hp.lock_remaining_s(), 45.0F, 0.5F, "correction starts 45s lock");
            hp.update(false, 45.1F);
            require(!hp.is_locked(), "corrected lock expires after 45s");
        }

        {
            HitProgress hp;
            for (int lock = 1; lock <= 5; ++lock) {
                drive_until_locked(hp, lock);
                expire_lock(hp);
            }
            require(hp.is_exhausted(), "precondition: exhausted");
            require(!hp.note_official_countered(), "exhausted -> no correction");
            require(hp.lock_count() == 5, "lock_count capped at 5");
        }
```

- [ ] **Step 2: 构建确认失败**

Run: `./.script/build-laser && ctest --test-dir build -R hit_progress_test --output-on-failure`
Expected: 编译失败（reset/note_official_countered 未声明）。

- [ ] **Step 3: 实现头文件声明（hit_progress.hpp public 区）**

```cpp
    /// 整局归零（开局调用）：P/t/n/P0/stage/difficulty/lock_count/锁定/耗尽 全部复位
    void reset();
    /// 裁判系统 0x020C 官方反制状态上升沿校核。返回 true 表示本地漏检、补计一次锁定。
    [[nodiscard]] auto note_official_countered() -> bool;
```

- [ ] **Step 4: 实现（hit_progress.cpp，update() 之前插入）**

```cpp
void HitProgress::reset() {
    p_ = 0.0F;
    t_ = 0.0F;
    n_ = 0;
    p0_ = 50.0F;
    stage_ = 0;
    difficulty_ = 1;
    lock_count_ = 0;
    locked_ = false;
    exhausted_ = false;
    hitting_ = false;
    lock_timer_ = 0.0F;
}

auto HitProgress::note_official_countered() -> bool {
    if (exhausted_ || locked_)
        return false;
    if (lock_count_ >= kMaxLocks) {
        exhausted_ = true;
        return false;
    }
    trigger_lock();
    return true;
}
```

- [ ] **Step 5: 构建并确认通过**

Run: `./.script/build-laser && ctest --test-dir build -R hit_progress_test --output-on-failure`
Expected: PASS。

- [ ] **Step 6: 提交**

```bash
git add src/tracking/hit_progress.hpp src/tracking/hit_progress.cpp tests/tracking/hit_progress_test.cpp
git commit -m "feat(tracking): per-match hit progress reset and referee lock verification"
```

---

### Task 5: ControlLoop 集成与透出

**Files:**
- Modify: `src/runtime/control_loop.hpp`（成员 `RefereeLink referee_link_;`）
- Modify: `src/runtime/control_loop.cpp`（poll/门控/校核/快照）
- Modify: `src/runtime/overlay_renderer.hpp` / `.cpp`（OverlayRenderContext.referee + 绘制）
- Modify: `src/core/debug_renderer.hpp` / `.cpp`（`draw_referee_status`）
- Modify: `src/bridges/ros_bridge.hpp` / `.cpp`（referee 话题发布）
- Test: `tests/runtime/overlay_renderer_test.cpp`

**Interfaces:**
- Consumes: Task 2/3 `RefereeLink`、Task 4 `HitProgress::reset/note_official_countered`、Task 1 `RefereeConfig`。

- [ ] **Step 1: 写失败测试（overlay_renderer_test.cpp 追加，末尾 return 0 之前）**

```cpp
        {
            ControlLoopFrame gated_frame;
            gated_frame.display = cv::Mat::zeros(240, 320, CV_8UC3);
            const auto gated_before = gated_frame.display.clone();
            rmcs_laser_guidance::RefereeSnapshot referee;
            referee.signal_available = true;
            referee.guidance_gated = true;
            referee.game_progress = 1;
            referee.match_elapsed_s = -1;
            renderer.render(
                gated_frame,
                OverlayRenderContext{
                    .guidance_enabled = true,
                    .guidance_ready = true,
                    .hit_progress = nullptr,
                    .streaming_active = false,
                    .recording_active = false,
                    .enemy_color = rmcs_laser_guidance::EnemyColor::red,
                    .using_tensorrt = false,
                    .referee = &referee,
                });
            require(
                cv::norm(gated_frame.display, gated_before, cv::NORM_INF) > 0.0,
                "referee line should be drawn when gated");
        }
```

- [ ] **Step 2: 构建确认失败**

Run: `./.script/build-laser && ctest --test-dir build -R overlay_renderer_test --output-on-failure`
Expected: 编译失败（RefereeSnapshot 不存在，若 Task 3 已提前添加则 `OverlayRenderContext.referee` 字段缺失）。

- [ ] **Step 3: RefereeSnapshot 结构**

已在 Task 3 Step 3 加入 `include/laser_guidance/runtime.hpp`。本任务无需重复；若 Task 3 被跳过（不应发生），补加同款结构体与 `RuntimeSnapshot::referee` 字段。

- [ ] **Step 4: ControlLoop 头文件**

`src/runtime/control_loop.hpp` 加入 `#include "runtime/referee_link.hpp"` 与成员（`HitProgress hit_progress_;` 之后）：

```cpp
    RefereeLink referee_link_{RefereeConfig{}};
```

**注意：** 成员初始化顺序必须与声明顺序一致，`RefereeLink` 需在构造体中显式初始化：

`src/runtime/control_loop.cpp` 构造列表（`hit_progress_()` 之后）：

```cpp
    , referee_link_(config.referee)
```

- [ ] **Step 5: 主循环门控（control_loop.cpp run_loop，guidance 执行块之前）**

在帧读取循环内、`frame.track = select_target_track(...)` 之后插入：

```cpp
        referee_link_.poll();
        if (referee_link_.consume_match_started()) {
            hit_progress_.reset();
            std::println(stderr, "[REFEREE] match started, hit progress reset");
        }
        if (referee_link_.consume_countered_edge()) {
            if (hit_progress_.note_official_countered()) {
                std::println(
                    stderr,
                    "[REFEREE] official countered missed locally, lock corrected to {}",
                    hit_progress_.lock_count());
            }
        }
```

将 guidance 执行块的条件由 `if (guidance != nullptr) {` 改为：

```cpp
        const bool guidance_allowed = referee_link_.guidance_allowed();
        if (guidance != nullptr && guidance_allowed) {
```

块内部（set_offset / execute / diag 逻辑）保持不变。

- [ ] **Step 6: update_hit_progress 门控（control_loop.cpp）**

在 `const auto now = Clock::now();` 之后、`float dt_s` 计算之前插入：

```cpp
    if (!referee_link_.hit_progress_allowed()) {
        last_hit_progress_time_ = now;
        return;
    }
```

- [ ] **Step 7: 快照填充（control_loop.cpp assemble_snapshot 或 update_status_locked 中）**

在 `assemble_snapshot` 组装 `RuntimeSnapshot` 处追加：

```cpp
    snapshot.referee = referee_link_.snapshot();
```

（若 `assemble_snapshot` 为 const 方法，`referee_link_.snapshot()` 本身 const，无需改动签名。）

- [ ] **Step 8: overlay 透出**

`src/runtime/overlay_renderer.hpp` 的 `OverlayRenderContext` 追加：

```cpp
    const RefereeSnapshot* referee = nullptr;
```

`src/core/debug_renderer.hpp` 追加声明：

```cpp
auto draw_referee_status(cv::Mat& image, const RefereeSnapshot& referee) -> void;
```

`src/core/debug_renderer.cpp` 实现（`draw_hit_progress` 之后）：

```cpp
auto draw_referee_status(cv::Mat& image, const RefereeSnapshot& referee) -> void {
    std::string line;
    if (!referee.signal_available) {
        line = "REF: no signal (ungated)";
    } else {
        line = std::format(
            "REF: pg={} t={}s {}{}{}",
            static_cast<int>(referee.game_progress),
            referee.match_elapsed_s,
            referee.guidance_gated ? "[GATED] " : "",
            referee.signal_stale ? "[STALE] " : "",
            referee.official_aerial_countered ? "[CT]" : "");
    }
    cv::putText(
        image, line, {10, 130}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {200, 200, 200}, 1);
}
```

`src/runtime/overlay_renderer.cpp` 的 `render()` 末尾（`draw_status_bar` 之前）追加：

```cpp
    if (context.referee != nullptr) {
        draw_referee_status(frame.display, *context.referee);
    }
```

- [ ] **Step 9: ROS 透出（ros_bridge）**

`src/bridges/ros_bridge.hpp` 的 `Impl` 前追加私有发布方法声明（public 区）：

```cpp
    auto publish_referee(const RefereeSnapshot& referee) -> void;
```

`src/bridges/ros_bridge.cpp`：
- include 追加 `#include <std_msgs/msg/float64_multi_array.hpp>`
- `Impl` 加成员 `rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr referee_pub;`
- `init()` 中创建：

```cpp
            referee_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
                std::string(kTopicPrefix) + "referee", 10);
```

- 实现：

```cpp
auto RosBridge::publish_referee(const RefereeSnapshot& referee) -> void {
    if (!impl_->initialized || !impl_->referee_pub) return;
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        static_cast<double>(referee.game_progress),
        static_cast<double>(referee.match_elapsed_s),
        referee.guidance_gated ? 1.0 : 0.0,
        referee.official_aerial_targeted ? 1.0 : 0.0,
        referee.official_aerial_countered ? 1.0 : 0.0,
    };
    impl_->referee_pub->publish(msg);
}
```

- `publish_snapshot` 末尾追加：

```cpp
    impl_->publish_referee(snapshot.referee);
```

- [ ] **Step 10: 构建 + 全部相关测试**

Run: `./.script/build-laser && ctest --test-dir build -R "referee_link_test|hit_progress_test|overlay_renderer_test|config_test" --output-on-failure`
Expected: 全部 PASS。

- [ ] **Step 11: 提交**

```bash
git add src/runtime/control_loop.hpp src/runtime/control_loop.cpp src/runtime/overlay_renderer.hpp src/runtime/overlay_renderer.cpp src/core/debug_renderer.hpp src/core/debug_renderer.cpp src/bridges/ros_bridge.hpp src/bridges/ros_bridge.cpp tests/runtime/overlay_renderer_test.cpp
git commit -m "feat(runtime): gate guidance and hit progress on referee match window"
```

---

### Task 6: mock 裁判发布器 tools/referee_sim

**Files:**
- Create: `tools/referee_sim.cpp`

**Interfaces:**
- Consumes: 无（仅 libzmq + nlohmann_json，经 `rmcs_laser_guidance_core` 传递可用）。
- Produces: `tool_referee_sim` 可执行（CMake GLOB 自动注册）。

- [ ] **Step 1: 实现 `tools/referee_sim.cpp`**

```cpp
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <print>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

namespace {

using namespace std::chrono_literals;

struct Options {
    std::string bind = "tcp://*:5558";
    int interval_ms = 1000;
    int prep_s = 300;        // progress 1
    int selfcheck_s = 15;    // progress 2
    int countdown_s = 5;     // progress 3
    int match_s = 420;       // progress 4
    int settle_s = 10;       // progress 5
    int counter_at_s = 60;   // 比赛开始后第几秒触发 0x020C 反制
    int counter_duration_s = 45;
    bool loop = false;
    bool show_help = false;
};

auto print_help() -> void {
    std::println(
        "usage: tool_referee_sim [--bind tcp://*:5558] [--interval-ms 1000] "
        "[--prep-s 300] [--selfcheck-s 15] [--countdown-s 5] [--match-s 420] "
        "[--settle-s 10] [--counter-at-s 60] [--counter-duration-s 45] [--loop]");
}

auto parse_options(int argc, char** argv) -> Options {
    Options opt;
    const auto need_value = [&](int& i, const char* flag) -> std::string {
        if (i + 1 >= argc) {
            std::println(stderr, "error: {} requires a value", flag);
            std::exit(1);
        }
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--bind") opt.bind = need_value(i, "--bind");
        else if (arg == "--interval-ms") opt.interval_ms = std::atoi(need_value(i, "--interval-ms").c_str());
        else if (arg == "--prep-s") opt.prep_s = std::atoi(need_value(i, "--prep-s").c_str());
        else if (arg == "--selfcheck-s") opt.selfcheck_s = std::atoi(need_value(i, "--selfcheck-s").c_str());
        else if (arg == "--countdown-s") opt.countdown_s = std::atoi(need_value(i, "--countdown-s").c_str());
        else if (arg == "--match-s") opt.match_s = std::atoi(need_value(i, "--match-s").c_str());
        else if (arg == "--settle-s") opt.settle_s = std::atoi(need_value(i, "--settle-s").c_str());
        else if (arg == "--counter-at-s") opt.counter_at_s = std::atoi(need_value(i, "--counter-at-s").c_str());
        else if (arg == "--counter-duration-s") opt.counter_duration_s = std::atoi(need_value(i, "--counter-duration-s").c_str());
        else if (arg == "--loop") opt.loop = true;
        else if (arg == "--help" || arg == "-h") opt.show_help = true;
        else {
            std::println(stderr, "error: unknown option {}", arg);
            std::exit(1);
        }
    }
    return opt;
}

auto game_state_json(int progress, int remain_s) -> std::string {
    nlohmann::json j;
    j["cmd_id"] = 0x0001;
    j["game_type"] = 1;
    j["game_progress"] = progress;
    j["stage_remain_time"] = remain_s;
    j["sync_timestamp"] = 0;
    return j.dump();
}

auto mark_json(bool targeted, bool countered) -> std::string {
    nlohmann::json j;
    j["cmd_id"] = 0x020C;
    j["opponent_hero_vulnerable"] = false;
    j["opponent_engineer_vulnerable"] = false;
    j["opponent_infantry_3_vulnerable"] = false;
    j["opponent_infantry_4_vulnerable"] = false;
    j["opponent_aerial_marked"] = false;
    j["opponent_sentry_vulnerable"] = false;
    j["ally_hero_marked"] = false;
    j["ally_engineer_marked"] = false;
    j["ally_infantry_3_marked"] = false;
    j["ally_infantry_4_marked"] = false;
    j["ally_aerial_marked"] = false;
    j["ally_sentry_marked"] = false;
    j["opponent_aerial_targeted"] = targeted;
    j["opponent_aerial_countered"] = countered;
    j["ally_aerial_targeted"] = false;
    j["ally_aerial_countered"] = false;
    return j.dump();
}

} // namespace

int main(int argc, char** argv) {
    const auto opt = parse_options(argc, argv);
    if (opt.show_help) {
        print_help();
        return 0;
    }

    zmq::context_t ctx(1);
    zmq::socket_t pub(ctx, zmq::socket_type::pub);
    pub.bind(opt.bind);
    std::println(stderr, "referee_sim: PUB on {}", opt.bind);

    const auto publish = [&](const std::string& payload) { pub.send(zmq::buffer(payload), zmq::send_flags::dontwait); };

    const auto phases = std::vector<std::pair<int, int>>{
        {1, opt.prep_s}, {2, opt.selfcheck_s}, {3, opt.countdown_s}, {4, opt.match_s}, {5, opt.settle_s}};

    do {
        for (const auto& [progress, duration_s] : phases) {
            for (int s = 0; s < duration_s; ++s) {
                const int remain = progress == 4 ? opt.match_s - s : duration_s - s;
                publish(game_state_json(progress, remain));
                if (progress == 4) {
                    const bool in_counter =
                        s >= opt.counter_at_s && s < opt.counter_at_s + opt.counter_duration_s;
                    publish(mark_json(in_counter, in_counter));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(opt.interval_ms));
            }
        }
    } while (opt.loop);

    return 0;
}
```

（若编译缺 `#include <vector>`，在 include 区补充。）

- [ ] **Step 2: 构建**

Run: `./.script/build-laser`
Expected: `tool_referee_sim` 构建成功（`build/tool_referee_sim`，二进制平铺在 build/ 根）。

- [ ] **Step 3: 冒烟验证**

Run: `./build/tool_referee_sim --help`
Expected: 打印 usage，退出码 0。

- [ ] **Step 4: 提交**

```bash
git add tools/referee_sim.cpp
git commit -m "feat(tools): referee_sim mock publisher for local gating verification"
```

---

### Task 7: 文档同步

**Files:**
- Modify: `docs/architecture.md`
- Modify: `docs/runtime_operations.md`
- Modify: `AGENTS.md`、`docs/AGENTS.md`、`README.md`

**Interfaces:**
- Consumes: 全部前序任务的最终形态。

- [ ] **Step 1: docs/architecture.md 补充 RefereeLink 组件**

在运行组件描述部分新增小节（内容取自 spec「数据流」一节，简述 RefereeLink 职责、数据源 0x0001/0x020C、门控与校核语义、no_signal 降级行为）。

- [ ] **Step 2: docs/runtime_operations.md 补充配置与调试**

新增 `referee` 配置节说明 + `tool_referee_sim` 本地验证方法（起 mock → 跑 competition → 观察 overlay REF 行与门控行为）。

- [ ] **Step 3: AGENTS.md / docs/AGENTS.md / README.md 运行时状态行**

在 AGENTS.md 运行时状态列表中追加一行（与现有条目风格一致）：

```
- `RefereeLink` 订阅裁判系统 ZMQ（0x0001/0x020C），门控引导与 HitProgress；无信号时退化为不门控；`tools/referee_sim` 提供本地 mock。
```

- [ ] **Step 4: 提交**

```bash
git add docs/architecture.md docs/runtime_operations.md AGENTS.md docs/AGENTS.md README.md
git commit -m "docs: referee gating configuration and verification tooling"
```

---

### Task 8: 取消引导门控（全程引导 + 赛外本地计算）

**用户裁决（2026-08-03）：** 引导执行与 HitProgress 本地计算永不门控；`game_progress` 只用于每局开局 HitProgress 重置、0x020C 校核、相机阶段切换、看门狗与显示。本任务撤销 Task 5 的门控接线，保留状态机/边沿/校核/看门狗。

**Files:**
- Modify: `src/runtime/referee_link.hpp` / `referee_link.cpp`
- Modify: `include/laser_guidance/runtime.hpp`
- Modify: `src/runtime/control_loop.cpp`
- Modify: `src/core/debug_renderer.cpp`
- Modify: `src/bridges/ros_bridge.cpp`
- Modify: `tests/runtime/referee_link_test.cpp`
- Modify: `tests/runtime/overlay_renderer_test.cpp`
- Modify: 文档（AGENTS.md、docs/AGENTS.md、docs/architecture.md、docs/runtime_operations.md、README.md）

**Interfaces:**
- Consumes: Task 2/3 的 `RefereeWindow`（保留 update/consume_match_started/match_elapsed_s/expire_if_over/in_window/signal_available）、`RefereeLink`（保留 poll/consume_match_started/consume_countered_edge/snapshot）、Task 4 的 HitProgress reset/note_official_countered。
- Produces: 删除 `RefereeWindow::allowed()`、`RefereeLink::guidance_allowed()`/`hit_progress_allowed()`；`RefereeSnapshot` 删除 `guidance_gated`；ROS 话题 4 元素数组。

- [ ] **Step 1: 更新测试（referee_link_test.cpp，先改后实现）**

删除以下断言（依赖已删 API）：
- 状态机区：`w.allowed()` 的全部调用（"no signal -> ungated"、准备阶段、比赛进入、超时、断流、re-arm 用例）改为断言 `w.in_window()` + `w.signal_available()`：
  - `require(!w.signal_available(), "no signal flag");` 保持不变
  - 准备阶段：`require(!w.in_window(), "prep phase not in window");`
  - 4 进入：`require(w.in_window(), "match enters window");`
  - 5 退出：`require(!w.in_window(), "settle exits window");`
  - 420s 超时：`require(!w.in_window(), "exits at 420s despite progress still 4");`
  - 断流：`require(w.in_window(), "stale signal keeps window");` / `require(!w.in_window(), "stale signal still expires at 420s");`
  - timed_out 用例：`!w.in_window()` 与 `!w.consume_match_started()` 断言保留，`allowed()` 调用替换
- RefereeLink 禁用用例：`require(link.guidance_allowed(), ...)` 删除，改为 `require(!link.snapshot().signal_available, "disabled link no signal");`

- [ ] **Step 2: 构建确认失败**

Run: `./.script/build-laser && ctest --test-dir build -R referee_link_test --output-on-failure`
Expected: FAIL（allowed/guidance_allowed 未定义）。

- [ ] **Step 3: 实现 API 删除（referee_link.hpp/.cpp）**

`RefereeWindow` 删除 `allowed()` 声明与实现；`RefereeLink` 删除 `guidance_allowed()`/`hit_progress_allowed()` 声明与实现（`window_.allowed()` 的实现行一并删除）。其余保留。

- [ ] **Step 4: RefereeSnapshot 删除 guidance_gated（include/laser_guidance/runtime.hpp）**

删除 `bool guidance_gated = false;` 行及注释。

- [ ] **Step 5: ControlLoop 撤销门控（src/runtime/control_loop.cpp）**

- 删除 `const bool guidance_allowed = referee_link_.guidance_allowed();` 行，`if (guidance != nullptr && guidance_allowed) {` 还原为 `if (guidance != nullptr) {`
- `update_hit_progress` 中删除：
```cpp
    if (!referee_link_.hit_progress_allowed()) {
        last_hit_progress_time_ = now;
        return;
    }
```
（`const auto now = Clock::now();` 之后的 dt 计算恢复原样，HitProgress 赛外照常本地计算）

- [ ] **Step 6: overlay 去掉 [GATED]（src/core/debug_renderer.cpp）**

`draw_referee_status` 改为：

```cpp
auto draw_referee_status(cv::Mat& image, const RefereeSnapshot& referee) -> void {
    std::string line;
    if (!referee.signal_available) {
        line = "REF: no signal (local calc)";
    } else {
        line = std::format(
            "REF: pg={} t={}s {}{}",
            static_cast<int>(referee.game_progress),
            referee.match_elapsed_s,
            referee.signal_stale ? "[STALE] " : "",
            referee.official_aerial_countered ? "[CT]" : "");
    }
    cv::putText(
        image, line, {10, 130}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {200, 200, 200}, 1);
}
```

- [ ] **Step 7: ROS 4 元素数组（src/bridges/ros_bridge.cpp）**

`publish_referee` 的 `msg.data` 改为：

```cpp
    msg.data = {
        static_cast<double>(referee.game_progress),
        static_cast<double>(referee.match_elapsed_s),
        referee.official_aerial_targeted ? 1.0 : 0.0,
        referee.official_aerial_countered ? 1.0 : 0.0,
    };
```

- [ ] **Step 8: overlay 测试去掉 guidance_gated（tests/runtime/overlay_renderer_test.cpp）**

删除 `referee.guidance_gated = true;` 行；其余（`.referee = &referee` 传参与 norm 断言）保留。

- [ ] **Step 9: 构建 + 全部相关测试**

Run: `./.script/build-laser && ctest --test-dir build -R "referee_link_test|hit_progress_test|overlay_renderer_test|config_test" --output-on-failure`
Expected: 全部 PASS；全量 `ctest --test-dir build --output-on-failure` 27/27。

- [ ] **Step 10: 文档同步**

- AGENTS.md / docs/AGENTS.md 的状态行改为：`RefereeLink` 订阅裁判系统 ZMQ（0x0001/0x020C），全程引导；game_progress 用于每局 HitProgress 重置与 0x020C 锁定校核，无信号/赛外退化纯本地计算；`tools/referee_sim` 提供本地 mock。
- docs/architecture.md：RefereeLink 小节改为"全程引导，不门控；用于每局重置/校核/相机阶段切换/看门狗"。
- docs/runtime_operations.md：referee 节说明"引导永不门控；overlay REF 行显示 pg/计时/STALE/CT；referee_sim 用法不变"。
- README.md：如提及门控，同步改为全程引导表述。

- [ ] **Step 11: 提交**

```bash
git add src/runtime/referee_link.hpp src/runtime/referee_link.cpp include/laser_guidance/runtime.hpp src/runtime/control_loop.cpp src/core/debug_renderer.cpp src/bridges/ros_bridge.cpp tests/runtime/referee_link_test.cpp tests/runtime/overlay_renderer_test.cpp AGENTS.md docs/AGENTS.md docs/architecture.md docs/runtime_operations.md README.md
git commit -m "feat(runtime): always-on guidance, referee data for per-match reset and lock verification only"
```

---

## 自审记录

- **Spec 覆盖**：§1 RefereeLink（Task 2/3）、§2 ControlLoop 集成（Task 5 门控 + Task 8 撤销门控改全程引导）、§3 HitProgress（Task 4）、§4 配置（Task 1 + Task 3 Step 4a 的 signal_timeout_s）、§5 快照/overlay/ROS（Task 3 Step 3 + Task 5 + Task 8 移除 guidance_gated）、§6 mock 工具（Task 6）、§7 测试（各任务 Step 1）、§8 排除项（无任务，符合 YAGNI）。看门狗（断流告警）在 Task 3/5 落地，相机难度阶段切换由现有 `maybe_switch_hik_profile` 读取已校核同步的 `hit_progress_.difficulty()` 天然覆盖。
- **类型一致性**：`RefereeConfig`（Task 1）→ `RefereeLink(RefereeConfig)`（Task 3）；`RefereeSnapshot` 在 Task 3 Step 3 统一定义，Task 5 消费、Task 8 移除 `guidance_gated`；`consume_match_started/consume_countered_edge` 三处使用签名一致；Task 8 删除 `allowed()/guidance_allowed()/hit_progress_allowed()` 时同步更新其全部调用点与测试。
- **无占位**：所有代码块为完整实现；Task 5 Step 5 的 guidance 块仅改条件、内部逻辑不变。
- **头文件卫生**：`RefereeLink` 用 Impl PIMPL（referee_link.cpp 内持有 zmq::context_t/socket_t），公共头不暴露 libzmq，与 `ZmqSender`/`RosBridge` 的既有 PIMPL 风格一致。
- **构建注意**：`src/runtime/referee_link.cpp` 手动加入 `laser_guidance_runtime_SOURCES`（CMakeLists.txt:221-231）；`referee_link_` 成员在构造列表按声明顺序初始化。
