# Modern C++ Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Harden the control plane, error model, and realtime lifecycle without changing FIFO wire protocol, ROS/ZMQ/UDP layouts, or HitProgress rules.

**Architecture:** Surgical vertical slices (Approach A). Each PR is independently mergeable. Command surface becomes a closed `std::variant`; failures become `Error{kind,message}`; `LatestValue` shutdown stops throwing; then boundary hygiene (Hik header sink, explicit CMake sources).

**Tech Stack:** C++23, CMake, existing `tests/*_test.cpp` harness (`require` / `require_contains` in `tests/test_utils.hpp`), `build-laser` / `ctest --test-dir build`.

**Spec:** `docs/superpowers/specs/2026-07-25-modern-cpp-refactor-design.md`

## Global Constraints

- FIFO text protocol unchanged: `stream on|off`, `record on|off`, `enemy red|blue|auto`, `backend onnx|tensorrt`, `ekf on|off`, `quit`.
- Do not reintroduce `ControlLoopBuilder`, `ModeHooks`, `GuidanceController`, `ICaptureDevice`, service locators.
- Do not change ROS2 topic contracts, ZMQ `cmd_id`, UDP binary layout, or HitProgress formulas.
- Hardware degrade semantics stay: missing camera/FT4222 must not kill the whole process path that already degrades today.
- One concept per commit/PR; no mixed feature work (Hik profiles, recording QP, etc.).
- Prefer TDD: update/add failing tests first, then minimal implementation.
- Commits only when the user explicitly asks to commit (default: implement + verify; leave staging to user unless requested).

## File map (create / modify)

| Path | Role |
|------|------|
| `include/laser_guidance/runtime.hpp` | Command ADT + factories live here / nearby |
| `include/laser_guidance/error.hpp` | **Create** — `ErrorKind`, `Error`, helpers |
| `src/runtime/runtime_command.cpp` | `runtime_command::set_*` factories |
| `src/runtime/error.cpp` | **Create** if helpers need a TU (`format_error`) |
| `src/runtime/control_loop.cpp` | `submit_command` via `std::visit` |
| `src/bridges/bridges.cpp` | `parse_command` factories + later Error |
| `include/laser_guidance/bridges.hpp` | Signatures for FIFO expected types |
| `tests/runtime/fifo_control_test.cpp` | Variant + Error assertions |
| `src/tracking/freshness_queue.hpp` | Non-throwing shutdown |
| `tests/tracking/freshness_queue_test.cpp` | Shutdown returns nullopt |
| `src/capture/capture_device.*` | Consumers of LatestValue; later Hik sink |
| `src/runtime/perception_runner.*` | LatestValue consumer; optional jthread |
| `src/runtime/guidance_session.*` | Error migration (phase 2b) |
| `src/io/ft4222_spi.*` | Error migration (phase 2b) |
| `CMakeLists.txt` | Explicit sources for runtime lib (phase 4c) |
| `docs/design-patterns.md` | Document variant command + Error after merges |

---

### Task 1: Baseline guardrails

**Files:**
- Modify: none (read-only verify)
- Optional note: `docs/todolist.md`

**Interfaces:**
- Consumes: existing build/test entrypoints
- Produces: known-green baseline before refactor commits

- [ ] **Step 1: Record current branch and dirty state**

```bash
cd /home/yukikaze/Documents/workspace/laser_guidance
git status -sb
git rev-parse --short HEAD
```

Expected: note any unrelated dirty files (`config/camera_calib.yaml`, etc.) and **do not** include them in refactor commits.

- [ ] **Step 2: Build**

```bash
build-laser
# or: cmake --build build --parallel
```

Expected: exit 0.

- [ ] **Step 3: Run focused tests that refactor will touch**

```bash
ctest --test-dir build -R 'fifo_control_test|freshness_queue_test' --output-on-failure
```

Expected: both PASS.

- [ ] **Step 4: Optionally run full suite**

```bash
ctest --test-dir build --output-on-failure
```

Expected: PASS, or document pre-existing failures with names (do not “fix” unrelated failures in refactor PRs).

---

### Task 2: RuntimeCommand as `std::variant` (Phase 1)

**Files:**
- Modify: `include/laser_guidance/runtime.hpp`
- Modify: `src/runtime/runtime_command.cpp`
- Modify: `src/bridges/bridges.cpp`
- Modify: `src/runtime/control_loop.cpp` (`submit_command` ~155–196)
- Modify: `tests/runtime/fifo_control_test.cpp`
- Verify no remaining `RuntimeCommand::set_` / `.type` on commands: `rg 'RuntimeCommand::|RuntimeCommandType|command->type|command\\.type' --glob '*.{hpp,cpp}'`

**Interfaces:**
- Consumes: existing FIFO strings; existing `EnemyColor`, `RuntimeBackend`
- Produces:

```cpp
namespace rmcs_laser_guidance {

struct CmdSetStreaming { bool enabled = false; };
struct CmdSetRecording { bool enabled = false; };
struct CmdSetEnemyColor { EnemyColor enemy_color = EnemyColor::auto_select; };
struct CmdSetBackend { RuntimeBackend backend = RuntimeBackend::onnx; };
struct CmdSetEkf { bool enabled = false; };
struct CmdShutdown {};

using RuntimeCommand = std::variant<
    CmdSetStreaming,
    CmdSetRecording,
    CmdSetEnemyColor,
    CmdSetBackend,
    CmdSetEkf,
    CmdShutdown>;

namespace runtime_command {
auto set_streaming(bool enabled) -> RuntimeCommand;
auto set_recording(bool enabled) -> RuntimeCommand;
auto set_enemy_color(EnemyColor color) -> RuntimeCommand;
auto set_backend(RuntimeBackend backend) -> RuntimeCommand;
auto set_ekf(bool enabled) -> RuntimeCommand;
auto shutdown() -> RuntimeCommand;
} // namespace runtime_command

} // namespace rmcs_laser_guidance
```

- Remove: `enum class RuntimeCommandType`, old `struct RuntimeCommand { type, enabled, ... }`, and `RuntimeCommand::set_*` static methods.

- [ ] **Step 1: Rewrite fifo tests to the new shape (will fail to compile until implementation)**

Replace type/field checks with variant checks. Example blocks:

```cpp
#include <variant>

// stream on
{
    const auto command = FifoControlServer::parse_command("stream on\n");
    require(command.has_value(), "stream on should parse");
    const auto* streaming = std::get_if<rmcs_laser_guidance::CmdSetStreaming>(&*command);
    require(streaming != nullptr, "stream on type mismatch");
    require(streaming->enabled, "stream on enabled mismatch");
}

// record off
{
    const auto command = FifoControlServer::parse_command("record off");
    require(command.has_value(), "record off should parse");
    const auto* recording = std::get_if<rmcs_laser_guidance::CmdSetRecording>(&*command);
    require(recording != nullptr, "record off type mismatch");
    require(!recording->enabled, "record off enabled mismatch");
}

// enemy blue
{
    const auto command = FifoControlServer::parse_command("enemy blue");
    require(command.has_value(), "enemy blue should parse");
    const auto* enemy = std::get_if<rmcs_laser_guidance::CmdSetEnemyColor>(&*command);
    require(enemy != nullptr, "enemy blue type mismatch");
    require(enemy->enemy_color == EnemyColor::blue, "enemy blue value mismatch");
}

// backend tensorrt
{
    const auto command = FifoControlServer::parse_command("backend tensorrt");
    require(command.has_value(), "backend tensorrt should parse");
    const auto* backend = std::get_if<rmcs_laser_guidance::CmdSetBackend>(&*command);
    require(backend != nullptr, "backend tensorrt type mismatch");
    require(backend->backend == RuntimeBackend::tensorrt, "backend tensorrt value mismatch");
}

// ekf off
{
    const auto command = FifoControlServer::parse_command("ekf off");
    require(command.has_value(), "ekf off should parse");
    const auto* ekf = std::get_if<rmcs_laser_guidance::CmdSetEkf>(&*command);
    require(ekf != nullptr, "ekf off type mismatch");
    require(!ekf->enabled, "ekf off enabled mismatch");
}

// quit
{
    const auto command = FifoControlServer::parse_command("quit");
    require(command.has_value(), "quit should parse");
    require(std::holds_alternative<rmcs_laser_guidance::CmdShutdown>(*command),
        "quit type mismatch");
}
```

Also fix multi-line / partial / recovery sections that use `command->type`, `command->enabled`, `command->backend`, `command->enemy_color` the same way.

Remove `using ... RuntimeCommandType`.

- [ ] **Step 2: Build only the test target to confirm failure mode**

```bash
cmake --build build --target fifo_control_test --parallel
```

Expected: FAIL compile (missing types / old members).

- [ ] **Step 3: Update `include/laser_guidance/runtime.hpp`**

1. Add `#include <variant>`.
2. Delete `enum class RuntimeCommandType` and the old `struct RuntimeCommand` with factories.
3. Insert the payload structs, `using RuntimeCommand = std::variant<...>`, and `namespace runtime_command { ... }` declarations as in **Interfaces** above.
4. Leave `CompetitionRuntime`, snapshots, detections unchanged.

- [ ] **Step 4: Rewrite `src/runtime/runtime_command.cpp`**

```cpp
#include "laser_guidance/runtime.hpp"

namespace rmcs_laser_guidance::runtime_command {

auto set_streaming(const bool enabled) -> RuntimeCommand {
    return CmdSetStreaming{.enabled = enabled};
}
auto set_recording(const bool enabled) -> RuntimeCommand {
    return CmdSetRecording{.enabled = enabled};
}
auto set_enemy_color(const EnemyColor color) -> RuntimeCommand {
    return CmdSetEnemyColor{.enemy_color = color};
}
auto set_backend(const RuntimeBackend backend) -> RuntimeCommand {
    return CmdSetBackend{.backend = backend};
}
auto set_ekf(const bool enabled) -> RuntimeCommand {
    return CmdSetEkf{.enabled = enabled};
}
auto shutdown() -> RuntimeCommand { return CmdShutdown{}; }

} // namespace rmcs_laser_guidance::runtime_command
```

- [ ] **Step 5: Update `FifoControlServer::parse_command` in `src/bridges/bridges.cpp`**

Replace every `RuntimeCommand::set_*` / `RuntimeCommand::shutdown()` with `runtime_command::set_*` / `runtime_command::shutdown()`. FIFO string matching stays identical.

- [ ] **Step 6: Rewrite `ControlLoop::submit_command` with exhaustive visit**

In `src/runtime/control_loop.cpp`, replace the switch on `command.type` with:

```cpp
auto ControlLoop::submit_command(const RuntimeCommand& command)
    -> std::expected<void, std::string> {
    // Backend switch may run outside the state lock (matches current order:
    // validate backend first, then update state).
    if (const auto* set_backend = std::get_if<CmdSetBackend>(&command)) {
        if (!perception_.has_backend(set_backend->backend)) {
            return std::unexpected("requested backend is not available");
        }
        if (!perception_.set_active_backend(set_backend->backend)) {
            return std::unexpected("failed to switch active backend");
        }
    }

    EnemyColor enemy_color = EnemyColor::auto_select;
    bool request_shutdown = false;
    {
        std::scoped_lock lock(state_mutex_);
        std::visit(
            [&](const auto& cmd) {
                using T = std::decay_t<decltype(cmd)>;
                if constexpr (std::is_same_v<T, CmdSetStreaming>) {
                    state_.streaming_requested = cmd.enabled && allows_streaming();
                } else if constexpr (std::is_same_v<T, CmdSetRecording>) {
                    state_.recording_requested = cmd.enabled && allows_recording();
                } else if constexpr (std::is_same_v<T, CmdSetEnemyColor>) {
                    state_.enemy_color = cmd.enemy_color;
                } else if constexpr (std::is_same_v<T, CmdSetBackend>) {
                    // already applied
                } else if constexpr (std::is_same_v<T, CmdSetEkf>) {
                    state_.ekf_enabled = cmd.enabled;
                } else if constexpr (std::is_same_v<T, CmdShutdown>) {
                    state_.stop_requested = true;
                    request_shutdown = true;
                } else {
                    static_assert(sizeof(T) == 0, "non-exhaustive RuntimeCommand visit");
                }
            },
            command);
        enemy_color = state_.enemy_color;
        update_status_locked();
    }

    perception_.set_enemy_color(enemy_color);
    if (request_shutdown) {
        request_stop();
    }
    return {};
}
```

Add `#include <type_traits>` / `<variant>` if not already visible via headers.

- [ ] **Step 7: Grep for leftovers**

```bash
rg 'RuntimeCommandType|RuntimeCommand::set_|RuntimeCommand::shutdown|command->type|command\\.type' \
  --glob '*.{hpp,cpp}' -n
```

Expected: no matches in app/tests (except this plan/spec docs).

- [ ] **Step 8: Build and run fifo test**

```bash
cmake --build build --parallel
ctest --test-dir build -R fifo_control_test --output-on-failure
```

Expected: PASS.

- [ ] **Step 9: Commit (only if user requested commits)**

```bash
git add include/laser_guidance/runtime.hpp \
  src/runtime/runtime_command.cpp \
  src/bridges/bridges.cpp \
  src/runtime/control_loop.cpp \
  tests/runtime/fifo_control_test.cpp
git commit -m "refactor(runtime): RuntimeCommand as std::variant sum type"
```

---

### Task 3: Introduce `Error` / `ErrorKind` and migrate public control path (Phase 2a)

**Files:**
- Create: `include/laser_guidance/error.hpp`
- Create: `src/runtime/error.cpp` (optional; can be header-only helpers)
- Modify: `include/laser_guidance/runtime.hpp` (signatures using `Error`)
- Modify: `include/laser_guidance/bridges.hpp`
- Modify: `src/bridges/bridges.cpp`
- Modify: `src/runtime/control_loop.hpp` / `.cpp`
- Modify: `src/runtime/runtime.cpp`
- Modify: `tools/competition.cpp`, `tools/preview.cpp` (print `format_error`)
- Modify: `tests/runtime/fifo_control_test.cpp`
- Modify: `CMakeLists.txt` only if new `.cpp` must be listed (support GLOB already picks `src/runtime/runtime_command.cpp` via support sources — **if** `error.cpp` is under `src/runtime/`, either add to `laser_guidance_support_SOURCES` GLOB path `src/runtime/support.cpp` pattern or put helpers header-only; **prefer header-only helpers in `error.hpp`** to avoid CMake churn this task)

**Interfaces:**
- Produces:

```cpp
// include/laser_guidance/error.hpp
#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace rmcs_laser_guidance {

enum class ErrorKind : std::uint8_t {
    config,
    device,
    unavailable,
    timeout,
    internal,
};

struct Error {
    ErrorKind kind = ErrorKind::internal;
    std::string message{};
};

inline auto make_error(ErrorKind kind, std::string message) -> Error {
    return Error{.kind = kind, .message = std::move(message)};
}

inline auto error_kind_name(ErrorKind kind) -> std::string_view {
    switch (kind) {
    case ErrorKind::config: return "config";
    case ErrorKind::device: return "device";
    case ErrorKind::unavailable: return "unavailable";
    case ErrorKind::timeout: return "timeout";
    case ErrorKind::internal: return "internal";
    }
    return "internal";
}

inline auto format_error(const Error& error) -> std::string {
    return std::string("[") + std::string(error_kind_name(error.kind)) + "] " + error.message;
}

} // namespace rmcs_laser_guidance
```

Public signature migrations (this task only):

```cpp
// CompetitionRuntime / ControlLoop
auto start() -> std::expected<void, Error>;
auto run() -> std::expected<void, Error>;
auto submit_command(const RuntimeCommand& command) -> std::expected<void, Error>;

// FifoControlServer
auto start() -> std::expected<void, Error>;
static auto parse_command(std::string_view text) -> std::expected<RuntimeCommand, Error>;
```

**ErrorKind mapping for FIFO parse:**

| Condition | ErrorKind | message (keep existing text) |
|-----------|-----------|------------------------------|
| empty / blank | `config` | `empty FIFO command` |
| unsupported | `config` | `unsupported FIFO command: ...` |
| FIFO open fail | `device` | `failed to open FIFO: ...` |

**ErrorKind mapping for `submit_command`:**

| Condition | ErrorKind | message |
|-----------|-----------|---------|
| backend missing | `unavailable` | `requested backend is not available` |
| backend switch fail | `unavailable` | `failed to switch active backend` |

- [ ] **Step 1: Add `error.hpp` as above**

- [ ] **Step 2: Update fifo tests for `Error`**

```cpp
{
    const auto command = FifoControlServer::parse_command("backend invalid");
    require(!command.has_value(), "invalid backend should fail");
    require(command.error().kind == rmcs_laser_guidance::ErrorKind::config,
        "invalid backend should be config error");
    require_contains(command.error().message, "unsupported FIFO command",
        "invalid backend error");
}

{
    const auto command = FifoControlServer::parse_command("   ");
    require(!command.has_value(), "blank command should fail");
    require(command.error().kind == rmcs_laser_guidance::ErrorKind::config,
        "blank should be config error");
    require_contains(command.error().message, "empty FIFO command", "blank command error");
}
```

Where tools print errors, use `format_error(result.error())` or at least `result.error().message`.

- [ ] **Step 3: Change `parse_command` / `FifoControlServer::start` return types**

```cpp
if (normalized.empty()) {
    return std::unexpected(make_error(ErrorKind::config, "empty FIFO command"));
}
// ...
return std::unexpected(
    make_error(ErrorKind::config, "unsupported FIFO command: " + normalized));
```

For FIFO open failure:

```cpp
return std::unexpected(
    make_error(ErrorKind::device, "failed to open FIFO: " + fifo_path_.string()));
```

`last_error_` can store `format_error(err)` or `err.message` — keep `last_error()` as `const std::string&` for now (no public break on that accessor).

- [ ] **Step 4: Change `ControlLoop::submit_command` / `start` / `run` and facade**

Propagate `Error` through:

- `ControlLoop::start/run/submit_command`
- `CompetitionRuntime::start/run/submit_command` in `runtime.cpp`

For any remaining internal `std::expected<void, std::string>` used only inside ControlLoop init, either:

- leave internal as `string` and wrap at the public boundary with `make_error(ErrorKind::internal, msg)`, **or**
- convert call sites in the same task if cheap.

Minimum for Task 3: **public** methods use `Error`. Internal helpers may still use `string` if conversion is at the edge.

- [ ] **Step 5: Fix tools**

```cpp
if (auto result = runtime.submit_command(*command); !result) {
    std::println(stderr, "command failed: {}", format_error(result.error()));
}
```

Include `laser_guidance/error.hpp`.

- [ ] **Step 6: Build + test**

```bash
cmake --build build --parallel
ctest --test-dir build -R fifo_control_test --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Commit (if requested)**

```bash
git add include/laser_guidance/error.hpp include/laser_guidance/runtime.hpp \
  include/laser_guidance/bridges.hpp src/bridges/bridges.cpp \
  src/runtime/control_loop.hpp src/runtime/control_loop.cpp src/runtime/runtime.cpp \
  tools/competition.cpp tools/preview.cpp tests/runtime/fifo_control_test.cpp
git commit -m "refactor(error): introduce Error domain on public control path"
```

---

### Task 4: Migrate capture open/reconnect + guidance/FT4222 factories to `Error` (Phase 2b)

**Files:**
- Modify: `src/capture/capture_backend.hpp`
- Modify: `src/capture/capture_device.hpp` / `.cpp`
- Modify: `src/capture/v4l2_capture.hpp` / `.cpp` (if they return expected)
- Modify: `src/runtime/guidance_session.hpp` / `.cpp`
- Modify: `src/io/ft4222_spi.hpp` / `.cpp`
- Modify: `src/runtime/control_loop.cpp` (call sites that append error strings)
- Modify: any tests that assert `.error()` as string equality

**Interfaces:**
- Produces (examples):

```cpp
auto CaptureDevice::open() -> std::expected<CaptureFormat, Error>;
auto CaptureDevice::reconnect() -> std::expected<void, Error>;
auto CaptureDevice::read_frame() -> std::expected<Frame, Error>; // after Task 5, shutdown is not an error
static auto Ft4222Spi::open(Ft4222Config) -> std::expected<Ft4222Spi, Error>;
static auto GuidanceSession::create_auto(...) -> std::expected<GuidanceSession, Error>;
```

**Kind rules:**

| Failure | ErrorKind |
|---------|-----------|
| open/read/reconnect hardware | `device` |
| backend missing / wrong state | `unavailable` or `internal` (pick one; prefer `unavailable` for “not open”) |
| calib file missing / bad path | `config` |

- [ ] **Step 1: Change capture/guidance/SPI signatures to `expected<T, Error>`**

Wrap existing string messages with `make_error(ErrorKind::device, msg)` (or config as appropriate). Do not change retry timing in ControlLoop.

- [ ] **Step 2: Update ControlLoop logging**

```cpp
if (!open_result) {
    std::println(stderr, "camera init failed: {}, will retry...", format_error(open_result.error()));
    sync_last_error(format_error(open_result.error()));
}
```

- [ ] **Step 3: Build full tree + run tests**

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

- [ ] **Step 4: Commit (if requested)**

```bash
git commit -m "refactor(error): migrate capture/guidance/FT4222 to Error"
```

---

### Task 5: `LatestValue` non-throwing shutdown (Phase 3a)

**Files:**
- Modify: `src/tracking/freshness_queue.hpp`
- Modify: `tests/tracking/freshness_queue_test.cpp`
- Modify: `src/capture/capture_device.cpp` (`read_frame`, `capture_loop`)
- Modify: `src/runtime/perception_runner.cpp` (`submit`, `run`)

**Interfaces — target API:**

```cpp
template <typename T>
class LatestValue {
public:
    // Returns overwrite_count after push. On shutdown: returns current overwrite_count
    // and does not store the value (non-throwing).
    auto push(T value) -> std::size_t;

    // Blocking pop. nullopt means shutdown.
    auto pop() -> std::optional<T>;

    // Non-blocking. false if empty OR shutdown without value.
    auto try_pop(T& value) -> bool;

    auto shutdown() -> void;
    [[nodiscard]] auto overwrite_count() const -> std::size_t;
    [[nodiscard]] auto is_shutdown() const -> bool;
};
```

**Behavior locks:**

- `push` after `shutdown`: **do not throw**; discard value; return `overwrite_count_`.
- `pop` after `shutdown` with no value: return `std::nullopt` (unblocks waiters).
- Overwrite counting semantics for non-shutdown pushes unchanged.

- [ ] **Step 1: Update `freshness_queue_test.cpp` for non-throwing shutdown**

Replace the shutdown block that expects exceptions:

```cpp
{
    rmcs_laser_guidance::LatestValue<int> queue;
    std::promise<std::optional<int>> popped;
    auto future = popped.get_future();

    const std::jthread worker([&](const std::stop_token&) {
        popped.set_value(queue.pop());
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    queue.shutdown();
    const auto value = future.get();
    require(!value.has_value(), "shutdown should unblock pop with nullopt");

    int tmp = 0;
    require(!queue.try_pop(tmp), "shutdown empty queue should stay empty");

    // push after shutdown: non-throwing discard
    require(queue.push(7) == queue.overwrite_count(), "push after shutdown should not throw");
    require(!queue.try_pop(tmp), "push after shutdown must not store value");
}
```

Update the blocking-pop success case: `queue.pop()` returns `std::optional<int>`; assert `*value == 42` or `value == 42` after require has_value.

- [ ] **Step 2: Build test — expect fail until header updated**

```bash
cmake --build build --target freshness_queue_test --parallel
```

- [ ] **Step 3: Implement `LatestValue` changes in `freshness_queue.hpp`**

Key fragments:

```cpp
auto push(T value) -> std::size_t {
    std::scoped_lock lock(mutex_);
    if (shutdown_) {
        return overwrite_count_;
    }
    if (value_.has_value()) {
        ++overwrite_count_;
    }
    value_.emplace(std::move(value));
    cv_.notify_one();
    return overwrite_count_;
}

auto pop() -> std::optional<T> {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [this] { return shutdown_ || value_.has_value(); });
    if (!value_.has_value()) {
        return std::nullopt; // shutdown
    }
    T value = std::move(*value_);
    value_.reset();
    return value;
}

auto try_pop(T& value) -> bool {
    std::scoped_lock lock(mutex_);
    if (!value_.has_value()) {
        return false;
    }
    value = std::move(*value_);
    value_.reset();
    return true;
}

auto shutdown() -> void {
    std::scoped_lock lock(mutex_);
    shutdown_ = true;
    cv_.notify_all();
}

[[nodiscard]] auto is_shutdown() const -> bool {
    std::scoped_lock lock(mutex_);
    return shutdown_;
}
```

Remove `#include <stdexcept>` if unused. Keep `#include <optional>`.

- [ ] **Step 4: Update capture consumers**

`CaptureDevice::read_frame`:

```cpp
auto CaptureDevice::read_frame() -> std::expected<Frame, Error> { // or string if Task 4 not done
    if (!frame_queue_) {
        return std::unexpected(/* not open */);
    }
    auto item = frame_queue_->pop();
    if (!item.has_value()) {
        return std::unexpected(make_error(ErrorKind::unavailable, "capture stopped"));
    }
    return std::move(*item); // item is expected<Frame, ...> today
}
```

Note: today the queue is `LatestValue<std::expected<Frame, std::string>>`. After pop:

```cpp
auto item = frame_queue_->pop();
if (!item.has_value()) {
    return std::unexpected(make_error(ErrorKind::unavailable, "capture stopped"));
}
return std::move(*item); // still expected<Frame, string> or Error depending on Task 4
```

`capture_loop` push:

```cpp
frame_queue_->push(std::move(result));
// no try/catch needed for shutdown; push is non-throwing
if (frame_queue_->is_shutdown()) {
    break;
}
```

Or check `capture_stop_` only (already present).

- [ ] **Step 5: Update perception consumers**

`submit`:

```cpp
if (!frame_queue_ || frame_queue_->is_shutdown()) {
    return false;
}
frame_queue_->push(...);
return true;
```

`run`:

```cpp
while (true) {
    auto queued = frame_queue_->pop();
    if (!queued.has_value()) {
        break; // shutdown
    }
    QueuedFrame queued_frame = std::move(*queued);
    // ... rest unchanged
}
```

- [ ] **Step 6: Build + tests**

```bash
cmake --build build --parallel
ctest --test-dir build -R 'freshness_queue_test|fifo_control_test' --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Commit (if requested)**

```bash
git commit -m "refactor(concurrency): LatestValue non-throwing shutdown"
```

---

### Task 6: Optional `jthread` for PerceptionRunner worker (Phase 3b)

**Files:**
- Modify: `src/runtime/perception_runner.hpp`
- Modify: `src/runtime/perception_runner.cpp`

**Interfaces:**
- Replace `std::thread worker_` with `std::jthread worker_`.
- `run` becomes `run(std::stop_token st)` **or** keep `run()` and rely on `LatestValue::shutdown` as the wake mechanism (preferred minimal change).

**Minimal recommended approach (no stop_token plumbing into queue):**

1. `worker_` type → `std::jthread`.
2. `stop()`: call `frame_queue_->shutdown()` then `worker_ = std::jthread{}` (joins) or `if (worker_.joinable())` — for `jthread`, assign default / call `request_stop` + join via destructor.
3. Do **not** migrate CaptureDevice in this task.

```cpp
// start
worker_ = std::jthread([this](std::stop_token) { run(); });

// stop
shutdown(); // shuts down LatestValue
worker_ = std::jthread{}; // joins previous
```

- [ ] **Step 1: Implement jthread swap as above**
- [ ] **Step 2: Build + full ctest**
- [ ] **Step 3: Manual smoke if hardware available** — start/stop `tool_preview`
- [ ] **Step 4: Commit (if requested)** — `refactor(concurrency): use jthread for PerceptionRunner`

---

### Task 7: Sink Hik types out of `capture_device.hpp` (Phase 4B)

**Files:**
- Modify: `src/capture/capture_device.hpp` — remove `#include <hikcamera/capturer.hpp>` and `HikBackend` / `V4l2Backend` definitions if moved
- Create: `src/capture/v4l2_backend.hpp` (optional) / keep V4L2 in cpp
- Create: `src/capture/hik_backend.cpp` (+ private header `src/capture/hik_backend.hpp` not installed)
- Modify: `src/capture/capture_device.cpp` — `make_backend()` implementations
- Ensure `laser_guidance_runtime` still links `hikcamera`

**Target:**

```cpp
// capture_device.hpp — no hikcamera includes
class CaptureDevice {
  // ...
  [[nodiscard]] auto make_backend() const -> std::unique_ptr<CaptureBackend>;
  std::unique_ptr<CaptureBackend> backend_{};
  // ...
};
```

`HikBackend` and `V4l2Backend` live in `.cpp` or private headers under `src/capture/`.

- [ ] **Step 1: Move `struct V4l2Backend` / `struct HikBackend` out of the public capture header**
- [ ] **Step 2: Confirm no other TU needed full definitions (only capture_device.cpp)**
- [ ] **Step 3: Build full tree**
- [ ] **Step 4: Commit (if requested)** — `refactor(capture): hide Hik backend types from header`

---

### Task 8: Explicit sources for `laser_guidance_runtime` (Phase 4C)

**Files:**
- Modify: `CMakeLists.txt` — the `set(laser_guidance_runtime_SOURCES ...)` block is already explicit; extend the same pattern to **support/tracking/vision/guidance/bridges** that still use `GLOB_RECURSE`, **or** only document that runtime is already explicit and replace GLOB for the next-most-edited lib.

**Minimal done definition for this task:**

1. Replace `file(GLOB_RECURSE laser_guidance_support_SOURCES ...)` with an explicit list of every current file under `src/core/*.cpp`, `src/runtime/support.cpp`, `src/runtime/runtime_command.cpp` (and `error.cpp` if added).
2. Leave other GLOBs for a follow-up if the list is huge — but **do** runtime (already explicit) + support (small).

- [ ] **Step 1: `ls src/core/*.cpp src/runtime/support.cpp src/runtime/runtime_command.cpp`**
- [ ] **Step 2: Paste explicit `set(laser_guidance_support_SOURCES ...)`**
- [ ] **Step 3: Clean configure**

```bash
rm -rf build && build-laser
ctest --test-dir build -R 'fifo_control_test|freshness_queue_test' --output-on-failure
```

- [ ] **Step 4: Commit (if requested)** — `build: explicit sources for laser_guidance_support`

---

### Task 9: Documentation sync

**Files:**
- Modify: `docs/design-patterns.md` — Value Types / command: document `std::variant` command ADT; Error domain; fix CaptureDevice note (virtual backend today, not variant)
- Modify: `docs/todolist.md` — mark refactor slices done / remaining
- Modify: `docs/superpowers/specs/2026-07-25-modern-cpp-refactor-design.md` — status → `accepted` / `implemented-in-progress`
- Touch `docs/architecture.md` only if control surface description needs a sentence

- [ ] **Step 1: Patch design-patterns.md section on Value Types / commands**
- [ ] **Step 2: Fix Capture backend selection sentence to match code**
- [ ] **Step 3: Update todolist completed items**

---

## Spec coverage checklist

| Spec item | Task |
|-----------|------|
| Phase 0 baseline | Task 1 |
| Phase 1 command variant | Task 2 |
| Phase 2 Error public path | Task 3 |
| Phase 2 capture/guidance/SPI | Task 4 |
| Phase 3 LatestValue | Task 5 |
| Phase 3 jthread one worker | Task 6 |
| Phase 4B Hik sink | Task 7 |
| Phase 4C CMake explicit | Task 8 |
| Docs | Task 9 |
| Non-goals (no DI, no units, no ROS change) | Global Constraints |
| Banned abstractions | Global Constraints |

## Deferred (not in this plan)

- Public OpenCV DTO strip (`Point2f` / `Rect2f` in `runtime.hpp`)
- Full jthread migration for CaptureDevice / ScanController / ControlLoop main thread
- `std::span` on FT4222 APIs
- TensorRT `unique_ptr` deleters
- Strong units (mm/deg/V)

---

## Execution handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-25-modern-cpp-refactor.md`.

**Two execution options:**

1. **Subagent-Driven (recommended)** — fresh subagent per task, review between tasks  
2. **Inline Execution** — same session with executing-plans checkpoints  

**Which approach?**
