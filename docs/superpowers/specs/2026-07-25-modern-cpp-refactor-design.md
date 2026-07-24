# Modern C++ Refactor Design

**Date:** 2026-07-25  
**Status:** implemented (tasks 1-8 complete); docs synced in task 9  
**Default goal assumption:** stability / maintainability first (A), with selective public-boundary hardening (B). Full “modernize everything” (D) is out of scope for a single cycle.  
**Assumption note:** Defaults are A+selective B. Override before Phase 1 if wrong.  
**Plan:** `docs/superpowers/plans/2026-07-25-modern-cpp-refactor.md`

## 1. Context

The repository is already a disciplined real-time vision→galvo system:

- Public facade: `CompetitionRuntime` + value types (`RuntimeCommand`, `RuntimeSnapshot`, …)
- Internal composition: `ControlLoop` owns concrete components
- C++23 with widespread `std::expected` / `std::optional` / PIMPL
- Explicit anti-patterns already removed: builders, mode hooks, deep interface forests

Design review debt (prioritized):

| Priority | Debt | Why it matters |
|----------|------|----------------|
| P0 | `RuntimeCommand` is a tagged flat struct | Invalid field combos type-check; hard to extend safely |
| P0 | Errors are mostly `std::string` | Callers cannot classify reconnectable vs config vs fatal |
| P0 | `LatestValue` shutdown throws | Exception path across realtime loops is brittle |
| P1 | Production threads use `std::thread` + ad-hoc stop flags | Tests already show `jthread`/`stop_token`; lifecycle is inconsistent |
| P1 | Public headers pull OpenCV / yaml-cpp | Public surface heavier than needed; ABI/compile coupling |
| P1 | Hik types visible in `capture_device.hpp` | Capture compile graph heavier than public API claim |
| P2 | SPI buffers use raw pointer + length | Length mismatch risk; `span` is local and cheap |
| P2 | CMake `GLOB_RECURSE` | New files can silently miss builds depending on configure |
| P2 | Units are bare floats | Domain bugs (mm/deg/V mix) possible; larger churn |

## 2. Goals

1. **Make the control plane safer** without changing FIFO wire protocol semantics.
2. **Make failures classifiable** while keeping human-readable messages for logs.
3. **Unify stop / queue-close semantics** so realtime loops do not rely on exceptions for control flow.
4. **Harden compile boundaries** where ROI is high and behavior risk is low.
5. **Preserve architecture principles** in `docs/design-patterns.md` and `docs/development.md`:
   - no service locator
   - no mega runtime manager
   - no reintroduction of builders / ModeHooks / ICaptureDevice forests
   - concrete composition first

## 3. Non-goals (this cycle)

- Rewriting the vision/TensorRT pipeline
- Introducing a DI container or plugin framework
- Strong physical units everywhere (mm/deg/V newtypes) as a first pass
- Splitting `ControlLoop` into many files unless a phase forces it
- Changing ROS2 topic contracts, ZMQ `cmd_id`, or UDP binary layout
- Making ONNX/TensorRT/ROS2 optional again (product choice stays)

## 4. Approaches considered

### Approach A — Surgical vertical slices (recommended)

Ship 3–4 small, independently mergeable refactors:

1. Command ADT + visitor/switch on variant  
2. `Error` domain + migrate hot paths  
3. Queue/thread lifecycle (`optional`/`expected` close + optional `jthread`)  
4. Boundary hygiene (OpenCV DTO split **or** Hik header sink — pick one first)

**Pros:** low blast radius; each slice testable; matches “no big rewrite” culture.  
**Cons:** temporary dual shapes during migration; more PRs.

### Approach B — Big-bang public API modernization

Change `runtime.hpp` heavily in one PR: variant commands, plain geometry DTOs, Error type, span SPI, jthread everywhere.

**Pros:** clean end state faster on paper.  
**Cons:** high merge risk on active feature branch (Hik profiles/recording); hard bisect; tools+tests all break together.

### Approach C — Boundary-only (headers/PIMPL/CMake)

Only compile isolation and CMake source lists; leave command/error/concurrency as-is.

**Pros:** safer for competition week.  
**Cons:** does not fix the highest-value type-safety debts.

**Recommendation:** Approach A.

## 5. Design principles for every slice

1. **Behavior-preserving first** — FIFO strings, snapshot field meanings, degraded hardware semantics stay.
2. **TDD on existing tests** — extend `tests/runtime/fifo_control_test.cpp` and module tests before behavior changes.
3. **One concept per PR** — do not mix command ADT with OpenCV DTO strip in the same PR.
4. **Internal-first migration** — prefer changing `runtime_internal` then thin public adapters when needed.
5. **No new abstraction layers** unless they replace an existing concrete pain with a smaller surface.

## 6. Phase design

### Phase 0 — Guardrails (before code moves)

**Deliverables**

- Confirm baseline: `build-laser` / existing `ctest` green on the target branch.
- Freeze “do not touch” list for the cycle (ROS msg layouts, ZMQ payload schema, HitProgress formula).
- Add a short section to `docs/todolist.md` pointing at this spec (optional).

**Success:** green baseline recorded; no feature work mixed into refactor PRs.

---

### Phase 1 — `RuntimeCommand` as a closed sum type

**Problem**  
`RuntimeCommand` holds `type` plus unrelated payload fields. Factories help, but the type system does not.

**Target shape (public API)**

Prefer a **variant of small structs** (or equivalent closed set), with factories remaining the primary construction path:

```cpp
struct CmdSetStreaming { bool enabled; };
struct CmdSetRecording { bool enabled; };
struct CmdSetEnemyColor { EnemyColor enemy_color; };
struct CmdSetBackend { RuntimeBackend backend; };
struct CmdSetEkf { bool enabled; };
struct CmdShutdown {};

using RuntimeCommand = std::variant<
    CmdSetStreaming,
    CmdSetRecording,
    CmdSetEnemyColor,
    CmdSetBackend,
    CmdSetEkf,
    CmdShutdown>;
```

**Factory API (required choice — locked default below)**

`RuntimeCommand` as a `using` alias to `std::variant` **cannot** host `static` member factories like today’s `RuntimeCommand::set_streaming`. Default:

```cpp
namespace runtime_command {
auto set_streaming(bool enabled) -> RuntimeCommand;
auto set_recording(bool enabled) -> RuntimeCommand;
auto set_enemy_color(EnemyColor color) -> RuntimeCommand;
auto set_backend(RuntimeBackend backend) -> RuntimeCommand;
auto set_ekf(bool enabled) -> RuntimeCommand;
auto shutdown() -> RuntimeCommand;
}
```

In-tree call sites (`runtime_command.cpp` factories, bridges, tools if any) switch to `runtime_command::set_*`. No dual representation.

**Migration strategy**

1. Introduce payload structs + `using RuntimeCommand = std::variant<...>` in `include/laser_guidance/runtime.hpp`.
2. Replace `RuntimeCommand::set_*` with `runtime_command::set_*` in `src/runtime/runtime_command.cpp`.
3. Update `FifoControlServer::parse_command` to construct variant alternatives.
4. Update `ControlLoop::submit_command` to `std::visit` with an overloaded lambda set; default arm `static_assert`/`std::unreachable` for exhaustiveness.
5. Update `tests/runtime/fifo_control_test.cpp` to use `std::holds_alternative` / `std::get_if` instead of `.type` / `.enabled`.

**Compatibility note**  
If external out-of-tree code depends on member `.type`, either:

- document a **breaking public API** change (acceptable for this app repo), or  
- keep a thin deprecated accessor for one release (not preferred; adds dual representation).

**Recommendation for this monorepo:** break cleanly; fix in-tree tools/tests only.

**Out of scope**  
Do not invent a command bus, queue of commands, or async command processor.

**Success criteria**

- All FIFO parse tests pass.
- `submit_command` is exhaustive (compiler warns on missing alternative if using `visit` + overload set with `static_assert` default).
- Impossible states like “streaming command with enemy_color set” cannot be represented.

---

### Phase 2 — Classified errors (`Error` + `expected`)

**Problem**  
`std::expected<T, std::string>` is consistent but not actionable for control flow (retry vs abort vs user config fix).

**Target shape**

```cpp
enum class ErrorKind : std::uint8_t {
    config,       // bad path / invalid YAML / unsupported command
    device,       // camera / FT4222 / open failure (often retryable)
    unavailable,  // backend missing, feature disabled by profile
    timeout,      // optional; only if a real timeout path exists
    internal,     // invariant / programmer error surfaced as error
};

struct Error {
    ErrorKind kind = ErrorKind::internal;
    std::string message{};
};

// Prefer: std::expected<T, Error>
```

**Helpers**

```cpp
auto make_error(ErrorKind kind, std::string message) -> Error;
auto format_error(const Error& e) -> std::string; // for logging: "[device] camera open failed: ..."
```

**Migration order (hot paths first)**

1. Define `Error` in a small public header (e.g. `include/laser_guidance/error.hpp`) or in `runtime.hpp` if kept minimal.
2. Migrate **public** signatures first where callers already branch on success only:
   - `CompetitionRuntime::start/run/submit_command`
   - `FifoControlServer::parse_command` / `start`
3. Migrate capture/guidance factories used by control loop:
   - `CaptureDevice::open/reconnect`
   - `GuidanceSession::create_*`
   - `Ft4222Spi::open`
4. Leave deep vision/TensorRT internals on `string` until a later pass if needed.

**Logging policy**  
Always log `format_error(err)` (or `err.message` with kind prefix). Do not drop messages.

**Retry policy mapping (explicit)**

| ErrorKind | Default control-loop reaction |
|-----------|-------------------------------|
| `device` | keep existing reconnect / guidance retry behavior |
| `config` | fail start or reject command; do not spin reconnect forever |
| `unavailable` | degrade feature (guidance off / backend switch fail) |
| `internal` | log loudly; prefer fail-closed for that subsystem |

**Success criteria**

- At least FIFO + `submit_command` + capture open/reconnect use `Error`.
- No behavior change in reconnect timing without a dedicated test/doc note.
- Call sites that only print errors keep working via `format_error`.

---

### Phase 3 — Lifecycle: queues and threads

**Problem**

- `LatestValue::pop` throws on shutdown → control-flow exception.
- Production uses `std::thread` + `atomic stop` / manual join; tests already use `jthread`.

**3a. `LatestValue` close semantics**

Change close to non-throwing for normal shutdown:

```cpp
// Preferred:
auto try_pop() -> std::optional<T>;
auto pop_wait() -> std::optional<T>; // nullopt means shutdown
// push on shutdown: ignore or return false (choose one; document)
```

Rules:

- Shutdown is **not** a programmer error for consumer loops.
- Overwrite counting behavior stays.
- Document single-producer / multi? (current: effectively SP/SC usage) — keep as-is.

**3b. Thread stop protocol (optional in same phase if small)**

Where stop is a simple flag:

- Prefer `std::jthread` + `std::stop_token` for new or touched workers.
- Do **not** force-migrate every thread in one PR if capture/perception need careful join order.

Minimum for this phase:

- `LatestValue` non-throwing close.
- One worker path (prefer `PerceptionRunner` **or** `CaptureDevice`) migrated as a template for the rest.

**Success criteria**

- No `throw` on normal `LatestValue` shutdown path.
- Existing capture/perception integration tests (if any) still pass; manual smoke: start/stop competition/preview cleanly.
- No deadlock on stop (join order documented in code comments near teardown).

---

### Phase 4 — Boundary hygiene (pick one track first)

#### Track 4A — Public DTO without OpenCV in `runtime.hpp` (higher churn)

Introduce plain geometry in public surface:

```cpp
struct Point2f { float x = 0; float y = 0; };
struct Rect2f { float x = 0; float y = 0; float width = 0; float height = 0; };
```

Map at internal boundaries with free converters in `src/` (OpenCV stays implementation detail).

**Do only if** Phase 1–2 are stable. This touches many snapshot/detection fields and tests.

#### Track 4B — Sink Hik types out of `capture_device.hpp` (recommended first boundary win)

- Move `HikBackend` implementation to `.cpp` / private header under `src/capture/`.
- Keep `CaptureDevice` public methods unchanged.
- Goal: including capture headers for non-Hik tests should not force hikcamera parameter headers where avoidable.

#### Track 4C — CMake explicit sources

- Replace `file(GLOB_RECURSE ...)` for core libs with explicit lists (or `CONFIGURE_DEPENDS` kept only temporarily).
- Prefer explicit lists for `laser_guidance_runtime` first (fattest / most edited).

**Recommendation order:** 4B → 4C → 4A.

---

### Phase 5 — Small local API cleanups (opportunistic)

Only when touching adjacent code:

- `Ft4222Spi::write/transfer` take `std::span<const std::uint8_t>` / `std::span<std::uint8_t>`.
- TensorRT engine cleanup via `unique_ptr` + custom deleters (no behavior change).
- Remove unused includes from `support.hpp` (yaml-cpp if truly unused).

Not a standalone epic.

## 7. Testing strategy

| Slice | Required tests |
|-------|----------------|
| Phase 1 | Extend `tests/runtime/fifo_control_test.cpp`; compile-time exhaustiveness on visit |
| Phase 2 | FIFO parse errors keep messages; add kind assertions for invalid command / blank |
| Phase 3 | Unit test `LatestValue` shutdown: pop returns nullopt, no throw; start/stop smoke if available |
| Phase 4B | Build still links hik path; open/reconnect tests if present |
| Phase 4C | Clean configure + build from empty build dir |

Manual gates after each merge:

- `tool_preview` start/stop
- `tool_competition` FIFO: `stream on`, `enemy red`, `quit`
- Guidance optional path still degrades without FT4222

## 8. Documentation updates (when a phase merges)

Per `docs/development.md` / AGENTS rules, update only what changed:

- `docs/design-patterns.md` — document command as variant; error domain
- `docs/architecture.md` — only if control/observation surface description changes
- `docs/AGENTS.md` / root `AGENTS.md` — boundary notes if public headers change
- `docs/todolist.md` — mark completed refactor items

## 9. Explicit “do not reintroduce” list

- `ControlLoopBuilder`, `ModeHooks`, `GuidanceController`, `ICaptureDevice`, `InferenceFacade`
- Global service locator / singleton runtime registry
- Parallel “v2” pipeline beside the main one
- Exception-based API for normal command/device failures (exceptions remain only at third-party boundaries, converted to `Error`)

## 10. Suggested PR / commit sequence

1. `refactor(runtime): RuntimeCommand as std::variant`  
2. `refactor(error): introduce Error/ErrorKind and migrate command+FIFO`  
3. `refactor(error): migrate capture open/reconnect + guidance factories`  
4. `refactor(concurrency): LatestValue non-throwing shutdown`  
5. `refactor(concurrency): jthread stop_token for one worker`  
6. `refactor(capture): hide Hik backend types from header`  
7. `build: explicit sources for laser_guidance_runtime`  
8. (later) public geometry DTO / span SPI / TensorRT deleters

Each PR must be mergeable alone and leave `main`/`develop` shippable.

## 11. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| Variant breaks external scripts relying on C++ members | In-tree only; FIFO protocol unchanged |
| Error migration half-done (`string` vs `Error`) | Alias or convert at edges; finish public surface first |
| Thread join order regressions | Change one worker at a time; manual start/stop smoke |
| Feature work collides (Hik profiles) | Do not mix feature commits into refactor PRs; rebase often |
| Over-scoping units/concepts | Park in P2 backlog |

## 12. Success definition for the whole cycle

Cycle is done when:

1. Commands are a closed sum type with exhaustive handling.
2. Public control-path errors use `Error` with kinds that drive retry vs reject.
3. `LatestValue` normal shutdown does not throw.
4. At least one boundary win shipped (Hik header sink **or** CMake explicit sources).
5. Architecture docs reflect the new patterns; no banned abstractions reintroduced.
6. Baseline automated tests + manual preview/competition smoke still pass.

## 13. Open decisions (defaults if no override)

| Decision | Default |
|----------|---------|
| Primary goal | A (stability) + selective B |
| Command representation | `std::variant` of small structs |
| Breaking public C++ API | Yes, in-tree only |
| First boundary track | 4B Hik header sink, then 4C CMake |
| Public OpenCV removal | Deferred after 1–3 |
| Full jthread migration | Only after LatestValue; one worker first |

---

## Appendix A — Touch map (expected files)

**Phase 1**

- `include/laser_guidance/runtime.hpp`
- `src/runtime/runtime_command.cpp`
- `src/runtime/control_loop.cpp` (`submit_command`)
- `src/bridges/bridges.cpp` (`parse_command`)
- `include/laser_guidance/bridges.hpp` (if signatures mention command fields)
- `tests/runtime/fifo_control_test.cpp`
- `tools/competition.cpp` / `tools/preview.cpp` (only if they inspect fields)

**Phase 2**

- new: `include/laser_guidance/error.hpp` (or equivalent)
- same public APIs as Phase 1
- `src/capture/*`, `src/runtime/guidance_session.*`, `src/io/ft4222_spi.*` (incremental)

**Phase 3**

- `src/tracking/freshness_queue.hpp` (+ tests)
- optionally `src/capture/capture_device.*`, `src/runtime/perception_runner.*`

**Phase 4B**

- `src/capture/capture_device.hpp/.cpp`
- possibly new `src/capture/hik_backend.cpp` / `v4l2_backend.cpp`

## Appendix B — Alignment with existing docs

This design **extends** current principles:

- Still: concrete composition, value-type control/observation surfaces, PIMPL at SDK edges.
- Changes: control surface becomes a true ADT; errors become structured; concurrency close path becomes expected-based.

It does **not** reverse the removal of builder/hook forests documented in `docs/design-patterns.md`.

**Doc drift (out of scope for code, fix when editing design-patterns):**  
`docs/design-patterns.md` claims `CaptureDevice` selects backends via `std::variant`. Current code uses a thin `CaptureBackend` virtual base + `unique_ptr` (`V4l2Backend` / `HikBackend`). Refactor does **not** reintroduce a large interface forest; optional later cleanup is either (a) update the doc to match code, or (b) replace the two backends with `std::variant` *inside* `CaptureDevice` only — not a Phase 1–3 requirement.

## Appendix C — Spec self-review (2026-07-25)

| Check | Result |
|-------|--------|
| Placeholders (TBD/TODO) | None remaining after factory API lock |
| Internal consistency | Phases 1→2→3→4 order consistent; OpenCV DTO deferred |
| Scope | One cycle = Approach A; D/full units out of scope |
| Ambiguity | Factory namespace locked to `runtime_command::`; Error kinds mapped to retry policy |
| Goal assumption | Explicit A+B default; user may override |
