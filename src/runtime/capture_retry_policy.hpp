#pragma once

#include <chrono>
#include <optional>

namespace rmcs_laser_guidance::runtime_internal {

// Shared reconnect/guidance-retry state machine used by both ControlLoop and
// GuidanceOpsApp's run_loop(). Pure logic: no locks, no I/O, no logging. The
// two loops differ in what they do around each transition (locking,
// telemetry, log wording), so this only owns the "when" decisions, not the
// actions themselves.
class CaptureRetryPolicy {
public:
    using Clock = std::chrono::steady_clock;

    explicit CaptureRetryPolicy(
        int max_consecutive_errors = 3,
        Clock::duration reconnect_retry_delay = std::chrono::seconds(1),
        Clock::duration guidance_retry_delay = std::chrono::seconds(1))
        : max_consecutive_errors_(max_consecutive_errors)
        , reconnect_retry_delay_(reconnect_retry_delay)
        , guidance_retry_delay_(guidance_retry_delay) {}

    // Called after a successful capture_.read_frame(). Clears the error
    // streak and any pending reconnect.
    auto on_read_success() -> void {
        consecutive_errors_ = 0;
        next_reconnect_at_.reset();
    }

    // Called after a failed capture_.read_frame(). Returns true once the
    // caller should tear down guidance and enter the reconnect state (i.e.
    // consecutive_errors_ just crossed max_consecutive_errors_).
    auto on_read_error(const Clock::time_point now) -> bool {
        ++consecutive_errors_;
        if (consecutive_errors_ < max_consecutive_errors_) {
            return false;
        }
        consecutive_errors_ = max_consecutive_errors_;
        next_reconnect_at_ = now + reconnect_retry_delay_;
        next_guidance_retry_at_ = now + guidance_retry_delay_;
        return true;
    }

    [[nodiscard]] auto reconnect_pending() const -> bool { return next_reconnect_at_.has_value(); }

    [[nodiscard]] auto reconnect_due(const Clock::time_point now) const -> bool {
        return next_reconnect_at_.has_value() && now >= *next_reconnect_at_;
    }

    auto on_reconnect_succeeded() -> void {
        consecutive_errors_ = 0;
        next_reconnect_at_.reset();
    }

    auto on_reconnect_failed(const Clock::time_point now) -> void {
        consecutive_errors_ = max_consecutive_errors_;
        next_reconnect_at_ = now + reconnect_retry_delay_;
    }

    [[nodiscard]] auto guidance_retry_due(const Clock::time_point now) const -> bool {
        return !next_guidance_retry_at_.has_value() || now >= *next_guidance_retry_at_;
    }

    auto arm_guidance_retry(const Clock::time_point now) -> void {
        next_guidance_retry_at_ = now;
    }

    auto defer_guidance_retry(const Clock::time_point now) -> void {
        next_guidance_retry_at_ = now + guidance_retry_delay_;
    }

    auto clear_guidance_retry() -> void { next_guidance_retry_at_.reset(); }

private:
    int max_consecutive_errors_;
    Clock::duration reconnect_retry_delay_;
    Clock::duration guidance_retry_delay_;

    int consecutive_errors_ = 0;
    std::optional<Clock::time_point> next_reconnect_at_{};
    std::optional<Clock::time_point> next_guidance_retry_at_{};
};

} // namespace rmcs_laser_guidance::runtime_internal
