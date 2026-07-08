#include <chrono>
#include <print>
#include <stdexcept>

#include "runtime/capture_retry_policy.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::runtime_internal::CaptureRetryPolicy;
        using rmcs_laser_guidance::tests::require;
        using Clock = CaptureRetryPolicy::Clock;

        const auto t0 = Clock::now();

        {
            // Below the error threshold: stays out of the reconnect state and
            // keeps deferring, never actually triggering a reconnect.
            CaptureRetryPolicy policy(/*max_consecutive_errors=*/3);
            require(!policy.on_read_error(t0), "1st error should not trip reconnect");
            require(!policy.on_read_error(t0), "2nd error should not trip reconnect");
            require(!policy.reconnect_pending(), "reconnect should not be pending yet");
        }

        {
            // Crossing the threshold flips into the reconnect-pending state
            // and arms a guidance retry at the same time.
            CaptureRetryPolicy policy(
                /*max_consecutive_errors=*/3, std::chrono::seconds(1), std::chrono::seconds(1));
            require(!policy.on_read_error(t0), "1st error should not trip reconnect");
            require(!policy.on_read_error(t0), "2nd error should not trip reconnect");
            require(policy.on_read_error(t0), "3rd error should trip reconnect");
            require(policy.reconnect_pending(), "reconnect should now be pending");
            require(!policy.reconnect_due(t0), "reconnect should not be due immediately");
            require(
                policy.reconnect_due(t0 + std::chrono::seconds(2)),
                "reconnect should be due after the retry delay");
        }

        {
            // A single success mid-streak resets the error count, so a
            // subsequent error does not immediately trip reconnect again.
            CaptureRetryPolicy policy(/*max_consecutive_errors=*/3);
            require(!policy.on_read_error(t0), "1st error should not trip reconnect");
            require(!policy.on_read_error(t0), "2nd error should not trip reconnect");
            policy.on_read_success();
            require(!policy.on_read_error(t0), "error streak should have reset after success");
        }

        {
            // Successful reconnect clears the pending state entirely.
            CaptureRetryPolicy policy(/*max_consecutive_errors=*/1);
            require(policy.on_read_error(t0), "single error should trip reconnect (threshold=1)");
            policy.on_reconnect_succeeded();
            require(!policy.reconnect_pending(), "reconnect should clear after success");
        }

        {
            // Failed reconnect re-arms the reconnect delay from "now".
            CaptureRetryPolicy policy(
                /*max_consecutive_errors=*/1, std::chrono::seconds(1), std::chrono::seconds(1));
            require(policy.on_read_error(t0), "single error should trip reconnect (threshold=1)");
            policy.on_reconnect_failed(t0 + std::chrono::seconds(5));
            require(
                !policy.reconnect_due(t0 + std::chrono::seconds(5)),
                "reconnect should not be due immediately after a failed attempt");
            require(
                policy.reconnect_due(t0 + std::chrono::seconds(6)),
                "reconnect should be due one delay period after the failed attempt");
        }

        {
            // Guidance retry gating: due immediately by default, then
            // deferred after a failed retry, then cleared on success.
            CaptureRetryPolicy policy(
                /*max_consecutive_errors=*/3, std::chrono::seconds(1), std::chrono::seconds(1));
            require(policy.guidance_retry_due(t0), "guidance retry should be due with no arm/defer");

            policy.defer_guidance_retry(t0);
            require(
                !policy.guidance_retry_due(t0), "guidance retry should not be due immediately after defer");
            require(
                policy.guidance_retry_due(t0 + std::chrono::seconds(1)),
                "guidance retry should be due after the retry delay");

            policy.clear_guidance_retry();
            require(policy.guidance_retry_due(t0), "guidance retry should be due again after clear");

            policy.arm_guidance_retry(t0);
            require(policy.guidance_retry_due(t0), "arm_guidance_retry should make it due at that instant");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "capture_retry_policy_test failed: {}", e.what());
        return 1;
    }
}
