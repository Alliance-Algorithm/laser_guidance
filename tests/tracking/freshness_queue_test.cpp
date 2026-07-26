#include <chrono>
#include <future>
#include <optional>
#include <print>
#include <stop_token>
#include <thread>

#include "test_utils.hpp"
#include "tracking/freshness_queue.hpp"

int main() {
    try {
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_contains;

        {
            rmcs_laser_guidance::LatestValue<int> queue;
            int value = 0;
            require(!queue.try_pop(value), "empty queue should not pop");
        }

        {
            rmcs_laser_guidance::LatestValue<int> queue;
            require(queue.push(1) == 0, "first push should not overwrite");
            require(queue.push(2) == 1, "second push should report one overwrite");
            require(queue.push(3) == 2, "third push should report two overwrites");

            const auto value = queue.pop();
            require(value.has_value(), "pop should return a value");
            require(*value == 3, "pop should return newest value");
            require(queue.overwrite_count() == 2, "overwrite count mismatch");
        }

        {
            rmcs_laser_guidance::LatestValue<int> queue;
            std::promise<std::optional<int>> popped;
            auto future = popped.get_future();

            const std::jthread worker([&](const std::stop_token&) {
                popped.set_value(queue.pop());
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            require(
                future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready,
                "pop should block until push");

            require(queue.push(42) == 0, "push after block should not overwrite");
            auto result = future.get();
            require(result.has_value(), "blocked pop should return a value");
            require(*result == 42, "blocked pop should wake with pushed value");
        }

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

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "freshness_queue_test failed: {}", e.what());
        return 1;
    }
}
