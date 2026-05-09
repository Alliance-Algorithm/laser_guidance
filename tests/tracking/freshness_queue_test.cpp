#include <chrono>
#include <exception>
#include <future>
#include <print>
#include <stop_token>
#include <thread>

#include "tracking/freshness_queue.hpp"
#include "test_utils.hpp"

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
            require(value == 3, "pop should return newest value");
            require(queue.overwrite_count() == 2, "overwrite count mismatch");
        }

        {
            rmcs_laser_guidance::LatestValue<int> queue;
            std::promise<int> popped;
            auto future = popped.get_future();

            const std::jthread worker([&](const std::stop_token&) {
                try {
                    popped.set_value(queue.pop());
                } catch (...) {
                    popped.set_exception(std::current_exception());
                }
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            require(future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready,
                "pop should block until push");

            require(queue.push(42) == 0, "push after block should not overwrite");
            require(future.get() == 42, "blocked pop should wake with pushed value");
        }

        {
            rmcs_laser_guidance::LatestValue<int> queue;
            std::promise<bool> finished;
            auto future = finished.get_future();

            const std::jthread worker([&](const std::stop_token&) {
                try {
                    (void)queue.pop();
                    finished.set_value(false);
                } catch (const std::exception&) {
                    finished.set_value(true);
                }
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            queue.shutdown();
            require(future.get(), "shutdown should unblock waiting pop");
            int value = 0;
            require(!queue.try_pop(value), "shutdown empty queue should stay empty");

            bool push_rejected = false;
            try {
                (void)queue.push(7);
            } catch (const std::exception&) {
                push_rejected = true;
            }
            require(push_rejected, "push after shutdown should be rejected");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "freshness_queue_test failed: {}", e.what());
        return 1;
    }
}
