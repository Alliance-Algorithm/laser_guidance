#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace rmcs_laser_guidance {

template <typename T>
class LatestValue {
public:
    LatestValue() = default;

    auto push(T value) -> std::size_t {
        std::scoped_lock lock(mutex_);
        if (shutdown_) {
            throw std::runtime_error("LatestValue is shutdown");
        }

        if (value_.has_value()) {
            ++overwrite_count_;
        }

        value_.emplace(std::move(value));
        cv_.notify_one();
        return overwrite_count_;
    }

    auto pop() -> T {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this] { return shutdown_ || value_.has_value(); });

        if (shutdown_) {
            throw std::runtime_error("LatestValue is shutdown");
        }

        T value = std::move(*value_);
        value_.reset();
        return value;
    }

    auto try_pop(T& value) -> bool {
        std::scoped_lock lock(mutex_);
        if (!value_.has_value()) return false;

        value = std::move(*value_);
        value_.reset();
        return true;
    }

    auto shutdown() -> void {
        std::scoped_lock lock(mutex_);
        shutdown_ = true;
        cv_.notify_all();
    }

    [[nodiscard]] auto overwrite_count() const -> std::size_t {
        std::scoped_lock lock(mutex_);
        return overwrite_count_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<T> value_{};
    std::size_t overwrite_count_ = 0;
    bool shutdown_ = false;
};

} // namespace rmcs_laser_guidance
