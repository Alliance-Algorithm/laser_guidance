#pragma once

#include <cstdint>

namespace rmcs_laser_guidance {

// NOLINTBEGIN(readability-identifier-naming)
enum class HitState : std::uint8_t {
    None,
    Candidate,
    Confirmed,
};

class HitStateMachine {
public:
    explicit HitStateMachine(int confirm_frames = 3, int release_frames = 5)
        : confirm_frames_(confirm_frames), release_frames_(release_frames) {}

    auto update(bool is_purple) -> HitState {
        if (!has_last_ || is_purple != last_is_purple_) {
            consecutive_ = 1;
            last_is_purple_ = is_purple;
            has_last_ = true;
        } else {
            ++consecutive_;
        }

        if (is_purple) {
            state_ = consecutive_ >= confirm_frames_ ? HitState::Confirmed : HitState::Candidate;
        } else if (state_ == HitState::Confirmed && consecutive_ >= release_frames_) {
            state_ = HitState::None;
        }

        return state_;
    }

    [[nodiscard]] auto state() const -> HitState {
        return state_;
    }

    [[nodiscard]] auto consecutive() const -> int {
        return consecutive_;
    }

    auto reset() -> void {
        state_ = HitState::None;
        consecutive_ = 0;
        last_is_purple_ = false;
        has_last_ = false;
    }

private:
    int confirm_frames_;
    int release_frames_;
    HitState state_ = HitState::None;
    int consecutive_ = 0;
    bool last_is_purple_ = false;
    bool has_last_ = false;
};
// NOLINTEND(readability-identifier-naming)

} // namespace rmcs_laser_guidance
