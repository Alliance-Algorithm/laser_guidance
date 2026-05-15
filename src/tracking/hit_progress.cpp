#include "tracking/hit_progress.hpp"

#include <algorithm>

namespace rmcs_laser_guidance {

auto HitProgress::progress_ratio() const noexcept -> float {
    if (p0_ <= 0.0F) return 0.0F;
    return std::clamp(p_ / p0_, 0.0F, 1.0F);
}

void HitProgress::update(bool is_hit, float dt_s) {
    if (exhausted_) return;

    if (locked_) {
        lock_timer_ = std::max(0.0F, lock_timer_ - dt_s);
        if (lock_timer_ <= 0.0F) {
            locked_ = false;
            if (lock_count_ >= 3) {
                exhausted_ = true;
                return;
            }
        }
        return;
    }

    if (!is_hit) {
        p_ = std::max(0.0F, p_ - 0.5F * dt_s);
        t_ = 0.0F;
        n_ = 0;
        return;
    }

    t_ += dt_s;
    constexpr float kTick = 0.1F;
    int ticks = static_cast<int>(t_ / kTick + 1.0e-6F);
    if (ticks > 0) {
        p_ += static_cast<float>(n_ * ticks + ticks * (ticks + 1) / 2);
        n_ += ticks;
        t_ -= static_cast<float>(ticks) * kTick;
    }

    if (p_ >= p0_) {
        trigger_lock();
    }
}

void HitProgress::trigger_lock() {
    ++lock_count_;
    locked_     = true;
    lock_timer_ = 45.0F;
    p_ = 0.0F;
    t_ = 0.0F;
    n_ = 0;
    advance_stage();
}

void HitProgress::advance_stage() {
    stage_ = std::min(lock_count_, 2);
    switch (stage_) {
    case 0: p0_ = 50.0F;  break;
    case 1: p0_ = 100.0F; break;
    case 2: p0_ = 100.0F; break;
    default: break;
    }
}

}
