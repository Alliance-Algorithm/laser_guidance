#include "tracking/hit_progress.hpp"

#include <algorithm>

namespace rmcs_laser_guidance {
namespace {

constexpr float kTickSeconds = 0.1F;
constexpr float kProgressDecayPerSecond = 0.5F;
constexpr float kProgressIncrementScale = 0.6F;
constexpr float kMaxProgress = 100.0F;
constexpr int kMaxLocks = 5;

auto difficulty_for_stage(const int stage) -> int {
    switch (std::clamp(stage, 0, 4)) {
    case 0: return 1;
    case 1:
    case 2: return 2;
    case 3:
    case 4: return 3;
    default: return 1;
    }
}

} // namespace

auto HitProgress::progress_ratio() const noexcept -> float {
    if (p0_ <= 0.0F)
        return 0.0F;
    return std::clamp(p_ / p0_, 0.0F, 1.0F);
}

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

void HitProgress::update(bool is_purple, float dt_s) {
    update(is_purple, is_purple, dt_s);
}

void HitProgress::update(bool is_purple, bool is_colorless, float dt_s) {
    if (exhausted_)
        return;

    if (locked_) {
        hitting_ = false;
        lock_timer_ = std::max(0.0F, lock_timer_ - dt_s);
        if (lock_timer_ <= 0.0F) {
            locked_ = false;
            if (lock_count_ >= kMaxLocks) {
                exhausted_ = true;
                return;
            }
        }
        return;
    }

    const bool is_hitting_target = is_purple || (difficulty_ >= 3 && is_colorless);
    if (!is_hitting_target) {
        hitting_ = false;
        p_ = std::max(0.0F, p_ - kProgressDecayPerSecond * dt_s);
        t_ = 0.0F;
        n_ = 0;
        return;
    }

    hitting_ = true;
    t_ += dt_s;
    int ticks = static_cast<int>(t_ / kTickSeconds + 1.0e-6F);
    if (ticks > 0) {
        const int increment_series = n_ * ticks + ticks * (ticks + 1) / 2;
        p_ = std::clamp(
            p_ + kProgressIncrementScale * static_cast<float>(increment_series), 0.0F,
            kMaxProgress);
        n_ += ticks;
        t_ -= static_cast<float>(ticks) * kTickSeconds;
    }

    // RM2026 §5.6.3: reaching P0 locks the launcher immediately; P resets and
    // the 45s lock begins. There is no local confirmation step.
    if (p_ >= p0_) {
        trigger_lock();
    }
}

void HitProgress::trigger_lock() {
    ++lock_count_;
    locked_ = true;
    lock_timer_ = 45.0F;
    p_ = 0.0F;
    t_ = 0.0F;
    n_ = 0;
    hitting_ = false;
    advance_stage();
}

void HitProgress::sync_official_counter(const int count) {
    const int clamped = std::clamp(count, 0, kMaxLocks);
    const bool advanced = clamped > lock_count_;
    lock_count_ = clamped;
    stage_ = std::min(clamped, kMaxLocks - 1);
    difficulty_ = difficulty_for_stage(stage_);
    p0_ = difficulty_ == 1 ? 50.0F : 100.0F;
    if (advanced) {
        // 官方确认新一次反制成功：重置 P 并进入 45s 锁定（与本地触发一致）。
        // 官方边沿不被本地锁定期门控，避免漏计。
        locked_ = true;
        lock_timer_ = 45.0F;
        p_ = 0.0F;
        t_ = 0.0F;
        n_ = 0;
        hitting_ = false;
    }
    // exhausted_ 不在此设置：第 5 次锁定仍持续 45s，由 update() 在锁定期结束后置位
}

void HitProgress::advance_stage() {
    stage_ = std::min(lock_count_, kMaxLocks - 1);
    // The first lock (stage 0) runs against the constructor default p0_=50;
    // every lock after it requires the full bar. trigger_lock() has already
    // incremented lock_count_, so lock_count_ is never 0 here.
    p0_ = 100.0F;
    difficulty_ = difficulty_for_stage(stage_);
}

} // namespace rmcs_laser_guidance
