#include <cmath>
#include <exception>
#include <print>

#include "tracking/hit_progress.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::HitProgress;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_near;

        constexpr float kTick = 0.1F;

        {
            HitProgress hp;
            require(hp.progress() == 0.0F, "initial progress = 0");
            require(hp.stage() == 0, "initial stage = 0");
            require(hp.p0() == 50.0F, "initial P0 = 50");
            require(hp.lock_count() == 0, "initial lock_count = 0");
            require(!hp.is_locked(), "not locked initially");
            require(!hp.is_exhausted(), "not exhausted initially");
            require(!hp.is_hitting(), "not hitting initially");
        }

        {
            HitProgress hp;
            hp.update(false, 2.0F);
            require(hp.progress() == 0.0F, "decay clamped at 0");
        }

        {
            HitProgress hp;
            hp.update(true, 0.5F);
            require_near(hp.progress(), 15.0F, 0.1F, "5 ticks: 1+2+3+4+5=15");
        }

        {
            HitProgress hp;
            for (int i = 0; i < 10; ++i)
                hp.update(true, kTick);
            require(hp.progress() == 0.0F, "P resets on lock");
            require(hp.lock_count() == 1, "lock triggered at P0=50");
        }

        {
            HitProgress hp;
            hp.update(true, kTick);
            require_near(hp.progress(), 1.0F, 0.01F, "1st 0.1s: P += 1");
            hp.update(true, kTick);
            require_near(hp.progress(), 3.0F, 0.01F, "2nd 0.1s: P += 2, total=3");
            hp.update(true, kTick);
            require_near(hp.progress(), 6.0F, 0.01F, "3rd 0.1s: P += 3, total=6");
            hp.update(true, kTick);
            require_near(hp.progress(), 10.0F, 0.01F, "4th 0.1s: P += 4, total=10");
        }

        {
            HitProgress hp;
            hp.update(true, 0.05F);
            require(hp.progress() == 0.0F, "no accumulation before 0.1s");
            hp.update(false, 0.01F);
            require(hp.progress() == 0.0F, "interrupt <0.1s resets t/n");
        }

        {
            HitProgress hp;
            hp.update(true, 0.09F);
            hp.update(false, 0.01F);
            require(hp.progress() == 0.0F, "interrupt resets, no accumulation");
            hp.update(true, kTick);
            require_near(hp.progress(), 1.0F, 0.01F, "starts fresh: P+=1");
        }

        {
            HitProgress hp;
            hp.update(true, 0.3F);
            const float after_hit = hp.progress();
            require(after_hit > 0.0F, "P accumulated after 3 ticks (6)");
            hp.update(false, 0.5F);
            require(hp.progress() < after_hit, "decay reduces P");
            require(!hp.is_hitting(), "not hitting after decay");
        }

        {
            HitProgress hp;
            for (int i = 0; i < 9; ++i)
                hp.update(true, kTick);
            require(hp.progress() < 50.0F, "P < P0 after 9 ticks (45)");
            hp.update(true, kTick);
            require(hp.lock_count() == 1, "10th tick crosses P0=50 → lock");
            require(hp.is_locked(), "lock triggered");
            require(hp.progress() == 0.0F, "P resets on lock");
            require(hp.lock_count() == 1, "lock_count = 1");
            require(hp.stage() == 1, "stage advances to 1");
            require_near(hp.p0(), 100.0F, 0.01F, "P0 = 100 at stage 1");
            require_near(hp.lock_remaining_s(), 45.0F, 0.5F, "lock timer = 45s");
        }

        {
            HitProgress hp;
            hp.update(true, 1.0F);
            require(hp.is_locked(), "locked after stage 0");
            hp.update(true, 44.9F);
            require(hp.is_locked(), "still locked at 44.9s");
            require(hp.lock_remaining_s() > 0.0F, "timer > 0");
            hp.update(false, 0.2F);
            require(!hp.is_locked(), "unlocked after 45s");
            require(hp.lock_remaining_s() == 0.0F, "timer expired");
        }

        {
            HitProgress hp;
            hp.update(true, 1.0F);
            require(hp.lock_count() == 1, "lock 1 done");
            hp.update(false, 46.0F);
            require(!hp.is_locked(), "lock 1 expired");
            hp.update(true, 5.0F);
            require(hp.lock_count() == 2, "lock 2 done");
            require(hp.stage() == 2, "stage 2");
            require_near(hp.p0(), 100.0F, 0.01F, "P0 = 100 at stage 2");
            hp.update(false, 46.0F);
            require(!hp.is_locked(), "lock 2 expired");
            hp.update(true, 5.0F);
            require(hp.lock_count() == 3, "lock 3 done");
            require(hp.is_locked(), "locked after lock 3");
            hp.update(false, 46.0F);
            require(hp.is_exhausted(), "exhausted after 3rd lock expires");
            const float p_before = hp.progress();
            hp.update(true, 1.0F);
            require(hp.progress() == p_before, "no change when exhausted");
            require(hp.lock_count() == 3, "lock_count stays at 3");
        }

        {
            HitProgress hp;
            hp.update(true, 0.8F);
            require(hp.progress_ratio() > 0.0F, "ratio > 0 when hitting");
            hp.update(false, 2.0F);
            require(hp.progress_ratio() < 1.0F, "ratio < 1.0 during decay");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "hit_progress_test failed: {}", e.what());
        return 1;
    }
}
