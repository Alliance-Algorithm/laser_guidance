#include <exception>
#include <print>

#include "test_utils.hpp"
#include "tracking/hit_progress.hpp"

namespace {

constexpr float kTick = 0.1F;

auto drive_until_locked(rmcs_laser_guidance::HitProgress& hp, const int expected_lock_count)
    -> void {
    using rmcs_laser_guidance::tests::require;

    for (int i = 0; i < 64 && !hp.is_locked(); ++i)
        hp.update(true, kTick);

    require(hp.is_locked(), "expected lock to trigger");
    require(hp.lock_count() == expected_lock_count, "unexpected lock count after trigger");
    require(hp.progress() == 0.0F, "P resets on lock");
}

auto expire_lock(rmcs_laser_guidance::HitProgress& hp) -> void {
    using rmcs_laser_guidance::tests::require;

    hp.update(false, 45.1F);
    require(!hp.is_locked(), "lock should expire after 45s");
}

} // namespace

int main() {
    try {
        using rmcs_laser_guidance::HitProgress;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_near;

        {
            HitProgress hp;
            require(hp.progress() == 0.0F, "initial progress = 0");
            require(hp.stage() == 0, "initial stage = 0");
            require(hp.difficulty() == 1, "initial difficulty = 1");
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
            require_near(hp.progress(), 9.0F, 0.01F, "5 ticks: 0.6*(1+2+3+4+5)=9");
        }

        {
            HitProgress hp;
            hp.update(true, kTick);
            require_near(hp.progress(), 0.6F, 0.01F, "1st 0.1s: P += 0.6");
            require(hp.is_hitting(), "is_hitting reflects current illumination");
            hp.update(true, kTick);
            require_near(hp.progress(), 1.8F, 0.01F, "2nd 0.1s: total=0.6+1.2");
            hp.update(true, kTick);
            require_near(hp.progress(), 3.6F, 0.01F, "3rd 0.1s: total=3.6");
            hp.update(true, kTick);
            require_near(hp.progress(), 6.0F, 0.01F, "4th 0.1s: total=6.0");
        }

        {
            HitProgress hp;
            hp.update(true, 0.05F);
            require(hp.progress() == 0.0F, "no accumulation before 0.1s");
            hp.update(false, 0.01F);
            require(hp.progress() == 0.0F, "interrupt <0.1s resets t/n");
            hp.update(true, kTick);
            require_near(hp.progress(), 0.6F, 0.01F, "starts fresh after interruption");
        }

        {
            HitProgress hp;
            hp.update(true, 0.3F);
            require_near(hp.progress(), 3.6F, 0.01F, "3 ticks: 0.6+1.2+1.8=3.6");
            const float after_hit = hp.progress();
            hp.update(false, 0.5F);
            require_near(hp.progress(), after_hit - 0.25F, 0.01F, "decay is 0.5/s");
            require(!hp.is_hitting(), "not hitting after decay");
        }

        {
            HitProgress hp;
            for (int i = 0; i < 12; ++i)
                hp.update(true, kTick);
            require_near(hp.progress(), 46.8F, 0.05F, "12 ticks: 0.6*(1..12)=46.8");
            require(hp.progress() < 50.0F, "P < P0 after 12 ticks");
            hp.update(true, kTick);
            require(hp.lock_count() == 1, "13th tick crosses P0=50");
            require(hp.is_locked(), "lock triggered");
            require(hp.progress() == 0.0F, "P resets on lock");
            require(hp.stage() == 1, "stage advances to 1");
            require(hp.difficulty() == 2, "difficulty 2 before second lock");
            require_near(hp.p0(), 100.0F, 0.01F, "P0 = 100 after first lock");
            require_near(hp.lock_remaining_s(), 45.0F, 0.5F, "lock timer = 45s");
        }

        {
            HitProgress hp;
            drive_until_locked(hp, 1);
            hp.update(true, 44.9F);
            require(hp.is_locked(), "still locked at 44.9s");
            require(hp.lock_remaining_s() > 0.0F, "timer > 0");
            hp.update(false, 0.2F);
            require(!hp.is_locked(), "unlocked after 45s");
            require(hp.lock_remaining_s() == 0.0F, "timer expired");
        }

        {
            HitProgress hp;
            drive_until_locked(hp, 1);
            expire_lock(hp);
            require(hp.stage() == 1, "stage 1 before second lock");
            require(hp.difficulty() == 2, "difficulty 2 before second lock");

            drive_until_locked(hp, 2);
            expire_lock(hp);
            require(hp.stage() == 2, "stage 2 before third lock");
            require(hp.difficulty() == 2, "difficulty 2 before third lock");

            drive_until_locked(hp, 3);
            expire_lock(hp);
            require(hp.stage() == 3, "stage 3 before fourth lock");
            require(hp.difficulty() == 3, "difficulty 3 before fourth lock");

            drive_until_locked(hp, 4);
            expire_lock(hp);
            require(hp.stage() == 4, "stage 4 before fifth lock");
            require(hp.difficulty() == 3, "difficulty 3 before fifth lock");

            drive_until_locked(hp, 5);
            require(hp.is_locked(), "fifth lock holds for 45s");
            expire_lock(hp);
            require(hp.is_exhausted(), "exhausted after 5th lock expires");
            const float p_before = hp.progress();
            hp.update(true, 1.0F);
            require(hp.progress() == p_before, "no change when exhausted");
            require(hp.lock_count() == 5, "lock_count stays at 5");
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
