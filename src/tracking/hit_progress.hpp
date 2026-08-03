#pragma once

namespace rmcs_laser_guidance {

/// @brief 空中机器人被激光瞄准进度计算器 (RoboMaster 2026 规则 §5.6.3)
///
/// 阶段 / difficulty / 相机参数:
///   Stage 0  第一次反制前: difficulty=1, P0=50,  检测区无缩减, 发光 → 相机 lit (第一套)
///   Stage 1  第二次反制前: difficulty=2, P0=100, 纵向 3/5, 发光 → 相机 lit (第一套)
///   Stage 2  第三次反制前: difficulty=2, P0=100, 纵向 3/5, 发光 → 相机 lit (第一套)
///   Stage 3  第四次反制前: difficulty=3, P0=100, 纵向 1/5, 不发光 → 相机 unlit (第二套)
///   Stage 4  第五次反制前: difficulty=3, P0=100, 纵向 1/5, 不发光 → 相机 unlit (第二套)
///   锁定 5 次后: 耗尽, 不再计算
///
/// 前三阶段由 Purple HIT (class_id==2) 累计并等待 Colorless (class_id==3)
/// 确认；difficulty 3 阶段由 Colorless 持续检测驱动。该状态来自本地视觉，非裁判系统。
///
/// P 值规则:
///   - 未命中时: P 以 0.5/s 衰减至 0, t/n 归零
///   - 命中时:   每累计 0.1s → P += 0.6*n (n = 1,2,3...)
///   - 照射不足 0.1s 中断: t/n 归零
class HitProgress {
public:
    HitProgress() = default;

    /// 每帧更新
    /// @param is_purple  当前帧目标是否为紫色 (class_id == 2)
    /// @param is_colorless 当前帧是否识别为无色 (class_id == 3)
    /// @param dt_s    帧间隔 (秒)
    void update(bool is_purple, bool is_colorless, float dt_s);

    // Compatibility for existing non-competition diagnostics that model a hit
    // as both the illumination and completion signal.
    void update(bool is_purple, float dt_s);

    [[nodiscard]] float progress() const noexcept { return p_; }
    [[nodiscard]] float progress_ratio() const noexcept;
    [[nodiscard]] bool is_hitting() const noexcept { return hitting_; }
    [[nodiscard]] bool is_locked() const noexcept { return locked_; }
    [[nodiscard]] float lock_remaining_s() const noexcept { return lock_timer_; }
    [[nodiscard]] int lock_count() const noexcept { return lock_count_; }
    [[nodiscard]] int stage() const noexcept { return stage_; }
    [[nodiscard]] int difficulty() const noexcept { return difficulty_; }
    [[nodiscard]] float p0() const noexcept { return p0_; }
    [[nodiscard]] bool is_exhausted() const noexcept { return exhausted_; }

    /// 整局归零（开局调用）：P/t/n/P0/stage/difficulty/lock_count/锁定/耗尽 全部复位
    void reset();
    /// 裁判系统 0x020C 官方反制状态上升沿校核。返回 true 表示本地漏检、补计一次锁定。
    [[nodiscard]] auto note_official_countered() -> bool;
    /// 用官方反制成功次数（0x020C 边沿计数，每局归零）权威同步阶段：次数
    /// 增加时进入 45s 锁定并重置 P；比赛窗口内每帧调用，离线/窗口外不用。
    void sync_official_counter(int count);

private:
    void trigger_lock();
    void advance_stage();

    float p_ = 0.0F;
    float t_ = 0.0F;
    int n_ = 0;
    float p0_ = 50.0F;
    int stage_ = 0;
    int difficulty_ = 1;
    int lock_count_ = 0;
    bool locked_ = false;
    bool exhausted_ = false;
    bool hitting_ = false;
    float lock_timer_ = 0.0F;
};

} // namespace rmcs_laser_guidance
