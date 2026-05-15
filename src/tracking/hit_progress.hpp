#pragma once

namespace rmcs_laser_guidance {

/// @brief 空中机器人被激光瞄准进度计算器 (RoboMaster 2026 规则 §5.6.3)
///
/// 阶段与 P0 阈值:
///   Stage 0 (初始): P0 = 50,  检测区域无缩减
///   Stage 1 (已锁定1次): P0 = 100, 检测区域纵向缩减为 3/5
///   Stage 2 (已锁定2次): P0 = 100, 检测区域纵向缩减为 1/5, 不再主动发光
///   锁定3次后: 耗尽, 不再计算
///
/// P 值规则:
///   - 未命中时: P 以 0.5/s 衰减至 0, t/n 归零
///   - 命中时:   每累计 0.1s → P += n (n = 1,2,3...)
///   - 照射不足 0.1s 中断: t/n 归零
class HitProgress {
public:
    HitProgress() = default;

    /// 每帧更新
    /// @param is_hit  当前帧目标是否为紫色 (class_id == 0)
    /// @param dt_s    帧间隔 (秒)
    void update(bool is_hit, float dt_s);

    [[nodiscard]] float  progress()        const noexcept { return p_; }
    [[nodiscard]] float  progress_ratio()  const noexcept;
    [[nodiscard]] bool   is_hitting()      const noexcept { return t_ > 0.0F; }
    [[nodiscard]] bool   is_locked()       const noexcept { return locked_; }
    [[nodiscard]] float  lock_remaining_s() const noexcept { return lock_timer_; }
    [[nodiscard]] int    lock_count()      const noexcept { return lock_count_; }
    [[nodiscard]] int    stage()           const noexcept { return stage_; }
    [[nodiscard]] float  p0()              const noexcept { return p0_; }
    [[nodiscard]] bool   is_exhausted()    const noexcept { return exhausted_; }

private:
    void trigger_lock();
    void advance_stage();

    float p_     = 0.0F;
    float t_     = 0.0F;
    int   n_     = 0;
    float p0_    = 50.0F;
    int   stage_      = 0;
    int   lock_count_ = 0;
    bool  locked_     = false;
    bool  exhausted_  = false;
    float lock_timer_ = 0.0F;
};

} // namespace rmcs_laser_guidance
