#pragma once

namespace rmcs_laser_guidance {

// 检测置信度阈值按类别区分：
//   - 红蓝（class 0/1，敌机颜色）调高，抑制误报导致的引导抖动；
//   - 紫色/无色（class 2/3，RM2026 反制目标）保持低阈值，不丢反制检测。
inline constexpr float kConfidenceThreshold = 0.25F;
inline constexpr float kEnemyColorConfidenceThreshold = 0.5F;

inline auto confidence_threshold_for(const int class_id) -> float {
    return (class_id == 0 || class_id == 1) ? kEnemyColorConfidenceThreshold
                                            : kConfidenceThreshold;
}

} // namespace rmcs_laser_guidance
