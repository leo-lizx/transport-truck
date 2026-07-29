#ifndef CHASSIS_MOTION_PROFILE_H
#define CHASSIS_MOTION_PROFILE_H

#include <math.h>
#include <stdint.h>


typedef struct
{
    float speed_cap_mps;
    float accel_limit_mps2;
    uint8_t braking;
} chassis_motion_profile_t;

/* 依据实测轴向速度计算短程加速/制动包络。
 * latency_s 同时覆盖 20ms 控制拍和轮速低通延迟；解析 v_allow 时使用
 * d = v²/(2a) + v·latency + margin 的正根，使进入制动点速度连续。 */
static inline chassis_motion_profile_t chassis_motion_profile_calculate(
    float axis_error_m,
    float axis_velocity_mps,
    float arrival_tolerance_m,
    float normal_accel_mps2,
    float short_accel_mps2,
    float brake_decel_mps2,
    float speed_limit_mps,
    float short_move_dist_m,
    float latency_s,
    float margin_m,
    float brake_floor_mps)
{
    chassis_motion_profile_t profile;
    float abs_error_m = (axis_error_m >= 0.0f) ? axis_error_m : -axis_error_m;
    float remaining_m;
    float toward_speed_mps;
    float stop_distance_m;

    if (arrival_tolerance_m < 0.0f) arrival_tolerance_m = 0.0f;
    if (latency_s < 0.0f) latency_s = 0.0f;
    if (margin_m < 0.0f) margin_m = 0.0f;

    remaining_m = abs_error_m - arrival_tolerance_m;
    if (remaining_m < 0.0f) remaining_m = 0.0f;

    profile.speed_cap_mps = (speed_limit_mps > 0.0f) ? speed_limit_mps : 0.0f;
    profile.accel_limit_mps2 = normal_accel_mps2;
    profile.braking = 0U;

    if (remaining_m <= 0.0f) {
        profile.speed_cap_mps = 0.0f;
        profile.braking = 1U;
        return profile;
    }

    toward_speed_mps = (axis_error_m >= 0.0f)
                     ? axis_velocity_mps : -axis_velocity_mps;
    if (toward_speed_mps < 0.0f) toward_speed_mps = 0.0f;

    if (brake_decel_mps2 > 1e-6f) {
        stop_distance_m = toward_speed_mps * toward_speed_mps
                        / (2.0f * brake_decel_mps2)
                        + toward_speed_mps * latency_s
                        + margin_m;

        if (remaining_m <= stop_distance_m + 1e-6f) {
            float usable_distance_m = remaining_m - margin_m;
            float allowed_speed_mps = 0.0f;

            profile.braking = 1U;
            if (usable_distance_m > 0.0f) {
                float latency_speed_mps = brake_decel_mps2 * latency_s;
                float discriminant = latency_speed_mps * latency_speed_mps
                                   + 2.0f * brake_decel_mps2
                                   * usable_distance_m;
                allowed_speed_mps = sqrtf(discriminant) - latency_speed_mps;
            }
            if (allowed_speed_mps < brake_floor_mps) {
                allowed_speed_mps = brake_floor_mps;
            }
            if (allowed_speed_mps < profile.speed_cap_mps) {
                profile.speed_cap_mps = allowed_speed_mps;
            }
            return profile;
        }
    }

    if ((abs_error_m <= short_move_dist_m) &&
        (short_accel_mps2 > normal_accel_mps2)) {
        profile.accel_limit_mps2 = short_accel_mps2;
    }
    return profile;
}

/* 动态停车包络已按实测速度给出速度上限，进入制动段后由速度上限和
 * 非对称 ramp 负责主动制动。此时若驱动轴继续叠加 D 阻尼，末端可能
 * 被重复减速到零；车速归零后 D 项消失，又会重新起步。 */
static inline float chassis_motion_drive_damping_gain(
    uint8_t braking,
    float normal_damping_gain)
{
    return (braking != 0U) ? 0.0f : normal_damping_gain;
}

/* 同一速度目标下，减小幅值或反向属于主动制动，使用更大的减速度；
 * 增大同向幅值才使用加速度。 */
static inline float chassis_motion_ramp_step(float current_mps,
                                             float target_mps,
                                             float accel_limit_mps2,
                                             float decel_limit_mps2,
                                             float dt_s)
{
    float abs_current_mps = (current_mps >= 0.0f) ? current_mps : -current_mps;
    float abs_target_mps = (target_mps >= 0.0f) ? target_mps : -target_mps;
    uint8_t is_reversing = (uint8_t)((current_mps * target_mps) < 0.0f);
    uint8_t is_decelerating = (uint8_t)(
        (is_reversing != 0U) ||
        (abs_target_mps < abs_current_mps));
    float rate_limit_mps2 = (is_decelerating != 0U)
                          ? decel_limit_mps2 : accel_limit_mps2;
    float delta_mps;

    if ((rate_limit_mps2 <= 0.0f) || (dt_s <= 0.0f)) {
        return current_mps;
    }

    delta_mps = rate_limit_mps2 * dt_s;
    if (is_reversing != 0U) {
        if ((current_mps > delta_mps) || (current_mps < -delta_mps)) {
            return (current_mps > 0.0f)
                 ? current_mps - delta_mps
                 : current_mps + delta_mps;
        }
        return 0.0f;
    }
    if (target_mps > current_mps + delta_mps) {
        return current_mps + delta_mps;
    }
    if (target_mps < current_mps - delta_mps) {
        return current_mps - delta_mps;
    }
    return target_mps;
}

/* 静摩擦前馈只负责起步/追速。目标速度已低于同向反馈时属于制动，
 * 必须撤掉正向前馈，避免与轮速 PID 的制动力互相抵消。 */
static inline uint8_t chassis_motion_breakaway_allowed(
    float target_mps,
    float feedback_mps,
    float target_threshold_mps,
    float stopped_threshold_mps)
{
    float abs_target_mps = (target_mps >= 0.0f) ? target_mps : -target_mps;
    float abs_feedback_mps = (feedback_mps >= 0.0f)
                           ? feedback_mps : -feedback_mps;

    if (abs_target_mps <= target_threshold_mps) {
        return 0U;
    }
    if (((target_mps * feedback_mps) > 0.0f) &&
        (abs_target_mps < abs_feedback_mps)) {
        return 0U;
    }
    if (abs_feedback_mps < stopped_threshold_mps) {
        return 1U;
    }
    if ((target_mps * feedback_mps) > 0.0f) {
        return 1U;
    }
    return 0U;
}

#endif /* CHASSIS_MOTION_PROFILE_H */
