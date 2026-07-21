#ifndef CHASSIS_ARRIVAL_H
#define CHASSIS_ARRIVAL_H

#include <stdint.h>

typedef struct
{
    uint8_t hold_translation;
    uint8_t arrived;
} chassis_arrival_decision_t;

/* 到点驻留决策保持为纯函数式小状态机，便于在无硬件环境验证。
 * 进入位置窗口后立即冻结平移；速度或航向尚未稳定时不累计，但也不重新加速。
 * 只有实时位置离开窗口才释放冻结并清零驻留计数。 */
static inline chassis_arrival_decision_t chassis_arrival_dwell_update(
    uint16_t *dwell_count,
    uint8_t position_ok,
    uint8_t velocity_ok,
    uint8_t yaw_ok,
    uint16_t required_count)
{
    chassis_arrival_decision_t decision = {0U, 0U};

    if (position_ok == 0U) {
        *dwell_count = 0U;
        return decision;
    }

    decision.hold_translation = 1U;
    if ((velocity_ok == 0U) || (yaw_ok == 0U)) {
        *dwell_count = 0U;
        return decision;
    }

    if (*dwell_count < required_count) {
        ++(*dwell_count);
    }
    decision.arrived = (uint8_t)(*dwell_count >= required_count);
    return decision;
}

/* 保持主动制动直到所有轮速足够低，再清 PID/PWM，避免增量式 PID 残留输出。 */
static inline uint8_t chassis_wheels_idle_stop_ready(
    const float *wheel_targets_mps,
    const float *wheel_feedback_mps,
    uint8_t wheel_count,
    float target_epsilon_mps,
    float feedback_epsilon_mps)
{
    uint8_t i;

    if (wheel_count == 0U) {
        return 0U;
    }

    for (i = 0U; i < wheel_count; ++i) {
        float abs_target = (wheel_targets_mps[i] >= 0.0f)
                         ? wheel_targets_mps[i] : -wheel_targets_mps[i];
        float abs_feedback = (wheel_feedback_mps[i] >= 0.0f)
                           ? wheel_feedback_mps[i] : -wheel_feedback_mps[i];

        if ((abs_target >= target_epsilon_mps) ||
            (abs_feedback >= feedback_epsilon_mps)) {
            return 0U;
        }
    }

    return 1U;
}

/* IMU 的静止授权只反映车轮是否真实运动，不能与“目标是否为零”的停轮条件共用。
 * 否则角度环刚产生非零目标时，即使车轮尚未转动，也会永久禁止零偏校正。 */
static inline uint8_t chassis_wheels_feedback_stationary(
    const float *wheel_feedback_mps,
    uint8_t wheel_count,
    float feedback_epsilon_mps)
{
    uint8_t i;

    if (wheel_count == 0U) {
        return 0U;
    }

    for (i = 0U; i < wheel_count; ++i) {
        float abs_feedback = (wheel_feedback_mps[i] >= 0.0f)
                           ? wheel_feedback_mps[i] : -wheel_feedback_mps[i];
        if (abs_feedback >= feedback_epsilon_mps) {
            return 0U;
        }
    }

    return 1U;
}

#endif /* CHASSIS_ARRIVAL_H */
