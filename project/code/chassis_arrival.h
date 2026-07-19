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

#endif /* CHASSIS_ARRIVAL_H */
