#ifndef STEPPER_PROFILE_H
#define STEPPER_PROFILE_H

#include <math.h>
#include <stdint.h>

/* 逐步梯形速度规划：每步调用一次，返回该步使用的速度（步/秒）。
 * 速度按 v² = v0² + 2·a·(1 步) 递推，剩余步数不足以减速到起停速度时开始减速。
 * 纯计算、无硬件依赖，便于主机测试；驱动层在定时中断里调用。 */
typedef struct
{
    uint32_t remaining;   /* 尚未发出的步数 */
    float v;              /* 上一步速度（步/秒） */
    float vmin;           /* 起停速度 */
    float vmax;           /* 巡航速度上限 */
    float accel;          /* 加速度（步/秒²），<=0 表示恒速 vmax */
    uint8_t started;
} stepper_profile_t;

static inline void stepper_profile_start(stepper_profile_t *p,
                                         uint32_t steps,
                                         float vmin,
                                         float vmax,
                                         float accel)
{
    p->remaining = steps;
    p->vmin = vmin;
    p->vmax = (vmax < vmin) ? vmin : vmax;
    p->accel = accel;
    p->v = (accel > 0.0f) ? vmin : p->vmax;
    p->started = 0U;
}

static inline float stepper_profile_next(stepper_profile_t *p)
{
    float v = p->v;

    if (p->remaining == 0U) {
        return 0.0f;
    }

    if ((p->started != 0U) && (p->accel > 0.0f)) {
        float two_a = 2.0f * p->accel;
        float brake_steps = (v * v - p->vmin * p->vmin) / two_a;

        if ((float)p->remaining <= brake_steps + 1.0f) {
            float v2 = v * v - two_a;
            v = (v2 > p->vmin * p->vmin) ? sqrtf(v2) : p->vmin;
        } else if (v < p->vmax) {
            v = sqrtf(v * v + two_a);
            if (v > p->vmax) {
                v = p->vmax;
            }
        }
    }

    p->started = 1U;
    p->v = v;
    --p->remaining;
    return v;
}

#endif /* STEPPER_PROFILE_H */
