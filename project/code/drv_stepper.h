#ifndef DRV_STEPPER_H
#define DRV_STEPPER_H

#include <stdint.h>

/* 升降步进驱动：D12 脉冲、D13 方向、D4 限位开关；GPT1 定时中断产生脉冲。
 * 位置单位为微步，正方向 = 远离限位开关。 */
void stepper_init(void);

/* 相对移动 delta_steps 微步，梯形加减速；speed/accel 单位为步/秒、步/秒²。
 * 朝限位开关方向运动时，开关触发即停，并置 stepper_home_hit()。
 * 运动中返回 0 且不接受新指令；delta 为 0 时直接返回 1。 */
uint8_t stepper_move(int32_t delta_steps, float speed_sps, float accel_sps2);

/* 立即停止脉冲（不减速）；当前只用于低速台架测试的手动急停。 */
void stepper_stop(void);

uint8_t stepper_is_busy(void);
int32_t stepper_get_position(void);
void stepper_set_position(int32_t steps);

/* 最近一次运动是否因限位开关触发而结束。 */
uint8_t stepper_home_hit(void);

/* 由 isr.c 的 GPT1_IRQHandler 调用。 */
void stepper_isr(void);

#endif /* DRV_STEPPER_H */
