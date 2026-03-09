/*===========================================================================
 * [chassis_motor.c] 电机驱动模块实现
 *===========================================================================*/

#include "chassis_motor.h"

void chassis_motor_init(chassis_motor_t *motor)
{
    /* 初始化 PWM 输出引脚，初始占空比为 0（电机不转） */
    pwm_init(motor->pwm_channel, CHASSIS_MOTOR_PWM_FREQ_HZ, 0U);

    /* 初始化方向控制引脚为推挽输出，默认高电平（正向） */
    gpio_init(motor->dir_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    /* 确保启动时电机处于停止状态 */
    pwm_set_duty(motor->pwm_channel, 0U);
}

void chassis_motor_set_pwm(chassis_motor_t *motor, float pwm_signed)
{
    float pwm_abs;
    uint32 duty;

    /* 取绝对值并限幅到 PWM 最大值 */
    pwm_abs = (pwm_signed >= 0.0f) ? pwm_signed : (-pwm_signed);
    if (pwm_abs > CHASSIS_MOTOR_PWM_MAX)
    {
        pwm_abs = CHASSIS_MOTOR_PWM_MAX;
    }
    duty = (uint32)pwm_abs;

    /*
     * 方向控制逻辑：
     *   pwm_signed ≥ 0  →  GPIO_HIGH（正转）
     *   pwm_signed <  0  →  GPIO_LOW （反转）
     *
     * 注意：若实车方向与预期相反，不要修改这里的逻辑，
     *       请到 chassis_config.h 修改对应轮的 DIR_SIGN 宏。
     */
    if (pwm_signed >= 0.0f)
    {
        gpio_high(motor->dir_pin);
    }
    else
    {
        gpio_low(motor->dir_pin);
    }

    pwm_set_duty(motor->pwm_channel, duty);
}

void chassis_motor_stop(chassis_motor_t *motor)
{
    pwm_set_duty(motor->pwm_channel, 0U);
}
