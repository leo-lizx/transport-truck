#ifndef CHASSIS_MOTOR_H
#define CHASSIS_MOTOR_H

/*===========================================================================
 * [chassis_motor.h] 电机驱动模块
 *
 *   封装 DRV8701E 驱动板的 PWM + 方向控制逻辑。
 *   每个电机需要两个引脚：
 *     PWM 引脚 — 控制转速（占空比越高，转速越快）
 *     DIR 引脚 — 控制转向（高电平正转，低电平反转）
 *
 *   dir_sign 为方向修正系数，用于修正实车安装方向：
 *     若某轮转向与预期相反，在 chassis_config.h 中将
 *     对应的 DIR_SIGN 改为 -1.0f 即可，不需改此模块代码。
 *
 *   使用流程：
 *     1. chassis_motor_init()    — 初始化 PWM 频率和方向引脚
 *     2. chassis_motor_set_pwm() — 设置有符号 PWM 值
 *     3. chassis_motor_stop()    — 紧急停止
 *===========================================================================*/

#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "chassis_config.h"

/** 单个电机的驱动配置 */
typedef struct
{
    pwm_channel_enum pwm_channel;   /**< PWM 硬件通道（见 chassis_config.h 引脚宏） */
    gpio_pin_enum    dir_pin;       /**< 方向控制 GPIO 引脚 */
    float            dir_sign;      /**< 方向修正系数: +1.0=正向, -1.0=反向 */
} chassis_motor_t;

/**
 * @brief  初始化电机驱动（配置 PWM 频率和方向引脚）
 *         初始化后电机处于停止状态（占空比为 0）
 * @param  motor  电机结构体指针
 */
void chassis_motor_init(chassis_motor_t *motor);

/**
 * @brief  设置电机 PWM 输出
 * @param  motor       电机结构体指针
 * @param  pwm_signed  有符号 PWM 值：
 *                       正值 → 正转，负值 → 反转，0 → 停止
 *                     绝对值超过 CHASSIS_MOTOR_PWM_MAX 会被自动钳位
 */
void chassis_motor_set_pwm(chassis_motor_t *motor, float pwm_signed);

/**
 * @brief  停止电机输出（PWM 设为 0）
 * @param  motor  电机结构体指针
 */
void chassis_motor_stop(chassis_motor_t *motor);

#endif /* CHASSIS_MOTOR_H */
