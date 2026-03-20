/*===========================================================================
 * [chassis_encoder.c] 编码器测速模块实现
 *===========================================================================*/

#include "chassis_encoder.h"

/**
 * @brief  初始化单路编码器
 * @param  enc 编码器对象指针
 * @note   初始化后会清零计数器，首帧速度输出为 0。
 */
void chassis_encoder_init(chassis_encoder_t *enc)
{
    /* 初始化正交编码器硬件计数器 */
    encoder_quad_init(enc->index, enc->ch1_pin, enc->ch2_pin);

    /* 清零计数器，确保从 0 开始累计 */
    encoder_clear_count(enc->index);

    enc->speed_mps = 0.0f;
}

/**
 * @brief  按固定周期更新编码器速度
 * @param  enc  编码器对象指针
 * @param  dt_s 采样周期（秒）
 */
void chassis_encoder_update(chassis_encoder_t *enc, float dt_s)
{
    int16 pulse_delta_count;      /* 本周期脉冲增量（计数器差分值） */
    float wheel_meter_per_count;  /* 每个脉冲对应的轮线位移（米） */

    /* 读取自上次清零以来的脉冲增量 */
    pulse_delta_count = encoder_get_count(enc->index);

    /* 立即清零计数器，为下一个采样周期做准备 */
    encoder_clear_count(enc->index);

    /*
     * 脉冲增量 → 轮线速度 (m/s) 换算：
     *   meter_per_count = 轮子周长 / 每转脉冲数
     *                   = (2 × π × R) / N
     *   speed = count_delta × meter_per_count / dt × dir_sign
     */
    wheel_meter_per_count = (2.0f * CHASSIS_PI_F * CHASSIS_WHEEL_RADIUS_M)
                          / CHASSIS_ENCODER_COUNTS_PER_REV;

    enc->speed_mps = (float)pulse_delta_count * wheel_meter_per_count / dt_s * enc->dir_sign;
}

/**
 * @brief  读取最近一次更新后的轮速
 * @param  enc 编码器对象指针
 * @return 轮速（m/s）
 */
float chassis_encoder_get_speed(const chassis_encoder_t *enc)
{
    return enc->speed_mps;
}
