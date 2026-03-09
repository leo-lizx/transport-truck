/*===========================================================================
 * [chassis_encoder.c] 编码器测速模块实现
 *===========================================================================*/

#include "chassis_encoder.h"

void chassis_encoder_init(chassis_encoder_t *enc)
{
    /* 初始化正交编码器硬件计数器 */
    encoder_quad_init(enc->index, enc->ch1_pin, enc->ch2_pin);

    /* 清零计数器，确保从 0 开始累计 */
    encoder_clear_count(enc->index);

    enc->speed_mps = 0.0f;
}

void chassis_encoder_update(chassis_encoder_t *enc, float dt_s)
{
    int16 count_delta;
    float meter_per_count;

    /* 读取自上次清零以来的脉冲增量 */
    count_delta = encoder_get_count(enc->index);

    /* 立即清零计数器，为下一个采样周期做准备 */
    encoder_clear_count(enc->index);

    /*
     * 脉冲增量 → 轮线速度 (m/s) 换算：
     *   meter_per_count = 轮子周长 / 每转脉冲数
     *                   = (2 × π × R) / N
     *   speed = count_delta × meter_per_count / dt × dir_sign
     */
    meter_per_count = (2.0f * CHASSIS_PI_F * CHASSIS_WHEEL_RADIUS_M)
                    / CHASSIS_ENCODER_COUNTS_PER_REV;

    enc->speed_mps = (float)count_delta * meter_per_count / dt_s * enc->dir_sign;
}

float chassis_encoder_get_speed(const chassis_encoder_t *enc)
{
    return enc->speed_mps;
}
