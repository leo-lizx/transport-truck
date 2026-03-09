/*===========================================================================
 * [chassis_imu.c] IMU 姿态模块实现
 *===========================================================================*/

#include "chassis_imu.h"

/* ---- 模块内部状态 ---- */
static volatile float s_yaw_deg     = 0.0f;    /**< 当前积分航向角（度） */
static volatile uint8 s_imu_ready   = 0U;      /**< IMU 是否初始化成功标志 */
static float          s_gyro_z_bias = 0.0f;    /**< Z 轴陀螺零偏（°/s） */

uint8 chassis_imu_init(void)
{
    uint8 result = imu660rb_init();

    s_imu_ready   = (result == 0U) ? 1U : 0U;
    s_yaw_deg     = 0.0f;
    s_gyro_z_bias = 0.0f;

    return result;
}

void chassis_imu_update_5ms(void)
{
    float gyro_z_dps;

    /* 若 IMU 未就绪则跳过，避免读到错误数据 */
    if (0U == s_imu_ready)
    {
        return;
    }

    /* 读取原始陀螺仪数据（SPI 通信） */
    imu660rb_get_gyro();

    /* 将原始 ADC 值换算为角速度（°/s），并减去零偏 */
    gyro_z_dps = imu660rb_gyro_transition(imu660rb_gyro_z) - s_gyro_z_bias;

    /* 欧拉积分：角度 += 角速度 × 步长 */
    s_yaw_deg += gyro_z_dps * CHASSIS_TASK_DT_5MS_S;

    /* 归一化到 [-180, +180]，避免长时间运行后数值溢出 */
    s_yaw_deg = chassis_normalize_angle_deg(s_yaw_deg);
}

float chassis_imu_get_yaw_deg(void)
{
    return s_yaw_deg;
}

void chassis_imu_set_yaw_deg(float yaw_deg)
{
    s_yaw_deg = chassis_normalize_angle_deg(yaw_deg);
}

void chassis_imu_set_gyro_bias(float bias_dps)
{
    s_gyro_z_bias = bias_dps;
}
