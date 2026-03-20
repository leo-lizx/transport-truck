/*===========================================================================
 * [chassis_imu.c] IMU 航向采样与积分模块实现
 *===========================================================================*/

#include "chassis_imu.h"
#include "chassis_config.h"
#include "zf_device_imu660rb.h"
#include "zf_driver_delay.h"
#include <math.h>

/* IMU 积分与滤波参数（统一使用配置头中的宏） */
#define IMU_DT_S                 CHASSIS_TASK_DT_5MS_S
#define IMU_GYRO_DEADZONE_DPS    CHASSIS_IMU_GYRO_DEADZONE_DPS
#define IMU_GYRO_LPF_ALPHA       CHASSIS_IMU_GYRO_LPF_ALPHA
#define IMU_BIAS_ADAPT_ALPHA     CHASSIS_IMU_BIAS_ADAPT_ALPHA
#define IMU_YAW_SIGN             CHASSIS_IMU_YAW_SIGN

/** 全局欧拉角输出（当前控制链只使用 yaw） */
volatile EulerAngle_t car_angle = {0.0f, 0.0f, 0.0f};

/** Z 轴陀螺仪静态零偏（单位：度/秒） */
static float s_gyro_z_bias_dps = 0.0f;

/** Z 轴角速度一阶低通状态（单位：度/秒） */
static float s_yaw_rate_lpf_dps = 0.0f;

/**
 * @brief  初始化 IMU 硬件并执行静态零偏标定
 * @note   标定期间车体必须静止，否则会把运动角速度误认为零偏。
 */
void chassis_imu_init(void)
{
    float gyro_z_sum_dps = 0.0f; /* 标定窗口内角速度累计值 */
    uint16 sample_count = 1000U; /* 标定采样次数 */
    uint16 i;                    /* 采样循环计数 */

    /* 步骤 1: 初始化底层 IMU 设备。 */
    imu660rb_init();

    /* 步骤 2: 在静止状态采样 Z 轴角速度，估计零偏。 */
    for (i = 0U; i < sample_count; ++i)
    {
        imu660rb_get_gyro();
        gyro_z_sum_dps += imu660rb_gyro_transition(imu660rb_gyro_z);
        system_delay_ms(1);
    }

    /* 步骤 3: 计算并保存平均零偏。 */
    s_gyro_z_bias_dps = gyro_z_sum_dps / (float)sample_count;

    /* 步骤 4: 复位航向积分状态。 */
    car_angle.roll = 0.0f;
    car_angle.pitch = 0.0f;
    car_angle.yaw = 0.0f;
    s_yaw_rate_lpf_dps = 0.0f;
}

/**
 * @brief  外部设置航向角（用于重定位校正）
 * @param  yaw_deg 目标航向角（度）
 */
void chassis_imu_set_yaw_deg(float yaw_deg)
{
    car_angle.yaw = chassis_normalize_angle_deg(yaw_deg);
}

/**
 * @brief  读取当前航向角
 * @return 航向角（度，范围 [-180, 180]）
 */
float chassis_imu_get_yaw_deg(void)
{
    return car_angle.yaw;
}

/**
 * @brief  5ms 周期更新航向角
 * @note   处理链路：零偏补偿 -> 死区抑噪 -> 零偏自适应 -> 低通 -> 欧拉积分。
 */
void chassis_imu_update_5ms(void)
{
    float gyro_z_raw_dps;  /* 原始 Z 轴角速度（度/秒） */
    float yaw_rate_dps;    /* 零偏补偿与符号修正后的角速度（度/秒） */

    /* 步骤 1: 采样底层传感器数据。 */
    imu660rb_get_acc();
    imu660rb_get_gyro();
    gyro_z_raw_dps = imu660rb_gyro_transition(imu660rb_gyro_z);

    /* 步骤 2: 做零偏补偿与安装方向修正。 */
    yaw_rate_dps = (gyro_z_raw_dps - s_gyro_z_bias_dps) * IMU_YAW_SIGN;

    /* 步骤 3: 小角速度死区抑噪。 */
    if (fabsf(yaw_rate_dps) < IMU_GYRO_DEADZONE_DPS)
    {
        yaw_rate_dps = 0.0f;
    }

    /* 步骤 4: 静止时缓慢更新零偏，抑制长期漂移。 */
    if (0.0f == yaw_rate_dps)
    {
        s_gyro_z_bias_dps += IMU_BIAS_ADAPT_ALPHA * (gyro_z_raw_dps - s_gyro_z_bias_dps);
    }

    /* 步骤 5: 一阶低通滤波，降低角速度噪声。 */
    s_yaw_rate_lpf_dps += IMU_GYRO_LPF_ALPHA * (yaw_rate_dps - s_yaw_rate_lpf_dps);

    /* 步骤 6: 欧拉积分并归一化到 [-180, 180]。 */
    car_angle.yaw += s_yaw_rate_lpf_dps * IMU_DT_S;
    car_angle.yaw = chassis_normalize_angle_deg(car_angle.yaw);
}
