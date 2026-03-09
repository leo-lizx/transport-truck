#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

/*===========================================================================
 * [chassis_imu.h] IMU 姿态模块
 *
 *   封装 IMU660RB 6 轴惯性传感器的初始化和航向角估计。
 *   当前仅使用 Z 轴陀螺仪做航向角（yaw）积分。
 *
 *   使用的传感器：IMU660RB
 *     接口：SPI
 *     量程：陀螺 ±2000°/s，加速度 ±16g
 *     传感器引脚在逐飞库 zf_device_imu660rb.h 中配置
 *
 *   调用流程：
 *     1. chassis_imu_init()         — 启动时调用一次
 *     2. chassis_imu_update_5ms()   — 在 5ms PIT 中断中调用
 *     3. chassis_imu_get_yaw_deg()  — 随时获取当前航向角
 *
 *   零偏标定建议：
 *     开机后让车静止 1~2 秒，采样 200~500 次 Z 轴角速度取平均，
 *     作为零偏调用 chassis_imu_set_gyro_bias() 写入。
 *===========================================================================*/

#include "chassis_config.h"

/**
 * @brief  初始化 IMU 传感器
 * @return 0 = 成功，非 0 = 失败（传感器通信异常，检查接线和SPI配置）
 */
uint8 chassis_imu_init(void);

/**
 * @brief  5ms 周期任务：读取陀螺仪 Z 轴并积分航向角
 *         必须以 5ms 固定间隔调用，积分精度依赖于调用周期的稳定性
 */
void chassis_imu_update_5ms(void);

/**
 * @brief  获取当前航向角
 * @return 航向角（度），范围 [-180, +180]，逆时针为正
 */
float chassis_imu_get_yaw_deg(void);

/**
 * @brief  直接设置航向角值（视觉重定位时使用）
 * @param  yaw_deg  新的航向角（度），会自动归一化到 [-180, +180]
 */
void chassis_imu_set_yaw_deg(float yaw_deg);

/**
 * @brief  设置陀螺仪 Z 轴零偏补偿值
 * @param  bias_dps  零偏值（°/s），后续每次读取会自动减去此值
 */
void chassis_imu_set_gyro_bias(float bias_dps);

#endif /* CHASSIS_IMU_H */
