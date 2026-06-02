#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

/*===========================================================================
 * [chassis_imu.h] IMU 航向采样与积分模块
 *
 *   本模块对外提供统一的 IMU 接口，核心职责：
 *     1) 初始化陀螺仪并执行静态零偏标定
 *     2) 以 5ms 固定周期更新航向角（yaw）
 *     3) 提供航向角读写接口给 ctrl 模块
 *
 *   注意：当前闭环控制只使用 yaw，roll/pitch 保留为扩展字段。
 *===========================================================================*/

#include "zf_common_typedef.h"

/** IMU 欧拉角输出（单位：度） */
typedef struct
{
    float roll;   /**< 横滚角 roll（度） */
    float pitch;  /**< 俯仰角 pitch（度） */
    float yaw;    /**< 航向角 yaw（度） */
} EulerAngle_t;

/** 全局 IMU 欧拉角输出（由 5ms 任务持续更新） */
extern volatile EulerAngle_t car_angle;

/**
 * @brief  初始化 IMU 模块
 * @note   包含硬件初始化与静态零偏标定，调用时车体需静止。
 */
void chassis_imu_init(void);

/**
 * @brief  5ms 周期更新 IMU 航向角
 * @note   需在严格 5ms 节拍下调用，保证积分精度。
 */
void chassis_imu_update_5ms(void);

/**
 * @brief  读取当前航向角
 * @return 航向角（度，范围 [-180, 180]）
 */
float chassis_imu_get_yaw_deg(void);

/**
 * @brief  外部写入航向角（用于位姿校正）
 * @param  yaw_deg 目标航向角（度）
 */
void chassis_imu_set_yaw_deg(float yaw_deg);

/**
 * @brief  读取滑窗 LPF 后的偏航角速度 (供姿态环 D 项使用)
 * @return 偏航角速度 (°/s, 已完成零偏补偿 / 符号修正 / LPF)
 */
float chassis_imu_get_yaw_rate_dps(void);

/**
 * @brief  KF 角度量测更新接口 (P0-改进 2026-05-02 编码器融合):
 *         外部模块 (如 chassis_ctrl 的 odom yaw) 提供一次 yaw 观测,
 *         KF 以 H=[1,0] 做标量观测更新, R 越大代表越不信这一观测.
 *         典型用法: ctrl 模块每 20ms 算 yaw_from_odom, 用 R=100°² 弱约束.
 * @param yaw_obs_deg  观测到的 yaw 角 (度)
 * @param R_deg2       观测噪声方差 (°²); 越大 = 越弱的修正力度
 */
void chassis_imu_kf_correct_angle(float yaw_obs_deg, float R_deg2);

#endif /* CHASSIS_IMU_H */
