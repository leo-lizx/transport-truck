#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

/*===========================================================================
 * [chassis_ctrl.h] 底盘顶层控制模块
 *
 *   整合 IMU 姿态、编码器测速、麦轮运动学、PID 轮控、里程计定位、
 *   定点移动导航等功能，对外提供简洁统一的 API。
 *
 *   本模块是底盘子系统的唯一对外入口，上层代码（如 app_game_logic）
 *   不需要直接调用 motor/encoder/imu/mecanum 等底层模块。
 *
 *   典型调用流程：
 *     main.c 中调用: chassis_ctrl_init()
 *     5ms PIT 中断:  chassis_ctrl_task_5ms()
 *     20ms PIT 中断: chassis_ctrl_task_20ms()
 *     业务层:        chassis_ctrl_move_to_grid(x, y)
 *                    while (!chassis_ctrl_is_arrived()) { ... }
 *
 *   内部控制流程（20ms 周期内依次执行）：
 *     ① 读取编码器 → 计算四轮实际速度
 *     ② 麦轮逆运动学 → 估算车体速度
 *     ③ 里程计积分 → 更新全局位姿 (x, y)
 *     ④ 导航 P 控制器 → 计算目标车体速度
 *     ⑤ 缓加速滤波 → 麦轮正运动学 → PID → 电机输出
 *
 * [模块依赖]:
 *   chassis_config.h  — 所有引脚和参数配置
 *   chassis_imu.h     — IMU 姿态采样
 *   chassis_encoder.h — 编码器测速
 *   chassis_motor.h   — 电机驱动
 *   chassis_pid.h     — PID 控制器
 *   chassis_mecanum.h — 麦轮运动学
 *===========================================================================*/

#include "chassis_config.h"

/* ========================== 公共数据类型 ========================== */

/** 车体位姿：全局坐标系下的位置和航向角 */
typedef struct
{
    float x_m;       /**< 全局 X 坐标（米），向右为正 */
    float y_m;       /**< 全局 Y 坐标（米），向前为正 */
    float yaw_deg;   /**< 航向角（度），逆时针为正，范围 [-180, +180] */
} chassis_pose_t;

/** 车体速度指令：车体坐标系下的三自由度速度 */
typedef struct
{
    float vx_body_mps;   /**< X 方向速度（m/s），向右为正 */
    float vy_body_mps;   /**< Y 方向速度（m/s），向前为正 */
    float wz_dps;        /**< 转向角速度（°/s），逆时针为正 */
} chassis_body_speed_cmd_t;

/* ========================== 公共 API ========================== */

/**
 * @brief  初始化底盘控制子系统
 *         包括：IMU、4 路编码器、4 路电机 PWM/DIR、4 路 PID
 *         初始化后电机处于停止状态
 */
void chassis_ctrl_init(void);

/**
 * @brief  5ms 高频任务：读取 IMU 陀螺仪并积分航向角
 *         在 5ms PIT 中断中调用
 */
void chassis_ctrl_task_5ms(void);

/**
 * @brief  20ms 低频任务：编码器 → 里程计 → 导航控制 → PID → 电机
 *         在 20ms PIT 中断中调用
 */
void chassis_ctrl_task_20ms(void);

/**
 * @brief  下发网格坐标目标，底盘自动移动到该位置
 *         网格索引范围: x=[0, CHASSIS_GRID_MAX_X], y=[0, CHASSIS_GRID_MAX_Y]
 *         超出范围自动钳位到边界
 * @param  target_x_grid  目标 X 网格索引
 * @param  target_y_grid  目标 Y 网格索引
 */
void chassis_ctrl_move_to_grid(uint8 target_x_grid, uint8 target_y_grid);

/**
 * @brief  切换到航向保持模式：停止位置移动，仅保持指定航向角
 * @param  target_yaw_deg  目标航向角（度）
 */
void chassis_ctrl_hold_yaw(float target_yaw_deg);

/**
 * @brief  查询是否已到达目标点
 * @return 1 = 已到达或空闲，0 = 移动中
 */
uint8 chassis_ctrl_is_arrived(void);

/**
 * @brief  获取当前位姿副本（结构体拷贝）
 * @return 当前位姿 {x_m, y_m, yaw_deg}
 */
chassis_pose_t chassis_ctrl_get_pose(void);

/**
 * @brief  获取最近一次输出的车体速度指令（调试用）
 * @return 车体速度指令
 */
chassis_body_speed_cmd_t chassis_ctrl_get_last_cmd(void);

/**
 * @brief  紧急停止：立即清零所有控制状态并切断 PWM 输出
 */
void chassis_ctrl_stop(void);

/**
 * @brief  外部校正位姿（如视觉重定位矫正里程计累积误差）
 * @param  x_m      校正后 X 坐标（米）
 * @param  y_m      校正后 Y 坐标（米）
 * @param  yaw_deg  校正后航向角（度）
 */
void chassis_ctrl_set_pose(float x_m, float y_m, float yaw_deg);

/**
 * @brief  设置陀螺仪零偏补偿值
 * @param  gyro_z_bias_dps  Z 轴零偏（°/s）
 */
void chassis_ctrl_set_gyro_bias(float gyro_z_bias_dps);

#endif /* CHASSIS_CTRL_H */
