#ifndef CHASSIS_POSE_CTRL_H
#define CHASSIS_POSE_CTRL_H

/*===========================================================================
 * [兼容性头文件]
 *
 *   原 chassis_pose_ctrl 模块已拆分为独立子模块：
 *     chassis_config.h    — 引脚与参数配置（所有宏集中在此）
 *     chassis_pid.c/.h    — 增量式 PID 控制器
 *     chassis_motor.c/.h  — 电机驱动（PWM + 方向控制）
 *     chassis_encoder.c/.h— 编码器测速
 *     chassis_imu.c/.h    — IMU 姿态采样与航向积分
 *     chassis_mecanum.c/.h— 麦轮正/逆运动学
 *     chassis_ctrl.c/.h   — 顶层控制（整合以上模块，对外唯一入口）
 *
 *   本文件保留旧 API 名称的宏映射，确保已有代码无需修改即可编译。
 *   新代码请直接 #include "chassis_ctrl.h"
 *===========================================================================*/

#include "chassis_ctrl.h"

/* ---- 旧 API → 新 API 名称映射 ---- */
#define chassis_pose_ctrl_init          chassis_ctrl_init
#define chassis_pose_ctrl_task_5ms      chassis_ctrl_task_5ms
#define chassis_pose_ctrl_task_20ms     chassis_ctrl_task_20ms
#define chassis_pose_ctrl_move_to_grid  chassis_ctrl_move_to_grid
#define chassis_pose_ctrl_hold_yaw      chassis_ctrl_hold_yaw
#define chassis_pose_ctrl_is_arrived    chassis_ctrl_is_arrived
#define chassis_pose_ctrl_get_pose      chassis_ctrl_get_pose
#define chassis_pose_ctrl_get_last_cmd  chassis_ctrl_get_last_cmd
#define chassis_pose_ctrl_stop          chassis_ctrl_stop
#define chassis_pose_ctrl_set_pose      chassis_ctrl_set_pose
#define chassis_pose_ctrl_set_gyro_bias chassis_ctrl_set_gyro_bias

#endif /* CHASSIS_POSE_CTRL_H */