/*===========================================================================
 * [chassis_pose_ctrl_call_example.c] 底盘控制调用层实现
 *===========================================================================*/

#include "chassis_pose_ctrl_call_example.h"

void app_control_pipeline_init(void)
{
    /* 启动阶段调用一次：初始化姿态控制与定位模块 */
    chassis_ctrl_init();
}

void app_control_pipeline_on_pit_5ms(void)
{
    /* 5ms 高速任务：IMU 姿态采样与航向角积分 */
    chassis_ctrl_task_5ms();
}

void app_control_pipeline_on_pit_20ms(void)
{
    /* 20ms 低速任务：编码器 + 里程计 + 定点控制 + 电机输出 */
    chassis_ctrl_task_20ms();
}

void app_control_pipeline_move_to_grid(uint8 target_x_grid, uint8 target_y_grid)
{
    /* 上层下发网格目标点，底盘自动执行定点移动 */
    chassis_ctrl_move_to_grid(target_x_grid, target_y_grid);
}

uint8 app_control_pipeline_is_arrived(void)
{
    /* 上层轮询该状态即可判断是否到达目标点 */
    return chassis_ctrl_is_arrived();
}

void app_control_pipeline_correct_pose(float x_m, float y_m, float yaw_deg)
{
    /* 预留给视觉重定位：矫正里程计累积误差 */
    chassis_ctrl_set_pose(x_m, y_m, yaw_deg);
}

void app_control_pipeline_set_gyro_bias(float gyro_z_bias_dps)
{
    /* 预留给上层静止标定：写入陀螺零偏 */
    chassis_ctrl_set_gyro_bias(gyro_z_bias_dps);
}
