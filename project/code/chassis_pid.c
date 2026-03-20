/*===========================================================================
 * [chassis_pid.c] 增量式 PID 控制器实现
 *===========================================================================*/

#include "chassis_pid.h"

/**
 * @brief  初始化增量式 PID 控制器
 */
void chassis_pid_init(chassis_pid_t *pid, float kp, float ki, float kd, float output_max)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->error_k    = 0.0f;
    pid->error_k_1  = 0.0f;
    pid->error_k_2  = 0.0f;
    pid->output     = 0.0f;
    pid->output_max = output_max;
}

/**
 * @brief  执行一次增量式 PID 计算
 * @param  pid      PID 对象指针
 * @param  target   目标值
 * @param  feedback 反馈值
 * @return 限幅后的输出值
 */
float chassis_pid_step(chassis_pid_t *pid, float target, float feedback)
{
    float output_delta; /* 本周期输出增量 Δu(k) */

    /* 误差历史向后移动一拍 */
    pid->error_k_2 = pid->error_k_1;
    pid->error_k_1 = pid->error_k;
    pid->error_k   = target - feedback;

    /* 计算增量 Δu(k) */
    output_delta =
        pid->kp * (pid->error_k - pid->error_k_1) +               /* P 项 */
        pid->ki *  pid->error_k +                                  /* I 项 */
        pid->kd * (pid->error_k - 2.0f * pid->error_k_1 + pid->error_k_2); /* D 项 */

    /* 累加到输出并做对称限幅 */
    pid->output += output_delta;
    pid->output  = chassis_clamp_f(pid->output, -pid->output_max, pid->output_max);

    return pid->output;
}

/**
 * @brief  清零 PID 内部状态
 */
void chassis_pid_reset(chassis_pid_t *pid)
{
    pid->error_k   = 0.0f;
    pid->error_k_1 = 0.0f;
    pid->error_k_2 = 0.0f;
    pid->output    = 0.0f;
}
