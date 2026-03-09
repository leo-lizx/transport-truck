/*===========================================================================
 * [chassis_pid.c] 增量式 PID 控制器实现
 *===========================================================================*/

#include "chassis_pid.h"

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

float chassis_pid_step(chassis_pid_t *pid, float target, float feedback)
{
    float delta_output;

    /* 误差历史向后移动一拍 */
    pid->error_k_2 = pid->error_k_1;
    pid->error_k_1 = pid->error_k;
    pid->error_k   = target - feedback;

    /* 计算增量 Δu(k) */
    delta_output =
        pid->kp * (pid->error_k - pid->error_k_1) +               /* P 项 */
        pid->ki *  pid->error_k +                                  /* I 项 */
        pid->kd * (pid->error_k - 2.0f * pid->error_k_1 + pid->error_k_2); /* D 项 */

    /* 累加到输出并做对称限幅 */
    pid->output += delta_output;
    pid->output  = chassis_clamp_f(pid->output, -pid->output_max, pid->output_max);

    return pid->output;
}

void chassis_pid_reset(chassis_pid_t *pid)
{
    pid->error_k   = 0.0f;
    pid->error_k_1 = 0.0f;
    pid->error_k_2 = 0.0f;
    pid->output    = 0.0f;
}
