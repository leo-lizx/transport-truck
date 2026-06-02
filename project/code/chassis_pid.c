/*===========================================================================
 * [chassis_pid.c] 增量式 PID 控制器实现
 *===========================================================================*/

#include "chassis_pid.h"
#include "chassis_ctrl.h"
#include <stdio.h>

/* ---------------------- 单轮 PID 调试缓存 ---------------------- */
static volatile uint8 s_pid_debug_wheel_index = (uint8)CHASSIS_WHEEL_RF; /* 当前选中的调试轮子 */
static volatile float s_pid_debug_target_value = 0.0f;                    /* 最近一次目标值 */
static volatile float s_pid_debug_actual_value = 0.0f;                    /* 最近一次实际值 */

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

void chassis_pid_debug_select_wheel(chassis_wheel_index_t wheel_index)
{
    if ((uint8)wheel_index >= (uint8)CHASSIS_WHEEL_COUNT)
    {
        return;
    }

    s_pid_debug_wheel_index = (uint8)wheel_index;
}

void chassis_pid_debug_feed_sample(chassis_wheel_index_t wheel_index,
                                   float target_value,
                                   float actual_value)
{
    if ((uint8)wheel_index != s_pid_debug_wheel_index)
    {
        return;
    }

    s_pid_debug_target_value = target_value;
    s_pid_debug_actual_value = actual_value;
}

void chassis_pid_debug_get_snapshot(chassis_pid_debug_snapshot_t *out_snapshot)
{
    if (0 == out_snapshot)
    {
        return;
    }

    out_snapshot->wheel_index  = s_pid_debug_wheel_index;
    out_snapshot->target_value = s_pid_debug_target_value;
    out_snapshot->actual_value = s_pid_debug_actual_value;
}

void chassis_pid_debug_reset(void)
{
    s_pid_debug_target_value = 0.0f;
    s_pid_debug_actual_value = 0.0f;
}

void chassis_pid_debug_task_5ms(void)
{
    static uint8 div = 0U;
    chassis_pid_debug_snapshot_t snapshot;
    // const char *wheel_name;
    float wheel_fb[4];

    div++;
    if (div < 20U)
    {
        return; /* 100ms 分频打印 */
    }
    div = 0U;

    chassis_pid_debug_get_snapshot(&snapshot);

    /* 同步取 4 路 LPF 速度快照, 用于"接线诊断": 手转任一物理轮观察哪个 index 在动 */
    chassis_ctrl_get_wheel_feedback_snapshot(wheel_fb);

    /*
     * Firewater 协议两条曲线分组同时输出:
     *   pid_<wheel>:target,actual           — 当前调试轮 PID 跟踪曲线
     *   wfb:LF,RF,LB,RB                     — 4 路反馈巡检 (诊断接线用)
     * 接线自检方法: 让车停稳, 手转某一物理轮, 看 wfb 4 个数值里哪一列变化,
     *   该列下标 0/1/2/3 即该物理轮在软件里实际对应的 index.
     *   若与你期望不符, 去 chassis_config.h 把对应 LF/RF/LB/RB 的 ENC_INDEX
     *   /ENC_CH1/ENC_CH2 三个宏改正 (整组对调, 不要只改一个).
     */
    printf("%.4f,%.4f\n",
           snapshot.target_value,
           snapshot.actual_value);
    // printf("wfb:%.4f,%.4f,%.4f,%.4f\n",
    //        wheel_fb[0], wheel_fb[1], wheel_fb[2], wheel_fb[3]);
}
