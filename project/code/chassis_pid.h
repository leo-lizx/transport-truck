#ifndef CHASSIS_PID_H
#define CHASSIS_PID_H

/*===========================================================================
 * [chassis_pid.h] 增量式 PID 控制器
 *
 *   提供轻量级增量式 PID，用于四轮独立轮速闭环。
 *
 *   增量式 PID 相比位置式 PID 的优点：
 *     1. 输出自带积分效果，不存在积分饱和问题
 *     2. 切换目标时更平滑，不会产生大幅跳变
 *     3. 只需保存最近 3 次误差，内存占用小
 *
 *   增量公式：
 *     Δu(k) = Kp × [e(k) - e(k-1)]
 *            + Ki × e(k)
 *            + Kd × [e(k) - 2·e(k-1) + e(k-2)]
 *     u(k)  = u(k-1) + Δu(k)
 *
 *   使用流程：
 *     1. chassis_pid_init()  — 设置增益和输出限幅
 *     2. chassis_pid_step()  — 每个控制周期调用一次
 *     3. chassis_pid_reset() — 模式切换或急停时清零状态
 *===========================================================================*/

#include "chassis_config.h"

/** 增量式 PID 控制器状态结构体 */
typedef struct
{
    /* ---- PID 增益参数（由 init 设置，运行中也可动态修改） ---- */
    float kp;           /**< 比例增益 */
    float ki;           /**< 积分增益 */
    float kd;           /**< 微分增益 */

    /* ---- 运行时状态（内部维护，外部不要手动修改） ---- */
    float error_k;      /**< 当前误差 e(k)    */
    float error_k_1;    /**< 上一次误差 e(k-1) */
    float error_k_2;    /**< 上上次误差 e(k-2) */
    float output;       /**< 当前 PID 累计输出 */

    /* ---- 输出限幅 ---- */
    float output_max;   /**< 输出上限（正值），对称限幅 [-max, +max] */
} chassis_pid_t;

/**
 * @brief  初始化 PID 控制器
 * @param  pid         PID 结构体指针
 * @param  kp          比例增益
 * @param  ki          积分增益
 * @param  kd          微分增益
 * @param  output_max  输出限幅上限（正值）
 */
void chassis_pid_init(chassis_pid_t *pid, float kp, float ki, float kd, float output_max);

/**
 * @brief  PID 单步计算（每个控制周期调用一次）
 * @param  pid       PID 结构体指针
 * @param  target    目标值（如期望轮速 m/s）
 * @param  feedback  反馈值（如实际轮速 m/s）
 * @return           经过限幅的 PID 输出值（可直接作为 PWM 值使用）
 */
float chassis_pid_step(chassis_pid_t *pid, float target, float feedback);

/**
 * @brief  重置 PID 内部状态（清零误差和输出）
 *         急停或模式切换时调用，避免残留误差影响后续控制
 * @param  pid  PID 结构体指针
 */
void chassis_pid_reset(chassis_pid_t *pid);

/** 单轮 PID 调试快照（用于串口打印） */
typedef struct
{
    uint8 wheel_index;   /**< 当前调试轮子索引 */
    float target_value;  /**< 当前目标值 */
    float actual_value;  /**< 当前实际值 */
} chassis_pid_debug_snapshot_t;

/**
 * @brief  选择当前 PID 调试轮子（每次仅跟踪一个轮子）
 * @param  wheel_index 轮子索引：CHASSIS_WHEEL_LF/RF/LB/RB
 */
void chassis_pid_debug_select_wheel(chassis_wheel_index_t wheel_index);

/**
 * @brief  喂入单轮调试样本（由控制层在 20ms 控制任务中调用）
 * @param  wheel_index  样本所属轮子索引
 * @param  target_value 目标值
 * @param  actual_value 实际反馈值
 */
void chassis_pid_debug_feed_sample(chassis_wheel_index_t wheel_index,
                                   float target_value,
                                   float actual_value);

/**
 * @brief  读取当前单轮 PID 调试快照
 * @param  out_snapshot 输出快照指针
 */
void chassis_pid_debug_get_snapshot(chassis_pid_debug_snapshot_t *out_snapshot);

/**
 * @brief  清零单轮 PID 调试缓存
 */
void chassis_pid_debug_reset(void);

/**
 * @brief  单轮 PID 调试打印任务（建议主循环每 5ms 调用）
 *         内部 100ms 打印一次，printf 仅输出：目标值 实际值
 */
void chassis_pid_debug_task_5ms(void);

#endif /* CHASSIS_PID_H */
