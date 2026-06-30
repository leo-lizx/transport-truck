#ifndef CHASSIS_MPC_H
#define CHASSIS_MPC_H
/*===========================================================================
 * [chassis_mpc.h] 1D 直线行驶 MPC — 驱动轴纵向速度规划
 *
 *   推箱子场景: 车沿网格轴线走直线, MPC 提前 N 拍预知目标距离,
 *   规划最优速度曲线, 干扰后自动重规划。
 *   只接管驱动轴速度, 横向保持和航向保持沿用现有机制。
 *
 *   @owner  rt1064-main
 *   @periph none                  纯数学运算 (FISTA QP 求解器)
 *===========================================================================*/

#include "zf_common_typedef.h"
#include "config/configChassis.h"

#if CHASSIS_MPC_ENABLE

/* ==================================================================
 *  MPC 维度常量 (由 config 宏推导)
 * ================================================================== */

/** 预测步数, 决策变量数 = N (compile-time constant for array sizing) */
#define MPC_N            CHASSIS_MPC_HORIZON_N

/** packed 对称 H 矩阵元素数 = N*(N+1)/2 */
#define MPC_H_PACKED     CHASSIS_MPC_H_PACKED

/* ==================================================================
 *  API
 * ================================================================== */

/**
 * @brief  MPC 模块初始化 (启动时调用一次)
 * @note   预计算 FISTA 步长等不变量
 */
void chassis_mpc_init(void);

/**
 * @brief  重置 MPC 内部状态 (切换目标时调用)
 * @note   清零热启动缓冲和上一拍解
 */
void chassis_mpc_reset(void);

/**
 * @brief  执行一步 MPC, 返回驱动轴最优速度指令
 *
 * @param  dist_to_target_m  到目标的带符号距离 (m), 正值=目标在前方
 * @param  out_velocity_mps  输出: MPC 规划的速度指令 (m/s)
 * @return 1 = MPC 正常输出; 0 = 触发 fallback (调用者应回退 sqrt_controller)
 *
 * @note   在 20ms ISR 上下文中调用, 耗时 ~5-10μs @ 600MHz
 *         内部有 NaN 保护, 异常时自动回退
 */
uint8 chassis_mpc_step(float dist_to_target_m, float *out_velocity_mps);

#endif /* CHASSIS_MPC_ENABLE */
#endif /* CHASSIS_MPC_H */
