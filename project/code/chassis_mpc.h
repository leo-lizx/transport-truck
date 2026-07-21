#ifndef CHASSIS_MPC_H
#define CHASSIS_MPC_H
/*===========================================================================
 * [chassis_mpc.h] 1D 直线行驶/航向 MPC — FISTA QP 求解器 (实例化)
 *
 *   推箱子场景: 车沿网格轴线走直线 / 原地旋转对齐航向,
 *   MPC 提前 N 拍预知目标距离/角度, 规划最优速度曲线。
 *
 *   @owner  rt1064-main
 *   @periph none                  纯数学运算 (FISTA QP 求解器)
 *===========================================================================*/

#include "zf_common_typedef.h"
#include "config/configChassis.h"

#if CHASSIS_MPC_ENABLE || CHASSIS_MPC_YAW_ENABLE

/* ==================================================================
 *  MPC 维度常量 (由 config 宏推导)
 * ================================================================== */

/** 预测步数, 决策变量数 = N (compile-time constant for array sizing) */
#define MPC_N            CHASSIS_MPC_HORIZON_N

/** packed 对称 H 矩阵元素数 = N*(N+1)/2 */
#define MPC_H_PACKED     CHASSIS_MPC_H_PACKED

/* ==================================================================
 *  MPC 求解器实例 (每个控制轴一个实例)
 * ================================================================== */

typedef struct {
    /* ---- 配置 (init 时设定, build_H 使用) ---- */
    float q_pos;            /* 位置/角度精度权重 */
    float r_vel;            /* 速度代价权重 */
    float r_smooth;         /* 平滑代价权重 */
    float qf_factor;        /* 终端位置附加权重因子 */
    float v_limit;          /* 控制量上限 (m/s 或 °/s) */
    float fallback_err;     /* 误差超此值 → fallback (m 或 °) */
    float min_err;          /* 误差低于此值 → fallback (m 或 °) */

    /* ---- 预计算 (init 时构建, 之后不变) ---- */
    float H[MPC_H_PACKED];  /* 对称 H 矩阵 packed 下三角 */
    float alpha;            /* FISTA 步长 = 1/||H||_F */

    /* ---- 约束 (init 时按 v_limit 填充) ---- */
    float v_min[MPC_N];
    float v_max[MPC_N];

    /* ---- 运行时状态 ---- */
    float V_warm[MPC_N];    /* 热启动种子 + shift */
    uint8 warm_valid;       /* 1 = V_warm 有效 */
} mpc_solver_t;

/* ==================================================================
 *  通用求解器 API
 * ================================================================== */

/**
 * @brief  初始化一个 MPC 求解器实例 (启动时调用一次)
 * @param  s              求解器实例指针
 * @param  q_pos          位置/角度精度权重
 * @param  r_vel          速度代价权重
 * @param  r_smooth       平滑代价权重 (相邻步速度差惩罚)
 * @param  qf_factor      终端位置权重放大因子
 * @param  v_limit        控制量上限
 * @param  fallback_err   误差绝对值超此值 → fallback
 * @param  min_err        误差绝对值低于此值 → fallback
 */
void mpc_solver_init(mpc_solver_t *s,
                     float q_pos, float r_vel, float r_smooth,
                     float qf_factor, float v_limit,
                     float fallback_err, float min_err);

/**
 * @brief  重置求解器热启动状态 (切换目标时调用)
 */
void mpc_solver_reset(mpc_solver_t *s);

/**
 * @brief  执行一步 MPC, 返回最优控制量
 *
 * @param  s               求解器实例
 * @param  dist_to_target  到目标的带符号距离/角度 (正值=目标在前方)
 * @param  out_control     输出: 最优速度/角速度指令 (m/s 或 °/s)
 * @return 1 = 正常输出; 0 = fallback (调用者应回退备用控制器)
 *
 * @note   在 20ms ISR 上下文中调用, 耗时 ~5-10μs @ 600MHz
 */
uint8 mpc_solver_step(mpc_solver_t *s, float dist_to_target,
                      float *out_control);

/* ==================================================================
 *  位置 MPC (向后兼容, 内部使用 g_mpc_pos 实例)
 * ================================================================== */

#if CHASSIS_MPC_ENABLE
void chassis_mpc_init(void);
void chassis_mpc_reset(void);
uint8 chassis_mpc_step(float dist_to_target_m, float *out_velocity_mps);
#endif /* CHASSIS_MPC_ENABLE */

/* ==================================================================
 *  Yaw MPC (原地旋转航向对齐)
 * ================================================================== */

#if CHASSIS_MPC_YAW_ENABLE
void chassis_mpc_yaw_init(void);
void chassis_mpc_yaw_reset(void);

/**
 * @brief  执行一步 Yaw MPC
 * @param  angle_err_deg   带符号航向误差 (°), 正值=CCW
 * @param  out_rate_dps    输出: 最优角速度指令 (°/s)
 * @return 1 = MPC 正常; 0 = fallback (调用者回退 sqrt_controller)
 */
uint8 chassis_mpc_yaw_step(float angle_err_deg, float *out_rate_dps);
#endif /* CHASSIS_MPC_YAW_ENABLE */

#endif /* CHASSIS_MPC_ENABLE || CHASSIS_MPC_YAW_ENABLE */
#endif /* CHASSIS_MPC_H */
