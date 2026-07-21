/*===========================================================================
 * [chassis_mpc.c] 1D MPC — FISTA QP 求解器 (实例化, 支持位置+yaw)
 *
 *   模型: x_{k+1} = x_k - dt·u_k  (x = 距离或角度, u = 速度或角速度)
 *   控制: u_k ∈ [-v_limit, +v_limit]
 *
 *   代价:
 *     J = Σ_{k=1..N} Q_POS · x_k² + Q_term · x_N²
 *       + Σ_{k=0..N-1} R_VEL · u_k²
 *       + Σ_{k=1..N-1} R_SMOOTH · (u_k - u_{k-1})²
 *
 *   @owner  rt1064-main
 *   @periph none                  纯数学运算
 *===========================================================================*/

#include "chassis_mpc.h"

#if CHASSIS_MPC_ENABLE || CHASSIS_MPC_YAW_ENABLE

#include <math.h>

/* ==================================================================
 *  全局实例 (N=20 时 BSS 每个约 1.1KB；仅分配已启用的实例)
 * ================================================================== */

#if CHASSIS_MPC_ENABLE
static mpc_solver_t g_mpc_pos;   /* 位置 MPC (驱动轴纵向) */
#endif
#if CHASSIS_MPC_YAW_ENABLE
static mpc_solver_t g_mpc_yaw;   /* Yaw MPC (原地旋转) */
#endif

/* ==================================================================
 *  Packed 对称矩阵索引: H(i,j) = s_H[i*(i+1)/2 + j]  (i ≥ j)
 * ================================================================== */
static inline uint16 H_IDX(uint8 i, uint8 j)
{
    uint8 r, c;
    if (i >= j) { r = i; c = j; }
    else        { r = j; c = i; }
    return (uint16)((uint16)r * ((uint16)r + 1U) / 2U + (uint16)c);
}

/** Packed 对称矩阵-向量乘: y = H * x */
static void mpc_matvec(const float *H_packed,
                       const float *x,
                       float *restrict y,
                       uint8 n)
{
    uint8 i;
    uint8 j;
    for (i = 0U; i < n; ++i) {
        float sum = 0.0f;
        for (j = 0U; j <= i; ++j) {
            sum += H_packed[H_IDX(i, j)] * x[j];
        }
        for (j = (uint8)(i + 1U); j < n; ++j) {
            sum += H_packed[H_IDX(j, i)] * x[j];
        }
        y[i] = sum;
    }
}

/* ==================================================================
 *  H 矩阵解析构建 (init 时调用一次, 使用实例配置)
 * ================================================================== */
static void mpc_build_H(mpc_solver_t *s)
{
    const float dt       = CHASSIS_TASK_DT_20MS_S;
    const float dt2      = dt * dt;
    const float q_pos    = s->q_pos;
    const float qf_fact  = s->qf_factor;
    const float r_vel    = s->r_vel;
    const float r_smooth = s->r_smooth;
    const uint8 N        = MPC_N;
    const float pos_w    = q_pos * dt2;
    uint8 i;
    uint8 j;

    for (i = 0U; i < N; ++i) {
        uint8 max_ij;
        for (j = 0U; j <= i; ++j) {
            float val = 0.0f;

            /* 位置代价 (含终端放大) */
            max_ij = (i >= j) ? i : j;
            val += pos_w * ((float)(N - max_ij) + qf_fact);

            /* 速度幅度 */
            if (i == j) {
                val += r_vel;
            }

            /* 速度平滑 (拉普拉斯) */
            if (i == j) {
                if (i == 0U || i == (uint8)(N - 1U)) {
                    val += r_smooth;
                } else {
                    val += 2.0f * r_smooth;
                }
            } else if ((uint8)(i - j) == 1U) {
                val -= r_smooth;
            }

            s->H[H_IDX(i, j)] = val;
        }
    }
}

/* ==================================================================
 *  g 向量构建 (每拍重算, 仅依赖当前误差)
 *
 *  g[i] = -2 · Q_POS · dt · x₀ · [(N - i) + QF_FACTOR]
 * ================================================================== */
static void mpc_build_g(const mpc_solver_t *s, float x0, float *g)
{
    const float dt      = CHASSIS_TASK_DT_20MS_S;
    const float q_pos   = s->q_pos;
    const float qf_fact = s->qf_factor;
    const uint8 N       = MPC_N;
    const float base    = -2.0f * q_pos * dt * x0;
    uint8 i;

    for (i = 0U; i < N; ++i) {
        g[i] = base * ((float)(N - i) + qf_fact);
    }
}

/* ==================================================================
 *  FISTA 求解器 (Nesterov 加速投影梯度)
 *
 *  求解: min ½·Vᵀ·H·V + gᵀ·V  s.t. v_min ≤ V ≤ v_max
 * ================================================================== */
static void mpc_fista_solve(mpc_solver_t *s,
                            const float *g,
                            float *V)
{
    const uint8 n      = MPC_N;
    const float *H_p   = s->H;
    const float *v_min = s->v_min;
    const float *v_max = s->v_max;
    const float alpha  = s->alpha;
    float y[MPC_N];
    float grad[MPC_N];
    float t;
    uint8 iter;
    uint8 i;

    /* 热启动 */
    if (s->warm_valid) {
        for (i = 0U; i < n; ++i) { V[i] = s->V_warm[i]; }
    } else {
        for (i = 0U; i < n; ++i) { V[i] = 0.0f; }
    }

    for (i = 0U; i < n; ++i) { y[i] = V[i]; }
    t = 1.0f;

    for (iter = 0U; iter < (uint8)CHASSIS_MPC_FISTA_MAX_ITER; ++iter) {
        float t_new;
        float beta;
        float grad_norm_sq = 0.0f;
        uint8 converged;

        /* ∇f(y) = H·y + g */
        mpc_matvec(H_p, y, grad, n);
        for (i = 0U; i < n; ++i) { grad[i] += g[i]; }

        /* 投影梯度步: V_new = clip(y - α·∇f, v_min, v_max) */
        {
            float V_new[MPC_N];
            for (i = 0U; i < n; ++i) {
                float step = y[i] - alpha * grad[i];
                if (step < v_min[i]) { step = v_min[i]; }
                if (step > v_max[i]) { step = v_max[i]; }
                V_new[i] = step;
            }

            /* Nesterov 动量更新 */
            t_new = 0.5f + 0.5f * sqrtf(1.0f + 4.0f * t * t);
            if (t_new < 1.0f) { t_new = 1.0f; }
            beta = (t - 1.0f) / t_new;
            for (i = 0U; i < n; ++i) {
                float diff = V_new[i] - V[i];
                y[i] = V_new[i] + beta * diff;
                V[i] = V_new[i];
            }
            t = t_new;
        }

        /* 收敛检查 */
        {
            float max_diff = 0.0f;
            for (i = 0U; i < n; ++i) {
                float diff = y[i] - V[i];
                if (diff < 0.0f) { diff = -diff; }
                if (diff > max_diff) { max_diff = diff; }
            }

            for (i = 0U; i < n; ++i) {
                float gi = grad[i];
                if ((V[i] <= v_min[i] && gi > 0.0f) ||
                    (V[i] >= v_max[i] && gi < 0.0f)) {
                    gi = 0.0f;
                }
                grad_norm_sq += gi * gi;
            }

            converged = ((max_diff < CHASSIS_MPC_FISTA_TOL) &&
                         (grad_norm_sq < CHASSIS_MPC_FISTA_TOL *
                                         CHASSIS_MPC_FISTA_TOL))
                        ? 1U : 0U;
            if (converged) { break; }
        }
    }

    /* 写回热启动种子: shift + pad 0 */
    for (i = 0U; i < (uint8)(n - 1U); ++i) {
        s->V_warm[i] = V[i + 1U];
    }
    s->V_warm[n - 1U] = 0.0f;
    s->warm_valid = 1U;
}

/* ==================================================================
 *  公共 API — 通用求解器
 * ================================================================== */

void mpc_solver_init(mpc_solver_t *s,
                     float q_pos, float r_vel, float r_smooth,
                     float qf_factor, float v_limit,
                     float fallback_err, float min_err)
{
    float frob_sq = 0.0f;
    uint8 i;
    uint8 j;

    /* 存配置 */
    s->q_pos        = q_pos;
    s->r_vel        = r_vel;
    s->r_smooth     = r_smooth;
    s->qf_factor    = qf_factor;
    s->v_limit      = v_limit;
    s->fallback_err = fallback_err;
    s->min_err      = min_err;

    /* 1) 构建 H (常数, 之后不改) */
    mpc_build_H(s);

    /* 2) 计算 FISTA 步长 α = 1/||H||_F */
    for (i = 0U; i < MPC_N; ++i) {
        for (j = 0U; j <= i; ++j) {
            float hij = s->H[H_IDX(i, j)];
            frob_sq += hij * hij;
        }
    }
    if (frob_sq > 1e-12f) {
        if (CHASSIS_MPC_FISTA_STEP > 1e-9f) {
            s->alpha = CHASSIS_MPC_FISTA_STEP;
        } else {
            s->alpha = 1.0f / sqrtf(frob_sq);
        }
    } else {
        s->alpha = 1.0f;
    }

    /* 3) 初始化约束向量 */
    for (i = 0U; i < MPC_N; ++i) {
        s->v_min[i] = -v_limit;
        s->v_max[i] =  v_limit;
    }

    /* 4) 清热启动 */
    mpc_solver_reset(s);
}

void mpc_solver_reset(mpc_solver_t *s)
{
    uint8 i;
    for (i = 0U; i < MPC_N; ++i) {
        s->V_warm[i] = 0.0f;
    }
    s->warm_valid = 0U;
}

uint8 mpc_solver_step(mpc_solver_t *s, float dist_to_target,
                      float *out_control)
{
    float g[MPC_N];
    float V[MPC_N];
    float abs_err;

    if (out_control == NULL) { return 0U; }

    /* NaN 保护 */
    if (dist_to_target != dist_to_target) {
        *out_control = 0.0f;
        return 0U;
    }

    /* 距离判断: 太远或太近 → fallback */
    abs_err = (dist_to_target >= 0.0f) ? dist_to_target : -dist_to_target;
    if (abs_err > s->fallback_err || abs_err < s->min_err) {
        *out_control = 0.0f;
        return 0U;
    }

    /* 1) 构建 g 向量 (依赖当前 x₀) */
    mpc_build_g(s, dist_to_target, g);

    /* 2) 求解 QP */
    mpc_fista_solve(s, g, V);

    /* 3) 提取第一步控制 */
    {
        float u_out = V[0];

        if (u_out != u_out) {
            *out_control = 0.0f;
            s->warm_valid = 0U;
            return 0U;
        }

        /* 硬限幅兜底 */
        if (u_out > s->v_limit) {
            u_out = s->v_limit;
        } else if (u_out < -s->v_limit) {
            u_out = -s->v_limit;
        }

        *out_control = u_out;
    }

    return 1U;
}

/* ==================================================================
 *  位置 MPC (向后兼容包装)
 * ================================================================== */

#if CHASSIS_MPC_ENABLE

void chassis_mpc_init(void)
{
    mpc_solver_init(&g_mpc_pos,
                    CHASSIS_MPC_Q_POS,
                    CHASSIS_MPC_R_VEL,
                    CHASSIS_MPC_R_SMOOTH,
                    CHASSIS_MPC_QF_FACTOR,
                    CHASSIS_MPC_V_MAX_MPS,
                    CHASSIS_MPC_FALLBACK_ERR_M,
                    CHASSIS_MPC_MIN_DIST_M);
}

void chassis_mpc_reset(void)
{
    mpc_solver_reset(&g_mpc_pos);
}

uint8 chassis_mpc_step(float dist_to_target_m, float *out_velocity_mps)
{
    return mpc_solver_step(&g_mpc_pos, dist_to_target_m, out_velocity_mps);
}

#endif /* CHASSIS_MPC_ENABLE */

/* ==================================================================
 *  Yaw MPC
 * ================================================================== */

#if CHASSIS_MPC_YAW_ENABLE

void chassis_mpc_yaw_init(void)
{
    mpc_solver_init(&g_mpc_yaw,
                    CHASSIS_MPC_YAW_Q_POS,
                    CHASSIS_MPC_YAW_R_VEL,
                    CHASSIS_MPC_YAW_R_SMOOTH,
                    CHASSIS_MPC_YAW_QF_FACTOR,
                    CHASSIS_MAX_YAW_SPEED_DPS,
                    CHASSIS_MPC_YAW_FALLBACK_ERR_DEG,
                    CHASSIS_MPC_YAW_MIN_DEG);
}

void chassis_mpc_yaw_reset(void)
{
    mpc_solver_reset(&g_mpc_yaw);
}

uint8 chassis_mpc_yaw_step(float angle_err_deg, float *out_rate_dps)
{
    return mpc_solver_step(&g_mpc_yaw, angle_err_deg, out_rate_dps);
}

#endif /* CHASSIS_MPC_YAW_ENABLE */

#endif /* CHASSIS_MPC_ENABLE || CHASSIS_MPC_YAW_ENABLE */
