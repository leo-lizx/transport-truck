/*===========================================================================
 * [chassis_mpc.c] 1D 直线行驶 MPC — FISTA QP 求解器 + 驱动轴速度规划
 *
 *   模型: d_{k+1} = d_k - dt·v_k  (距目标剩余距离, 驱动轴)
 *   控制: v_k ∈ [-v_max, +v_max]
 *
 *   代价:
 *     J = Σ_{k=1..N} Q_POS · d_k² + Q_term · d_N²
 *       + Σ_{k=0..N-1} R_VEL · v_k²
 *       + Σ_{k=1..N-1} R_SMOOTH · (v_k - v_{k-1})²
 *
 *   等价 QP: min ½·Vᵀ·H·V + gᵀ·V  s.t. -v_max ≤ v_k ≤ v_max
 *
 *   H 矩阵解析构建 (仅 init 一次, 不依赖 d₀); g 每拍按 d₀ 重算。
 *
 *   @owner  rt1064-main
 *   @periph none                  纯数学运算
 *===========================================================================*/

#include "chassis_mpc.h"

#if CHASSIS_MPC_ENABLE

#include <math.h>

/* ==================================================================
 *  静态存储 (BSS, 避免 ISR 栈上分配)
 * ================================================================== */

/** H 矩阵 packed 下三角 (对称), N×N → N*(N+1)/2 元素, init 时构建一次 */
static float s_H[MPC_H_PACKED];

/** FISTA 步长 α = 1/||H||_F (init 时预计算) */
static float s_alpha = 0.0f;

/** 速度上下界: [-v_max, +v_max] */
static float s_v_min[MPC_N];
static float s_v_max[MPC_N];

/** 当前拍解 V (mpc_step 输出, 同时做下拍热启动种子) */
static float s_V_warm[MPC_N];

/** 热启动是否有效的标志: 切换目标时 reset 清零 */
static uint8 s_warm_valid = 0U;

/* ==================================================================
 *  Packed 对称矩阵索引: H(i,j) = s_H[i*(i+1)/2 + j]  (i ≥ j)
 * ================================================================== */
static inline uint16 H_IDX(uint8 i, uint8 j)
{
    /* 保证 i ≥ j: 若传反了自动 flip */
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
            sum += H_packed[H_IDX(j, i)] * x[j];  /* H(i,j)=H(j,i) */
        }
        y[i] = sum;
    }
}

/* ==================================================================
 *  H 矩阵解析构建 (init 时调用一次)
 *
 *  H[i][j] = Q_POS·dt²·[(N - max(i,j)) + QF_FACTOR]   ← 位置+终端
 *          + R_VEL · δ_{i,j}                              ← 速度幅值
 *          + R_SMOOTH · L_{i,j}                            ← 速度平滑
 *
 *  L 是离散拉普拉斯: L[i][i]   = 2  (1 < i < N-1)
 *                    L[0][0]   = 1
 *                    L[N-1][N-1] = 1
 *                    L[i][i+1] = L[i+1][i] = -1
 * ================================================================== */
static void mpc_build_H(void)
{
    const float dt       = CHASSIS_TASK_DT_20MS_S;
    const float dt2      = dt * dt;
    const float q_pos    = CHASSIS_MPC_Q_POS;
    const float qf_fact  = CHASSIS_MPC_QF_FACTOR;
    const float r_vel    = CHASSIS_MPC_R_VEL;
    const float r_smooth = CHASSIS_MPC_R_SMOOTH;
    const uint8 N        = MPC_N;
    const float pos_w    = q_pos * dt2;    /* Q_POS * dt² 基础块 */
    uint8 i;
    uint8 j;

    for (i = 0U; i < N; ++i) {
        uint8 max_ij;
        for (j = 0U; j <= i; ++j) {
            float val = 0.0f;

            /* ---- 位置代价 (含终端放大) ---- */
            max_ij = (i >= j) ? i : j;
            /* H_pos[i][j] = pos_w * [(N - max_ij) + qf_fact] */
            val += pos_w * ((float)(N - max_ij) + qf_fact);

            /* ---- 速度幅度 ---- */
            if (i == j) {
                val += r_vel;
            }

            /* ---- 速度平滑 (拉普拉斯) ---- */
            if (i == j) {
                if (i == 0U || i == (uint8)(N - 1U)) {
                    val += r_smooth;          /* 端点: 1 */
                } else {
                    val += 2.0f * r_smooth;   /* 内部: 2 */
                }
            } else if ((uint8)(i - j) == 1U) {
                val -= r_smooth;               /* 次对角线: -1 */
            }
            /* 其余非对角线位置无平滑贡献 */

            s_H[H_IDX(i, j)] = val;
        }
    }
}

/* ==================================================================
 *  g 向量构建 (每拍重算, 仅依赖 d₀)
 *
 *  g[i] = -2 · Q_POS · dt · d₀ · [(N - i) + QF_FACTOR]
 *
 *  物理: d₀ > 0 (目标在前) → g < 0 → 推 v 正 (前进)
 *        d₀ < 0 (已过冲)   → g > 0 → 推 v 负 (后退)
 * ================================================================== */
static void mpc_build_g(float d0, float *g)
{
    const float dt      = CHASSIS_TASK_DT_20MS_S;
    const float q_pos   = CHASSIS_MPC_Q_POS;
    const float qf_fact = CHASSIS_MPC_QF_FACTOR;
    const uint8 N       = MPC_N;
    const float base    = -2.0f * q_pos * dt * d0;
    uint8 i;

    for (i = 0U; i < N; ++i) {
        g[i] = base * ((float)(N - i) + qf_fact);
    }
}

/* ==================================================================
 *  FISTA 求解器 (Nesterov 加速投影梯度)
 *
 *  求解: min ½·Vᵀ·H·V + gᵀ·V  s.t. V_min ≤ V ≤ V_max
 *
 *  热启动: 使用 s_V_warm 作为初值, 迭代后写回供下拍复用
 * ================================================================== */
static void mpc_fista_solve(const float *H_packed,
                            const float *g,
                            const float *v_min,
                            const float *v_max,
                            float *V,
                            uint8 n)
{
    float y[MPC_N];     /* Nesterov 外推点 */
    float grad[MPC_N];  /* 梯度 H·y + g */
    float t;            /* Nesterov 动量参数 */
    float alpha;
    uint8 iter;
    uint8 i;

    alpha = s_alpha;

    /* 热启动: 用上一拍解 */
    if (s_warm_valid) {
        for (i = 0U; i < n; ++i) {
            V[i] = s_V_warm[i];
        }
    } else {
        for (i = 0U; i < n; ++i) {
            V[i] = 0.0f;
        }
    }

    /* Nesterov 初值 */
    for (i = 0U; i < n; ++i) {
        y[i] = V[i];
    }
    t = 1.0f;

    for (iter = 0U; iter < (uint8)CHASSIS_MPC_FISTA_MAX_ITER; ++iter) {
        float t_new;
        float beta;
        float grad_norm_sq = 0.0f;
        uint8 converged;

        /* ∇f(y) = H·y + g */
        mpc_matvec(H_packed, y, grad, n);
        for (i = 0U; i < n; ++i) {
            grad[i] += g[i];
        }

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
            if (t_new < 1.0f) { t_new = 1.0f; }  /* 数值兜底 */
            beta = (t - 1.0f) / t_new;
            for (i = 0U; i < n; ++i) {
                float diff = V_new[i] - V[i];
                y[i] = V_new[i] + beta * diff;
                V[i]  = V_new[i];
            }
            t = t_new;
        }

        /* 收敛检查: ||V_new - V|| 近似 (用 y 与 V 的差替) */
        {
            float max_diff = 0.0f;
            for (i = 0U; i < n; ++i) {
                float diff = y[i] - V[i];
                if (diff < 0.0f) { diff = -diff; }
                if (diff > max_diff) { max_diff = diff; }
            }

            /* 同时检查梯度范数 */
            for (i = 0U; i < n; ++i) {
                float gi = grad[i];
                /* 在约束边界上, 有效梯度是投影后的残差 */
                if ((V[i] <= v_min[i] && gi > 0.0f) ||
                    (V[i] >= v_max[i] && gi < 0.0f)) {
                    gi = 0.0f;  /* 被约束"吃掉"的梯度不算 */
                }
                grad_norm_sq += gi * gi;
            }

            converged = ((max_diff < CHASSIS_MPC_FISTA_TOL) &&
                         (grad_norm_sq < CHASSIS_MPC_FISTA_TOL * CHASSIS_MPC_FISTA_TOL))
                        ? 1U : 0U;
            if (converged) {
                break;
            }
        }
    }

    /* 写回热启动种子: shift + pad 0 */
    for (i = 0U; i < (uint8)(n - 1U); ++i) {
        s_V_warm[i] = V[i + 1U];
    }
    s_V_warm[n - 1U] = 0.0f;
    s_warm_valid = 1U;
}

/* ==================================================================
 *  公共 API
 * ================================================================== */

void chassis_mpc_init(void)
{
    float frob_sq = 0.0f;  /* Frobenius 范数平方 */
    uint8 i;
    uint8 j;

    /* 1) 构建 H (常数, 之后不改) */
    mpc_build_H();

    /* 2) 计算 FISTA 步长 α = 1/||H||_F (保守, 保证收敛) */
    for (i = 0U; i < MPC_N; ++i) {
        for (j = 0U; j <= i; ++j) {
            float hij = s_H[H_IDX(i, j)];
            frob_sq += hij * hij;
        }
        /* 对称部分 (i < j 的 hij 与 hji 相同, Frobenius 范数计一次平方) */
    }
    /* 对角线元素只计一次 (上面 j≤i 循环已包含 i=j) */
    if (frob_sq > 1e-12f) {
        float frob = sqrtf(frob_sq);
        /* 使用配置步长或自动计算 */
        if (CHASSIS_MPC_FISTA_STEP > 1e-9f) {
            s_alpha = CHASSIS_MPC_FISTA_STEP;
        } else {
            s_alpha = 1.0f / frob;
        }
    } else {
        s_alpha = 1.0f;  /* 兜底: shouldn't happen */
    }

    /* 3) 初始化约束向量 */
    {
        float v_lim = CHASSIS_MPC_V_MAX_MPS;
        for (i = 0U; i < MPC_N; ++i) {
            s_v_min[i] = -v_lim;
            s_v_max[i] =  v_lim;
        }
    }

    /* 4) 清热启动 */
    chassis_mpc_reset();
}

void chassis_mpc_reset(void)
{
    uint8 i;
    for (i = 0U; i < MPC_N; ++i) {
        s_V_warm[i] = 0.0f;
    }
    s_warm_valid = 0U;
}

uint8 chassis_mpc_step(float dist_to_target_m, float *out_velocity_mps)
{
    float g[MPC_N];
    float V[MPC_N];
    float abs_dist;

    if (out_velocity_mps == NULL) {
        return 0U;
    }

    /* NaN 保护: 输入异常 → fallback (NaN 是唯一 x!=x 的 float) */
    if (dist_to_target_m != dist_to_target_m) {
        *out_velocity_mps = 0.0f;
        return 0U;
    }

    /* 距离判断: 太远或太近 → fallback */
    abs_dist = (dist_to_target_m >= 0.0f) ? dist_to_target_m : -dist_to_target_m;
    if (abs_dist > CHASSIS_MPC_FALLBACK_ERR_M ||
        abs_dist < CHASSIS_MPC_MIN_DIST_M) {
        *out_velocity_mps = 0.0f;
        return 0U;
    }

    /* 1) 构建 g 向量 (依赖当前 d₀) */
    mpc_build_g(dist_to_target_m, g);

    /* 2) 求解 QP */
    mpc_fista_solve(s_H, g, s_v_min, s_v_max, V, MPC_N);

    /* 3) 提取第一步控制 */
    {
        float v_out = V[0];

        /* NaN 保护: 输出异常 → fallback */
        if (v_out != v_out) {
            *out_velocity_mps = 0.0f;
            s_warm_valid = 0U;
            return 0U;
        }

        /* 硬限幅兜底 (理论上 FISTA 投影已保证, 此处二次确认) */
        if (v_out > CHASSIS_MPC_V_MAX_MPS) {
            v_out = CHASSIS_MPC_V_MAX_MPS;
        } else if (v_out < -CHASSIS_MPC_V_MAX_MPS) {
            v_out = -CHASSIS_MPC_V_MAX_MPS;
        }

        *out_velocity_mps = v_out;
    }

    return 1U;
}

#endif /* CHASSIS_MPC_ENABLE */
