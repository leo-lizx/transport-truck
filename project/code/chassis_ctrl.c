/*===========================================================================
 *  chassis_ctrl.c — 底盘顶层控制
 *
 *  链路: IMU → 航向角 → 里程计 → 导航/航向闭环 → 运动学 → PID → 电机
 *
 *  管理:
 *    · 四轮硬件实例化（引脚定义在 chassis_config.h）
 *    · 缓加速斜坡滤波
 *    · 四种互斥控制模式（枚举管理）
 *    · 里程计位姿追踪
 *==========================================================================*/

#include "chassis_ctrl.h"
#include "chassis_imu.h"
#include "chassis_encoder.h"
#include "chassis_motor.h"
#include "chassis_pid.h"
#include "chassis_mecanum.h"
#include <math.h>

/* 软限位依赖：地图由 app_game_logic.c 维护 */
extern uint8 g_game_map[CHASSIS_GRID_ROWS][CHASSIS_GRID_COLS];

/* ---------------------- 航向闭环常量 ---------------------- */
#define YAW_DEADZONE_DEG   0.30f    /* 航向误差死区               */
#define YAW_KI             0.030f   /* 航向 I 增益                */
#define YAW_I_LIMIT        120.0f   /* 积分限幅 (°·s)             */
#define YAW_MIN_WZ_DPS     10.0f    /* 原地保持最小角速度补偿     */

/* 单轮 PID 调试起步补偿参数（用于克服静摩擦） */
#define WHEEL_DEBUG_START_SPEED_EPS_MPS   (0.03f)   /* 低于此反馈速度视为静止 */
#define WHEEL_DEBUG_START_TARGET_EPS_MPS  (0.05f)   /* 低于此目标速度不启用补偿 */
#define WHEEL_DEBUG_START_PWM_MIN         (1200.0f) /* 起步最小 PWM 幅值 */
#define WHEEL_DEBUG_SIGN_FIX_CONFIRM_CNT  (6U)      /* 反号连续计数达到该值后自动翻转反馈符号 */

/* 轮速闭环抗抖参数（抑制低速量化噪声和来回翻向） */
#define WHEEL_FB_LPF_ALPHA                (0.35f)   /* 轮速反馈一阶低通系数，越小越平滑 */
#define WHEEL_STOP_TARGET_EPS_MPS         (0.015f)  /* 目标接近 0 的判据 */
#define WHEEL_STOP_FEEDBACK_EPS_MPS       (0.030f)  /* 反馈接近 0 的判据 */

/* 软限位保护参数（防止里程计漂移导致虚拟地图“撞墙”） */
#define SOFT_LIMIT_LOOKAHEAD_S             (0.22f)   /* 前瞻时间窗，越大越保守 */
#define SOFT_LIMIT_SIDE_OFFSET_M           (0.08f)   /* 左右试探偏移 */
#define SOFT_LIMIT_BRAKE_SCALE             (0.28f)   /* 触发时线速度缩放 */
#define SOFT_LIMIT_AVOID_WZ_DPS            (35.0f)   /* 避障微调角速度 */
#define SOFT_LIMIT_MIN_MOVE_EPS_MPS        (0.01f)   /* 小于此速度不触发预测 */
#define SOFT_LIMIT_MAP_WALL                (1U)      /* 与地图编码 MAP_WALL 保持一致 */

/* ---------------------- 控制模式 ---------------------- */
/*
 * 单轮 PID 调试专用开关：
 * 1 = 仅保留 MODE_SINGLE_WHEEL_PID_DEBUG 主链路，其余模式逻辑临时屏蔽（保留在 #else 以便恢复）
 * 0 = 启用全部控制模式
 */
#define CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY   (1)

typedef enum {
    MODE_YAW_HOLD = 0,      /* 原地航向保持（默认）            */
    MODE_POINT_NAV,         /* 网格点位导航                    */
    MODE_MOVE_YAW,          /* 外部平移 + 航向指令             */
    MODE_ATT_DEBUG,         /* 航向闭环调试（行为同 YAW_HOLD） */
    MODE_SINGLE_WHEEL_PID_DEBUG /* 单轮 PID 调试（仅一个轮子给目标） */
} ctrl_mode_t;

/* ====================== 硬件实例 ====================== */

static chassis_motor_t s_mot[CHASSIS_WHEEL_COUNT] = {
    { CHASSIS_LF_PWM_CHANNEL, CHASSIS_LF_DIR_PIN, CHASSIS_LF_DIR_SIGN },  /* LF */
    { CHASSIS_RF_PWM_CHANNEL, CHASSIS_RF_DIR_PIN, CHASSIS_RF_DIR_SIGN },  /* RF */
    { CHASSIS_LB_PWM_CHANNEL, CHASSIS_LB_DIR_PIN, CHASSIS_LB_DIR_SIGN },  /* LB */
    { CHASSIS_RB_PWM_CHANNEL, CHASSIS_RB_DIR_PIN, CHASSIS_RB_DIR_SIGN },  /* RB */
};

static chassis_encoder_t s_enc[CHASSIS_WHEEL_COUNT] = {
    { CHASSIS_LF_ENC_INDEX, CHASSIS_LF_ENC_CH1, CHASSIS_LF_ENC_CH2, CHASSIS_LF_ENC_SIGN, 0.0f },
    { CHASSIS_RF_ENC_INDEX, CHASSIS_RF_ENC_CH1, CHASSIS_RF_ENC_CH2, CHASSIS_RF_ENC_SIGN, 0.0f },
    { CHASSIS_LB_ENC_INDEX, CHASSIS_LB_ENC_CH1, CHASSIS_LB_ENC_CH2, CHASSIS_LB_ENC_SIGN, 0.0f },
    { CHASSIS_RB_ENC_INDEX, CHASSIS_RB_ENC_CH1, CHASSIS_RB_ENC_CH2, CHASSIS_RB_ENC_SIGN, 0.0f },
};

static chassis_pid_t s_pid[CHASSIS_WHEEL_COUNT];

/* ====================== 运行时状态 ====================== */

/* 可调参数（默认值来自 chassis_config.h） */
volatile chassis_tune_params_t g_chassis_tune_params = {
    {
        CHASSIS_WHEEL_PID_LF_KP,
        CHASSIS_WHEEL_PID_RF_KP,
        CHASSIS_WHEEL_PID_LB_KP,
        CHASSIS_WHEEL_PID_RB_KP,
    },
    {
        CHASSIS_WHEEL_PID_LF_KI,
        CHASSIS_WHEEL_PID_RF_KI,
        CHASSIS_WHEEL_PID_LB_KI,
        CHASSIS_WHEEL_PID_RB_KI,
    },
    {
        CHASSIS_WHEEL_PID_LF_KD,
        CHASSIS_WHEEL_PID_RF_KD,
        CHASSIS_WHEEL_PID_LB_KD,
        CHASSIS_WHEEL_PID_RB_KD,
    },
    CHASSIS_POS_KP, CHASSIS_YAW_KP,
    CHASSIS_MAX_LINEAR_SPEED_MPS, CHASSIS_MAX_YAW_SPEED_DPS,
    CHASSIS_CMD_ACCEL_LIMIT_MPS2, CHASSIS_CMD_ACCEL_LIMIT_DPS2,
};

static volatile chassis_pose_t           s_pose     = {0};
static volatile chassis_body_speed_cmd_t s_last_cmd = {0};
static volatile chassis_body_speed_cmd_t s_ramp     = {0};  /* 斜坡滤波状态 */

static volatile ctrl_mode_t s_mode    = MODE_YAW_HOLD;
static volatile uint8       s_arrived = 1U;

/* 导航目标 */
static volatile float s_tgt_x_m       = 0.0f;
static volatile float s_tgt_y_m       = 0.0f;
static volatile float s_tgt_yaw_deg   = 0.0f;

/* 外部移动+航向模式的平移指令 */
static volatile float s_cmd_vx = 0.0f;
static volatile float s_cmd_vy = 0.0f;

/* 单轮 PID 调试参数 */
static volatile uint8 s_debug_wheel_index = (uint8)CHASSIS_WHEEL_LF;
static volatile float s_debug_wheel_target_mps = 0.0f;
static volatile float s_debug_fb_sign_mul[CHASSIS_WHEEL_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
static volatile uint8 s_debug_fb_sign_mismatch_cnt[CHASSIS_WHEEL_COUNT] = {0U, 0U, 0U, 0U};

/* 航向积分项 */
static volatile float s_yaw_i  = 0.0f;

/* 里程计反馈 */
static volatile float s_fb_vx  = 0.0f;
static volatile float s_fb_vy  = 0.0f;
static volatile float s_wheel_fb_lpf[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static volatile uint8 s_wheel_fb_lpf_inited = 0U;

/* ====================== 内部工具函数 ====================== */

/** 调试轮索引安全校验：非法值回退到左前轮 */
static uint8 debug_wheel_index_safe(uint8 wheel_index)
{
    return (wheel_index < (uint8)CHASSIS_WHEEL_COUNT) ? wheel_index
                                                       : (uint8)CHASSIS_WHEEL_LF;
}

/** 调试目标速度限幅：限制在轮速上限范围内 */
static float debug_target_speed_clamp(float target_speed_mps)
{
    return chassis_clamp_f(target_speed_mps,
                           -CHASSIS_MAX_WHEEL_SPEED_MPS,
                            CHASSIS_MAX_WHEEL_SPEED_MPS);
}

/** 停止指定轮子并清理 PID 状态 */
static void stop_wheel_with_pid_reset(uint8 wheel_index)
{
    chassis_pid_reset(&s_pid[wheel_index]);
    chassis_motor_stop(&s_mot[wheel_index]);
}

/** 参数安全限幅，防止异常值进入控制链 */
static chassis_tune_params_t sanitize(chassis_tune_params_t p)
{
    uint8 i;

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        p.wheel_pid_kp[i] = chassis_clamp_f(p.wheel_pid_kp[i], 0.0f, 400.0f);
        p.wheel_pid_ki[i] = chassis_clamp_f(p.wheel_pid_ki[i], 0.0f,  80.0f);
        p.wheel_pid_kd[i] = chassis_clamp_f(p.wheel_pid_kd[i], 0.0f,  40.0f);
    }

    p.pos_kp               = chassis_clamp_f(p.pos_kp,               0.0f,    5.0f);
    p.yaw_kp               = chassis_clamp_f(p.yaw_kp,               0.0f,   10.0f);
    p.max_linear_speed_mps = chassis_clamp_f(p.max_linear_speed_mps, 0.05f,
                                             CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS);
    p.max_yaw_speed_dps   = chassis_clamp_f(p.max_yaw_speed_dps,   10.0f,
                                             CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS);
    p.cmd_accel_limit_mps2 = chassis_clamp_f(p.cmd_accel_limit_mps2, 0.10f, 5.00f);
    p.cmd_accel_limit_dps2 = chassis_clamp_f(p.cmd_accel_limit_dps2, 20.0f, 1000.0f);
    return p;
}

/** 车体速度矢量限幅: (vx,vy)模长 ≤ max_linear, |wz| ≤ max_yaw */
static void limit_speed(chassis_body_speed_cmd_t *c)
{
    float norm;

    c->wz_dps = chassis_clamp_f(c->wz_dps,
                                -g_chassis_tune_params.max_yaw_speed_dps,
                                 g_chassis_tune_params.max_yaw_speed_dps);

    norm = sqrtf(c->vx_body_mps * c->vx_body_mps +
                 c->vy_body_mps * c->vy_body_mps);
    if (norm > g_chassis_tune_params.max_linear_speed_mps && norm > 1e-6f) {
        float s = g_chassis_tune_params.max_linear_speed_mps / norm;
        c->vx_body_mps *= s;
        c->vy_body_mps *= s;
    }
}

/** 缓加速斜坡滤波: 每 20ms 周期限制速度变化量，防止轮胎打滑 */
static chassis_body_speed_cmd_t ramp_filter(chassis_body_speed_cmd_t tgt)
{
    const float dv = g_chassis_tune_params.cmd_accel_limit_mps2 * CHASSIS_TASK_DT_20MS_S;
    const float dw = g_chassis_tune_params.cmd_accel_limit_dps2 * CHASSIS_TASK_DT_20MS_S;
    chassis_body_speed_cmd_t out;

    limit_speed(&tgt);

    out.vx_body_mps = chassis_clamp_f(tgt.vx_body_mps,
                                      s_ramp.vx_body_mps - dv,
                                      s_ramp.vx_body_mps + dv);
    out.vy_body_mps = chassis_clamp_f(tgt.vy_body_mps,
                                      s_ramp.vy_body_mps - dv,
                                      s_ramp.vy_body_mps + dv);
    out.wz_dps      = chassis_clamp_f(tgt.wz_dps,
                                      s_ramp.wz_dps - dw,
                                      s_ramp.wz_dps + dw);

    limit_speed(&out);   /* 斜坡后二次限幅 */
    s_ramp = out;
    return out;
}

/** 紧急停机: 清零滤波器、PID、PWM */
static void force_stop(void)
{
    uint8 i;

    s_ramp     = (chassis_body_speed_cmd_t){0};
    s_last_cmd = (chassis_body_speed_cmd_t){0};
    s_yaw_i    = 0.0f;

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        stop_wheel_with_pid_reset(i);
        s_wheel_fb_lpf[i] = 0.0f;
    }
    s_wheel_fb_lpf_inited = 0U;
}

/** 软限位：网格是否在可通行内场 */
static inline uint8 soft_limit_is_inner_grid(int16 gx, int16 gy)
{
    if (gx < (int16)CHASSIS_GRID_INNER_MIN_X || gx > (int16)CHASSIS_GRID_INNER_MAX_X) return 0;
    if (gy < (int16)CHASSIS_GRID_INNER_MIN_Y || gy > (int16)CHASSIS_GRID_INNER_MAX_Y) return 0;
    return 1;
}

/** 软限位：判断网格是否为墙/边界障碍 */
static uint8 soft_limit_is_wall_grid(int16 gx, int16 gy)
{
    if (!soft_limit_is_inner_grid(gx, gy)) return 1;
    return (g_game_map[gy][gx] == SOFT_LIMIT_MAP_WALL) ? 1 : 0;
}

/** 软限位：按全局坐标点判断是否会撞墙 */
static uint8 soft_limit_is_wall_point(float x_m, float y_m)
{
    uint8 gx = chassis_m_to_grid_x(x_m);
    uint8 gy = chassis_m_to_grid_y(y_m);
    return soft_limit_is_wall_grid((int16)gx, (int16)gy);
}

/**
 * 软限位保护：预测下一步位置，若即将撞墙则减速并施加微转向。
 *
 * 逻辑：
 * 1) 以前瞻时间预测车体中心落点。
 * 2) 若前方落点位于墙体，则线速度强制缩放。
 * 3) 对比前左/前右两个试探点，向更空旷一侧施加角速度偏置。
 */
static void apply_soft_limit_guard(chassis_body_speed_cmd_t *cmd)
{
    float yaw_rad;
    float cy, sy;
    float vxg, vyg;
    float v_norm;
    float nx, ny;
    float left_x, left_y;
    float right_x, right_y;
    uint8 front_hit;
    uint8 left_hit;
    uint8 right_hit;

    if (!cmd) return;

    v_norm = sqrtf(cmd->vx_body_mps * cmd->vx_body_mps +
                   cmd->vy_body_mps * cmd->vy_body_mps);
    if (v_norm < SOFT_LIMIT_MIN_MOVE_EPS_MPS) return;

    yaw_rad = s_pose.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cy = cosf(yaw_rad);
    sy = sinf(yaw_rad);

    /* 车体系速度 -> 全局速度 */
    vxg = cy * cmd->vx_body_mps - sy * cmd->vy_body_mps;
    vyg = sy * cmd->vx_body_mps + cy * cmd->vy_body_mps;

    nx = s_pose.x_m + vxg * SOFT_LIMIT_LOOKAHEAD_S;
    ny = s_pose.y_m + vyg * SOFT_LIMIT_LOOKAHEAD_S;
    front_hit = soft_limit_is_wall_point(nx, ny);

    if (!front_hit) return;

    /* 触发软限位：先强制减速，避免继续顶墙 */
    cmd->vx_body_mps *= SOFT_LIMIT_BRAKE_SCALE;
    cmd->vy_body_mps *= SOFT_LIMIT_BRAKE_SCALE;

    /* 使用法向偏移评估左右绕行可行性 */
    left_x  = nx - sy * SOFT_LIMIT_SIDE_OFFSET_M;
    left_y  = ny + cy * SOFT_LIMIT_SIDE_OFFSET_M;
    right_x = nx + sy * SOFT_LIMIT_SIDE_OFFSET_M;
    right_y = ny - cy * SOFT_LIMIT_SIDE_OFFSET_M;

    left_hit  = soft_limit_is_wall_point(left_x, left_y);
    right_hit = soft_limit_is_wall_point(right_x, right_y);

    if (left_hit && !right_hit) {
        cmd->wz_dps -= SOFT_LIMIT_AVOID_WZ_DPS;
    } else if (!left_hit && right_hit) {
        cmd->wz_dps += SOFT_LIMIT_AVOID_WZ_DPS;
    } else {
        /* 两侧同样拥挤/开阔：保留原命令转向方向，给一个固定偏置 */
        if (cmd->wz_dps >= 0.0f) {
            cmd->wz_dps += SOFT_LIMIT_AVOID_WZ_DPS;
        } else {
            cmd->wz_dps -= SOFT_LIMIT_AVOID_WZ_DPS;
        }
    }
}

/** 完整执行链路: 滤波 → 运动学 → PID → 电机 */
static void apply_speed(chassis_body_speed_cmd_t cmd,
                        const float wheel_fb_mps[CHASSIS_WHEEL_COUNT])
{
    chassis_body_speed_cmd_t f;
    float targets[CHASSIS_WHEEL_COUNT];
    uint8 i;

    f = ramp_filter(cmd);
    s_last_cmd = f;

    chassis_mecanum_forward(f.vx_body_mps, f.vy_body_mps,
                            f.wz_dps * CHASSIS_DEG_TO_RAD_F, targets);
    chassis_mecanum_clamp_wheels(targets, CHASSIS_MAX_WHEEL_SPEED_MPS);

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        float pwm_forward_domain;
        float pwm_motor_domain;

        /* 低速停轮抑抖：目标和反馈都接近 0 时直接停车并清 PID 累积。 */
        if ((fabsf(targets[i]) < WHEEL_STOP_TARGET_EPS_MPS) &&
            (fabsf(wheel_fb_mps[i]) < WHEEL_STOP_FEEDBACK_EPS_MPS))
        {
            stop_wheel_with_pid_reset(i);
            continue;
        }

        /* 速度环统一使用“前进符号域”：目标和反馈都直接用 m/s 前进为正。 */
        pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], wheel_fb_mps[i]);

        /* 仅在最终电机输出时再乘电机方向修正系数，避免符号链路混乱。 */
        pwm_motor_domain = pwm_forward_domain * s_mot[i].dir_sign;

        chassis_motor_set_pwm(&s_mot[i], pwm_motor_domain);
    }
}

/** 单轮 PID 调试链路：仅一个轮子给目标速度，其余轮子目标为 0 */
static void apply_single_wheel_pid_debug(const float wheel_fb_mps[CHASSIS_WHEEL_COUNT])
{
    float targets[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    float debug_feedback_value = 0.0f;
    uint8 debug_idx;
    uint8 i;

    debug_idx = debug_wheel_index_safe(s_debug_wheel_index);

    /* 仅指定调试轮子允许非零目标速度。 */
    targets[debug_idx] = debug_target_speed_clamp(s_debug_wheel_target_mps);

    /* 调试模式不输出车体运动指令。 */
    s_last_cmd = (chassis_body_speed_cmd_t){0};
    s_ramp     = (chassis_body_speed_cmd_t){0};
    s_yaw_i    = 0.0f;

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        if (i == debug_idx)
        {
            float feedback_for_pid;
            float pwm_forward_domain;
            float pwm_motor_domain;

            feedback_for_pid = wheel_fb_mps[i] * s_debug_fb_sign_mul[i];

            /* 调试模式低速停轮抑抖：目标/反馈都接近 0 时不进入闭环。 */
            if ((fabsf(targets[i]) < WHEEL_STOP_TARGET_EPS_MPS) &&
                (fabsf(feedback_for_pid) < WHEEL_STOP_FEEDBACK_EPS_MPS))
            {
                stop_wheel_with_pid_reset(i);
                debug_feedback_value = feedback_for_pid;
                continue;
            }

            /* 若目标与反馈长期反号，自动翻转该轮反馈符号，打断正反馈发散。 */
            if ((fabsf(targets[i]) > WHEEL_DEBUG_START_TARGET_EPS_MPS) &&
                (fabsf(wheel_fb_mps[i]) > WHEEL_DEBUG_START_SPEED_EPS_MPS))
            {
                if ((targets[i] * feedback_for_pid) < 0.0f)
                {
                    if (s_debug_fb_sign_mismatch_cnt[i] < 255U)
                    {
                        s_debug_fb_sign_mismatch_cnt[i]++;
                    }
                }
                else
                {
                    s_debug_fb_sign_mismatch_cnt[i] = 0U;
                }

                if (s_debug_fb_sign_mismatch_cnt[i] >= WHEEL_DEBUG_SIGN_FIX_CONFIRM_CNT)
                {
                    s_debug_fb_sign_mul[i] = -s_debug_fb_sign_mul[i];
                    s_debug_fb_sign_mismatch_cnt[i] = 0U;
                    chassis_pid_reset(&s_pid[i]);
                    feedback_for_pid = wheel_fb_mps[i] * s_debug_fb_sign_mul[i];
                }
            }
            else
            {
                s_debug_fb_sign_mismatch_cnt[i] = 0U;
            }

            /* 单轮调试同样在“前进符号域”做闭环，打印值和控制值保持一致。 */
            pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], feedback_for_pid);
            pwm_motor_domain = pwm_forward_domain * s_mot[i].dir_sign;

            /* 起步抗静摩擦：目标非零但轮速接近零时，给最小启动 PWM。 */
            if ((fabsf(targets[i]) > WHEEL_DEBUG_START_TARGET_EPS_MPS) &&
                (fabsf(feedback_for_pid) < WHEEL_DEBUG_START_SPEED_EPS_MPS) &&
                (fabsf(pwm_motor_domain) < WHEEL_DEBUG_START_PWM_MIN))
            {
                float target_sign = (targets[i] >= 0.0f) ? 1.0f : -1.0f;
                pwm_motor_domain = WHEEL_DEBUG_START_PWM_MIN * target_sign * s_mot[i].dir_sign;
            }

            chassis_motor_set_pwm(&s_mot[i], pwm_motor_domain);
            debug_feedback_value = feedback_for_pid;
        }
        else
        {
            stop_wheel_with_pid_reset(i);
        }
    }

    chassis_pid_debug_feed_sample((chassis_wheel_index_t)debug_idx,
                                  targets[debug_idx],
                                  debug_feedback_value);
}

/**
 * 航向 PI 闭环
 * @param err        航向误差(°)，已归一化
 * @param compensate 非零时启用最小角速度补偿（原地保持专用，抵消静摩擦）
 * @return           角速度指令(°/s)
 */
static float yaw_pi(float err, uint8 compensate)
{
    float wz;

    /* 死区内：积分缓慢衰减，输出零 */
    if (fabsf(err) <= YAW_DEADZONE_DEG) {
        s_yaw_i *= 0.80f;
        return 0.0f;
    }

    /* PI 计算 */
    s_yaw_i += err * CHASSIS_TASK_DT_20MS_S;
    s_yaw_i  = chassis_clamp_f(s_yaw_i, -YAW_I_LIMIT, YAW_I_LIMIT);

    wz = g_chassis_tune_params.yaw_kp * err + YAW_KI * s_yaw_i;
    wz = chassis_clamp_f(wz, -g_chassis_tune_params.max_yaw_speed_dps, g_chassis_tune_params.max_yaw_speed_dps);

    /* 低速补偿: 确保有误差就有可执行输出 */
    if (compensate && fabsf(wz) < YAW_MIN_WZ_DPS)
        wz = (err >= 0.0f) ? YAW_MIN_WZ_DPS : -YAW_MIN_WZ_DPS;

    return wz;
}

/** 模式切换辅助: 清积分 + 设模式 */
static void enter_mode(ctrl_mode_t m)
{
    s_yaw_i = 0.0f;
    s_mode  = m;
}

/* ====================== 公共 API ====================== */

/*--- 初始化 ---*/

void chassis_ctrl_init(void)
{
    uint8 i;

    g_chassis_tune_params = sanitize(g_chassis_tune_params);
    chassis_imu_init();

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        chassis_motor_init(&s_mot[i]);
        chassis_encoder_init(&s_enc[i]);
        chassis_pid_init(&s_pid[i],
                         g_chassis_tune_params.wheel_pid_kp[i],
                         g_chassis_tune_params.wheel_pid_ki[i],
                         g_chassis_tune_params.wheel_pid_kd[i],
                         CHASSIS_MOTOR_PWM_MAX);
    }

    s_pose         = (chassis_pose_t){0};
    s_tgt_x_m      = 0.0f;
    s_tgt_y_m      = 0.0f;
    s_tgt_yaw_deg  = 0.0f;
    s_cmd_vx       = 0.0f;
    s_cmd_vy       = 0.0f;
    s_debug_wheel_index = (uint8)CHASSIS_WHEEL_LF;
    s_debug_wheel_target_mps = 0.0f;
    s_fb_vx        = 0.0f;
    s_fb_vy        = 0.0f;
    s_mode         = MODE_YAW_HOLD;
    s_arrived      = 1U;

    chassis_pid_debug_select_wheel(CHASSIS_WHEEL_LF);
    chassis_pid_debug_reset();

    force_stop();
}

/*--- 周期任务 ---*/

void chassis_ctrl_task_5ms(void)
{
    chassis_imu_update_5ms();
    s_pose.yaw_deg = chassis_imu_get_yaw_deg();
}

void chassis_ctrl_task_20ms(void)
{
    float ws_raw[CHASSIS_WHEEL_COUNT]; /* 四轮原始速度反馈 */
    float ws_pid[CHASSIS_WHEEL_COUNT]; /* 供闭环使用的滤波速度反馈 */
#if (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY)
    float vx_raw, vy_raw;
    float yaw_rad, cy, sy;
    chassis_body_speed_cmd_t cmd = {0};
#endif
    uint8 i;

    /* 1) 编码器采样 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        chassis_encoder_update(&s_enc[i], CHASSIS_TASK_DT_20MS_S);
        ws_raw[i] = chassis_encoder_get_speed(&s_enc[i]);

        if (0U == s_wheel_fb_lpf_inited)
        {
            s_wheel_fb_lpf[i] = ws_raw[i];
        }
        else
        {
            s_wheel_fb_lpf[i] += WHEEL_FB_LPF_ALPHA * (ws_raw[i] - s_wheel_fb_lpf[i]);
        }

        ws_pid[i] = s_wheel_fb_lpf[i];
    }
    s_wheel_fb_lpf_inited = 1U;

#if (1 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY)
    /*
     * 当前仅调试单轮 PID：
     * - MODE_SINGLE_WHEEL_PID_DEBUG: 执行单轮闭环
     * - 其他模式: 强制停机（相当于暂时注释掉其他模式主控制逻辑）
     */
    if (MODE_SINGLE_WHEEL_PID_DEBUG == s_mode)
    {
        apply_single_wheel_pid_debug(ws_pid);
    }
    else
    {
        force_stop();
    }
    return;
#else
    /* 2) 逆运动学 → 车体速度反馈 */
    chassis_mecanum_inverse(ws_pid, &vx_raw, &vy_raw);
    s_fb_vx = vx_raw * CHASSIS_ODOM_SCALE_X;
    s_fb_vy = vy_raw * CHASSIS_ODOM_SCALE_Y;

    /* 3) 里程计积分: 车体速度旋转到全局坐标系后累加 */
    yaw_rad = s_pose.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cy = cosf(yaw_rad);
    sy = sinf(yaw_rad);
    s_pose.x_m += (cy * s_fb_vx - sy * s_fb_vy) * CHASSIS_TASK_DT_20MS_S;
    s_pose.y_m += (sy * s_fb_vx + cy * s_fb_vy) * CHASSIS_TASK_DT_20MS_S;

    /* 4) 模式分支 → 生成车体速度指令 */
    switch (s_mode) {

    case MODE_POINT_NAV: {
        float dx   = s_tgt_x_m - s_pose.x_m;
        float dy   = s_tgt_y_m - s_pose.y_m;
        float dist = sqrtf(dx * dx + dy * dy);

        /* 到达判定 */
        if (dist <= CHASSIS_TARGET_REACHED_EPSILON_M) {
            s_arrived = 1U;
            s_mode    = MODE_YAW_HOLD;
            force_stop();
            return;
        }

        /* 位置 P → 全局速度（含模长限速） */
        float vxg  = g_chassis_tune_params.pos_kp * dx;
        float vyg  = g_chassis_tune_params.pos_kp * dy;
        float norm = sqrtf(vxg * vxg + vyg * vyg);
        if (norm > g_chassis_tune_params.max_linear_speed_mps && norm > 1e-6f) {
            float sc = g_chassis_tune_params.max_linear_speed_mps / norm;
            vxg *= sc;
            vyg *= sc;
        }

        /* 航向追踪: 仅 P，朝向目标点 */
        float tgt_yaw = atan2f(dy, dx) * CHASSIS_RAD_TO_DEG_F;
        float yerr    = chassis_normalize_angle_deg(tgt_yaw - s_pose.yaw_deg);
        s_yaw_i = 0.0f;  /* 移动态不积分 */

        /* 全局 → 车体坐标变换 */
        cmd.vx_body_mps =  cy * vxg + sy * vyg;
        cmd.vy_body_mps = -sy * vxg + cy * vyg;
        cmd.wz_dps      = chassis_clamp_f(g_chassis_tune_params.yaw_kp * yerr,
                                          -g_chassis_tune_params.max_yaw_speed_dps,
                                           g_chassis_tune_params.max_yaw_speed_dps);
        break;
    }

    case MODE_MOVE_YAW: {
        float yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
        cmd.vx_body_mps = s_cmd_vx;
        cmd.vy_body_mps = s_cmd_vy;
        cmd.wz_dps      = yaw_pi(yerr, 0U);  /* 不补偿最小角速度 */
        break;
    }

    case MODE_SINGLE_WHEEL_PID_DEBUG: {
        apply_single_wheel_pid_debug(ws_pid);
        return;
    }

    default: {  /* MODE_YAW_HOLD / MODE_ATT_DEBUG */
        float yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
        cmd.vx_body_mps = 0.0f;
        cmd.vy_body_mps = 0.0f;
        cmd.wz_dps      = yaw_pi(yerr, 1U);  /* 补偿静摩擦 */
        break;
    }
    }

    /* 5) 软限位保护：即将撞墙时减速并微调方向 */
    apply_soft_limit_guard(&cmd);

    apply_speed(cmd, ws_pid);
#endif
}

/*--- 运动指令 ---*/

void chassis_ctrl_move_to_grid(uint8 target_x_grid, uint8 target_y_grid)
{
    uint8 x = chassis_clamp_grid_x_inner(target_x_grid);
    uint8 y = chassis_clamp_grid_y_inner(target_y_grid);

    s_tgt_x_m = chassis_grid_x_to_m(x);
    s_tgt_y_m = chassis_grid_y_to_m(y);

    enter_mode(MODE_POINT_NAV);
    s_arrived = 0U;
}

void chassis_ctrl_set_move_yaw_cmd(float vx_body_mps, float vy_body_mps,
                                   float target_yaw_deg)
{
    s_cmd_vx       = vx_body_mps;
    s_cmd_vy       = vy_body_mps;
    s_tgt_yaw_deg  = chassis_normalize_angle_deg(target_yaw_deg);

    enter_mode(MODE_MOVE_YAW);
    s_arrived = 1U;
}

void chassis_ctrl_hold_yaw(float target_yaw_deg)
{
    s_tgt_yaw_deg = chassis_normalize_angle_deg(target_yaw_deg);
    enter_mode(MODE_YAW_HOLD);
    s_arrived = 1U;
}

void chassis_ctrl_start_single_wheel_pid_debug(uint8 wheel_index,
                                               float target_speed_mps)
{
    uint8 safe_wheel;

    safe_wheel = debug_wheel_index_safe(wheel_index);

    s_debug_wheel_index = safe_wheel;
    s_debug_wheel_target_mps = debug_target_speed_clamp(target_speed_mps);
    s_debug_fb_sign_mul[safe_wheel] = 1.0f;
    s_debug_fb_sign_mismatch_cnt[safe_wheel] = 0U;

    force_stop();
    chassis_pid_debug_select_wheel((chassis_wheel_index_t)safe_wheel);
    chassis_pid_debug_reset();

    enter_mode(MODE_SINGLE_WHEEL_PID_DEBUG);
    s_arrived = 1U;
}

void chassis_ctrl_set_single_wheel_pid_debug_target(float target_speed_mps)
{
    s_debug_wheel_target_mps = debug_target_speed_clamp(target_speed_mps);
}

void chassis_ctrl_stop_single_wheel_pid_debug(void)
{
    if (MODE_SINGLE_WHEEL_PID_DEBUG == s_mode)
    {
        force_stop();
        chassis_pid_debug_reset();
        enter_mode(MODE_YAW_HOLD);
    }
    s_arrived = 1U;
}

void chassis_ctrl_stop(void)
{
    enter_mode(MODE_YAW_HOLD);
    s_arrived = 1U;
    force_stop();
}

/*--- 航向调试 ---*/

void chassis_ctrl_attitude_debug_start_zero(void)
{
    force_stop();
    chassis_ctrl_hold_yaw(0.0f);
    s_mode = MODE_ATT_DEBUG;  /* 覆盖 hold_yaw 设置的模式 */
}

void chassis_ctrl_attitude_debug_get_state(chassis_attitude_debug_info_t *out)
{
    if (!out) return;
    out->target_yaw_deg  = s_tgt_yaw_deg;
    out->current_yaw_deg = s_pose.yaw_deg;
    out->yaw_err_deg     = chassis_normalize_angle_deg(
                               s_tgt_yaw_deg - s_pose.yaw_deg);
    out->wz_cmd_dps      = s_last_cmd.wz_dps;
}

void chassis_ctrl_attitude_debug_task_5ms(void)
{
    static uint8 div = 0U;
    chassis_attitude_debug_info_t info;

    if (++div < 20U) return;   /* 100ms 分频打印 */
    div = 0U;

    chassis_ctrl_attitude_debug_get_state(&info);
    printf("[YawCL] tgt=%.2f cur=%.2f err=%.2f wz=%.2f\r\n",
           info.target_yaw_deg, info.current_yaw_deg,
           info.yaw_err_deg,    info.wz_cmd_dps);
}

/*--- 状态查询 ---*/

uint8 chassis_ctrl_is_arrived(void)
{
    return s_arrived;
}

chassis_pose_t chassis_ctrl_get_pose(void)
{
    chassis_pose_t c = { s_pose.x_m, s_pose.y_m, s_pose.yaw_deg };
    return c;
}

chassis_body_speed_cmd_t chassis_ctrl_get_last_cmd(void)
{
    chassis_body_speed_cmd_t c = { s_last_cmd.vx_body_mps,
                                   s_last_cmd.vy_body_mps,
                                   s_last_cmd.wz_dps };
    return c;
}

/*--- 外部校正 ---*/

void chassis_ctrl_set_pose(float x_m, float y_m, float yaw_deg)
{
    s_pose.x_m     = x_m;
    s_pose.y_m     = y_m;
    s_pose.yaw_deg = chassis_normalize_angle_deg(yaw_deg);
    chassis_imu_set_yaw_deg(s_pose.yaw_deg);  /* 同步 IMU 防覆盖 */
}

/*--- 运行时调参 ---*/

void chassis_ctrl_get_tune_params(chassis_tune_params_t *out)
{
    if (!out) return;
    *out = g_chassis_tune_params;
}

void chassis_ctrl_set_tune_params(const chassis_tune_params_t *in)
{
    chassis_tune_params_t safe;
    uint8 i;

    if (!in) return;

    safe = sanitize(*in);
    g_chassis_tune_params = safe;

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        s_pid[i].kp = safe.wheel_pid_kp[i];
        s_pid[i].ki = safe.wheel_pid_ki[i];
        s_pid[i].kd = safe.wheel_pid_kd[i];
    }
}