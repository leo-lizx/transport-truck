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
#include "app_link.h"        /* P0-3: 软限位改用 app_link_get_map_snapshot() 拿一致地图副本   */
#include "zf_common_headfile.h"  /* P0-3: __DMB() / __disable_irq() 内存屏障与临界区          */
#include <math.h>

/* 软限位依赖：P0-3 解耦 g_game_map 直访, 改为运行时 seq-lock 快照 (apply_soft_limit_guard 内部拷贝) */

/* ---------------------- 航向闭环常量 ----------------------
 * 这 4 个参数已迁移到 chassis_config.h, 这里仅做本地短别名转发,
 * 方便阅读 yaw_pi() 时不用记前缀。如要调参请改 chassis_config.h。
 */
#define YAW_DEADZONE_DEG   CHASSIS_YAW_DEADZONE_DEG
#define YAW_KI             CHASSIS_YAW_KI
#define YAW_I_LIMIT        CHASSIS_YAW_I_LIMIT
#define YAW_MIN_WZ_DPS     CHASSIS_YAW_MIN_WZ_DPS

/* 单轮 PID 调试起步补偿参数（用于克服静摩擦） */
#define WHEEL_DEBUG_START_SPEED_EPS_MPS   (0.03f)   /* 低于此反馈速度视为静止 */
#define WHEEL_DEBUG_START_TARGET_EPS_MPS  (0.05f)   /* 低于此目标速度不启用补偿 */
#define WHEEL_DEBUG_START_PWM_MIN         (800.0f)  /* 起步最小 PWM 幅值 */
#define WHEEL_DEBUG_TARGET_RAMP_MPS_PER_TICK (0.8f) /* 20ms 每拍目标最多变化量 */
#define WHEEL_DEBUG_SIGN_FIX_GUARD_TICKS  (40U)     /* 起步保护期: 800ms 内只允许一次自动反号 */
#define WHEEL_DEBUG_SIGN_FIX_CONFIRM_CNT  (12U)     /* 反号连续计数(@20ms)达到该值才翻转反馈符号 */
/*
 * P0-修复(抖动): 自动反号本意是开机查接线用, 运行中触发会清零 PID, 引发周期性
 * "一抖一抖"现象. 改为: 仅在启动保护期(s_debug_startup_guard_ticks > 0)期间
 * 允许检测/翻转, 且每次启动只允许翻转一次. 之后无论反馈相位如何都不再动符号.
 *
 * P0-修复(2026-04-25 顿挫): 即使加了启动保护期, 启动初几拍编码器/电机瞬态
 * 仍可能让"目标·反馈反号"连续 12 拍触发翻转, 翻转动作里 chassis_pid_reset()
 * 会把 pid->output 清零 -> 实际 PWM 跌到 0 -> "顿一下后再起来". 在
 * chassis_config.h 里 ENC_SIGN/DIR_SIGN 已用 wfb 自检 + 单轮验证标定到位的
 * 工程下, 这个自动机制只会破坏闭环, 默认关闭. 真要重新标轮时再打开.
 */
#ifndef WHEEL_DEBUG_AUTO_FLIP_FB_SIGN
#define WHEEL_DEBUG_AUTO_FLIP_FB_SIGN     (0)
#endif

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
 *
 * 2026-04-27 调姿态闭环, 必须放开全部模式 (否则 MODE_YAW_HOLD 在 task_20ms 里会被 force_stop 短路).
 */
#define CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY   (0)

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

/* ==================================================================
 * 【P0-3】s_pose Seq-Lock
 *   写者:
 *     · chassis_ctrl_task_5ms()        — 写 yaw_deg     (PIT_IRQn)
 *     · chassis_ctrl_task_20ms()       — 写 x_m / y_m   (PIT_IRQn, 与 5ms 同 IRQ, 不嵌套)
 *     · chassis_ctrl_set_pose()        — 写全部三字段   (主循环, 必须 __disable_irq 防 PIT 抢占)
 *   读者:
 *     · chassis_ctrl_get_pose()        — 主循环
 *     · apply_soft_limit_guard()       — PIT_IRQn 内, 与写者同 IRQ 互斥, 但 LPUART1 ISR 可能抢占,
 *                                         走 seq-lock 读 仍然必要 (规范化语义)
 * ================================================================== */
static volatile uint32 s_pose_seq = 0U;

static inline void pose_write_begin(void)
{
    s_pose_seq++;       /* 偶 -> 奇: 标记 updating */
    __DMB();
}
static inline void pose_write_end(void)
{
    __DMB();
    s_pose_seq++;       /* 奇 -> 偶: 标记 stable   */
}

/* seq-lock 读: dst 写入一致快照. 重试上限 4 次, 超限计数报警. */
#define POSE_READ_RETRY_MAX    (4U)
volatile uint32 g_chassis_pose_snapshot_retry_giveup = 0U;

static void pose_read_snapshot(chassis_pose_t *dst)
{
    uint32 retry;
    uint32 s1;
    uint32 s2;

    if (dst == NULL) { return; }

    for (retry = 0U; retry <= POSE_READ_RETRY_MAX; ++retry)
    {
        s1 = s_pose_seq;
        if ((s1 & 1U) != 0U) { continue; }
        __DMB();
        dst->x_m     = s_pose.x_m;
        dst->y_m     = s_pose.y_m;
        dst->yaw_deg = s_pose.yaw_deg;
        __DMB();
        s2 = s_pose_seq;
        if (s1 == s2) { return; }
    }

    ++g_chassis_pose_snapshot_retry_giveup;
    /* 兜底: 直接裸读 (与 P0-3 之前行为一致, 但极少进入此分支) */
    dst->x_m     = s_pose.x_m;
    dst->y_m     = s_pose.y_m;
    dst->yaw_deg = s_pose.yaw_deg;
}

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
static volatile uint8 s_debug_fb_sign_flipped[CHASSIS_WHEEL_COUNT] = {0U, 0U, 0U, 0U}; /* 本次启动周期内是否已自动翻转过, 防止反复跳 */
static volatile float  s_debug_target_ramp_mps      = 0.0f;
static volatile uint16 s_debug_startup_guard_ticks  = 0U;

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

/** 调试目标速度限幅：仅供单轮 PID 调试使用 */
static float debug_target_speed_clamp(float target_speed_mps)
{
    return chassis_clamp_f(target_speed_mps,
                           -CHASSIS_DEBUG_WHEEL_SPEED_LIMIT_MPS,
                            CHASSIS_DEBUG_WHEEL_SPEED_LIMIT_MPS);
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

#if (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY)
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
#endif /* (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY) */

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

#if (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY)
/** 软限位：网格是否在可通行内场 */
static inline uint8 soft_limit_is_inner_grid(int16 gx, int16 gy)
{
    if (gx < (int16)CHASSIS_GRID_INNER_MIN_X || gx > (int16)CHASSIS_GRID_INNER_MAX_X) return 0;
    if (gy < (int16)CHASSIS_GRID_INNER_MIN_Y || gy > (int16)CHASSIS_GRID_INNER_MAX_Y) return 0;
    return 1;
}

/** 软限位：判断网格是否为墙/边界障碍 (P0-3: 改读传入的本地地图快照, 不再访 g_game_map) */
static uint8 soft_limit_is_wall_grid(const uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS],
                                     int16 gx, int16 gy)
{
    if (!soft_limit_is_inner_grid(gx, gy)) return 1;
    return (map[gy][gx] == SOFT_LIMIT_MAP_WALL) ? 1 : 0;
}

/** 软限位：按全局坐标点判断是否会撞墙 */
static uint8 soft_limit_is_wall_point(const uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS],
                                      float x_m, float y_m)
{
    uint8 gx = chassis_m_to_grid_x(x_m);
    uint8 gy = chassis_m_to_grid_y(y_m);
    return soft_limit_is_wall_grid(map, (int16)gx, (int16)gy);
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
    /* P0-3: 在 PIT_IRQn 上下文里取一致地图 + 一致位姿副本 (LPUART1 ISR 可能抢占) */
    uint8           map_snap[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS];
    chassis_pose_t  pose_snap;
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

    app_link_get_map_snapshot(map_snap);   /* 192B 栈拷贝, 与 BFS 共享时序 */
    pose_read_snapshot(&pose_snap);

    yaw_rad = pose_snap.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cy = cosf(yaw_rad);
    sy = sinf(yaw_rad);

    /* 车体系速度 -> 全局速度 */
    vxg = cy * cmd->vx_body_mps - sy * cmd->vy_body_mps;
    vyg = sy * cmd->vx_body_mps + cy * cmd->vy_body_mps;

    nx = pose_snap.x_m + vxg * SOFT_LIMIT_LOOKAHEAD_S;
    ny = pose_snap.y_m + vyg * SOFT_LIMIT_LOOKAHEAD_S;
    front_hit = soft_limit_is_wall_point(map_snap, nx, ny);

    if (!front_hit) return;

    /* 触发软限位：先强制减速，避免继续顶墙 */
    cmd->vx_body_mps *= SOFT_LIMIT_BRAKE_SCALE;
    cmd->vy_body_mps *= SOFT_LIMIT_BRAKE_SCALE;

    /* 使用法向偏移评估左右绕行可行性 */
    left_x  = nx - sy * SOFT_LIMIT_SIDE_OFFSET_M;
    left_y  = ny + cy * SOFT_LIMIT_SIDE_OFFSET_M;
    right_x = nx + sy * SOFT_LIMIT_SIDE_OFFSET_M;
    right_y = ny - cy * SOFT_LIMIT_SIDE_OFFSET_M;

    left_hit  = soft_limit_is_wall_point(map_snap, left_x, left_y);
    right_hit = soft_limit_is_wall_point(map_snap, right_x, right_y);

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

    /*
     * 单轮速度兜底限幅 (P0-修复 2026-04-27):
     *   多指令叠加下某轮目标可能超 PID/PWM 能追上的范围, 单轮饱和会破坏
     *   vx/vy/wz 比例 -> 直线偏 / 转弯半径跳变. 这里等比例缩放保证车体
     *   运动方向不变, 上限见 chassis_config.h 的 CHASSIS_WHEEL_SPEED_CAP_MPS.
     */
    chassis_mecanum_clamp_wheels(targets, CHASSIS_WHEEL_SPEED_CAP_MPS);

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

        /* 速度环统一使用"前进符号域"：目标和反馈都直接用 m/s 前进为正。 */
        pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], wheel_fb_mps[i]);

        /*
         * 静摩擦突破 (P0-修复 2026-04-27 姿态闭环落地不动):
         * 增量 PID 启动时 output=0, 在地面上轮子动不起来 -> 反馈一直 0 ->
         * P 项不再贡献增量, 只剩 Ki*e 慢慢爬, 落地要好几秒才有动作。
         * 这里把 |output| 强行抬到 BREAKAWAY_PWM_MIN, 同步写回 pid->output 让
         * 下一拍增量从这个基线继续累加, 避免出现"突破一拍又跌回 0"的顿挫。
         */
        if ((fabsf(targets[i]) >= CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS) &&
            (fabsf(wheel_fb_mps[i]) < CHASSIS_WHEEL_BREAKAWAY_SPEED_EPS_MPS) &&
            (fabsf(pwm_forward_domain) < CHASSIS_WHEEL_BREAKAWAY_PWM_MIN))
        {
            pwm_forward_domain = (targets[i] >= 0.0f)
                                 ?  CHASSIS_WHEEL_BREAKAWAY_PWM_MIN
                                 : -CHASSIS_WHEEL_BREAKAWAY_PWM_MIN;
            s_pid[i].output    = pwm_forward_domain;
        }

        /* 仅在最终电机输出时再乘电机方向修正系数，避免符号链路混乱。 */
        pwm_motor_domain = pwm_forward_domain * s_mot[i].dir_sign;

        chassis_motor_set_pwm(&s_mot[i], pwm_motor_domain);
    }
}
#endif /* (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY) */

/** 单轮 PID 调试链路：仅一个轮子给目标速度，其余轮子目标为 0 */
static void apply_single_wheel_pid_debug(const float wheel_fb_mps[CHASSIS_WHEEL_COUNT])
{
    float targets[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    float debug_feedback_value = 0.0f;
    uint8 debug_idx;
    uint8 i;

    debug_idx = debug_wheel_index_safe(s_debug_wheel_index);

    /* 仅指定调试轮子允许非零目标速度，并用斜坡避免目标速度突变。 */
    {
        float set_target = debug_target_speed_clamp(s_debug_wheel_target_mps);
        float diff = set_target - s_debug_target_ramp_mps;
        float step_lim = WHEEL_DEBUG_TARGET_RAMP_MPS_PER_TICK;
        if (diff > step_lim)
        {
            s_debug_target_ramp_mps += step_lim;
        }
        else if (diff < -step_lim)
        {
            s_debug_target_ramp_mps -= step_lim;
        }
        else
        {
            s_debug_target_ramp_mps = set_target;
        }
    }
    targets[debug_idx] = s_debug_target_ramp_mps;
    if (s_debug_startup_guard_ticks > 0U)
    {
        s_debug_startup_guard_ticks--;
    }

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

            /*
             * 若目标与反馈长期反号, 自动翻转该轮反馈符号, 打断正反馈发散.
             * P0-修复(抖动): 仅在启动保护期内、且本次启动还没翻过的前提下才允许触发,
             * 防止运行中 PID 振荡产生短时反号导致周期性翻转 + 重置 PID, 形成
             * "一抖一抖"的周期性抽搐.
             * P0-修复(2026-04-25 顿挫): 默认编译关闭. 见顶部
             *   WHEEL_DEBUG_AUTO_FLIP_FB_SIGN 注释.
             */
#if (WHEEL_DEBUG_AUTO_FLIP_FB_SIGN != 0)
            if ((s_debug_startup_guard_ticks > 0U) &&
                (s_debug_fb_sign_flipped[i] == 0U) &&
                (fabsf(targets[i]) > WHEEL_DEBUG_START_TARGET_EPS_MPS) &&
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
                    s_debug_fb_sign_flipped[i] = 1U; /* 本次启动只允许翻一次 */
                    chassis_pid_reset(&s_pid[i]);
                    feedback_for_pid = wheel_fb_mps[i] * s_debug_fb_sign_mul[i];
                }
            }
            else
            {
                s_debug_fb_sign_mismatch_cnt[i] = 0U;
            }
#else
            /* 自动反号默认关闭, 维持中性状态以备宏开启时不会误触发 */
            s_debug_fb_sign_mismatch_cnt[i] = 0U;
#endif

            /* 单轮调试同样在“前进符号域”做闭环，打印值和控制值保持一致。 */
            pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], feedback_for_pid);
            pwm_motor_domain = pwm_forward_domain * s_mot[i].dir_sign;

            /*
             * 起步抗静摩擦: 目标非零但轮速接近零时, 给最小启动 PWM.
             * P0-修复(抖动): 软启动覆盖电机 PWM 时, 同步把 PID 内部累加器 pid->output
             * 钳到与之等价的"前进域"PWM 上, 实现无扰切换 (bumpless transfer).
             * 否则等反馈一过 0.03 m/s 软启动释放, PID 累加器还停在低值或继续增量,
             * 会让最终 PWM 在 800 与 PID 自由值之间产生明显阶跃, 形成第二种抖动源.
             */
            if ((fabsf(targets[i]) > WHEEL_DEBUG_START_TARGET_EPS_MPS) &&
                (fabsf(feedback_for_pid) < WHEEL_DEBUG_START_SPEED_EPS_MPS) &&
                (fabsf(pwm_motor_domain) < WHEEL_DEBUG_START_PWM_MIN))
            {
                float target_sign = (targets[i] >= 0.0f) ? 1.0f : -1.0f;
                pwm_motor_domain = WHEEL_DEBUG_START_PWM_MIN * target_sign * s_mot[i].dir_sign;
                /* PID 累加器钳到前进域等价值, 让下次 chassis_pid_step 在此基础上做增量 */
                s_pid[i].output = WHEEL_DEBUG_START_PWM_MIN * target_sign;
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

#if (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY)
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
#endif /* (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY) */

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
    /*
     * 不在这里写死调试轮索引. 真正的"选轮"由 chassis_ctrl_start_single_wheel_pid_debug()
     * 统一负责 (它会同时设置 s_debug_wheel_index 和 chassis_pid.c 里的
     * s_pid_debug_wheel_index). 这里写死任何一个轮都会变成跟 main 里 MAIN_PID_DEBUG_WHEEL_INDEX
     * 不一致的隐藏 footgun.
     */
    s_debug_wheel_target_mps = 0.0f;
    s_fb_vx        = 0.0f;
    s_fb_vy        = 0.0f;
    s_mode         = MODE_YAW_HOLD;
    s_arrived      = 1U;

    chassis_pid_debug_reset();

    force_stop();
}

/*--- 周期任务 ---*/

void chassis_ctrl_task_5ms(void)
{
    float yaw_now;
    chassis_imu_update_5ms();
    yaw_now = chassis_imu_get_yaw_deg();
    /* P0-3: seq-lock 写 yaw_deg */
    pose_write_begin();
    s_pose.yaw_deg = yaw_now;
    pose_write_end();
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

    /* 3) 里程计积分: 车体速度旋转到全局坐标系后累加 (P0-3: seq-lock 写 x/y) */
    yaw_rad = s_pose.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cy = cosf(yaw_rad);
    sy = sinf(yaw_rad);
    {
        float new_x = s_pose.x_m + (cy * s_fb_vx - sy * s_fb_vy) * CHASSIS_TASK_DT_20MS_S;
        float new_y = s_pose.y_m + (sy * s_fb_vx + cy * s_fb_vy) * CHASSIS_TASK_DT_20MS_S;
        pose_write_begin();
        s_pose.x_m = new_x;
        s_pose.y_m = new_y;
        pose_write_end();
    }

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
    s_debug_fb_sign_flipped[safe_wheel] = 0U; /* 新一轮启动, 重置"已翻过"标记 */
    s_debug_target_ramp_mps = 0.0f;
    s_debug_startup_guard_ticks = WHEEL_DEBUG_SIGN_FIX_GUARD_TICKS;

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

void chassis_ctrl_get_wheel_feedback_snapshot(float out_wheel_fb_mps[4])
{
    uint8 i;
    if (0 == out_wheel_fb_mps) return;
    /* 直接读 LPF 后的速度快照, 与 PID 闭环用的反馈完全一致 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        out_wheel_fb_mps[i] = s_wheel_fb_lpf[i];
    }
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
    chassis_pose_t pose_snap;
    if (!out) return;
    pose_read_snapshot(&pose_snap);     /* P0-3: 一致性快照, 避免与 5ms ISR 写竞争 */
    out->target_yaw_deg  = s_tgt_yaw_deg;
    out->current_yaw_deg = pose_snap.yaw_deg;
    out->yaw_err_deg     = chassis_normalize_angle_deg(
                               s_tgt_yaw_deg - pose_snap.yaw_deg);
    out->wz_cmd_dps      = s_last_cmd.wz_dps;
}

/** 调试用: 拷贝四轮反馈 + PID 输出快照, 方便诊断方向/PWM */
void chassis_ctrl_attitude_debug_get_wheel_pwm(float out_pwm[CHASSIS_WHEEL_COUNT])
{
    uint8 i;
    if (!out_pwm) return;
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        out_pwm[i] = s_pid[i].output;   /* 前进符号域 PWM, 未乘 dir_sign */
    }
}

void chassis_ctrl_attitude_debug_task_5ms(void)
{
    static uint8 div = 0U;
    chassis_attitude_debug_info_t info;
    float pwm_snap[CHASSIS_WHEEL_COUNT];
    float fb_snap[CHASSIS_WHEEL_COUNT];

    /* 50ms 分频 (10 * 5ms tick): 比 100ms 更密, 上位机绘曲线更平滑 */
    if (++div < 10U) return;
    div = 0U;

    chassis_ctrl_attitude_debug_get_state(&info);
    chassis_ctrl_attitude_debug_get_wheel_pwm(pwm_snap);
    chassis_ctrl_get_wheel_feedback_snapshot(fb_snap);

    /*
     * 一行打印, 空格分隔, 单位:
     *   tgt actual err [deg]   wz [dps]
     *   PWM[LF RF LB RB] 前进符号域 (未乘 dir_sign)
     *   FB[LF RF LB RB]  m/s
     *
     * 方向自检:
     *   - err > 0  应有 wz > 0  (yaw_pi 同号)
     *   - wz > 0   按麦轮公式: PWM_LF<0 PWM_RF>0 PWM_LB<0 PWM_RB>0 (左侧后转, 右侧前转 -> 车体俯视逆时针)
     *   - 如果实车转向跟 "逆时针" 反, 翻 chassis_config.h 的 CHASSIS_IMU_YAW_SIGN
     */
    printf("[YawCL] tgt=%6.2f act=%6.2f err=%6.2f wz=%6.2f | PWM=%6.0f %6.0f %6.0f %6.0f | FB=%5.2f %5.2f %5.2f %5.2f\r\n",
           info.target_yaw_deg,
           info.current_yaw_deg,
           info.yaw_err_deg,
           info.wz_cmd_dps,
           pwm_snap[CHASSIS_WHEEL_LF], pwm_snap[CHASSIS_WHEEL_RF],
           pwm_snap[CHASSIS_WHEEL_LB], pwm_snap[CHASSIS_WHEEL_RB],
           fb_snap[CHASSIS_WHEEL_LF], fb_snap[CHASSIS_WHEEL_RF],
           fb_snap[CHASSIS_WHEEL_LB], fb_snap[CHASSIS_WHEEL_RB]);
}

/*--- 状态查询 ---*/

uint8 chassis_ctrl_is_arrived(void)
{
    return s_arrived;
}

chassis_pose_t chassis_ctrl_get_pose(void)
{
    /* P0-3: seq-lock 读, 防止主循环看到 (新yaw, 旧x, 旧y) 的撕裂 */
    chassis_pose_t c;
    pose_read_snapshot(&c);
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
    /* P0-3: 主循环写者会被 PIT 抢占; 用临界区避免与 5ms/20ms ISR 写者交错破坏 seq 奇偶 */
    float yaw_norm = chassis_normalize_angle_deg(yaw_deg);
    __disable_irq();
    pose_write_begin();
    s_pose.x_m     = x_m;
    s_pose.y_m     = y_m;
    s_pose.yaw_deg = yaw_norm;
    pose_write_end();
    __enable_irq();
    chassis_imu_set_yaw_deg(yaw_norm);  /* 同步 IMU 防覆盖 (内部已自带防护) */
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

/* ==================================================================
 * 【P0-8】发车区 / 越界几何判定 实现
 * ----------------------------------------------------------------
 * 设计取舍:
 *   1) 全部判定基于 chassis_ctrl_get_pose() 的 seq-lock 快照 (已自带 P0-3 保护),
 *      不读 g_game_map, 与软限位解耦.
 *   2) 速度估算用 5ms 差分 + IIR 平滑, 避免占用 chassis_imu / chassis_encoder
 *      的内部状态; 调用方必须保证 chassis_zone_tick() 周期 ≈ 5ms.
 *   3) "完全离开发车区" 用车体外接圆与发车区矩形不相交判据,
 *      比"几何中心在区外" 在边界处更稳, 与赛规"完全离开"语义一致.
 *   4) OOB 滞回: 单向置位, 显式 chassis_zone_clear_oob() 才能清.
 *      避免位姿抖动反复触发刹停清 PID 积分.
 *   5) 全部状态为 file-static, 仅主循环单线程访问, 无锁.
 * ================================================================== */

/* 发车区矩形定义 (X_min, X_max, Y_min, Y_max) — Y 向下为正 */
typedef struct {
    float x_min_m;
    float x_max_m;
    float y_min_m;
    float y_max_m;
} chassis_rect_m_t;

/* 上一帧位姿 (用于差分速度); inited=0 时跳过当帧差分 */
static chassis_pose_t  s_zone_last_pose      = {0.0f, 0.0f, 0.0f};
static uint8           s_zone_last_pose_inited = 0U;

/* 一阶 IIR 平滑后的车体平移速度模长 (m/s) */
static float           s_zone_speed_lpf_mps  = 0.0f;

/* 静止累计计数 (单位: tick), 与 CHASSIS_STATIC_HOLD_MS 比较 */
static uint16          s_zone_static_ticks   = 0U;
#define ZONE_TICK_PERIOD_MS                 (5U)
#define ZONE_STATIC_HOLD_TICKS              ((uint16)(CHASSIS_STATIC_HOLD_MS / ZONE_TICK_PERIOD_MS))

/* 越界滞回标志 (1 = 已置位, 业务不复位则永久保持) */
static uint8           s_zone_oob_latched    = 0U;

/* ----- 内部辅助 ----- */

/** 取指定发车区矩形 (LAUNCH_ZONE_ANY 由上层拆 LEFT/RIGHT 两次调用) */
static void zone_get_launch_rect(LaunchZone_e zone, chassis_rect_m_t *out)
{
    /* Y 轴向下, "下边界" 即 Y = CHASSIS_MAP_HEIGHT_M */
    out->y_max_m = CHASSIS_MAP_HEIGHT_M - CHASSIS_LAUNCH_ZONE_BOTTOM_OFFSET_M;
    out->y_min_m = out->y_max_m - CHASSIS_LAUNCH_ZONE_H_M;

    if (zone == LAUNCH_ZONE_RIGHT) {
        out->x_max_m = CHASSIS_MAP_WIDTH_M;
        out->x_min_m = CHASSIS_MAP_WIDTH_M - CHASSIS_LAUNCH_ZONE_W_M;
    } else {
        /* 默认左发车区 */
        out->x_min_m = 0.0f;
        out->x_max_m = CHASSIS_LAUNCH_ZONE_W_M;
    }
}

/** 点是否在矩形内 (闭区间) */
static uint8 zone_point_in_rect(float x, float y, const chassis_rect_m_t *r)
{
    return ((x >= r->x_min_m) && (x <= r->x_max_m) &&
            (y >= r->y_min_m) && (y <= r->y_max_m)) ? 1U : 0U;
}

/**
 * 圆 (cx, cy, R) 是否与矩形 r 不相交 (即 "完全离开矩形").
 * 判据: 圆心到矩形最近点的距离 > R.
 */
static uint8 zone_circle_outside_rect(float cx, float cy, float radius_m,
                                      const chassis_rect_m_t *r)
{
    float dx = 0.0f;
    float dy = 0.0f;

    if      (cx < r->x_min_m) dx = r->x_min_m - cx;
    else if (cx > r->x_max_m) dx = cx - r->x_max_m;

    if      (cy < r->y_min_m) dy = r->y_min_m - cy;
    else if (cy > r->y_max_m) dy = cy - r->y_max_m;

    return ((dx * dx + dy * dy) > (radius_m * radius_m)) ? 1U : 0U;
}

/* ----- 对外 API ----- */

void chassis_zone_tick(void)
{
    chassis_pose_t  cur;
    float           dx_m;
    float           dy_m;
    float           inst_speed_mps;
    float           cx;
    float           cy;
    uint8           outside_field;

    cur = chassis_ctrl_get_pose();

    /* (1) 速度估算: 5ms 差分 + 一阶 IIR 平滑 */
    if (s_zone_last_pose_inited) {
        dx_m = cur.x_m - s_zone_last_pose.x_m;
        dy_m = cur.y_m - s_zone_last_pose.y_m;
        /* 已知 dt = ZONE_TICK_PERIOD_MS/1000 = 0.005s,
         * 用乘以 200.0f 等价除以 0.005, 省一次浮点除法 */
        inst_speed_mps = sqrtf(dx_m * dx_m + dy_m * dy_m) * 200.0f;
        s_zone_speed_lpf_mps = (1.0f - CHASSIS_SPEED_LPF_ALPHA) * s_zone_speed_lpf_mps
                             + CHASSIS_SPEED_LPF_ALPHA * inst_speed_mps;
    } else {
        s_zone_last_pose_inited = 1U;
        s_zone_speed_lpf_mps    = 0.0f;
    }
    s_zone_last_pose = cur;

    /* (2) 静止累计: 速度低于阈值才累加; 否则立即清零 */
    if (s_zone_speed_lpf_mps < CHASSIS_STATIC_SPEED_EPS_MPS) {
        if (s_zone_static_ticks < 0xFFFFU) {
            s_zone_static_ticks++;
        }
    } else {
        s_zone_static_ticks = 0U;
    }

    /* (3) OOB 滞回: 单向置位
     *   已置位 → 不再判, 必须 chassis_zone_clear_oob() 才能解锁 */
    if (s_zone_oob_latched) {
        return;
    }

    /* 车体外接圆穿出最外圈围墙 > HYSTERESIS 才置位 */
    cx = cur.x_m;
    cy = cur.y_m;
    outside_field = 0U;
    if ((cx + CHASSIS_BODY_RADIUS_M) > (CHASSIS_MAP_WIDTH_M  + CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;
    if ((cx - CHASSIS_BODY_RADIUS_M) < (0.0f                 - CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;
    if ((cy + CHASSIS_BODY_RADIUS_M) > (CHASSIS_MAP_HEIGHT_M + CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;
    if ((cy - CHASSIS_BODY_RADIUS_M) < (0.0f                 - CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;

    if (outside_field) {
        s_zone_oob_latched = 1U;
    }
}

uint8 chassis_zone_is_in_launch(LaunchZone_e zone)
{
    chassis_pose_t   pose_snap;
    chassis_rect_m_t rect;

    pose_snap = chassis_ctrl_get_pose();

    if (zone == LAUNCH_ZONE_ANY) {
        zone_get_launch_rect(LAUNCH_ZONE_LEFT, &rect);
        if (zone_point_in_rect(pose_snap.x_m, pose_snap.y_m, &rect)) return 1U;
        zone_get_launch_rect(LAUNCH_ZONE_RIGHT, &rect);
        return zone_point_in_rect(pose_snap.x_m, pose_snap.y_m, &rect);
    }

    zone_get_launch_rect(zone, &rect);
    return zone_point_in_rect(pose_snap.x_m, pose_snap.y_m, &rect);
}

uint8 chassis_zone_is_fully_outside_launch(LaunchZone_e zone)
{
    chassis_pose_t   pose_snap;
    chassis_rect_m_t rect;

    pose_snap = chassis_ctrl_get_pose();

    if (zone == LAUNCH_ZONE_ANY) {
        /* 必须同时离开左 + 右两个发车区 */
        zone_get_launch_rect(LAUNCH_ZONE_LEFT, &rect);
        if (!zone_circle_outside_rect(pose_snap.x_m, pose_snap.y_m,
                                      CHASSIS_BODY_RADIUS_M, &rect)) return 0U;
        zone_get_launch_rect(LAUNCH_ZONE_RIGHT, &rect);
        return zone_circle_outside_rect(pose_snap.x_m, pose_snap.y_m,
                                        CHASSIS_BODY_RADIUS_M, &rect);
    }

    zone_get_launch_rect(zone, &rect);
    return zone_circle_outside_rect(pose_snap.x_m, pose_snap.y_m,
                                    CHASSIS_BODY_RADIUS_M, &rect);
}

uint8 chassis_zone_is_out_of_bounds(void)
{
    return s_zone_oob_latched;
}

void chassis_zone_clear_oob(void)
{
    s_zone_oob_latched = 0U;
}

uint8 chassis_zone_is_static(void)
{
    return (s_zone_static_ticks >= ZONE_STATIC_HOLD_TICKS) ? 1U : 0U;
}