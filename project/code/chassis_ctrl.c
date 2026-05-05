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
/* P0-修复 2026-04-29 姿态环“一段一段”真凶:
 * 原阈值 0.015 m/s, 但 yaw 转 1° 需 wheel target ≈ 0.023 m/s, 仅高出 53%,
 * wz 一抖 target 跌破 → stop_wheel_with_pid_reset 把 PWM 拍 0 → 下一拍
 * target 变大又恢复 → PWM 跳变 → 表现为输出“段段起止”。
 * 将阈值调到 0.005 m/s, 让 yaw±1° 场景 (target ≈0.023) 有 4.5x 余量,
 * 小抖动不会再跳变 → PWM 输出连续. */
#define WHEEL_STOP_TARGET_EPS_MPS         (0.005f)  /* 原 0.015, 调小防小角度阈值跳变 */
#define WHEEL_STOP_FEEDBACK_EPS_MPS       (0.010f)  /* 原 0.030, 同步调小 */

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
/* 1 = chassis_ctrl_move_to_m() 主动锁定航向, 禁止 POINT_NAV 任务用 atan2 覆盖 */
static volatile uint8 s_nav_lock_yaw  = 0U;
/* 直线路径方向单位向量 (move_to_m 调用时按起点→终点计算).
 * 与 s_tgt_yaw_deg (机器人头部朝向) 完全解耦, 麦轮平移时两者通常不同. */
static volatile float s_path_cos_phi  = 1.0f;
static volatile float s_path_sin_phi  = 0.0f;

/* D 项低通状态 (一阶 IIR, 消除 odom 高频噪声对 KD 的放大) */
static float s_v_along_lpf = 0.0f;
static float s_v_cross_lpf = 0.0f;

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
/* In-Position Schmitt 锁状态: 1 = 已在位 (输出硬归零, I 冻结)
 * 由 yaw_pi() 内部按双阈值滞回切换, 模式切换/积分复位时强制清 0. */
static volatile uint8 s_yaw_in_position = 0;

/* 里程计反馈 */
static volatile float s_fb_vx  = 0.0f;
static volatile float s_fb_vy  = 0.0f;
static volatile float s_wheel_fb_lpf[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* P0-改进 2026-05-02 编码器 yaw 弱观测: odom 推算的 yaw 角积分量,
 * 每 20ms 用 (-vLF + vRF - vLB + vRB) / (4*K) 计算瞬时 wz_odom 后积分.
 * 通过 chassis_imu_kf_correct_angle() 喂给 IMU KF, R 设大让长期对齐, 短期不打架. */
#if (CHASSIS_ODOM_YAW_FUSION_ENABLE != 0)
static volatile float s_yaw_odom_deg = 0.0f;
static volatile uint8 s_yaw_odom_inited = 0U;
#endif

/* P0-重构 2026-04-29 PWM 连续化: 原静摩擦突破状态机已移除,
 * 改为 apply_speed() 里的加性前馈 (不覆写 PID 内部状态, PWM 输出连续).
 * 用不到了, 代码保留 stop_wheel_with_pid_reset() 仅供 force_stop() 使用. */
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
        const float abs_target = fabsf(targets[i]);

        /* P0-重构 2026-04-29 PWM 连续化: PID 永远连续计算, 不再做 WHEEL_STOP
         * 硬清零 (旧版 stop_wheel_with_pid_reset 会把 pid->output 拍 0 ->
         * 下一拍从 0 重新累加 -> PWM 跌崖式跳变, 表现为"一段一段").
         * target=0 且 fb≈0 时增量 PID 自然衰减到 0, 不需要手动中断. */
        pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], wheel_fb_mps[i]);

        /*
         * 静摩擦前馈 (P0-重构 2026-04-29):
         *
         * 旧版状态机问题: 进入 breakaway 时直接 s_pid[i].output = kick (覆盖!),
         * PID 输出从 <FLOOR 阶跃到 ≥FLOOR; 退出时又恢复 PID 自累加.
         * 状态机 on/off 翻转 = PWM 阶跃 = "一段一段"输出.
         *
         * 新方案: 去状态机, 改纯加性平滑前馈
         *   ff = sign(target) * FLOOR * smooth_ramp(|target|)
         * 性质:
         *   · target=0      → ff=0,                输出 = PID
         *   · |target|=EPS  → ff=0   (边界连续过零)
         *   · |target|≥2EPS → ff=±FLOOR (静摩擦补偿到位)
         *   · 不覆盖 PID 内部状态, 闭环不会被打断
         *   · PID 看到反馈起来后会主动减小输出, 与 ff 自然平衡
         */
        if (abs_target > CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS) {
            float ramp = (abs_target - CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS)
                       /  CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS;
            float ff_pwm;
            if (ramp > 1.0f) ramp = 1.0f;
            ff_pwm = CHASSIS_WHEEL_BREAKAWAY_PWM_FLOOR * ramp;
            if (targets[i] < 0.0f) ff_pwm = -ff_pwm;
            pwm_forward_domain += ff_pwm;
        }

        /* PID + 前馈叠加后再做总限幅, 避免硬件溢出 */
        pwm_forward_domain = chassis_clamp_f(pwm_forward_domain,
                                             -CHASSIS_MOTOR_PWM_MAX,
                                              CHASSIS_MOTOR_PWM_MAX);

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
 * sqrt_controller —— 时间最优"位置 → 速度"控制器
 * 来源: ArduPilot AC_AttitudeControl::sqrt_controller (GPLv3, 已工业验证多年).
 *
 * 数学原理:
 *   远处:  v(err) = sign(err) * sqrt(2 * a_max * |err|)   <- 最大减速曲线
 *   近处:  v(err) = p * err                              <- 线性 P, 平滑无 chatter
 *   切点:  |err| = a_max / p^2  (此处两段一阶光滑衔接, 速率与斜率连续)
 *
 * 几何含义: 远处按"以 a_max 减速恰好停在目标"的速度走 -> 不会超调;
 *          近处用线性 P 避免 sqrt 在 0 附近无穷大斜率引起的抖动.
 *
 * 调参口诀:
 *   - p:     只决定"近 0"段快慢, 大胆加大 (4~10), 不影响大角度
 *   - a_max: 决定减速距离 = v_max^2 / (2*a_max), 受底盘物理减速能力限制
 *
 * @param error    位置误差 (°)
 * @param p        线性段比例增益 (°/s per °)
 * @param accel_max 最大减速度 (°/s²); <=0 退化为纯 P
 * @return         期望角速度 (°/s)
 */
static float sqrt_controller(float error, float p, float accel_max)
{
    float correction_rate;

    if (accel_max <= 0.0f) {
        correction_rate = error * p;
    } else if (p <= 0.0f) {
        if (error > 0.0f) {
            correction_rate = sqrtf(2.0f * accel_max * error);
        } else if (error < 0.0f) {
            correction_rate = -sqrtf(2.0f * accel_max * (-error));
        } else {
            correction_rate = 0.0f;
        }
    } else {
        /* linear_dist: 线性段长度, p 越大切换点越近 (平方反比) */
        float linear_dist = accel_max / (p * p);
        if (error > linear_dist) {
            /* 远处 sqrt 段, 减 0.5*linear_dist 让两段在切点处 v 与 dv/derr 都连续 */
            correction_rate = sqrtf(2.0f * accel_max * (error - linear_dist * 0.5f));
        } else if (error < -linear_dist) {
            correction_rate = -sqrtf(2.0f * accel_max * ((-error) - linear_dist * 0.5f));
        } else {
            correction_rate = error * p;
        }
    }
    return correction_rate;
}

/**
 * 航向 PI 闭环
 * @param err              航向误差(°)，已归一化
 * @param enable_rate_loop 是否启用内环 rATE PI 反馈:
 *                          1 = MODE_YAW_HOLD / POINT_NAV (静止或低振动) -> 完整级联, 享KI学摩擦
 *                          0 = MODE_MOVE_YAW (平动中) -> 开环 rate, 避免 IMU 振动噪声被KI放大成限环
 *                          原因: 平动时底盘振动使 IMU yaw_rate 噪声 ±10°/s, 进 rate_err
 *                          进 I -> wz 护 -> 麦轮护 -> 车体更护. 平动期间只靠
 *                          前馈+外环P 足以完成轨迹跟踪, 稳态偏差由后续 YAW_HOLD 兜底.
 * @return                 角速度指令(°/s)
 */
/**
 * @param allow_inpos_lock  1=允许 Schmitt-trigger 在位锁 (YAW_HOLD 静止保持);
 *                          0=禁止锁 (POINT_NAV/MOVE_YAW 平动中, 锁会在 yaw_err≈0
 *                            时立即触发导致姿态环整拍输出 0, 车体漂转无纠正)
 */
static float yaw_pi(float err, uint8 enable_rate_loop, uint8 allow_inpos_lock)
{
    const float yaw_rate_dps = chassis_imu_get_yaw_rate_dps();   /* 实测车体角速度 */
    float wz;

#if (CHASSIS_YAW_USE_CASCADED_CTRL != 0)
    /* ==================================================================
     * 0) In-Position Schmitt-trigger 锁 (P0-改进 2026-05-02 收尾抖动)
     *    工业伺服通用结构, 解决 "wz_target 残值 + 轮端 breakaway 阶跃"
     *    引发的 20Hz 极限环. 一旦判为在位 -> 全链路硬归零, I 冻结,
     *    必须 |err| 越过更大的释放阈值才解锁.
     *
     *    allow_inpos_lock=0 时整个块旁路: 平动模式下 yaw_err≈0 会立即触发
     *    锁定 -> 姿态环输出恒为 0 -> 车体自由漂转, 这是严重 bug.
     * ================================================================== */
    if (allow_inpos_lock) {
        const float abs_err  = fabsf(err);
        const float abs_rate = fabsf(yaw_rate_dps);

        if (s_yaw_in_position) {
            /* 已在位: 误差超过释放阈值才解锁 */
            if (abs_err > CHASSIS_YAW_INPOS_EXIT_DEG) {
                s_yaw_in_position = 0;
            }
        } else {
            /* 未在位: 误差进入入锁阈值 + 车体已稳定 -> 锁死 */
            if ((abs_err < CHASSIS_YAW_INPOS_ENTER_DEG) &&
                (abs_rate < CHASSIS_YAW_INPOS_SETTLE_DPS)) {
                s_yaw_in_position = 1;
                s_yaw_i = 0.0f;        /* 锁定瞬间清空积分, 防解锁后甩出 */
            }
        }

        if (s_yaw_in_position) {
            /* 整条链路硬归零, 直接断开传染源 */
            return 0.0f;
        }
    } else {
        s_yaw_in_position = 0;   /* 平动中强制清除残留锁, 防 YAW_HOLD -> POINT_NAV 切换残留 */
    }

    /* ==================================================================
     * 级联 P-PI + 速度前馈 (P0-改进 2026-05-02 移植自 ArduPilot)
     * ================================================================== */
    {
        /* 1) 外环: 位置误差 -> 期望角速度 (sqrt 时间最优曲线) */
        float wz_target = sqrt_controller(err,
                                          g_chassis_tune_params.yaw_kp,
                                          CHASSIS_YAW_ACCEL_MAX_DPS2);
        float rate_err;
        float kp_v_term;
        float ki_v_term;

        wz_target = chassis_clamp_f(wz_target,
                                    -g_chassis_tune_params.max_yaw_speed_dps,
                                     g_chassis_tune_params.max_yaw_speed_dps);

        /* 2) 内环 PI 输入: 角速度跟踪误差 (仅静态开启反馈, 平动时 enable=0) */
        if (enable_rate_loop) {
            rate_err = wz_target - yaw_rate_dps;
        } else {
            /* 平动模式: rate_err 强制为 0 ->
             *   - kp_v_term = 0    (不被 IMU 振动噪声驱动)
             *   - I 仅按泄漏衰减, 不再累积振动能量
             * 输出 = wz_target (纯前馈+外环P), 足以跟上轻量级转向需求。 */
            rate_err = 0.0f;
        }

        /* 3) 反向积分卷绕保护: 误差换向时清零, 防止旧能量对抗新方向 */
        if ((err * s_yaw_i) < 0.0f) {
            s_yaw_i = 0.0f;
        }

        /* 4) I 全程累积 + 缓慢泄漏 (in-position 已在上方处理, 这里无需再判死区) */
        s_yaw_i = s_yaw_i * (1.0f - CHASSIS_YAW_RATE_I_LEAK)
                + rate_err * CHASSIS_TASK_DT_20MS_S;
        s_yaw_i = chassis_clamp_f(s_yaw_i,
                                  -CHASSIS_YAW_RATE_I_LIMIT,
                                   CHASSIS_YAW_RATE_I_LIMIT);

        kp_v_term = CHASSIS_YAW_RATE_KP * rate_err;
        ki_v_term = CHASSIS_YAW_RATE_KI * s_yaw_i;

        /* 5) 输出 = 前馈 + PI 修正 */
        wz = wz_target + kp_v_term + ki_v_term;
    }

    /* 6) 输出截幅 */
    wz = chassis_clamp_f(wz,
                         -g_chassis_tune_params.max_yaw_speed_dps,
                          g_chassis_tune_params.max_yaw_speed_dps);

#else /* ============== 旧的单环 PID 路径 (回退) ============== */
    {
        float p_term = sqrt_controller(err, g_chassis_tune_params.yaw_kp, CHASSIS_YAW_ACCEL_MAX_DPS2);
        float d_term = -CHASSIS_YAW_KD * yaw_rate_dps;
        float i_term;

        if ((err * s_yaw_i) < 0.0f) { s_yaw_i = 0.0f; }
        if (fabsf(err) <= YAW_DEADZONE_DEG) {
            float err_ratio = fabsf(err) / YAW_DEADZONE_DEG;
            float wz_ratio  = fabsf(yaw_rate_dps) / 5.0f;
            float scale = (err_ratio > wz_ratio) ? err_ratio : wz_ratio;
            if (scale > 1.0f) { scale = 1.0f; }
            scale = scale * scale;
            s_yaw_i *= 0.20f;
            p_term  *= scale;
            d_term  *= scale;
        } else if (fabsf(err) <= CHASSIS_YAW_I_BAND_DEG) {
            s_yaw_i += err * CHASSIS_TASK_DT_20MS_S;
            s_yaw_i  = chassis_clamp_f(s_yaw_i, -YAW_I_LIMIT, YAW_I_LIMIT);
        }
        i_term = YAW_KI * s_yaw_i;
        wz = p_term + i_term + d_term;
        if ((fabsf(err) <= YAW_DEADZONE_DEG) && (fabsf(wz) < 1.0f)) { wz = 0.0f; }
        wz = chassis_clamp_f(wz, -g_chassis_tune_params.max_yaw_speed_dps,
                                  g_chassis_tune_params.max_yaw_speed_dps);
    }
#endif

    return wz;
}
#endif /* (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY) */

/** 模式切换辅助: 清积分 + 设模式 */
static void enter_mode(ctrl_mode_t m)
{
    s_yaw_i           = 0.0f;
    s_yaw_in_position = 0;   /* 模式切换强制解锁, 防新目标被旧锁挡住 */
    s_nav_lock_yaw    = 0U;  /* 默认关闭锁航, move_to_m 主动调用才打开 */
    s_v_along_lpf     = 0.0f;  /* 切换目标时清零 LPF, 避免旧速度残值污染新路径 D 项 */
    s_v_cross_lpf     = 0.0f;
    s_mode            = m;
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

#if (CHASSIS_ODOM_YAW_FUSION_ENABLE != 0)
    /* 重置 odom yaw, 下一拍会自动 sync 到 IMU yaw */
    s_yaw_odom_deg = 0.0f;
    s_yaw_odom_inited = 0U;
#endif

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

    /* 2b) 编码器 yaw 弱观测 (P0-改进 2026-05-02):
     *   麦轮 O 型 yaw 逆运动学: wz_odom = (-vLF + vRF - vLB + vRB) / (4*K)  [rad/s]
     *   积分得 yaw_odom_deg, 喂给 IMU KF 做长期对齐. R 大 -> 短期不被打滑污染.
     *   首次进来 sync 到当前 IMU yaw, 之后两条独立积分, KF 慢慢把 IMU 拉过来. */
#if (CHASSIS_ODOM_YAW_FUSION_ENABLE != 0)
    {
        float wz_odom_rad = (-ws_pid[CHASSIS_WHEEL_LF] + ws_pid[CHASSIS_WHEEL_RF]
                             - ws_pid[CHASSIS_WHEEL_LB] + ws_pid[CHASSIS_WHEEL_RB])
                            / (4.0f * CHASSIS_MECANUM_K_M);
        float wz_odom_dps = wz_odom_rad * CHASSIS_RAD_TO_DEG_F;

        if (0U == s_yaw_odom_inited) {
            /* 启动时同步, 否则两路积分初值差会被 KF 立刻拉回去, 浪费 P 协方差 */
            s_yaw_odom_deg = chassis_imu_get_yaw_deg();
            s_yaw_odom_inited = 1U;
        } else {
            /*
             * P0-修复 2026-05-02 (抬车空转污染 KF):
             *   抬起车手转车体时, 麦轮空转 -> wz_odom 虚假大值 -> 持续注入
             *   KF 把 angle 拉到错误位置 -> err 被消 -> 车轮停 -> 永久卡死.
             *
             *   修法 (Innovation Gating, KF 工程标准):
             *     比较 IMU 直接测的 wz 与 odom 算的 wz, 差值超门限 ->
             *     编码器与陀螺严重不符 (典型: 抬车 / 卡死 / 编码器故障),
             *     本拍既不积分 odom 也不喂 KF, 把 odom 重新对齐到 IMU.
             *
             *   正常状态: 两者差值 < 10°/s, 直接喂 KF 完成长期对齐.
             */
            float wz_imu_dps = chassis_imu_get_yaw_rate_dps();
            float wz_diff    = fabsf(wz_odom_dps - wz_imu_dps);

            if (wz_diff > CHASSIS_ODOM_YAW_INNOV_GATE_DPS) {
                /* 编码器与陀螺不一致 -> 抬车 / 打滑 / 故障, 完全不信 odom */
                s_yaw_odom_deg = chassis_imu_get_yaw_deg();
            } else {
                float imu_yaw, diff;
                s_yaw_odom_deg = chassis_normalize_angle_deg(
                                    s_yaw_odom_deg + wz_odom_dps * CHASSIS_TASK_DT_20MS_S);

                /* 二级保护: 长期累积偏差也要兜底, 大跳过滤 */
                imu_yaw = chassis_imu_get_yaw_deg();
                diff = chassis_normalize_angle_deg(s_yaw_odom_deg - imu_yaw);
                if (fabsf(diff) < CHASSIS_ODOM_YAW_OUTLIER_DEG) {
                    chassis_imu_kf_correct_angle(s_yaw_odom_deg, CHASSIS_ODOM_YAW_R_DEG2);
                } else {
                    s_yaw_odom_deg = imu_yaw;
                }
            }
        }
    }
#endif

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
        float vxg, vyg, norm;
        float v_drive_min = CHASSIS_POS_MIN_DRIVE_SPEED_MPS;
        float yerr;

        /*
         * P0-修复 2026-05-02 (硬刹冲过头):
         * dist <= EPSILON 直接 force_stop, 但 ramp/电机/麦轮都有惯性, 实测
         * 会冲过 1~3cm. 这里改用"软到达": 进入 BRAKE_DIST 后线性减小最小推
         * 进速度, 进 EPSILON 时已基本停下, force_stop 只是兜底.
         */
        /*
         * Schmitt-trigger 到位判定 (双阈值抗震荡):
         *   进入保持: dist <= EPSILON (3cm)  -> 标记 s_arrived=1
         *   离开保持: dist >  HOLD_EXIT (8cm) -> 清除标志, 重启位置驱动
         *
         * 为什么需要滞后带?
         *   单阈值(只有 EPSILON=3cm): 车到位停下后, 任何 >3cm 小扰动都立即
         *   重激活位置 P, 以 ~0.12 m/s 冲回 -> 惯性过冲到另一侧 -> 反向冲 ->
         *   欠阻尼震荡. 8cm 的滞后带让小扰动在保持圈内自然衰减, 不触发驱动.
         */
        if (dist <= CHASSIS_TARGET_REACHED_EPSILON_M) {
            /* Fix1: 速度判据 (ROS Nav2 goal_checker 双判据):
             * 只在 dist<EPSILON 且车已基本停稳 (||v||<5cm/s) 时才置到位.
             * 防止车以高速穿过目标瞬间触发 s_arrived, 惯性冲出后
             * HOLD_EXIT 重启驱动 -> 反向冲 -> 来回震荡. */
            float v_norm = sqrtf(s_fb_vx * s_fb_vx + s_fb_vy * s_fb_vy);
            if (v_norm < CHASSIS_POS_ARRIVED_VEL_MPS) {
                s_arrived = 1U;
            }
        }
        if (s_arrived && (dist <= CHASSIS_POS_HOLD_EXIT_M)) {
            /* ============================================================
             * 到位保持 (ROS Nav2 goal_checker 标准方案):
             *   - 平移: 永远 0
             *   - 航向: 视调用接口决定
             *       move_to_grid (s_nav_lock_yaw=0): 不指定朝向 ->
             *         wz 硬归零, 不再纠正 yaw. 上层若想转头自己调
             *         hold_yaw 接口.
             *       move_to_m   (s_nav_lock_yaw=1): 调到 hold_yaw
             *         容忍带 (±2.5°) 内即停, 不咬残差死磕.
             *
             * 对比"让 yaw_pi 一直跑": 即使误差只有 0.5°, sqrt+前馈+PI
             * 也会输出几 °/s 的 wz, 配合轮端 breakaway/odom 噪声, 永远
             * 收敛不到 0 -> 极限环 -> 用户看到"到点后还在转".
             *
             * 关键: 立即清零平移 ramp 状态, 截断"进场惯性".
             * 根因: ramp_filter 以 accel_limit 速率缓慢从进场速度降向 0,
             * 期间仍向前驱动车 -> 冲出 HOLD_EXIT -> 位置 P 重启反向推 ->
             * ramp 再次缓降 -> 来回振荡. 保留 wz ramp 让航向平滑过渡.
             * ============================================================ */
            s_ramp.vx_body_mps = 0.0f;   /* 截断平移惯性, wz ramp 保留 */
            s_ramp.vy_body_mps = 0.0f;
            cmd.vx_body_mps = 0.0f;
            cmd.vy_body_mps = 0.0f;

            if (!s_nav_lock_yaw) {
                /* move_to_grid: 不指定朝向, 直接停 */
                cmd.wz_dps = 0.0f;
                s_yaw_in_position = 1;     /* 顺手锁住, 防 yaw_pi 残值漏出 */
                s_yaw_i = 0.0f;
            } else {
                yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
                if (fabsf(yerr) <= CHASSIS_YAW_GOAL_TOLERANCE_DEG) {
                    /* 进入 yaw 容忍带: 硬归零, 与 in-pos 锁形成滞回 */
                    cmd.wz_dps = 0.0f;
                    s_yaw_in_position = 1;
                    s_yaw_i = 0.0f;
                } else {
                    /* 仍需调向 hold_yaw, 用完整 yaw_pi (允许 in-pos 锁) */
                    cmd.wz_dps = yaw_pi(yerr, 1U, 1U);
                }
            }
            break;
        }
        /* 超出保持圈 (大扰动): 清除标志, 向下执行位置控制 */
        s_arrived = 0U;
        if ((CHASSIS_POS_BRAKE_DIST_M > 1e-6f) && (dist < CHASSIS_POS_BRAKE_DIST_M)) {
            /* 线性 ramp: dist=BRAKE -> v=V_MIN; dist=EPSILON -> v=0 */
            float ratio = (dist - CHASSIS_TARGET_REACHED_EPSILON_M)
                        / (CHASSIS_POS_BRAKE_DIST_M - CHASSIS_TARGET_REACHED_EPSILON_M);
            if (ratio < 0.0f) ratio = 0.0f;
            if (ratio > 1.0f) ratio = 1.0f;
            v_drive_min = CHASSIS_POS_MIN_DRIVE_SPEED_MPS * ratio;
        }

        /* ----------------------------------------------------------------
         * 位置 P 控制 (axis-by-axis 曼哈顿模式 + gain scheduling):
         *
         * 旧 CTE 方案: 沿程/侧偏分解, 走直线 -> 麦轮 X/Y 同时驱动,
         *   被扰动后回程方向变化大, 易引入对角震荡.
         *
         * 新方案 (CHASSIS_POS_AXIS_BY_AXIS_ENABLE=1):
         *   1) 取 |dx|,|dy| 较大者为"主轴", 只驱动该轴, 另一轴 = 0
         *   2) 主轴误差 < AXIS_SWITCH_TOL (4cm) 时切到次轴, 一格一格走
         *   3) 麦轮全局 X/Y 单轴运动 -> 对角耦合最小, 扰动恢复也只动一个轴
         *   4) 近场 dist < RECOVERY_DIST 时 KP 折半 (gain scheduling),
         *      避免高速冲回目标 -> 过冲 -> 反向冲 = 震荡
         *
         * D 项始终用全局速度 (vxg_meas, vyg_meas) 直接做 PD, 与 axis 独立. */
        {
            float vxg_meas      = cy * s_fb_vx - sy * s_fb_vy;  /* 全局速度 X */
            float vyg_meas      = sy * s_fb_vx + cy * s_fb_vy;  /* 全局速度 Y */

            /* D 项低通 (Tesla/Waymo: derivative-on-measurement + IIR LPF) */
            s_v_along_lpf = (1.0f - CHASSIS_POS_D_LPF_ALPHA) * s_v_along_lpf
                          + CHASSIS_POS_D_LPF_ALPHA * vxg_meas;  /* 借用 along 槽存 vxg LPF */
            s_v_cross_lpf = (1.0f - CHASSIS_POS_D_LPF_ALPHA) * s_v_cross_lpf
                          + CHASSIS_POS_D_LPF_ALPHA * vyg_meas;  /* 借用 cross 槽存 vyg LPF */

            /* Gain scheduling: 近场降增益, 避免回程过冲 */
            float kp_eff = g_chassis_tune_params.pos_kp;
            if (dist < CHASSIS_POS_RECOVERY_DIST_M) {
                kp_eff *= CHASSIS_POS_RECOVERY_KP_SCALE;
            }

#if (CHASSIS_POS_AXIS_BY_AXIS_ENABLE != 0)
            /* 曼哈顿: 选主轴, 只动主轴 */
            if (fabsf(dx) > CHASSIS_POS_AXIS_SWITCH_TOL_M) {
                /* X 轴优先 (绝对偏差大于切换容忍带) */
                vxg = kp_eff * dx - CHASSIS_POS_KD * s_v_along_lpf;
                vyg = 0.0f - CHASSIS_POS_KD * s_v_cross_lpf;  /* Y 仅做阻尼, 不主动推 */
            } else if (fabsf(dy) > CHASSIS_POS_AXIS_SWITCH_TOL_M) {
                /* X 已到位, 切到 Y 轴 */
                vxg = 0.0f - CHASSIS_POS_KD * s_v_along_lpf;
                vyg = kp_eff * dy - CHASSIS_POS_KD * s_v_cross_lpf;
            } else {
                /* 两轴都在容忍带内: 只做阻尼, 不再驱动 */
                vxg = -CHASSIS_POS_KD * s_v_along_lpf;
                vyg = -CHASSIS_POS_KD * s_v_cross_lpf;
            }
#else
            /* 旧方案保留: 径向 P, 修正力始终指向目标 */
            vxg = kp_eff * dx - CHASSIS_POS_KD * s_v_along_lpf;
            vyg = kp_eff * dy - CHASSIS_POS_KD * s_v_cross_lpf;
#endif
        }
        norm = sqrtf(vxg * vxg + vyg * vyg);

        /* P0-改进 2026-05-05 (PD 兼容的 MIN_DRIVE):
         * 旧: norm < V_MIN 直接按比例放大 -> PD 反向刹车时被反向放大成
         *     ±V_MIN 冲击, 引发 0.12 m/s 来回震荡.
         * 新: 只在 PD 输出方向"指向目标"时才补足最小推进, 否则让 PD
         *     自由刹车. 用沿程方向投影 v_along_cmd 判断:
         *       v_along_cmd > 0 (向目标) 且 norm < V_MIN -> 补到 V_MIN
         *       v_along_cmd <= 0 (PD 想刹车/倒车)        -> 不动
         * 这样末段 PD 自然平滑收敛, 不再被 MIN_DRIVE 反向冲击. */
        {
            /* 不重算: 用与 PD 同步的 cos_phi/sin_phi 投影需要重做, 这里
             * 用径向投影 (vxg*dx+vyg*dy)/dist 也等价判方向. */
            float v_align = (vxg * dx + vyg * dy);  /* 与 d_along 同号 -> 指向目标 */
            if ((v_drive_min > 0.0f) && (norm < v_drive_min) && (v_align > 0.0f) && (dist > 1e-6f)) {
                float sc = v_drive_min / (norm > 1e-6f ? norm : 1.0f);
                vxg *= sc;
                vyg *= sc;
                norm = v_drive_min;
            }
        }
        if (norm > g_chassis_tune_params.max_linear_speed_mps && norm > 1e-6f) {
            float sc = g_chassis_tune_params.max_linear_speed_mps / norm;
            vxg *= sc;
            vyg *= sc;
        }

        /*
         * P0-修复 2026-05-02 (atan2 噪声风暴 + 裸 P yaw 控):
         *   旧: tgt_yaw = atan2(dy,dx); wz = clamp(KP*err, ...)
         *   两个问题:
         *     1. dist 近 epsilon 时 atan2 对 odom 噪声 (1cm) 极敏感, 会跳 ±90°
         *        -> wz 饱和摆头 -> 抖
         *     2. 裸 P 绕过 yaw_pi() 全部改进 (sqrt_ctrl / in-position lock /
         *        平动模式禁内环), 两套 yaw 控同时存在容易潜伏 bug
         *
         *   新策略: 距离门控 + 复用 yaw_pi(平动模式)
         *     - 远场 (dist > YAW_TRACK_DIST): 把目标 yaw 实时更新为 atan2(dy,dx)
         *     - 近场 (dist <= YAW_TRACK_DIST): 冻结 s_tgt_yaw_deg, 不再更新
         *     - 一律走 yaw_pi(yerr, 0U) 平动模式, 共享所有保护逻辑
         *   带来的副作用: 远场到近场切换时 s_tgt_yaw_deg 跳 -> yaw_pi 内
         *   sqrt_ctrl 自动平滑收敛, 无需特殊处理.
         */
        /* s_nav_lock_yaw=1 时由调用者锁定航向, 禁止 atan2 覆盖 (move_to_m 接口) */
        if (!s_nav_lock_yaw && dist > CHASSIS_POS_YAW_TRACK_DIST_M) {
            s_tgt_yaw_deg = atan2f(dy, dx) * CHASSIS_RAD_TO_DEG_F;
        }
        yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);

        /* 全局 → 车体坐标变换 */
        cmd.vx_body_mps =  cy * vxg + sy * vyg;
        cmd.vy_body_mps = -sy * vxg + cy * vyg;
        cmd.wz_dps      = yaw_pi(yerr, 1U, 0U);  /* 完整级联PI + 禁in-pos锁(平动中不锁) */
        break;
    }

    case MODE_MOVE_YAW: {
        float yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
        cmd.vx_body_mps = s_cmd_vx;
        cmd.vy_body_mps = s_cmd_vy;
        cmd.wz_dps      = yaw_pi(yerr, 1U, 0U);  /* 完整级联PI + 禁in-pos锁(平动中不锁) */
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
        /*
         * P0-修复 2026-05-02 (姿态环"一段一段"潜伏 bug):
         * 旧代码这里传 compensate=1U, 启用 yaw_pi() 内的 YAW_MIN_WZ 阶跃补偿.
         * 当前 CHASSIS_YAW_MIN_WZ_DPS=0 使其暂时无效, 但只要有人把它调大,
         * wz 就会在零附近被一把抬到 ±YAW_MIN_WZ -> 麦轮目标阶跃 -> PWM 阶跃
         * -> 车体段段抽搐. 新方案约定: 静摩擦统一由轮端 breakaway 前馈处理,
         * yaw 输出全程保持连续, 不再做阶跃补偿. 与 MODE_MOVE_YAW 一致传 0U.
         */
        cmd.wz_dps      = yaw_pi(yerr, 1U, 1U);  /* 静止保持: 允许 in-pos 锁消除极限环 */
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

    enter_mode(MODE_POINT_NAV);  /* enter_mode 内 s_nav_lock_yaw 被重置为 0, atan2 跟踪生效 */
    s_arrived = 0U;
}

void chassis_ctrl_move_to_m(float x_m, float y_m, float hold_yaw_deg)
{
    float dx_path = x_m - s_pose.x_m;
    float dy_path = y_m - s_pose.y_m;
    float path_len = sqrtf(dx_path * dx_path + dy_path * dy_path);

    s_tgt_x_m     = x_m;
    s_tgt_y_m     = y_m;
    s_tgt_yaw_deg = chassis_normalize_angle_deg(hold_yaw_deg);

    /* 记录直线路径方向 (起点→终点 atan2), 与机器人头部朝向无关.
     * CTE 必须用这个方向做分解, 用 hold_yaw 会把麦轮平移的横向误当成侧偏纳入 CTE, 全算错. */
    if (path_len > 0.05f) {
        s_path_cos_phi = dx_path / path_len;
        s_path_sin_phi = dy_path / path_len;
    } else {
        /* 距离过近 (退化): 用目标航向角作为路径方向兑底 */
        float hr = s_tgt_yaw_deg * CHASSIS_DEG_TO_RAD_F;
        s_path_cos_phi = cosf(hr);
        s_path_sin_phi = sinf(hr);
    }

    enter_mode(MODE_POINT_NAV);  /* 先 enter_mode 重置标志, 再打开锁定, 顺序不能反 */
    s_nav_lock_yaw = 1U;         /* 锁住姿态, 任务层不再用 atan2 覆盖 */
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
     * VOFA+ FireWater 协议格式: 纯 ASCII, 通道用 ',' 分隔, 行尾 '\n', 无前缀。
     * 通道顺序 (共 12 路, 在 VOFA+ 里按这个顺序绑变量名即可):
     *   ch01 = target_yaw_deg          目标航向 (deg)
     *   ch02 = current_yaw_deg         当前航向 (deg)
     *   ch03 = yaw_err_deg             航向误差 (deg)
     *   ch04 = wz_cmd_dps              角速度指令 (dps)
     *   ch05 = pwm_LF (前进符号域, 未乘 dir_sign)
     *   ch06 = pwm_RF
     *   ch07 = pwm_LB
     *   ch08 = pwm_RB
     *   ch09 = fb_LF (m/s)
     *   ch10 = fb_RF
     *   ch11 = fb_LB
     *   ch12 = fb_RB
     */
    printf("%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\n",
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
#if (CHASSIS_ODOM_YAW_FUSION_ENABLE != 0)
    /* 重定位时也把 odom yaw 同步到新基准, 否则下一拍 KF 会被旧 odom 拉回去 */
    s_yaw_odom_deg = yaw_norm;
    s_yaw_odom_inited = 1U;
#endif
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