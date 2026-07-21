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
/*
 *  @owner  rt1064-main
 *  @periph none                  聚合层，通过 chassis_motor/encoder/imu/pid 间接使用外设
 */
#include "chassis_ctrl.h"
#include "chassis_imu.h"
#include "chassis_encoder.h"
#include "chassis_motor.h"
#include "chassis_pid.h"
#include "chassis_mecanum.h"
#include "chassis_mpc.h"     /* P0-MPC: 直线行驶 MPC 驱动轴纵向速度规划                   */
#include "chassis_arrival.h"
#include "chassis_velocity_handoff.h"
#include "app_link.h"        /* P0-3: 软限位改用 app_link_get_map_snapshot() 拿一致地图副本   */
#include "zf_common_headfile.h"  /* P0-3: __DMB() / __disable_irq() 内存屏障与临界区          */
#include <math.h>

/* 软限位依赖：P0-3 解耦 g_game_map 直访, 改为运行时 seq-lock 快照 (apply_soft_limit_guard 内部拷贝) */

/* 航向闭环参数集中在 chassis_config.h 顶部“用户常调参数区”。 */

/* 单轮 PID 调试起步补偿参数（用于克服静摩擦） */
#define WHEEL_DEBUG_START_SPEED_EPS_MPS   (0.03f)   /* 低于此反馈速度视为静止 */
#define WHEEL_DEBUG_START_TARGET_EPS_MPS  (0.05f)   /* 低于此目标速度不启用补偿 */
#define WHEEL_DEBUG_START_PWM_MIN         (800.0f)  /* 起步最小 PWM 幅值 */
#define WHEEL_DEBUG_TARGET_RAMP_MPS_PER_TICK (0.8f) /* 20ms 每拍目标最多变化量 */

/* 轮速闭环抗抖参数（抑制低速量化噪声和来回翻向） */
#define WHEEL_FB_LPF_ALPHA                (0.35f)   /* 轮速反馈一阶低通系数，越小越平滑 */

/* Yaw 串级 P-PI 内环: 前馈 + PI 直接输出 PWM, 自动克服静摩擦.
 * 外环(yaw_pi): angle_err → wz_cmd (纯 P)
 * 内环(本段):   wz_cmd - wz_actual → PWM (PI), 取代旧 GAIN×wz 开环 */
#define CHASSIS_YAW_PWM_GAIN              (80.0f)    /* 已废弃, 由内环 PI 替代 */

/* 保持轴直接 PWM 前馈: 绕过轮速 PID, 把车体速度 (vx,vy) 直接换算成 PWM.
 *   问题同 yaw: 保持轴输出 0.06m/s → 轮速 PID 只给 40×0.06=2.4PWM → 无力纠偏.
 *   新链路: 保持轴 → vx/vy(m/s) → 直接 PWM = GAIN × 麦轮分配系数 × 速度
 *   O 型麦轮: vy 四轮同号, vx 对角同号 (LF=-, RF=+, LB=+, RB=-).
 *   GAIN=1500 时, 保持 0.06m/s→90PWM/轮, 足以对抗 odom 漂移和耦合扰动. */
#define CHASSIS_HOLD_PWM_GAIN             (40.0f)
/* 仅当四轮目标和反馈同时接近 0 才停轮；0.003m/s 不会吞掉正常的小角度修正。 */
#define WHEEL_STOP_TARGET_EPS_MPS         (0.003f)
#define WHEEL_STOP_FEEDBACK_EPS_MPS       (0.030f)

/* 软限位保护参数 / 软限位 helpers / apply_soft_limit_guard()
 * 已于 2026-05-13 迁至 chassis_zone.c (chassis_zone_apply_soft_limit_guard).
 * 主任务调用见下方 task_20ms 末尾. */

/* ---------------------- 控制模式 ---------------------- */

typedef enum {
    MODE_STOPPED = 0,       /* PWM 持续关闭，不做位置或航向闭环           */
    MODE_YAW_HOLD,          /* 原地航向保持 / rotate_to_deg 复用          */
    MODE_POINT_NAV,         /* 网格点位导航                               */
    MODE_SINGLE_WHEEL_PID_DEBUG /* 单轮 PID 调试（仅一个轮子给目标）        */
} ctrl_mode_t;

/* ====================== 硬件实例 ====================== */

static chassis_motor_t s_mot[CHASSIS_WHEEL_COUNT] = {
    { MOTOR_LF_PWM_CHANNEL, MOTOR_LF_DIR_PIN, MOTOR_LF_OUTPUT_DIR },  /* LF */
    { MOTOR_RF_PWM_CHANNEL, MOTOR_RF_DIR_PIN, MOTOR_RF_OUTPUT_DIR },  /* RF */
    { MOTOR_LB_PWM_CHANNEL, MOTOR_LB_DIR_PIN, MOTOR_LB_OUTPUT_DIR },  /* LB */
    { MOTOR_RB_PWM_CHANNEL, MOTOR_RB_DIR_PIN, MOTOR_RB_OUTPUT_DIR },  /* RB */
};

static chassis_encoder_t s_enc[CHASSIS_WHEEL_COUNT] = {
    { ENCODER_LF_INDEX, ENCODER_LF_CH1, ENCODER_LF_CH2, ENCODER_LF_DIR, 0.0f },
    { ENCODER_RF_INDEX, ENCODER_RF_CH1, ENCODER_RF_CH2, ENCODER_RF_DIR, 0.0f },
    { ENCODER_LB_INDEX, ENCODER_LB_CH1, ENCODER_LB_CH2, ENCODER_LB_DIR, 0.0f },
    { ENCODER_RB_INDEX, ENCODER_RB_CH1, ENCODER_RB_CH2, ENCODER_RB_DIR, 0.0f },
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
    CHASSIS_POS_KP, CHASSIS_POS_Y_KP, CHASSIS_YAW_KP,
    CHASSIS_MAX_LINEAR_SPEED_MPS, CHASSIS_MAX_YAW_SPEED_DPS,
    CHASSIS_CMD_ACCEL_LIMIT_MPS2, CHASSIS_CMD_ACCEL_LIMIT_Y_MPS2,
    CHASSIS_CMD_ACCEL_LIMIT_DPS2,

    /* 静摩擦前馈 — 每轮独立默认值 */
    {
        CHASSIS_WHEEL_BREAKAWAY_LF_TARGET_EPS_MPS,
        CHASSIS_WHEEL_BREAKAWAY_RF_TARGET_EPS_MPS,
        CHASSIS_WHEEL_BREAKAWAY_LB_TARGET_EPS_MPS,
        CHASSIS_WHEEL_BREAKAWAY_RB_TARGET_EPS_MPS,
    },
    {
        CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR,
        CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR,
        CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR,
        CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR,
    },
    {
        CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR_XP,
        CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR_XP,
        CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR_XP,
        CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR_XP,
    },
    {
        CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR_XN,
        CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR_XN,
        CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR_XN,
        CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR_XN,
    },
    {
        CHASSIS_WHEEL_BREAKAWAY_LF_FB_STATIC_EPS_MPS,
        CHASSIS_WHEEL_BREAKAWAY_RF_FB_STATIC_EPS_MPS,
        CHASSIS_WHEEL_BREAKAWAY_LB_FB_STATIC_EPS_MPS,
        CHASSIS_WHEEL_BREAKAWAY_RB_FB_STATIC_EPS_MPS,
    },
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

static volatile ctrl_mode_t s_mode    = MODE_STOPPED;
static volatile uint8       s_arrived = 1U;
/* 1 = 正在执行 rotate_to_deg 原地旋转, 到达角度容忍带后置 s_arrived=1 并清零 */
static volatile uint8       s_rotate_active = 0U;
volatile uint32 g_chassis_arrival_count = 0U;  /* 到达计数, 不被清零, 串口可观察 */

/* 导航目标 */
static volatile float s_tgt_x_m       = 0.0f;
static volatile float s_tgt_y_m       = 0.0f;
static volatile float s_tgt_yaw_deg   = 0.0f;
/* 1 = chassis_ctrl_move_to_m() 主动锁定航向, 禁止任务层用 atan2 覆盖.
 * 曼哈顿轴模式下 move_to_grid() 也会保持起步航向, 不边走轴边转头. */
static volatile uint8 s_nav_lock_yaw  = 0U;

/* D 项低通状态 (一阶 IIR, 消除 odom 高频噪声对 KD 的放大) */
static float s_v_along_lpf = 0.0f;
static float s_v_cross_lpf = 0.0f;
/* 位置环沿程方向积分累积量 (m·s, 离散: 每帧 ×DT) */
static float s_pos_i       = 0.0f;
/* 保持轴独立积分 (P0-修复 2026-06-06): 驱动轴积分不帮保持轴纠偏, 保持轴需独立 I 消除稳态误差 */
static float s_pos_i_hold  = 0.0f;
/* yaw 环积分累积量 (°·s): 消除静摩擦稳态残差，仅在误差带内累积 */
static float s_yaw_i       = 0.0f;
/* 串级 P-PI 内环积分 (PWM 域): ∫(wz_cmd - wz_actual) → 自动克服静摩擦 */
static float s_yaw_rate_i  = 0.0f;
/* ADRC: z1=估计角速度(°/s), z2=估计总扰动(°/s²), u=上拍PWM输出 */
static float s_yaw_adrc_z1 = 0.0f;
static float s_yaw_adrc_z2 = 0.0f;
static float s_yaw_adrc_u  = 0.0f;
/* 到位驻留计数器: 连续 N 拍满足到位条件 → 判到达, 防止单拍噪声误触 */
static uint16 s_arrival_dwell_cnt = 0U;
/* 逐轴到位: X/Y 各自判断 |err|≤EPSILON, 两轴都到位→整体 arrived */
static uint8 s_axis_x_arrived = 0U;
static uint8 s_axis_y_arrived = 0U;
/* Schmitt 触发器: 1 = X 轴已到位, 当前锁定 Y 轴优先; 0 = X 未完成 */
static uint8 s_axis_y_locked = 0U;
/* POINT_NAV 起步 yaw 对齐标志: 进入模式时清零, 首次 |yerr|≤INPOS 后置1.
 * 门控仅在未对齐时阻断平移, 对齐后行进中靠 yaw_pi 温柔修正. */
static uint8_t s_nav_yaw_aligned = 1U;
/* axis-by-axis 非驱动轴保持坐标: 保持当前直线, 不是提前追最终目标造成斜线 */
static float s_axis_hold_x_m = 0.0f;
static float s_axis_hold_y_m = 0.0f;

/* 单轮 PID 调试参数 */
static volatile uint8 s_debug_wheel_index = (uint8)CHASSIS_WHEEL_LF;
static volatile float s_debug_wheel_target_mps = 0.0f;
static volatile float s_debug_fb_sign_mul[CHASSIS_WHEEL_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
static volatile float  s_debug_target_ramp_mps      = 0.0f;

/* In-Position Schmitt 锁状态: 1 = 已在位 (输出硬归零)
 * 由 yaw_pi() 内部按双阈值滞回切换, 模式切换时强制清 0. */
static volatile uint8 s_yaw_in_position = 0;

/* 里程计反馈 */
static volatile float s_fb_vx  = 0.0f;
static volatile float s_fb_vy  = 0.0f;
static volatile float s_wheel_fb_lpf[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* P0-诊断: 轮速目标快照, 供串口打印对比期望 vs 实际 */
volatile float g_chassis_diag_wheel_tgt[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float g_chassis_diag_body_spd_tgt_vx = 0.0f;
volatile float g_chassis_diag_body_spd_tgt_vy = 0.0f;
volatile float g_chassis_diag_body_spd_fb_vx  = 0.0f;
volatile float g_chassis_diag_body_spd_fb_vy  = 0.0f;

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

static float position_axis_velocity_cmd(float axis_error,
                                        float axis_velocity_lpf,
                                        float position_gain,
                                        float damping_gain,
                                        float accel_limit,
                                        float integral_term);

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

        /* 静摩擦前馈 — 每轮独立限幅 */
        p.wheel_breakaway_target_eps_mps[i]   = chassis_clamp_f(p.wheel_breakaway_target_eps_mps[i],   0.001f, 0.50f);
        p.wheel_breakaway_pwm_floor[i]        = chassis_clamp_f(p.wheel_breakaway_pwm_floor[i],        0.0f,   4000.0f);
        p.wheel_breakaway_pwm_floor_xp[i]    = chassis_clamp_f(p.wheel_breakaway_pwm_floor_xp[i],    0.0f,   4000.0f);
        p.wheel_breakaway_pwm_floor_xn[i]    = chassis_clamp_f(p.wheel_breakaway_pwm_floor_xn[i],    0.0f,   4000.0f);
        p.wheel_breakaway_fb_static_eps_mps[i] = chassis_clamp_f(p.wheel_breakaway_fb_static_eps_mps[i], 0.01f,  5.00f);
    }

    p.pos_kp   = chassis_clamp_f(p.pos_kp,   0.0f, CHASSIS_TUNE_POS_KP_LIMIT);
    p.pos_kp_y = chassis_clamp_f(p.pos_kp_y, 0.0f, CHASSIS_TUNE_POS_KP_LIMIT);
    p.yaw_kp               = chassis_clamp_f(p.yaw_kp,               0.0f,   10.0f);
    p.max_linear_speed_mps = chassis_clamp_f(p.max_linear_speed_mps, 0.05f,
                                             CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS);
    p.max_yaw_speed_dps   = chassis_clamp_f(p.max_yaw_speed_dps,   10.0f,
                                             CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS);
    p.cmd_accel_limit_mps2 = chassis_clamp_f(p.cmd_accel_limit_mps2, 0.10f, 5.00f);
    p.cmd_accel_limit_y_mps2 = chassis_clamp_f(p.cmd_accel_limit_y_mps2, 0.10f, 6.00f);
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
static chassis_body_speed_cmd_t ramp_filter(chassis_body_speed_cmd_t tgt,
                                             float accel_limit_mps2)
{
    const float dv = accel_limit_mps2 * CHASSIS_TASK_DT_20MS_S;
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

static float clamp_axis_hold_speed(float velocity_cmd)
{
    /* 非驱动轴保持速度上限 = max_linear_speed × 比例, 随全局限速自动跟随. */
    float hold_max = g_chassis_tune_params.max_linear_speed_mps
                   * CHASSIS_POS_HOLD_SPEED_RATIO;
    return chassis_clamp_f(velocity_cmd, -hold_max, hold_max);
}

static float axis_hold_velocity_cmd(float hold_error,
                                    float axis_velocity_lpf,
                                    float position_gain,
                                    float damping_gain,
                                    float accel_limit,
                                    float integral_term)
{
    /* 保持轴 1cm 死区: 滤除编码器量化噪声和 odom 短时抖动,
     * 防止 sqrt_controller 把小误差放大成 0.06~0.12 m/s 的修正脉冲.
     * 死区内修正力为 0, 让微小偏差自然衰减, 不触发无谓的来回修正.
     * 1cm 远小于 EPSILON=8cm, 不会造成保持轴长期漂移累积. */
    if (fabsf(hold_error) <= CHASSIS_POS_HOLD_DEAD_ZONE_M) {
        return 0.0f;
    }
    return clamp_axis_hold_speed(
            position_axis_velocity_cmd(hold_error,
                                       axis_velocity_lpf,
                                       position_gain,
                                       damping_gain,
                                       accel_limit,
                                       integral_term));
}

/** 紧急停机: 清零滤波器、PID、PWM */

static void force_stop(void)
{
    uint8 i;

    s_ramp     = (chassis_body_speed_cmd_t){0};
    s_last_cmd = (chassis_body_speed_cmd_t){0};
    s_pos_i    = 0.0f;
    s_pos_i_hold = 0.0f;
    s_yaw_i      = 0.0f;
    s_yaw_rate_i = 0.0f;
    s_yaw_adrc_z1 = 0.0f;
    s_yaw_adrc_z2 = 0.0f;
    s_yaw_adrc_u  = 0.0f;
    s_v_along_lpf = 0.0f;
    s_v_cross_lpf = 0.0f;

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        stop_wheel_with_pid_reset(i);
        s_wheel_fb_lpf[i] = 0.0f;
    }
    s_wheel_fb_lpf_inited = 0U;
}

/* 软限位 helpers + apply_soft_limit_guard() 已迁至 chassis_zone.c
 * (chassis_zone_apply_soft_limit_guard). 调用点见 task_20ms 末尾. */

/** 完整执行链路: 滤波 → 运动学 → PID → 电机 */
static void apply_speed(chassis_body_speed_cmd_t cmd,
                        const float wheel_fb_mps[CHASSIS_WHEEL_COUNT],
                        float accel_limit_mps2)
{
    chassis_body_speed_cmd_t f;
    float targets[CHASSIS_WHEEL_COUNT];
    uint8 i;

    f = ramp_filter(cmd, accel_limit_mps2);
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

    /* 诊断快照: 保存期望轮速和车体速度供串口打印 */
    {
        uint8 di;
        for (di = 0U; di < (uint8)CHASSIS_WHEEL_COUNT; ++di) {
            g_chassis_diag_wheel_tgt[di] = targets[di];
        }
        g_chassis_diag_body_spd_tgt_vx = f.vx_body_mps;
        g_chassis_diag_body_spd_tgt_vy = f.vy_body_mps;
        g_chassis_diag_body_spd_fb_vx  = s_fb_vx;
        g_chassis_diag_body_spd_fb_vy  = s_fb_vy;
    }

    /* 零目标时先由 PID 主动制动；四轮都低速后再清状态，防残留 PWM 正反抽动。 */
    if (chassis_wheels_idle_stop_ready(targets, wheel_fb_mps,
                                       (uint8)CHASSIS_WHEEL_COUNT,
                                       WHEEL_STOP_TARGET_EPS_MPS,
                                       WHEEL_STOP_FEEDBACK_EPS_MPS)) {
        for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
            stop_wheel_with_pid_reset(i);
        }
        return;
    }

    /* 方向检测: X 主导时按正负选 floor, Y 主导时沿用原 floor (不区分正负). */
    {
        float abs_vx = fabsf(f.vx_body_mps);
        float abs_vy = fabsf(f.vy_body_mps);
        const volatile float *pwm_floor_selected;
        if (abs_vx > abs_vy) {
            pwm_floor_selected = (f.vx_body_mps >= 0.0f)
                               ? g_chassis_tune_params.wheel_breakaway_pwm_floor_xp
                               : g_chassis_tune_params.wheel_breakaway_pwm_floor_xn;
        } else {
            pwm_floor_selected = g_chassis_tune_params.wheel_breakaway_pwm_floor;
        }

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
            float pwm_forward_domain;
            float pwm_motor_domain;
            const float abs_target = fabsf(targets[i]);
            const float abs_fb     = fabsf(wheel_fb_mps[i]);

            pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], wheel_fb_mps[i]);

            /* 静摩擦前馈: 线性衰减 + 方向门控 */
            if (abs_target > g_chassis_tune_params.wheel_breakaway_target_eps_mps[i]) {
                float fb_eps = g_chassis_tune_params.wheel_breakaway_fb_static_eps_mps[i];
                uint8 same_dir = ((targets[i] >= 0.0f) == (wheel_fb_mps[i] >= 0.0f));
                uint8 is_stopped = (abs_fb < fb_eps * 0.3f);
                if (same_dir || is_stopped) {
                    float ff_ratio;
                    if (fb_eps < 1e-6f) {
                        ff_ratio = 0.0f;
                    } else {
                        ff_ratio = 1.0f - (abs_fb / fb_eps);
                        if (ff_ratio < 0.0f) ff_ratio = 0.0f;
                        if (ff_ratio > 1.0f) ff_ratio = 1.0f;
                    }
                    {
                        float floor_pwm = pwm_floor_selected[i];
                        float ff_pwm = (targets[i] >= 0.0f) ? floor_pwm : -floor_pwm;
                        pwm_forward_domain += ff_pwm * ff_ratio;
                    }
                }
            }

            /* 保持轴直接 PWM 前馈: 绕过轮速 PID, 给保持轴纠偏力度.
             * O 型麦轮: vy 四轮同号(+1), vx 对角异号(LF=-,RF=+,LB=+,RB=-) */
            {
                static const float s_vx_signs[CHASSIS_WHEEL_COUNT] =
                    { -1.0f, +1.0f, +1.0f, -1.0f };
                float hold_pwm = CHASSIS_HOLD_PWM_GAIN
                               * (f.vy_body_mps + s_vx_signs[i] * f.vx_body_mps);
                pwm_forward_domain += hold_pwm;
            }

            /* Yaw 直接 PWM 前馈: O 型 LF=-,RF=+,LB=-,RB=+ */
            {
                static const float s_yaw_signs[CHASSIS_WHEEL_COUNT] =
                    { -1.0f, +1.0f, -1.0f, +1.0f };
                float yaw_pwm = s_yaw_signs[i] * CHASSIS_YAW_PWM_GAIN
                              * CHASSIS_MECANUM_K_M
                              * f.wz_dps * CHASSIS_DEG_TO_RAD_F;
                pwm_forward_domain += yaw_pwm;
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
}

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

    /* 调试模式不输出车体运动指令。 */
    s_last_cmd = (chassis_body_speed_cmd_t){0};
    s_ramp     = (chassis_body_speed_cmd_t){0};

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

            /* 单轮调试同样在“前进符号域”做闭环，打印值和控制值保持一致。 */
            pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], feedback_for_pid);
            pwm_motor_domain = pwm_forward_domain * s_mot[i].dir_sign;

            /* 起步抗静摩擦: 目标非零但轮速接近零时, 给最小启动 PWM,
             * 同时把 PID 累加器钳到等价值实现无扰切换 (bumpless transfer). */
            if ((fabsf(targets[i]) > WHEEL_DEBUG_START_TARGET_EPS_MPS) &&
                (fabsf(feedback_for_pid) < WHEEL_DEBUG_START_SPEED_EPS_MPS) &&
                (fabsf(pwm_motor_domain) < WHEEL_DEBUG_START_PWM_MIN))
            {
                float target_sign = (targets[i] >= 0.0f) ? 1.0f : -1.0f;
                pwm_motor_domain = WHEEL_DEBUG_START_PWM_MIN * target_sign * s_mot[i].dir_sign;
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
 * 位置环减速区半径 (m) 自动推导 (替代旧 CHASSIS_POS_BRAKE_DIST_M 常量).
 *
 * 物理约束: 从 max_linear_speed 以 accel_limit 减到 0 的滑行距离
 *   d_stop = v_max² / (2·accel_limit)
 * brake_dist 必须覆盖 d_stop + EPSILON 才能在到位前刹停, 再留 BRAKE_MARGIN 裕量.
 * 用运行时可调的 max_linear_speed / cmd_accel_limit 计算, 调速度/加速度时自动跟随. */
static float pos_brake_dist_auto(float accel_limit_mps2)
{
    float v_max = g_chassis_tune_params.max_linear_speed_mps;
    float accel = accel_limit_mps2;
    float d_stop = (accel > 1e-6f) ? (v_max * v_max / (2.0f * accel)) : 0.0f;
    return d_stop + CHASSIS_TARGET_REACHED_EPSILON_M + CHASSIS_POS_BRAKE_MARGIN_M;
}

static float position_axis_velocity_cmd(float axis_error,
                                        float axis_velocity_lpf,
                                        float position_gain,
                                        float damping_gain,
                                        float accel_limit,
                                        float integral_term)
{
    float position_cmd = sqrt_controller(axis_error, position_gain, accel_limit)
                       + integral_term;
    float velocity_cmd = position_cmd - damping_gain * axis_velocity_lpf;

    /* 反卷绕 (anti-windup): 用 position_cmd (P+I) 判断, 不用 velocity_cmd (P+I-D).
     * D 项仅做阻尼, 车速快时 -kd*vel 会让 velocity_cmd 变号, 但这不表示"控制反了",
     * 此时清零 velocity_cmd 会把积分效果也一起干掉 → KI 调了跟没调一样.
     * position_cmd 与 error 始终同号 (sqrt_controller 保证), 反卷绕仅在超出
     * brake_dist 时做 safety net 缩放, 近端不再触发. */
    if ((fabsf(axis_error) > pos_brake_dist_auto(accel_limit)) &&
        (((axis_error > 0.0f) && (position_cmd < 0.0f)) ||
         ((axis_error < 0.0f) && (position_cmd > 0.0f))))
    {
        velocity_cmd = position_cmd * 0.35f;
    }

    return velocity_cmd;
}

/**
 * 驱动轴速度指令 — MPC 包装器
 *
 * 优先尝试 MPC (直线行驶纵向前瞻规划);
 * MPC 未启用 / 距离超阈值 / 求解异常 → 回退 sqrt_controller。
 *
 * MPC 内部自行管理前瞻, 不需要外部积分累积 (ACCUMULATE_POS_I 在 MPC 路径下跳过).
 */
static float driving_axis_velocity_cmd(float axis_error,
                                       float axis_velocity_lpf,
                                       float position_gain,
                                       float damping_gain,
                                       float accel_limit,
                                       float integral_term,
                                       uint8 *out_mpc_active)
{
#if CHASSIS_MPC_ENABLE
    float mpc_v;
    if (chassis_mpc_step(axis_error, &mpc_v)) {
        float abs_error = fabsf(axis_error);

        if (abs_error < CHASSIS_MPC_BLEND_START_DIST_M) {
            float pid_v = position_axis_velocity_cmd(axis_error,
                                                     axis_velocity_lpf,
                                                     position_gain,
                                                     damping_gain,
                                                     accel_limit,
                                                     integral_term);
            /* 融合区按 PID 路径积累 I 项，让接管前后的指令保持连续。 */
            *out_mpc_active = 0U;
            return chassis_velocity_handoff_blend(
                abs_error,
                CHASSIS_MPC_BLEND_START_DIST_M,
                CHASSIS_MPC_MIN_DIST_M,
                mpc_v,
                pid_v);
        }

        *out_mpc_active = 1U;
        return mpc_v;
    }
#endif
    *out_mpc_active = 0U;
    return position_axis_velocity_cmd(axis_error,
                                      axis_velocity_lpf,
                                      position_gain,
                                      damping_gain,
                                      accel_limit,
                                      integral_term);
}

/**
 * 航向闭环: sqrt_controller + 内环速率 PI (级联)
 *
 * 外环: 位置误差 → wz_target (sqrt_controller 时间最优)
 * 内环: rate_err → PI 修正 → 追加到 wz
 *       平动时 enable_rate_loop=0 关闭内环反馈, 防 IMU 振动噪声入积分
 *
 * @param err              航向误差(°)，已归一化
 * @param enable_rate_loop  1=开启内环速率 PI (静止保持)
 * @param allow_inpos_lock  1=允许 Schmitt-trigger 在位锁
 * @return                 角速度指令(°/s)
 */
static float yaw_pi(float err, uint8 enable_rate_loop, uint8 allow_inpos_lock)
{
    const float yaw_rate_dps = chassis_imu_get_yaw_rate_dps();
    float wz;

#if (CHASSIS_YAW_USE_CASCADED_CTRL != 0)
    /* ── In-Position Schmitt 锁 ── */
    if (allow_inpos_lock) {
        const float abs_err  = fabsf(err);
        const float abs_rate = fabsf(yaw_rate_dps);
        if (s_yaw_in_position) {
            if (abs_err > CHASSIS_YAW_INPOS_EXIT_DEG) { s_yaw_in_position = 0; }
        } else {
            if ((abs_err < CHASSIS_YAW_INPOS_ENTER_DEG) &&
                (abs_rate < CHASSIS_YAW_INPOS_SETTLE_DPS)) {
                s_yaw_in_position = 1;
                s_yaw_i = 0.0f;  /* 锁定时清零积分 */
            }
        }
        if (s_yaw_in_position) { return 0.0f; }
    } else {
        s_yaw_in_position = 0;
    }

    /* ── 外环: MPC 优先, sqrt_controller 兜底 → wz_target ── */
    {
        float wz_target;
        float rate_err;
        float kp_v, ki_v;
        uint8 mpc_ok = 0U;

#if CHASSIS_MPC_YAW_ENABLE
        {
            float mpc_wz;
            if (chassis_mpc_yaw_step(err, &mpc_wz)) {
                wz_target = mpc_wz;
                mpc_ok = 1U;
            }
        }
#endif
        if (!mpc_ok) {
            /* 小误差增益调度: 微小角度(≤2°)时降低外环增益, 消 PI 追猎抖动.
             *   |err|≥2° → 全增益 (KP=10)
             *   |err|≤0.5° → 30% 增益 (KP≈3), wz_target 从 5→1.5dps, 不再过冲
             *   0.5~2° → 线性过渡, 避免增益突变引发新抖动 */
            float yaw_kp_eff = g_chassis_tune_params.yaw_kp;
            {
                float abs_err = fabsf(err);
                if (abs_err < 2.0f) {
                    float ratio;
                    if (abs_err < 0.5f) {
                        ratio = 0.30f;   /* 极小误差: 30% 增益, 几乎不追 */
                    } else {
                        /* 0.5°→2.0°: ratio 从 0.30 线性过渡到 1.00 */
                        ratio = 0.30f + (abs_err - 0.5f) / (2.0f - 0.5f) * (1.0f - 0.30f);
                    }
                    yaw_kp_eff *= ratio;
                }
            }
            wz_target = sqrt_controller(err, yaw_kp_eff,
                                        CHASSIS_YAW_ACCEL_MAX_DPS2);
        }

        wz_target = chassis_clamp_f(wz_target,
                                    -g_chassis_tune_params.max_yaw_speed_dps,
                                     g_chassis_tune_params.max_yaw_speed_dps);

        /* ── 内环 PI: 静止保持始终开启, 平动时可独立关闭 ── */
#if (CHASSIS_YAW_RATE_PI_MOVE_ENABLE != 0)
        if (enable_rate_loop)
#else
        /* 移动时关闭内环 PI, 仅静止保持(allow_inpos_lock=1)时开启 */
        if (enable_rate_loop && allow_inpos_lock)
#endif
        {
            rate_err = wz_target - yaw_rate_dps;
        } else {
            rate_err = 0.0f;
        }

        /* 反向卷绕保护 */
        if ((err * s_yaw_i) < 0.0f) { s_yaw_i = 0.0f; }

        /* I 累积 + 缓慢泄漏 */
        s_yaw_i = s_yaw_i * (1.0f - CHASSIS_YAW_RATE_I_LEAK)
                + rate_err * CHASSIS_TASK_DT_20MS_S;
        s_yaw_i = chassis_clamp_f(s_yaw_i,
                                  -CHASSIS_YAW_RATE_I_LIMIT,
                                   CHASSIS_YAW_RATE_I_LIMIT);

        kp_v = CHASSIS_YAW_RATE_KP * rate_err;
        ki_v = CHASSIS_YAW_RATE_KI * s_yaw_i;

        /* PI 总输出限幅: 防止 kp_v + ki_v 过大喧宾夺主, 盖过 sqrt_ctrl 规划 */
        {
            float pi_total = kp_v + ki_v;
            pi_total = chassis_clamp_f(pi_total,
                                       -CHASSIS_YAW_RATE_PI_LIMIT_DPS,
                                        CHASSIS_YAW_RATE_PI_LIMIT_DPS);
            wz = wz_target + pi_total;
        }
    }

    wz = chassis_clamp_f(wz,
                         -g_chassis_tune_params.max_yaw_speed_dps,
                          g_chassis_tune_params.max_yaw_speed_dps);
    return wz;

#else /* 旧单环回退 */
    (void)enable_rate_loop;
    /* ... fallback PID ... */
    wz = g_chassis_tune_params.yaw_kp * err;
    wz = chassis_clamp_f(wz,
                         -g_chassis_tune_params.max_yaw_speed_dps,
                          g_chassis_tune_params.max_yaw_speed_dps);
    return wz;
#endif
}

/** 模式切换辅助: 清状态 + 设模式 */
static void enter_mode(ctrl_mode_t m)
{
    s_yaw_in_position = 0;   /* 模式切换强制解锁, 防新目标被旧锁挡住 */
    s_rotate_active   = 0U;
    s_nav_lock_yaw    = 0U;  /* 默认关闭锁航, move_to_m 主动调用才打开 */
    s_v_along_lpf     = 0.0f;  /* 切换目标时清零 LPF, 避免旧速度残值污染新路径 D 项 */
    s_v_cross_lpf     = 0.0f;
    s_pos_i           = 0.0f;  /* 切换目标时清零位置积分, 防旧路径残留量误推新起点 */
    s_pos_i_hold      = 0.0f;  /* 保持轴积分同步清零 */
    s_yaw_i           = 0.0f;  /* yaw 积分同步清零 */
    s_yaw_rate_i      = 0.0f;  /* 内环 PI 积分清零 */
    s_yaw_adrc_z1     = 0.0f;  /* ADRC 状态清零 */
    s_yaw_adrc_z2     = 0.0f;
    s_yaw_adrc_u      = 0.0f;
    s_ramp = (chassis_body_speed_cmd_t){0};  /* P0-修复 2026-06-07:
                     * 原漏清 s_ramp: 方向反转时 ramp 保留旧方向速度残值,
                     * 新目标要求反向但 ramp_filter 每拍只能变 dv=0.012,
                     * 从 +0.58→0→-0.60 需 ~100 拍 ≈ 2s, 期间车轮往反方向转,
                     * 车大幅过冲后剧烈反向 → Y 保持轴被甩出震荡. */
    s_axis_y_locked     = 0U;   /* 切换目标时复位轴锁, 重新从 X 轴开始 */
    s_axis_x_arrived    = 0U;   /* 复位逐轴到达标志 */
    s_axis_y_arrived    = 0U;
    /* P0-修复 2026-07-17: 保持轴锚点初始化为当前位置。
     * 旧代码从未设过锚点, 残留上一目标的旧值 → Y 保持环拉偏 → 短距平移抖动. */
    s_axis_hold_x_m     = s_pose.x_m;
    s_axis_hold_y_m     = s_pose.y_m;
    s_nav_yaw_aligned   = 0U;   /* 起步 yaw 门控: 每个新目标都重新对齐一次 */
    s_arrival_dwell_cnt = 0U;   /* 到位驻留计数器清零: 新目标重新累计 */
    {
        uint8 pid_i;
        for (pid_i = 0U; pid_i < (uint8)CHASSIS_WHEEL_COUNT; ++pid_i) {
            chassis_pid_reset(&s_pid[pid_i]);
        }
    }
#if CHASSIS_MPC_ENABLE
    chassis_mpc_reset();  /* P0-MPC: 切换目标时清热启动, 旧解不污染新方向 */
#endif
#if CHASSIS_MPC_YAW_ENABLE
    chassis_mpc_yaw_reset();  /* Yaw MPC: 同步清热启动 */
#endif
    s_mode              = m;
}

/* ==========================================================================
 *  § 5. 公共 API — 初始化
 * ========================================================================== */

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
    /*
     * 不在这里写死调试轮索引. 真正的"选轮"由 chassis_ctrl_start_single_wheel_pid_debug()
     * 统一负责 (它会同时设置 s_debug_wheel_index 和 chassis_pid.c 里的
     * s_pid_debug_wheel_index). 这里写死任何一个轮都会变成跟 main 里 MAIN_PID_DEBUG_WHEEL_INDEX
     * 不一致的隐藏 footgun.
     */
    s_debug_wheel_target_mps = 0.0f;
    s_fb_vx        = 0.0f;
    s_fb_vy        = 0.0f;
    s_mode         = MODE_STOPPED;
    s_arrived      = 1U;

#if (CHASSIS_ODOM_YAW_FUSION_ENABLE != 0)
    /* 重置 odom yaw, 下一拍会自动 sync 到 IMU yaw */
    s_yaw_odom_deg = 0.0f;
    s_yaw_odom_inited = 0U;
#endif

    chassis_pid_debug_reset();

#if CHASSIS_MPC_ENABLE
    chassis_mpc_init();     /* P0-MPC: 构建 H 矩阵, 预计算 FISTA 步长 */
#endif
#if CHASSIS_MPC_YAW_ENABLE
    chassis_mpc_yaw_init(); /* Yaw MPC: 构建 H 矩阵, 预计算 FISTA 步长 */
#endif

    force_stop();
}

/* ==========================================================================
 *  § 6. 周期任务 — 5ms IMU 采样 / 20ms 闭环 (PIT_IRQ 调用点)
 * ========================================================================== */

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
    float vx_raw, vy_raw;
    float yaw_rad, cy, sy;
    chassis_body_speed_cmd_t cmd = {0};
    /* POINT_NAV 按全局 X/Y 逐轴驱动；下游虽处理车体系速度，斜坡上限仍应跟随当前全局驱动轴。 */
    float linear_accel_limit_mps2 = g_chassis_tune_params.cmd_accel_limit_mps2;
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

    /* 3) 里程计积分: 车体速度旋转到全局坐标系后累加 (P0-3: seq-lock 写 x/y)
     *
     * P0-修复 2026-07-08 (yaw=±90° 走错方向根因):
     *   本工程全局系为地图系 (X 向右, Y 向下=行号增大方向), 与车体系 (X 向右,
     *   Y 向前) 手性相反。由两条实车已验证事实可唯一确定变换:
     *     ① 发车航向 180° = 车头朝地图 -Y (APP_GAME_LAUNCH_FACE_YAW_DEG);
     *     ② 朝物体旋转公式 yaw = atan2f(dx, dy) (提交 ff2f828 实测修正)。
     *   ⇒ 车头方向在地图系 = (sin ψ, cos ψ), 车体→全局:
     *        gx = cosψ·vx + sinψ·vy
     *        gy = -sinψ·vx + cosψ·vy
     *   旧代码 sin 项符号相反 (gx=c·vx-s·vy, gy=s·vx+c·vy), 与指令端转置矩阵
     *   联立后实际运动 = R(2ψ)·期望: ψ=0/±180° 时无症状 (sin=0),
     *   ψ=±90° 时车沿反方向跑而里程计自认为正确 → 永远到不了目标点。
     *   本处与下方 D 项测量 / POINT_NAV 指令变换 / get_odom_velocity_global_mps
     *   四处同步修正; chassis_zone 软限位本就使用正确符号, 无需改动。 */
    yaw_rad = s_pose.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cy = cosf(yaw_rad);
    sy = sinf(yaw_rad);
    {
        float new_x = s_pose.x_m + ( cy * s_fb_vx + sy * s_fb_vy) * CHASSIS_TASK_DT_20MS_S;
        float new_y = s_pose.y_m + (-sy * s_fb_vx + cy * s_fb_vy) * CHASSIS_TASK_DT_20MS_S;
        pose_write_begin();
        s_pose.x_m = new_x;
        s_pose.y_m = new_y;
        pose_write_end();
    }

    /* 4) 模式分支 → 生成车体速度指令 */
    switch (s_mode) {

    case MODE_STOPPED:
        /* chassis_ctrl_stop()/init 已同步清零 PWM；停车期间禁止闭环重新产生命令。 */
        return;

    case MODE_POINT_NAV: {
        float dx   = s_tgt_x_m - s_pose.x_m;
        float dy   = s_tgt_y_m - s_pose.y_m;
        float dist = sqrtf(dx * dx + dy * dy);
        float accel_x = g_chassis_tune_params.cmd_accel_limit_mps2;
        float accel_y = g_chassis_tune_params.cmd_accel_limit_y_mps2;
        float vxg, vyg, norm;
        float yerr;

        /* ================================================================
         * 【层1~4】位置驱动
         *
         * 流程:
         *   层1: X/Y 实时误差均 ≤ EPSILON → 立即冻结平移并驻留确认
         *   层2: s_arrived && dist ≤ HOLD_EXIT → 驻车保持 (Schmitt上界)
         *   层3: s_arrived && dist > HOLD_EXIT → 大扰动, 重启驱动 (Schmitt下界)
         *   层4: 正常位置驱动 (brake_cap + axis-by-axis PD)
         * ================================================================*/

        /* ================================================================
         * 【层1】到位驻留判定
         *
         *   进入驻留: X/Y 实时误差均在 EPSILON 内 → 立即清平移 ramp
         *   到达确认: 位置 + 速度 + yaw 连续满足 N 拍 → 置 s_arrived=1
         *   驻留失败: 任一轴离开 EPSILON → 清计数并恢复该轴位置驱动
         *   到达后释放: dist > HOLD_EXIT → 清 s_arrived, 重启位置驱动
         *
         *   单阈值会让到位后的微小扰动立即重激活位置环；EPSILON 与
         *   HOLD_EXIT 之间的滞后区让扰动在驻车状态下自然衰减。
         * ----------------------------------------------------------------*/
        /* P0-修复 2026-06-07: 到达时清 s_ramp.
         * 原设计: 只设 cmd=0, 靠 ramp_filter 缓慢衰减 → 车带着速度
         *   冲出 10cm(HOLD_EXIT) → 下一拍 ISR 层3 清 s_arrived → 主循环
         *   永远抓不到 arrived=1 → 不切航点 → 串口显示 arrived 始终为 0.
         * 清 ramp 让轮速目标立刻归零, PID+breakaway 主动制动, 不再冲过头. */
        /* 到位驻留: 实时 X/Y 都进入窗口后立即冻结平移，再连续确认速度与航向。
         * 不能用锁存的 axis_arrived 代替实时位置，否则滑出窗口后仍会继续计数；
         * 也不能等计满后才清 ramp，否则 400ms 驻留期仍可能越点反复修正。 */
        if (0U == s_arrived) {
            uint8 pos_ok = (fabsf(dx) <= CHASSIS_TARGET_REACHED_EPSILON_M)
                        && (fabsf(dy) <= CHASSIS_TARGET_REACHED_EPSILON_M);
            float fb_speed = sqrtf(s_fb_vx * s_fb_vx + s_fb_vy * s_fb_vy);
            uint8 vel_ok = (fb_speed <= 0.12f);
            float yerr_arrival = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
            float wz_arrival = 0.0f;
            uint8 yaw_ok = 0U;
            chassis_arrival_decision_t arrival_decision;

            /* 只在已经进入位置窗口后运行静止航向环，避免正常导航路径重复调用 yaw_pi。
             * 驻留计数以 Schmitt 锁状态为准：进入 0.6° 后停止微调，漂到 0.8° 外才重启。 */
            if (pos_ok != 0U) {
                wz_arrival = yaw_pi(yerr_arrival, 1U, 1U);
                yaw_ok = s_yaw_in_position;
            }

            arrival_decision = chassis_arrival_dwell_update(
                &s_arrival_dwell_cnt,
                pos_ok,
                vel_ok,
                yaw_ok,
                CHASSIS_ARRIVAL_DWELL_COUNT);

            if (arrival_decision.hold_translation != 0U) {
                s_ramp.vx_body_mps = 0.0f;
                s_ramp.vy_body_mps = 0.0f;
                s_pos_i = 0.0f;
                s_pos_i_hold = 0.0f;
                cmd.vx_body_mps = 0.0f;
                cmd.vy_body_mps = 0.0f;
                cmd.wz_dps = wz_arrival;

                if (arrival_decision.arrived != 0U) {
                    s_arrived = 1U;
                    g_chassis_arrival_count++;
                    s_axis_x_arrived  = 0U;
                    s_axis_y_arrived  = 0U;
                    cmd.wz_dps        = 0.0f;
                }
                break;
            }

            /* 驻留期间若某轴滑出实时位置窗口，解除该轴锁存状态，
             * 让下方逐轴控制器确实重新拉回目标，而不是继续相信旧到位标志。 */
            if (fabsf(dx) > CHASSIS_TARGET_REACHED_EPSILON_M) {
                s_axis_x_arrived = 0U;
            }
            if (fabsf(dy) > CHASSIS_TARGET_REACHED_EPSILON_M) {
                s_axis_y_arrived = 0U;
            }
        }
        if (s_arrived && (dist <= CHASSIS_POS_HOLD_EXIT_M)) {
            /* 到位驻车: 平移停但航向保持.
             * 原 wz=0 让 yaw 在到位等待期间自由漂移 → 下次起步时
             * yaw 误差 → 坐标变换 cmd.vy_body = -sin(yaw)*vxg ≠ 0 → 走斜线. */
            {
                float yerr_hold = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
                cmd.vx_body_mps = 0.0f;
                cmd.vy_body_mps = 0.0f;
                /* inpos 锁内自然归零, 外层不硬切 → yaw PWM 前馈连续 */
                cmd.wz_dps = yaw_pi(yerr_hold, 1U, 1U);
            }
            break;
        }
        /* ================================================================
         * 【层3】大扰动 (Schmitt-trigger 下边界): 超出保持圈, 重启驱动
         * 清除保持状态, 同时取消进行中的矫正.
         * ================================================================*/
        if (s_arrived) {
            s_v_along_lpf   = 0.0f;  /* 清 D 项 LPF 缓存, 防止重新驱动时 D 项冲击 */
            s_v_cross_lpf   = 0.0f;
            s_pos_i         = 0.0f;  /* 清积分, 防止卷绕导致初始速度过大 */
            s_pos_i_hold    = 0.0f;
            s_yaw_i         = 0.0f;  /* yaw 积分同步清零 */
            s_yaw_rate_i    = 0.0f;  /* 内环 PI 积分清零 */
            s_arrival_dwell_cnt = 0U;  /* 驻留计数器清零: 重新驱动需重新确认 */
            s_ramp.vx_body_mps = 0.0f;  /* 清 ramp, 防旧残值污染新驱动方向 */
            s_ramp.vy_body_mps = 0.0f;
            s_axis_hold_x_m = s_pose.x_m;  /* 更新保持轴锚点到当前位置 */
            s_axis_hold_y_m = s_pose.y_m;
            /* P0-修复: 重置轴锁, 让状态机从 X 优先重新评估.
             * 不重置时: Y 阶段被大幅 X 扰动后仍保持 s_axis_y_locked=1,
             * 层4 进入 Y 相位, X 保持轴只能以 hold_max_speed 修正 X 偏差,
             * 同时 s_axis_hold_x_m 已被上方更新为扰动 X, X 保持环指向错误目标线,
             * 导致车永远停在扰动 X 处, dist 长期 > EPSILON, 无法到位. */
            s_axis_y_locked = 0U;
        }
        s_arrived = 0U;
        /* ================================================================
         * 【层4】正常位置驱动: brake_cap + axis-by-axis PD
         * dist > EPSILON, 车还在赶路或减速进场中.
         * ================================================================*/

        /* ----------------------------------------------------------------
         * 位置 P 控制 (axis-by-axis 曼哈顿模式):
         *   1) 取 |dx|,|dy| 较大者为"主轴", 只驱动该轴, 另一轴做保持
         *   2) 主轴误差 < switch_tol (= EPSILON×比例) 时切到次轴, 一格一格走
         *   3) 麦轮全局 X/Y 单轴运动 -> 对角耦合最小, 扰动恢复也只动一个轴
         *
         * 减速不再单独做 gain scheduling / recovery: 统一交给下方 brake cap
         * (sqrt 限速 + FLOOR) 这一个机制, 保持 KP 不变, 靠限速保证平滑停车.
         *
         * KD 自动推导: kd_eff = pos_kp × KD_RATIO (麦轮 ≈ KP/3, 阻尼比恒定);
         *              保持轴 kd_hold = pos_kp × HOLD_KD_RATIO (轻阻尼).
         * D 项始终用全局速度 (vxg_meas, vyg_meas) 直接做 PD, 与 axis 独立. */
        {
            /* P0-修复 2026-07-08: 与里程计积分同一套车体→全局变换 (见 task_20ms 步骤 3 注释) */
            float vxg_meas      =  cy * s_fb_vx + sy * s_fb_vy;  /* 全局速度 X */
            float vyg_meas      = -sy * s_fb_vx + cy * s_fb_vy;  /* 全局速度 Y */

            /* D 项低通 (Tesla/Waymo: derivative-on-measurement + IIR LPF) */
            s_v_along_lpf = (1.0f - CHASSIS_POS_D_LPF_ALPHA) * s_v_along_lpf
                          + CHASSIS_POS_D_LPF_ALPHA * vxg_meas;  /* 借用 along 槽存 vxg LPF */
            s_v_cross_lpf = (1.0f - CHASSIS_POS_D_LPF_ALPHA) * s_v_cross_lpf
                          + CHASSIS_POS_D_LPF_ALPHA * vyg_meas;  /* 借用 cross 槽存 vyg LPF */

            /* X/Y 位置环分别取参；D 项和保持阻尼由对应轴 Kp 独立推导。 */
            float kp_x = g_chassis_tune_params.pos_kp;
            float kp_y = g_chassis_tune_params.pos_kp_y;
            float kd_x = kp_x * CHASSIS_POS_KD_RATIO;
            float kd_y = kp_y * CHASSIS_POS_Y_KD_RATIO;
            float kd_hold_x = kp_x * CHASSIS_POS_HOLD_KD_RATIO;
            float kd_hold_y = kp_y * CHASSIS_POS_HOLD_KD_RATIO;

            /* 自动推导的积分/切轴参数 (替代旧 I_LIMIT / I_BAND / SWITCH_TOL 常量) */
            float brake_dist_x = pos_brake_dist_auto(accel_x);
            float brake_dist_y = pos_brake_dist_auto(accel_y);
            float i_limit    = g_chassis_tune_params.max_linear_speed_mps
                             * CHASSIS_POS_I_LIMIT_RATIO;
            float i_band_x   = brake_dist_x * CHASSIS_POS_I_BAND_RATIO;
            float i_band_y   = brake_dist_y * CHASSIS_POS_I_BAND_RATIO;

            /* 曼哈顿: 选主轴驱动, 非主轴命令 = 0 且立即清零其 ramp.
             *
             * 【定位不准根因】ramp 不清零时的过冲计算:
             *   切轴瞬间 ramp.vx ≈ kp * dx_prev ≈ 0.5 m/s
             *   ramp 以 3.0 m/s² 衰减 → 滑行距离 = 0.5²/(2*3.0) ≈ 42mm
             *   42mm > TOL=2cm → |dx| 超出容忍带 → X 重新抢优先权
             *   → X/Y 切轴振荡, Y 永远开不了头
             *
             * 修法: 每帧把非驱动轴的 s_ramp 清零, ramp_filter 下一步
             * 输出 = 0, 轮端 PID 立即接管制动. */
            /* 轴相位状态机 (逐轴到位):
             *   X 未到: 优先走 X, Y 保持。|dx|≤EPSILON → s_axis_x_arrived=1
             *   X 已到, Y 未到: 走 Y, X 保持。|dy|≤EPSILON → s_axis_y_arrived=1
             *   两轴都到 → layer1 置 s_arrived.
             *
             * P0-修复 2026-07-17: 替代 switch_tol+2D收敛, 每轴独立 EPSILON 判断. */
            {
                uint8 drive_y = ((s_axis_x_arrived != 0U) && (s_axis_y_arrived == 0U)) ? 1U : 0U;

                if (drive_y != 0U) {
                    linear_accel_limit_mps2 = accel_y;
                }

                if (!drive_y) {
                    /* ── X 驱动, Y 保持 ── */
                    if (fabsf(dx) <= CHASSIS_TARGET_REACHED_EPSILON_M
                        && fabsf(s_v_along_lpf) < 0.10f) {  /* 速度<10cm/s 才判到达, 防冲过头 */
                        s_axis_x_arrived = 1U;
                        s_axis_hold_x_m  = s_tgt_x_m;  /* 锚到目标 X */
                        s_pos_i          = 0.0f;
                        s_pos_i_hold     = 0.0f;
                        s_ramp.vx_body_mps = 0.0f;
                        s_ramp.vy_body_mps = 0.0f;
                        vxg = 0.0f;
                        vyg = 0.0f;
                    } else {
                        float y_hold_err = s_axis_hold_y_m - s_pose.y_m;
                        uint8 mpc_active;
                        vxg = driving_axis_velocity_cmd(dx, s_v_along_lpf,
                            kp_x, kd_x,
                            accel_x,
                            s_pos_i, &mpc_active);
                        if (!mpc_active && CHASSIS_POS_KI > 1e-6f && dist < i_band_x) {
                            s_pos_i += CHASSIS_POS_KI * dx * CHASSIS_TASK_DT_20MS_S;
                            if      (s_pos_i >  i_limit) s_pos_i =  i_limit;
                            else if (s_pos_i < -i_limit) s_pos_i = -i_limit;
                            if (dx * s_pos_i < 0.0f) s_pos_i = 0.0f;
                        }
                        if (fabsf(y_hold_err) > CHASSIS_POS_HOLD_DEAD_ZONE_M) {
                            if (CHASSIS_POS_Y_KI > 1e-6f) {
                                s_pos_i_hold += CHASSIS_POS_Y_KI * y_hold_err * CHASSIS_TASK_DT_20MS_S;
                                float hlim = i_limit * 0.5f;
                                if      (s_pos_i_hold >  hlim) s_pos_i_hold =  hlim;
                                else if (s_pos_i_hold < -hlim) s_pos_i_hold = -hlim;
                                if (y_hold_err * s_pos_i_hold < 0.0f) s_pos_i_hold = 0.0f;
                            }
                        } else { s_pos_i_hold = 0.0f; }
                        vyg = axis_hold_velocity_cmd(y_hold_err, s_v_cross_lpf,
                            kp_y, kd_hold_y,
                            accel_y,
                            s_pos_i_hold);
                    }
                } else {
                    /* ── Y 驱动, X 保持 ── */
                    if (fabsf(dy) <= CHASSIS_TARGET_REACHED_EPSILON_M
                        && fabsf(s_v_cross_lpf) < 0.10f) {  /* 速度<10cm/s 才判到达 */
                        s_axis_y_arrived = 1U;
                        s_axis_hold_y_m  = s_tgt_y_m;
                        s_pos_i          = 0.0f;
                        s_pos_i_hold     = 0.0f;
                        s_ramp.vx_body_mps = 0.0f;
                        s_ramp.vy_body_mps = 0.0f;
                        vxg = 0.0f;
                        vyg = 0.0f;
                    } else {
                        float x_hold_err = s_axis_hold_x_m - s_pose.x_m;
                        uint8 mpc_active;
                        /* X 漂移过大→回切: 放弃 Y 相位, 重新驱 X */
                        if ((fabsf(dx) > CHASSIS_TARGET_REACHED_EPSILON_M * 2.0f)
                            && (fabsf(dx) > fabsf(dy) * 1.5f)) {
                            s_axis_x_arrived = 0U;
                            s_pos_i          = 0.0f;
                            s_pos_i_hold     = 0.0f;
                            vxg = 0.0f;
                            vyg = 0.0f;
                        } else {
                            vyg = driving_axis_velocity_cmd(dy, s_v_cross_lpf,
                                kp_y, kd_y,
                                accel_y,
                                s_pos_i, &mpc_active);
                            if (!mpc_active && CHASSIS_POS_Y_KI > 1e-6f && dist < i_band_y) {
                                s_pos_i += CHASSIS_POS_Y_KI * dy * CHASSIS_TASK_DT_20MS_S;
                                if      (s_pos_i >  i_limit) s_pos_i =  i_limit;
                                else if (s_pos_i < -i_limit) s_pos_i = -i_limit;
                                if (dy * s_pos_i < 0.0f) s_pos_i = 0.0f;
                            }
                            if (fabsf(x_hold_err) > CHASSIS_POS_HOLD_DEAD_ZONE_M) {
                                if (CHASSIS_POS_KI > 1e-6f) {
                                    s_pos_i_hold += CHASSIS_POS_KI * x_hold_err * CHASSIS_TASK_DT_20MS_S;
                                    float hlim = i_limit * 0.5f;
                                    if      (s_pos_i_hold >  hlim) s_pos_i_hold =  hlim;
                                    else if (s_pos_i_hold < -hlim) s_pos_i_hold = -hlim;
                                    if (x_hold_err * s_pos_i_hold < 0.0f) s_pos_i_hold = 0.0f;
                                }
                            } else { s_pos_i_hold = 0.0f; }
                            vxg = axis_hold_velocity_cmd(x_hold_err, s_v_along_lpf,
                                kp_x, kd_hold_x,
                                accel_x,
                                s_pos_i_hold);
                        }
                    }
                }
            }
        }
        norm = sqrtf(vxg * vxg + vyg * vyg);

        /* 减速区 sqrt 限速 (逐轴独立刹车):
         *   P0-修复 2026-07-17: 旧代码用 dist(总距离) 算刹车, 导致 X 快到但 Y 远时
         *   dist 仍大 → 刹车不触发 → X 全速冲过头 → 保持环拉回 → 到点慢.
         *   修复: X 驱动时用 |dx|, Y 驱动时用 |dy| 独立算刹车. */
        {
            float brake_dist_x = pos_brake_dist_auto(accel_x);
            float brake_dist_y = pos_brake_dist_auto(accel_y);
            /* 入口也改用逐轴距离: 任一轴进刹车区就对该轴限速, 不再被另一轴拖累 */
            if ((brake_dist_x > 1e-6f || brake_dist_y > 1e-6f)
                && (fabsf(dx) < brake_dist_x || fabsf(dy) < brake_dist_y)) {
                float v_max_brake;

                /* 根据逐轴到位状态, 用驱动轴的误差独立算刹车速度 */
                if (s_axis_x_arrived == 0U) {
                    float brake_err = fabsf(dx) - CHASSIS_TARGET_REACHED_EPSILON_M;
                    if (brake_err < 0.0f) brake_err = 0.0f;
                    v_max_brake = sqrt_controller(brake_err,
                        g_chassis_tune_params.pos_kp,
                        accel_x);
                    if (v_max_brake < CHASSIS_POS_BRAKE_FLOOR_MPS)
                        v_max_brake = CHASSIS_POS_BRAKE_FLOOR_MPS;
                    if (fabsf(vxg) > v_max_brake)
                        vxg = (vxg > 0.0f) ? v_max_brake : -v_max_brake;
                } else if (s_axis_y_arrived == 0U) {
                    float brake_err = fabsf(dy) - CHASSIS_TARGET_REACHED_EPSILON_M;
                    if (brake_err < 0.0f) brake_err = 0.0f;
                    v_max_brake = sqrt_controller(brake_err,
                        g_chassis_tune_params.pos_kp_y,
                        accel_y);
                    if (v_max_brake < CHASSIS_POS_BRAKE_FLOOR_MPS)
                        v_max_brake = CHASSIS_POS_BRAKE_FLOOR_MPS;
                    if (fabsf(vyg) > v_max_brake)
                        vyg = (vyg > 0.0f) ? v_max_brake : -v_max_brake;
                } else {
                    /* 两轴都到位: 2D 等比例缩放 */
                    float brake_err = dist - CHASSIS_TARGET_REACHED_EPSILON_M;
                    float norm_2d;
                    if (brake_err < 0.0f) brake_err = 0.0f;
                    v_max_brake = sqrt_controller(brake_err,
                        g_chassis_tune_params.pos_kp,
                        accel_x);
                    if (v_max_brake < CHASSIS_POS_BRAKE_FLOOR_MPS)
                        v_max_brake = CHASSIS_POS_BRAKE_FLOOR_MPS;
                    norm_2d = sqrtf(vxg * vxg + vyg * vyg);
                    if (norm_2d > 1e-6f && norm_2d > v_max_brake) {
                        float sc = v_max_brake / norm_2d;
                        vxg *= sc;
                        vyg *= sc;
                    }
                }
            }
        }
        norm = sqrtf(vxg * vxg + vyg * vyg);  /* 刹车后重算, 供全局限速使用 */
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
        /* 曼哈顿轴模式按全局 X/Y 分段平移, yaw 在 move_to_grid/move_to_m 起步时
         * 锁到最近的 0/90/180/270° (见 move_to_grid), 行进中不再用 atan2 跟踪方位角,
         * 以免 X->Y 切轴时边旋转边走, 旋转耦合被保持环放大成横向偏移. */
        yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);

        /* 起步 yaw 门控: 先原地转到目标航向, 对齐后再平移.
         * P0-改进 2026-06-30: 恢复门控。边转边走时旋转耦合会被保持环放大
         * 成横向偏移 → 走不直。先对齐再平移消除这种耦合。 */
        if (!s_nav_yaw_aligned && (fabsf(yerr) > CHASSIS_YAW_INPOS_ENTER_DEG)) {
            s_ramp.vx_body_mps = 0.0f;
            s_ramp.vy_body_mps = 0.0f;
            cmd.vx_body_mps = 0.0f;
            cmd.vy_body_mps = 0.0f;
            cmd.wz_dps      = yaw_pi(yerr, 0U, 1U);  /* 原地旋转: 禁速率PI(噪声), 允在位锁 */
            break;
        }
        s_nav_yaw_aligned = 1U;

        /* 全局 → 车体坐标变换 (车体→全局矩阵的转置, 见 task_20ms 步骤 3 注释) */
        cmd.vx_body_mps =  cy * vxg - sy * vyg;
        cmd.vy_body_mps =  sy * vxg + cy * vyg;
        /* 平移时开内环 PI: 微小误差走增益调度消抖, 不小角度硬锁 */
        cmd.wz_dps = yaw_pi(yerr, 1U, 0U);
        break;
    }

    case MODE_SINGLE_WHEEL_PID_DEBUG: {
        apply_single_wheel_pid_debug(ws_pid);
        return;
    }

    default: {  /* MODE_YAW_HOLD / rotate_to_deg */
        float yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
        cmd.vx_body_mps = 0.0f;
        cmd.vy_body_mps = 0.0f;
        /* 静止保持: 速率PI + 在位锁 */
        cmd.wz_dps = yaw_pi(yerr, 1U, 1U);

        /* rotate_to_deg 到达判定: 误差进入容忍带且 in-pos 锁已触发 (车体稳定) */
        if (s_rotate_active && s_yaw_in_position) {
            s_arrived       = 1U;
            s_rotate_active = 0U;
        }
        break;
    }
    }

    /* 5) 软限位保护：即将撞墙时减速并微调方向.
     * POINT_NAV 模式下跳过: 目标始终是合法内场格, 0.22s 前瞻靠近边墙时
     * 会误触发 BRAKE_SCALE=0.28 + 随机 wz, 表现为行进中反复抖动.
     * 位置 PD + gain scheduling 已覆盖减速, 软限位只在自由运动模式保留. */
    if (s_mode != MODE_POINT_NAV) {
        chassis_zone_apply_soft_limit_guard(&cmd);
    }

    apply_speed(cmd, ws_pid, linear_accel_limit_mps2);
}

/* ==========================================================================
 *  § 7. 运动指令 — move_to_grid / move_to_m / hold_yaw / rotate_*
 * ========================================================================== */

/*--- 运动指令 ---*/

void chassis_ctrl_move_to_grid(uint8 target_x_grid, uint8 target_y_grid)
{
    uint8 x = chassis_clamp_grid_x_inner(target_x_grid);
    uint8 y = chassis_clamp_grid_y_inner(target_y_grid);
    chassis_pose_t pose_snap;
    float target_x_m;
    float target_y_m;

    pose_read_snapshot(&pose_snap);
    target_x_m = chassis_grid_x_to_m(x);
    target_y_m = chassis_grid_y_to_m(y);

    __disable_irq();
    s_tgt_x_m = target_x_m;
    s_tgt_y_m = target_y_m;
    /* 保持轴锚点用当前位置: axis-by-axis 模式下保持轴只维持另一轴不变,
     * 不提前追目标坐标, 避免 X 驱动阶段叠加 vy 修正 → 走斜线. */
    s_axis_hold_x_m = pose_snap.x_m;
    s_axis_hold_y_m = pose_snap.y_m;
    /* P0-修复 2026-05-12 (走斜线根因):
     * 曼哈顿轴模式下底盘只走 X/Y 网格方向, yaw 应锁到最近的 0/90/180/270°,
     * 让全局速度=车体速度, 不再有任何斜投影. 起点偏 ±45° 内会自动 snap, 偏更多则
     * 取最近 90° 倍数. 配合 yaw_pi P-only 阻尼, 起步时 IMU 会快速把 yaw 拉到位
     * (snap_err ≤45°, 以 max_yaw=150°/s 算 ≤0.3s 完成对齐). */
    {
        float yaw_now  = chassis_normalize_angle_deg(pose_snap.yaw_deg);
        float yaw_snap = roundf(yaw_now / 90.0f) * 90.0f;
        s_tgt_yaw_deg  = chassis_normalize_angle_deg(yaw_snap);
    }
    enter_mode(MODE_POINT_NAV);
    s_arrived = 0U;
    __enable_irq();
}

void chassis_ctrl_move_to_m(float x_m, float y_m, float hold_yaw_deg)
{
    chassis_pose_t pose_snap;
    float yaw_norm = chassis_normalize_angle_deg(hold_yaw_deg);

    pose_read_snapshot(&pose_snap);

    __disable_irq();
    s_tgt_x_m     = x_m;
    s_tgt_y_m     = y_m;
    s_tgt_yaw_deg = yaw_norm;     /* move_to_m 由调用者显式指定保持航向 */
    /* 保持轴锚点用当前位置, 不用目标坐标.
     * axis-by-axis 模式下保持轴职责是"维持另一轴不变", 不应提前追目标 Y.
     * 若锚点设为目标坐标, 里程计漂移会导致保持轴在 X 驱动阶段强行纠偏,
     * 车体 vy ≠ 0 → 走斜线 → 表现为"大范围 -X 移动时先往 -Y 冲一下".
     * Y 方向的修正留给 Y 驱动阶段完成, X 阶段只管走直线. */
    s_axis_hold_x_m = pose_snap.x_m;
    s_axis_hold_y_m = pose_snap.y_m;
    enter_mode(MODE_POINT_NAV);  /* 先 enter_mode 重置标志, 再打开锁定, 顺序不能反 */
    s_nav_lock_yaw = 1U;         /* 锁住姿态, 任务层不再用 atan2 覆盖 */
    s_arrived = 0U;
    __enable_irq();
}

void chassis_ctrl_hold_yaw(float target_yaw_deg)
{
    s_tgt_yaw_deg = chassis_normalize_angle_deg(target_yaw_deg);
    enter_mode(MODE_YAW_HOLD);
    s_arrived = 1U;
}

void chassis_ctrl_rotate_to_deg(float target_yaw_deg)
{
    s_tgt_yaw_deg = chassis_normalize_angle_deg(target_yaw_deg);
    enter_mode(MODE_YAW_HOLD);  /* 复用 YAW_HOLD: vx=vy=0, 只输出 wz */
    s_rotate_active = 1U;
    s_arrived       = 0U;
}

/* ==========================================================================
 *  § 8. 单轮 PID 调试 API (start/set_target/stop) + 紧急停机
 * ========================================================================== */

void chassis_ctrl_start_single_wheel_pid_debug(uint8 wheel_index,
                                               float target_speed_mps)
{
    uint8 safe_wheel;

    safe_wheel = debug_wheel_index_safe(wheel_index);

    s_debug_wheel_index = safe_wheel;
    s_debug_wheel_target_mps = debug_target_speed_clamp(target_speed_mps);
    s_debug_fb_sign_mul[safe_wheel] = 1.0f;
    s_debug_target_ramp_mps = 0.0f;

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
        enter_mode(MODE_STOPPED);
        force_stop();
        chassis_pid_debug_reset();
    }
    s_arrived = 1U;
}

void chassis_ctrl_stop(void)
{
    enter_mode(MODE_STOPPED);
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

/* ==========================================================================
 *  § 9. 航向调试 — attitude_debug_* + 50ms VOFA+ 输出链
 * ========================================================================== */

/*--- 航向调试 ---*/

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

void chassis_ctrl_attitude_debug_task_5ms(void)
{
    static uint8 div = 0U;
    chassis_pose_t pose_snap;
    float fb_snap[CHASSIS_WHEEL_COUNT];
    uint8 i;

    /* 50ms 分频 (10 * 5ms tick): 比 100ms 更密, 上位机绘曲线更平滑 */
    if (++div < 10U) return;
    div = 0U;

    pose_read_snapshot(&pose_snap);
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        fb_snap[i] = s_wheel_fb_lpf[i];
    }

    /*
     * VOFA+ FireWater 协议格式 (12 通道):
     *   ch01-04 = target/current/err/wz_cmd   (deg, deg, deg, dps)
     *   ch05-08 = pwm LF/RF/LB/RB             (前进符号域, 未乘 dir_sign)
     *   ch09-12 = fb  LF/RF/LB/RB             (m/s, LPF 后)
     */
    printf("%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\n",
           s_tgt_yaw_deg,
           pose_snap.yaw_deg,
           chassis_normalize_angle_deg(s_tgt_yaw_deg - pose_snap.yaw_deg),
           s_last_cmd.wz_dps,
           s_pid[CHASSIS_WHEEL_LF].output, s_pid[CHASSIS_WHEEL_RF].output,
           s_pid[CHASSIS_WHEEL_LB].output, s_pid[CHASSIS_WHEEL_RB].output,
           fb_snap[CHASSIS_WHEEL_LF], fb_snap[CHASSIS_WHEEL_RF],
           fb_snap[CHASSIS_WHEEL_LB], fb_snap[CHASSIS_WHEEL_RB]);
}

/* ==========================================================================
 *  § 10. 状态查询 / pose seq-lock 读 / 外部校正 / tune 参数 setter
 * ========================================================================== */

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

void chassis_ctrl_get_odom_velocity_global_mps(float *out_vx_g, float *out_vy_g)
{
    chassis_pose_t p;
    float yaw_rad;
    float cy;
    float sy;
    float vx_b;
    float vy_b;
    float vxg;
    float vyg;

    pose_read_snapshot(&p);
    yaw_rad = p.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cy      = cosf(yaw_rad);
    sy      = sinf(yaw_rad);
    vx_b    = s_fb_vx;
    vy_b    = s_fb_vy;
    /* 与里程计积分同一旋转: new_x += (cy*vx + sy*vy)*dt (P0-修复 2026-07-08) */
    vxg     =  cy * vx_b + sy * vy_b;
    vyg     = -sy * vx_b + cy * vy_b;

    if (out_vx_g != NULL) { *out_vx_g = vxg; }
    if (out_vy_g != NULL) { *out_vy_g = vyg; }
}

void chassis_ctrl_get_point_nav_target_m(float *out_x_m, float *out_y_m)
{
    if (out_x_m != NULL) { *out_x_m = s_tgt_x_m; }
    if (out_y_m != NULL) { *out_y_m = s_tgt_y_m; }
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
 *   原 1850-2141 行已于 2026-05-13 整体迁出到 chassis_zone.c.
 *   API (chassis_zone_*) / 行为 / 静态变量 / 阈值参数全部等价, 详见 chassis_zone.c.
 * ================================================================== */
