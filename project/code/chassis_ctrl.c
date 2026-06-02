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
 * 级联 P-PI 路径使用 CHASSIS_YAW_RATE_* 系列, 调参直接改 chassis_config.h.
 */
<<<<<<< HEAD
=======
#define YAW_DEADZONE_DEG   (g_chassis_tune_params.yaw.deadzone_deg)
#define YAW_KI             (g_chassis_tune_params.yaw.ki)
#define YAW_I_LIMIT        (g_chassis_tune_params.yaw.i_limit)
#define YAW_MIN_WZ_DPS     CHASSIS_YAW_MIN_WZ_DPS
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03

/* 单轮 PID 调试起步补偿参数（用于克服静摩擦） */
#define WHEEL_DEBUG_START_SPEED_EPS_MPS   (0.03f)   /* 低于此反馈速度视为静止 */
#define WHEEL_DEBUG_START_TARGET_EPS_MPS  (0.05f)   /* 低于此目标速度不启用补偿 */
#define WHEEL_DEBUG_START_PWM_MIN         (g_chassis_tune_params.wheel_ff.debug_start_pwm_min)
#define WHEEL_DEBUG_TARGET_RAMP_MPS_PER_TICK (0.8f) /* 20ms 每拍目标最多变化量 */

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

/* 软限位保护参数 / 软限位 helpers / apply_soft_limit_guard()
 * 已于 2026-05-13 迁至 chassis_zone.c (chassis_zone_apply_soft_limit_guard).
 * 主任务调用见下方 task_20ms 末尾. */

/* ---------------------- 控制模式 ---------------------- */

typedef enum {
    MODE_YAW_HOLD = 0,      /* 原地航向保持（默认） / rotate_to_deg 复用 */
    MODE_POINT_NAV,         /* 网格点位导航                              */
    MODE_SINGLE_WHEEL_PID_DEBUG /* 单轮 PID 调试（仅一个轮子给目标）        */
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
static volatile uint8       s_recovery_active = 0U;
/* 1 = 正在执行 rotate_to_deg 原地旋转, 到达角度容忍带后置 s_arrived=1 并清零 */
static volatile uint8       s_rotate_active = 0U;

/* 导航目标 */
static volatile float s_tgt_x_m       = 0.0f;
static volatile float s_tgt_y_m       = 0.0f;
static volatile float s_tgt_yaw_deg   = 0.0f;
/* 1 = chassis_ctrl_move_to_m() 主动锁定航向, 禁止 POINT_NAV 任务用 atan2 覆盖.
 * axis-by-axis 模式下 move_to_grid() 也会保持起步航向, 不边走轴边转头. */
static volatile uint8 s_nav_lock_yaw  = 0U;
/* 直线路径方向单位向量 (move_to_m 调用时按起点→终点计算).
 * 与 s_tgt_yaw_deg (机器人头部朝向) 完全解耦, 麦轮平移时两者通常不同. */
static volatile float s_path_cos_phi  = 1.0f;
static volatile float s_path_sin_phi  = 0.0f;

/* D 项低通状态 (一阶 IIR, 消除 odom 高频噪声对 KD 的放大) */
static float s_v_along_lpf = 0.0f;
static float s_v_cross_lpf = 0.0f;
/* 位置环沿程方向积分累积量 (m·s, 离散: 每帧 ×DT) */
static float s_pos_i       = 0.0f;
/* Schmitt 触发器: 1 = X 轴已到位, 当前锁定 Y 轴优先; 0 = X 未完成 */
static uint8_t s_axis_y_locked = 0U;
/* axis-by-axis 非驱动轴保持坐标: 保持当前直线, 不是提前追最终目标造成斜线 */
static float s_axis_hold_x_m = 0.0f;
static float s_axis_hold_y_m = 0.0f;

/* 单轮 PID 调试参数 */
static volatile uint8 s_debug_wheel_index = (uint8)CHASSIS_WHEEL_LF;
static volatile float s_debug_wheel_target_mps = 0.0f;
static volatile float s_debug_fb_sign_mul[CHASSIS_WHEEL_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
static volatile float  s_debug_target_ramp_mps      = 0.0f;

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

<<<<<<< HEAD
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

=======
#if (0 == CHASSIS_CTRL_SINGLE_WHEEL_PID_DEBUG_ONLY)
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
/** 车体速度矢量限幅: (vx,vy)模长 ≤ max_linear, |wz| ≤ max_yaw */
static void limit_speed(chassis_body_speed_cmd_t *c)
{
    float norm;

    c->wz_dps = chassis_clamp_f(c->wz_dps,
                                -g_chassis_tune_params.limit.max_yaw_speed_dps,
                                 g_chassis_tune_params.limit.max_yaw_speed_dps);

    norm = sqrtf(c->vx_body_mps * c->vx_body_mps +
                 c->vy_body_mps * c->vy_body_mps);
    if (norm > g_chassis_tune_params.limit.max_linear_speed_mps && norm > 1e-6f) {
        float s = g_chassis_tune_params.limit.max_linear_speed_mps / norm;
        c->vx_body_mps *= s;
        c->vy_body_mps *= s;
    }
}

/** 缓加速斜坡滤波: 每 20ms 周期限制速度变化量，防止轮胎打滑 */
static chassis_body_speed_cmd_t ramp_filter(chassis_body_speed_cmd_t tgt)
{
    const float dv = g_chassis_tune_params.limit.accel_limit_mps2 * CHASSIS_TASK_DT_20MS_S;
    const float dw = g_chassis_tune_params.limit.yaw_accel_limit_dps2 * CHASSIS_TASK_DT_20MS_S;
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
    return chassis_clamp_f(velocity_cmd,
                           -g_chassis_tune_params.position.axis_hold_max_speed_mps,
                            g_chassis_tune_params.position.axis_hold_max_speed_mps);
}

static float axis_hold_velocity_cmd(float hold_error,
                                    float axis_velocity_lpf,
                                    float position_gain,
                                    float damping_gain,
                                    float accel_limit)
{
    /* 保持轴不设到位死区: 任何偏离都持续输出修正力.
     * 原 EPSILON=5cm 死区是 X 偏移根因之一: Y 行驶时 X 可自由漂 ±5cm
     * 而不触发任何闭环修正, 累积后表现为明显 X 偏移. */
    return clamp_axis_hold_speed(
            position_axis_velocity_cmd(hold_error,
                                       axis_velocity_lpf,
                                       position_gain,
                                       damping_gain,
                                       accel_limit,
                                       0.0f));
}

/** 紧急停机: 清零滤波器、PID、PWM */

static void force_stop(void)
{
    uint8 i;

    s_ramp     = (chassis_body_speed_cmd_t){0};
    s_last_cmd = (chassis_body_speed_cmd_t){0};
    s_yaw_i    = 0.0f;
    s_pos_i    = 0.0f;
    s_v_along_lpf = 0.0f;
    s_v_cross_lpf = 0.0f;
    s_recovery_active = 0U;

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
        const float abs_fb     = fabsf(wheel_fb_mps[i]);

        pwm_forward_domain = chassis_pid_step(&s_pid[i], targets[i], wheel_fb_mps[i]);

<<<<<<< HEAD
        /* 静摩擦前馈 (仅起步注入, 运动中关闭):
         *   起步: |target|>EPS 且 |fb|<FB_STATIC_EPS -> 加性 ff = sign(t)*FLOOR
         *   一旦轮真正起转 (|fb|>=FB_STATIC_EPS), ff=0, 完全交给 PID
         * 这样消除 "PID + ff 双重补偿" 在运动中导致的 PWM 抖动. */
        if ((abs_target > CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS) &&
            (abs_fb     < CHASSIS_WHEEL_BREAKAWAY_FB_STATIC_EPS_MPS)) {
            float ff_pwm = (targets[i] >= 0.0f)
                         ?  CHASSIS_WHEEL_BREAKAWAY_PWM_FLOOR
                         : -CHASSIS_WHEEL_BREAKAWAY_PWM_FLOOR;
=======
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
        if ((g_chassis_tune_params.wheel_ff.breakaway_target_eps_mps > 1e-6f) &&
            (abs_target > g_chassis_tune_params.wheel_ff.breakaway_target_eps_mps)) {
            float ramp = (abs_target - g_chassis_tune_params.wheel_ff.breakaway_target_eps_mps)
                       /  g_chassis_tune_params.wheel_ff.breakaway_target_eps_mps;
            float ff_pwm;
            if (ramp > 1.0f) ramp = 1.0f;
            ff_pwm = g_chassis_tune_params.wheel_ff.breakaway_pwm_floor * ramp;
            if (targets[i] < 0.0f) ff_pwm = -ff_pwm;
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
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

    if ((fabsf(axis_error) > g_chassis_tune_params.position.brake_dist_m) &&
        (((axis_error > 0.0f) && (velocity_cmd < 0.0f)) ||
         ((axis_error < 0.0f) && (velocity_cmd > 0.0f))))
    {
        velocity_cmd = position_cmd * 0.25f;
    }
    else if ((fabsf(axis_error) > g_chassis_tune_params.position.target_reached_epsilon_m) &&
        (((axis_error > 0.0f) && (velocity_cmd < 0.0f)) ||
         ((axis_error < 0.0f) && (velocity_cmd > 0.0f))))
    {
        velocity_cmd = 0.0f;
    }

    return velocity_cmd;
}

/**
 * 航向 PI 闭环
 * @param err              航向误差(°)，已归一化
 * @param enable_rate_loop 是否启用内环 rate PI 反馈:
 *                          1 = MODE_YAW_HOLD (静止) -> 完整级联, 享 KI 学摩擦
 *                          0 = POINT_NAV 平动期间 -> 纯前馈, 避免 IMU 振动噪声被 KI 放大成限环
 *                          原因: 平动时底盘振动使 IMU yaw_rate 噪声 ±10°/s, 进 rate_err
 *                          进 I -> wz 抖 -> 麦轮抖 -> 车体更抖. 平动期间只靠
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
                                          g_chassis_tune_params.yaw.kp,
                                          CHASSIS_YAW_ACCEL_MAX_DPS2);
        float rate_err;
        float kp_v_term;
        float ki_v_term;

        wz_target = chassis_clamp_f(wz_target,
                                    -g_chassis_tune_params.limit.max_yaw_speed_dps,
                                     g_chassis_tune_params.limit.max_yaw_speed_dps);

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
        s_yaw_i = s_yaw_i * (1.0f - g_chassis_tune_params.yaw.rate_i_leak)
                + rate_err * CHASSIS_TASK_DT_20MS_S;
        s_yaw_i = chassis_clamp_f(s_yaw_i,
                                  -g_chassis_tune_params.yaw.rate_i_limit,
                                   g_chassis_tune_params.yaw.rate_i_limit);

        kp_v_term = g_chassis_tune_params.yaw.rate_kp * rate_err;
        ki_v_term = g_chassis_tune_params.yaw.rate_ki * s_yaw_i;

        /* 5) 输出 = 前馈 + PI 修正 */
        wz = wz_target + kp_v_term + ki_v_term;
    }

    /* 6) 输出截幅 */
    wz = chassis_clamp_f(wz,
                         -g_chassis_tune_params.limit.max_yaw_speed_dps,
                          g_chassis_tune_params.limit.max_yaw_speed_dps);

<<<<<<< HEAD
=======
#else /* ============== 旧的单环 PID 路径 (回退) ============== */
    {
        float p_term = sqrt_controller(err, g_chassis_tune_params.yaw.kp, CHASSIS_YAW_ACCEL_MAX_DPS2);
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
        wz = chassis_clamp_f(wz, -g_chassis_tune_params.limit.max_yaw_speed_dps,
                                  g_chassis_tune_params.limit.max_yaw_speed_dps);
    }
#endif

>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
    return wz;
}

/** 模式切换辅助: 清积分 + 设模式 */
static void enter_mode(ctrl_mode_t m)
{
    s_yaw_i           = 0.0f;
    s_yaw_in_position = 0;   /* 模式切换强制解锁, 防新目标被旧锁挡住 */
    s_rotate_active   = 0U;
    s_recovery_active = 0U;
    s_nav_lock_yaw    = 0U;  /* 默认关闭锁航, move_to_m 主动调用才打开 */
    s_v_along_lpf     = 0.0f;  /* 切换目标时清零 LPF, 避免旧速度残值污染新路径 D 项 */
    s_v_cross_lpf     = 0.0f;
    s_pos_i           = 0.0f;  /* 切换目标时清零位置积分, 防旧路径残留量误推新起点 */
    s_axis_y_locked   = 0U;   /* 切换目标时复位轴锁, 重新从 X 轴开始 */
    s_mode            = m;
}

/* ==========================================================================
 *  § 5. 公共 API — 初始化
 * ========================================================================== */

/* ====================== 公共 API ====================== */

/*--- 初始化 ---*/

void chassis_ctrl_init(void)
{
    uint8 i;
    chassis_tune_params_t tune_snapshot;

    tune_snapshot = g_chassis_tune_params;
    chassis_config_sanitize_tune(&tune_snapshot);
    chassis_config_apply(&tune_snapshot);
    chassis_imu_init();

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        chassis_motor_init(&s_mot[i]);
        chassis_encoder_init(&s_enc[i]);
        chassis_pid_init(&s_pid[i],
                         g_chassis_tune_params.wheel_pid.kp[i],
                         g_chassis_tune_params.wheel_pid.ki[i],
                         g_chassis_tune_params.wheel_pid.kd[i],
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
    s_fb_vx = vx_raw * g_chassis_tune_params.odom.scale_x;
    s_fb_vy = vy_raw * g_chassis_tune_params.odom.scale_y;

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
        float yerr;

        /* ================================================================
         * 【层1~4】位置驱动
         *
         * 流程:
         *   层1: dist ≤ EPSILON → 一拍姿态闭环保持, 然后置 s_arrived=1
         *   层2: s_arrived && dist ≤ HOLD_EXIT(10cm) → 驻车保持 (Schmitt上界)
         *   层3: s_arrived && dist > HOLD_EXIT → 大扰动, 重启驱动 (Schmitt下界)
         *   层4: 正常位置驱动 (brake_cap + gain_schedule + axis-by-axis PD)
         * ================================================================*/

        /* ================================================================
         * 【层1】到位进入判定 (Schmitt-trigger 上边界)
         *
         * Schmitt 双阈值说明:
         *   进入保持: dist ≤ EPSILON → 立即置 s_arrived=1
         *   离开保持: dist > HOLD_EXIT → 清 s_arrived, 重启位置驱动
         *
         *   单阈值的问题: 到位停下后任何小扰动(>6cm)立即重激活位置 P,
         *   以 ~0.12 m/s 冲回 → 过冲到另一侧 → 反向冲 → 欠阻尼震荡.
         *   4cm 的滞后带让小扰动在保持圈内自然衰减, 不触发驱动.
         *
         * 当前调试阶段先只看位置: 只要进误差圈, 就认为该目标完成.
         * yaw 精校/速度稳定都不参与到达判断, 避免车已经到点却长时间不切下一个目标.
         * ----------------------------------------------------------------*/
<<<<<<< HEAD
        if ((0U == s_arrived) && (dist <= CHASSIS_TARGET_REACHED_EPSILON_M)) {
            /* 到位: 输出 cmd=0 让 ramp_filter 平滑衰减惯性, PID 自然跟随
             * 反馈减速. 不再 force_stop 硬清 ramp/PID, 避免末段急刹一脚. */
=======
        if ((0U == s_arrived) && (dist <= g_chassis_tune_params.position.target_reached_epsilon_m)) {
            /* P0-修复 2026-05-12 (大位移潜在 bug):
             * 旧版到位帧调 yaw_pi(yerr_arrive, 1, 1) 输出 wz, 经 ramp 限制后
             * 送 apply_speed -> 麦轮接到 wz≠0 命令转一点点, 下一拍 layer2 force_stop
             * 又抹掉 -> 纯抖动无意义. 且 force_stop 不清 s_yaw_in_position 残留锁.
             *
             * 新版直接 force_stop + s_arrived=1, 等价 layer2 提前一拍执行,
             * 干净利落, 上层主循环下一拍即可切下个目标. */
            (void)dx; (void)dy;  /* avoid unused if compiler complains */
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
            s_arrived = 1U;
            s_recovery_active = 0U;
            s_yaw_in_position = 0;
            cmd.vx_body_mps = 0.0f;
            cmd.vy_body_mps = 0.0f;
            cmd.wz_dps      = 0.0f;
            break;
        }
        if (s_arrived && (dist <= CHASSIS_POS_HOLD_EXIT_M)) {
            /* 到位驻车: 同样让 ramp 自然收敛, 避免周期性 force_stop 抽搐. */
            cmd.vx_body_mps = 0.0f;
            cmd.vy_body_mps = 0.0f;
            cmd.wz_dps      = 0.0f;
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
            s_axis_hold_x_m = s_pose.x_m;  /* 更新保持轴锚点到当前位置 */
            s_axis_hold_y_m = s_pose.y_m;
            /* P0-修复: 重置轴锁, 让状态机从 X 优先重新评估.
             * 不重置时: Y 阶段被大幅 X 扰动后仍保持 s_axis_y_locked=1,
             * 层4 进入 Y 相位, X 保持轴只能以 AXIS_HOLD_MAX_SPEED(0.06m/s) 修正 X 偏差,
             * 同时 s_axis_hold_x_m 已被上方更新为扰动 X, X 保持环指向错误目标线,
             * 导致车永远停在扰动 X 处, dist 长期 > EPSILON, 无法到位. */
            s_axis_y_locked = 0U;
            s_recovery_active = 1U;         /* 进入扰动恢复模式: 弱增益缓慢归位 */
        }
        s_arrived = 0U;
        /* ================================================================
         * 【层4】正常位置驱动: brake_cap + gain_schedule + axis-by-axis PD
         * dist > EPSILON (6cm), 车还在赶路或减速进场中.
         * ================================================================*/
<<<<<<< HEAD
=======
        if ((g_chassis_tune_params.position.brake_dist_m > 1e-6f) && (dist < g_chassis_tune_params.position.brake_dist_m)) {
            /* 线性 ramp: dist=BRAKE -> v=V_MIN; dist=EPSILON -> v=0
             * 注意: 此处只影响 v_drive_min (最小推进速度下限),
             * 实际速度上限由下方 sqrt_controller brake_cap 控制. */
            float ratio = (dist - g_chassis_tune_params.position.target_reached_epsilon_m)
                        / (g_chassis_tune_params.position.brake_dist_m - g_chassis_tune_params.position.target_reached_epsilon_m);
            if (ratio < 0.0f) ratio = 0.0f;
            if (ratio > 1.0f) ratio = 1.0f;
            v_drive_min = CHASSIS_POS_MIN_DRIVE_SPEED_MPS * ratio;
            if ((ratio > 0.0f) &&
                (v_drive_min < (2.0f * g_chassis_tune_params.wheel_ff.breakaway_target_eps_mps)))
            {
                /* 只要还没进 EPSILON, 最小推进不能被衰减到静摩擦阈值以下.
                 * 否则车会停在 5cm 到位圈外, s_arrived 永远不置 1, 上层航点也不会切换. */
                v_drive_min = 2.0f * g_chassis_tune_params.wheel_ff.breakaway_target_eps_mps;
            }
        }
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03

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

            /* 线性 Gain scheduling (平滑版):
             *   dist = RECOVERY_DIST : 全量增益 (scale=1.0)
             *   dist = EPSILON       : 最小增益 (scale=KP_SCALE)
             *   中间段线性插值, 无硬跳变.
             *
             * 关键: KD 与 KP 等比缩放 → 阻尼比恒定.
             *   旧 step 方案: KP 突降 55%, KD 不变 → D 压过 P →
             *   在 35cm 处车速 0.5m/s: KP_new×0.35-KD×0.5=-0.1(负!) → 刹车
             *   → 停下来 → P 再大 → 再冲 → "停下抖动前进". */
            float kp_eff = g_chassis_tune_params.position.kp;
            float kd_eff = g_chassis_tune_params.position.kd;
            float cte_kp_eff = g_chassis_tune_params.position.cte_kp;
            float cte_kd_eff = g_chassis_tune_params.position.cte_kd;
            if (dist < CHASSIS_POS_RECOVERY_DIST_M) {
                float gs_t = (dist - g_chassis_tune_params.position.target_reached_epsilon_m)
                           / (CHASSIS_POS_RECOVERY_DIST_M - g_chassis_tune_params.position.target_reached_epsilon_m);
                if (gs_t < 0.0f) gs_t = 0.0f;
                if (gs_t > 1.0f) gs_t = 1.0f;
                float gs = CHASSIS_POS_RECOVERY_KP_SCALE
                         + (1.0f - CHASSIS_POS_RECOVERY_KP_SCALE) * gs_t;
                kp_eff     *= gs;
                kd_eff     *= gs;  /* D 与 P 同步缩, 保持 KD/KP 比不变 */
                cte_kp_eff *= gs;  /* CTE 增益同步缩放 */
                cte_kd_eff *= gs;
            }

#if (CHASSIS_POS_AXIS_BY_AXIS_ENABLE != 0)
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
            /* 轴相位状态机:
             *   X 未完成: 只要 |dx| > SWITCH_TOL, 先走 X.
             *   已进入 Y: 不再因为 X 抖过 RELOCK_TOL 而反复回切 X;
             *             Y 阶段用 X 保持环连续修正 X 偏差.
             * 这样避免 X/Y 来回抢轴导致的“Y 前进-停顿-回弹-再前进”. */
            {
                uint8 run_x_phase = ((0U == s_axis_y_locked) &&
                                      (fabsf(dx) > CHASSIS_POS_AXIS_SWITCH_TOL_M))
                                     ? 1U : 0U;

                /* 非驱动轴保持环: 直接使用 CTE 增益, 不与主轴增益取 min.
                 * 原 min(CTE_KD, POS_KD) 逻辑 bug: POS_KD=0 时 min=0,
                 * 把用户设定的 CTE_KD=1.0 完全清零 → 保持轴纯 P 无阻尼
                 * → 修正后振荡, 表现为 Y 行驶中 X 来回偏移. */
                float cte_kp_hold = cte_kp_eff;
                float cte_kd_hold = cte_kd_eff;

                /* 沿程方向条件积分: 仅在 I_BAND 内且 KI>0 时累积.
                 * 反向检测: 误差与积分异号 (已过冲/反向) 时清零, 防卷绕. */
#define ACCUMULATE_POS_I(e_driving)                                           \
    do {                                                                       \
        if (g_chassis_tune_params.position.ki > 1e-6f && dist < g_chassis_tune_params.position.i_band_m) {         \
            s_pos_i += g_chassis_tune_params.position.ki * (e_driving) * CHASSIS_TASK_DT_20MS_S;\
            if (s_pos_i >  g_chassis_tune_params.position.i_limit_mps) s_pos_i =  g_chassis_tune_params.position.i_limit_mps; \
            if (s_pos_i < -g_chassis_tune_params.position.i_limit_mps) s_pos_i = -g_chassis_tune_params.position.i_limit_mps; \
            if (((e_driving) * s_pos_i) < 0.0f)  s_pos_i = 0.0f;            \
        }                                                                      \
    } while (0)

                if (run_x_phase) {
                    float y_hold_err;
                    y_hold_err = s_axis_hold_y_m - s_pose.y_m;
                    s_axis_y_locked    = 0U;
                    ACCUMULATE_POS_I(dx);
                    vxg = position_axis_velocity_cmd(dx,
                                                     s_v_along_lpf,
                                                     kp_eff,
                                                     kd_eff,
                                                     g_chassis_tune_params.limit.accel_limit_mps2,
                                                     s_pos_i);
                    vyg = axis_hold_velocity_cmd(y_hold_err,
                                                 s_v_cross_lpf,
                                                 cte_kp_hold,
                                                 cte_kd_hold,
                                                 g_chassis_tune_params.limit.accel_limit_mps2);
                } else if ((s_axis_y_locked != 0U) ||
                           (fabsf(dy) > CHASSIS_POS_AXIS_SWITCH_TOL_M)) {
                    float x_hold_err;
                    if (!s_axis_y_locked) {
                        /* 切轴瞬间: 锚定 X 保持轴 + 清 D LPF/积分,
                         * 让 Y 主轴从干净基线起步, 避免 D 突变带来"切轴顿挫". */
                        s_axis_hold_x_m = s_pose.x_m;
                        s_v_along_lpf   = 0.0f;
                        s_v_cross_lpf   = 0.0f;
                        s_pos_i         = 0.0f;
                    }
                    x_hold_err         = s_axis_hold_x_m - s_pose.x_m;
                    s_axis_y_locked    = 1U;
                    ACCUMULATE_POS_I(dy);
                    vxg = axis_hold_velocity_cmd(x_hold_err,
                                                 s_v_along_lpf,
                                                 cte_kp_hold,
                                                 cte_kd_hold,
                                                 g_chassis_tune_params.limit.accel_limit_mps2);
                    vyg = position_axis_velocity_cmd(dy,
                                                     s_v_cross_lpf,
                                                     kp_eff,
                                                     kd_eff,
                                                     g_chassis_tune_params.limit.accel_limit_mps2,
                                                     s_pos_i);
                } else {
                    /* 两轴都在容忍带内: 2D 平滑收敛; 不累积 I (防到位后漂) */
                    s_axis_y_locked    = 1U;
                    vxg = position_axis_velocity_cmd(dx,
                                                     s_v_along_lpf,
                                                     kp_eff,
                                                     kd_eff,
                                                     g_chassis_tune_params.limit.accel_limit_mps2,
                                                     0.0f);
                    vyg = position_axis_velocity_cmd(dy,
                                                     s_v_cross_lpf,
                                                     kp_eff,
                                                     kd_eff,
                                                     g_chassis_tune_params.limit.accel_limit_mps2,
                                                     0.0f);
                }
#undef ACCUMULATE_POS_I
            }
#else
            /* CTE 直线路径 PD (CHASSIS_POS_AXIS_BY_AXIS_ENABLE=0 时生效):
             * 把误差/速度分解到路径坐标系 (沿程 t / 横向 n),
             * 沿程用 KP/KD, 横向用 CTE_KP/CTE_KD, 再旋转回全局输出.
             * 路径方向 (s_path_cos_phi, s_path_sin_phi) 由 move_to_m /
             * move_to_grid 调用时固化, 全程不随车位置变化 (固定参考线). */
            {
                float cp = s_path_cos_phi;   /* 路径单位向量 X 分量 */
                float sp = s_path_sin_phi;   /* 路径单位向量 Y 分量 */

                /* 误差分解到路径坐标系 */
                float e_along =  dx * cp + dy * sp;   /* 沿程误差: >0 表示目标在前 */
                float e_cross = -dx * sp + dy * cp;   /* 横向误差: >0 表示目标在左(车偏右) */

                /* LPF 速度旋转到路径坐标系 (s_v_along/cross_lpf 存的是全局 vx/vy LPF) */
                float v_along_lpf =  s_v_along_lpf * cp + s_v_cross_lpf * sp;
                float v_cross_lpf = -s_v_along_lpf * sp + s_v_cross_lpf * cp;

                /* 路径坐标系 PID 输出 (cte_kp_eff/cte_kd_eff 已在外层 gain-scheduling 处统一计算) */
                /* 沿程方向加 I 项, 消除地面摩擦/倾斜导致的稳态位置误差 */
                if (g_chassis_tune_params.position.ki > 1e-6f && dist < g_chassis_tune_params.position.i_band_m) {
                    s_pos_i += g_chassis_tune_params.position.ki * e_along * CHASSIS_TASK_DT_20MS_S;
                    if (s_pos_i >  g_chassis_tune_params.position.i_limit_mps) s_pos_i =  g_chassis_tune_params.position.i_limit_mps;
                    if (s_pos_i < -g_chassis_tune_params.position.i_limit_mps) s_pos_i = -g_chassis_tune_params.position.i_limit_mps;
                    if ((e_along * s_pos_i) < 0.0f)     s_pos_i = 0.0f;
                }
                float v_along_cmd =     kp_eff * e_along + s_pos_i -     kd_eff * v_along_lpf;
                float v_cross_cmd = cte_kp_eff * e_cross            - cte_kd_eff * v_cross_lpf;

                /* 旋转回全局坐标系: [vxg;vyg] = R^T * [v_along;v_cross] */
                vxg = v_along_cmd * cp - v_cross_cmd * sp;
                vyg = v_along_cmd * sp + v_cross_cmd * cp;
            }
#endif
        }
        norm = sqrtf(vxg * vxg + vyg * vyg);

        /* 接近目标时按位置误差限速, 避免冲过 EPSILON 反弹.
         * sqrt_ctrl 末段会算出 ~0 速度, FLOOR 保证仍能推进 EPSILON 内. */
        {
<<<<<<< HEAD
            if ((CHASSIS_POS_BRAKE_DIST_M > 1e-6f) && (dist < CHASSIS_POS_BRAKE_DIST_M) && (norm > 1e-6f)) {
                float brake_err = dist - CHASSIS_TARGET_REACHED_EPSILON_M;
=======
            /* 不重算: 用径向投影 (vxg*dx+vyg*dy) 等价判方向, 与 d_along 同号 -> 指向目标 */
            float v_align = (vxg * dx + vyg * dy);

            /* BRAKE_DIST 内限速: 用位置误差生成可停住的速度上限, 避免接近目标时先急停再续走. */
            if ((g_chassis_tune_params.position.brake_dist_m > 1e-6f) && (dist < g_chassis_tune_params.position.brake_dist_m) && (norm > 1e-6f)) {
                float brake_err = dist - g_chassis_tune_params.position.target_reached_epsilon_m;
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
                float v_max_brake;
                if (brake_err < 0.0f) brake_err = 0.0f;
                v_max_brake = sqrt_controller(brake_err,
<<<<<<< HEAD
                                              g_chassis_tune_params.pos_kp,
                                              g_chassis_tune_params.cmd_accel_limit_mps2);
                if (v_max_brake < CHASSIS_POS_BRAKE_FLOOR_MPS) {
                    v_max_brake = CHASSIS_POS_BRAKE_FLOOR_MPS;
=======
                                              g_chassis_tune_params.position.kp,
                                              g_chassis_tune_params.limit.accel_limit_mps2);
                /* P0-修复 2026-05-12 (拐点 5s 停留根因):
                 * 末段最小速度地板. brake_err→0 时 v_max_brake→0 把车锁死在 EPSILON 边缘,
                 * 4 麦轮 ≈0.011 m/s/轮 远低于静摩擦突破阈值. V_FLOOR 保证末段始终能推进. */
                if (v_max_brake < g_chassis_tune_params.position.brake_floor_mps) {
                    v_max_brake = g_chassis_tune_params.position.brake_floor_mps;
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
                }
                if (norm > v_max_brake) {
                    float sc = v_max_brake / norm;
                    vxg  *= sc;
                    vyg  *= sc;
                    norm  = v_max_brake;
                }
            }

            if ((s_recovery_active != 0U) &&
                (CHASSIS_POS_RECOVERY_MAX_SPEED_MPS > 1e-6f) &&
                (dist < CHASSIS_POS_RECOVERY_DIST_M) &&
                (norm > CHASSIS_POS_RECOVERY_MAX_SPEED_MPS))
            {
                float recovery_scale = CHASSIS_POS_RECOVERY_MAX_SPEED_MPS / norm;
                vxg *= recovery_scale;
                vyg *= recovery_scale;
                norm = CHASSIS_POS_RECOVERY_MAX_SPEED_MPS;
            }
        }
        if (norm > g_chassis_tune_params.limit.max_linear_speed_mps && norm > 1e-6f) {
            float sc = g_chassis_tune_params.limit.max_linear_speed_mps / norm;
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
        /* s_nav_lock_yaw=1 时由调用者锁定航向, 禁止 atan2 覆盖 (move_to_m 接口).
         * axis-by-axis 模式按全局 X/Y 分段平移, 若跟随 atan2 会在 X->Y 时边旋转边走,
         * 麦轮旋转耦合会被 X 保持环放大成横向偏移; 因此只在 CTE 直线模式更新 yaw. */
#if (CHASSIS_POS_AXIS_BY_AXIS_ENABLE == 0)
        if (!s_nav_lock_yaw && dist > CHASSIS_POS_YAW_TRACK_DIST_M) {
            s_tgt_yaw_deg = atan2f(dy, dx) * CHASSIS_RAD_TO_DEG_F;
        }
#endif
        yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);

        /* 全局 → 车体坐标变换 */
        cmd.vx_body_mps =  cy * vxg + sy * vyg;
        cmd.vy_body_mps = -sy * vxg + cy * vyg;
#if (CHASSIS_POS_AXIS_BY_AXIS_ENABLE != 0)
        if (fabsf(yerr) <= CHASSIS_YAW_GOAL_TOLERANCE_DEG)
        {
            s_yaw_i = 0.0f;
            cmd.wz_dps = 0.0f;
        }
        else
#endif
        {
            /* 平动期间用纯前馈 + 外环 sqrt_ctrl, 不开 rate-loop:
             * IMU 振动噪声进 PI 会被 KP_v=0.15 放大成 ±1.5°/s wz 抖,
             * 投影到麦轮 ≈4.6 mm/s 高频脉动, 表现为车身轻微颤抖.
             * yaw 长期偏差由 axis_by_axis 的 yaw_snap + 起步对齐 + 保持轴 CTE 兜底.
             * 静态保持时另一处仍用 rate PI + in-pos lock, 高精度. */
            cmd.wz_dps = yaw_pi(yerr, 0U, 0U);
        }
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
        /* 静止保持: 完整级联 PI + 允许 in-pos 锁消除极限环 */
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

    apply_speed(cmd, ws_pid);
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
    float path_cos_phi;
    float path_sin_phi;

    pose_read_snapshot(&pose_snap);
    target_x_m = chassis_grid_x_to_m(x);
    target_y_m = chassis_grid_y_to_m(y);

    /* 保存起点→终点路径方向 (供 AXIS_BY_AXIS=0 的 CTE 路径使用) */
    {
        float dx_path = target_x_m - pose_snap.x_m;
        float dy_path = target_y_m - pose_snap.y_m;
        float path_len = sqrtf(dx_path * dx_path + dy_path * dy_path);
        if (path_len > 0.05f) {
            path_cos_phi = dx_path / path_len;
            path_sin_phi = dy_path / path_len;
        } else {
            path_cos_phi = 1.0f;
            path_sin_phi = 0.0f;
        }
    }

    __disable_irq();
    s_tgt_x_m = target_x_m;
    s_tgt_y_m = target_y_m;
    s_axis_hold_x_m = pose_snap.x_m;
    s_axis_hold_y_m = pose_snap.y_m;
#if (CHASSIS_POS_AXIS_BY_AXIS_ENABLE != 0)
    /* P0-修复 2026-05-12 (走斜线根因):
     * 旧版 s_tgt_yaw_deg = pose_snap.yaw_deg (任意起点角), POINT_NAV 期间 yerr=0
     * yaw 不被纠正. 若起点 yaw=5° -> 主轴速度投影到全局产生 vyg=vx*sin(5°)=0.10 m/s
     * Y 漂移, CTE 上限 0.06 m/s 追不上 -> 走斜线.
     *
     * axis-by-axis 模式下底盘只走 X/Y 网格方向, yaw 应锁到最近的 0/90/180/270°,
     * 让全局速度=车体速度, 不再有任何斜投影. 起点偏 ±45° 内会自动 snap, 偏更多则
     * 取最近 90° 倍数. 配合 yaw_pi rate PI 闭环, 起步时 IMU 会快速把 yaw 拉到位
     * (snap_err ≤45°, 以 max_yaw=150°/s 算 ≤0.3s 完成对齐). */
    {
        float yaw_now  = chassis_normalize_angle_deg(pose_snap.yaw_deg);
        float yaw_snap = roundf(yaw_now / 90.0f) * 90.0f;
        s_tgt_yaw_deg  = chassis_normalize_angle_deg(yaw_snap);
    }
#endif
    s_path_cos_phi = path_cos_phi;
    s_path_sin_phi = path_sin_phi;
    enter_mode(MODE_POINT_NAV);
    s_arrived = 0U;
    __enable_irq();
}

void chassis_ctrl_move_to_m(float x_m, float y_m, float hold_yaw_deg)
{
    chassis_pose_t pose_snap;
    float dx_path;
    float dy_path;
    float path_len;
    float yaw_norm = chassis_normalize_angle_deg(hold_yaw_deg);
    float path_cos_phi;
    float path_sin_phi;

    pose_read_snapshot(&pose_snap);
    dx_path = x_m - pose_snap.x_m;
    dy_path = y_m - pose_snap.y_m;
    path_len = sqrtf(dx_path * dx_path + dy_path * dy_path);

    /* 记录直线路径方向 (起点→终点 atan2), 与机器人头部朝向无关.
     * CTE 必须用这个方向做分解, 用 hold_yaw 会把麦轮平移的横向误当成侧偏纳入 CTE, 全算错. */
    if (path_len > 0.05f) {
        path_cos_phi = dx_path / path_len;
        path_sin_phi = dy_path / path_len;
    } else {
        /* 距离过近 (退化): 用目标航向角作为路径方向兑底 */
        float hr = yaw_norm * CHASSIS_DEG_TO_RAD_F;
        path_cos_phi = cosf(hr);
        path_sin_phi = sinf(hr);
    }

    __disable_irq();
    s_tgt_x_m     = x_m;
    s_tgt_y_m     = y_m;
    s_tgt_yaw_deg = yaw_norm;
    s_axis_hold_x_m = pose_snap.x_m;
    s_axis_hold_y_m = pose_snap.y_m;
    s_path_cos_phi = path_cos_phi;
    s_path_sin_phi = path_sin_phi;
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
    /* 与里程计积分同一旋转: new_x += (cy*vx - sy*vy)*dt */
    vxg     = cy * vx_b - sy * vy_b;
    vyg     = sy * vx_b + cy * vy_b;

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
    chassis_config_get_snapshot(out);
}

void chassis_ctrl_set_tune_params(const chassis_tune_params_t *in)
{
    chassis_tune_params_t safe;
    uint8 i;

    if (!in) return;

    safe = *in;
    chassis_config_sanitize_tune(&safe);
    chassis_config_apply(&safe);

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        s_pid[i].kp = safe.wheel_pid.kp[i];
        s_pid[i].ki = safe.wheel_pid.ki[i];
        s_pid[i].kd = safe.wheel_pid.kd[i];
    }
}

uint8 chassis_ctrl_set_wheel_pid_tune(uint8 wheel_index, float kp, float ki, float kd)
{
    chassis_tune_params_t params;

    if (wheel_index >= (uint8)CHASSIS_WHEEL_COUNT)
    {
        return 0U;
    }

    chassis_ctrl_get_tune_params(&params);
    params.wheel_pid.kp[wheel_index] = kp;
    params.wheel_pid.ki[wheel_index] = ki;
    params.wheel_pid.kd[wheel_index] = kd;
    chassis_ctrl_set_tune_params(&params);
    return 1U;
}

/* ==================================================================
 * 【P0-8】发车区 / 越界几何判定 实现
 *   原 1850-2141 行已于 2026-05-13 整体迁出到 chassis_zone.c.
 *   API (chassis_zone_*) / 行为 / 静态变量 / 阈值参数全部等价, 详见 chassis_zone.c.
 * ================================================================== */
