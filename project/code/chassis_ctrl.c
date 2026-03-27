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

/* ---------------------- 航向闭环常量 ---------------------- */
#define YAW_DEADZONE_DEG   0.30f    /* 航向误差死区               */
#define YAW_KI             0.030f   /* 航向 I 增益                */
#define YAW_I_LIMIT        120.0f   /* 积分限幅 (°·s)             */
#define YAW_MIN_WZ_DPS     10.0f    /* 原地保持最小角速度补偿     */

/* ---------------------- 控制模式 ---------------------- */
typedef enum {
    MODE_YAW_HOLD = 0,      /* 原地航向保持（默认）            */
    MODE_POINT_NAV,         /* 网格点位导航                    */
    MODE_MOVE_YAW,          /* 外部平移 + 航向指令             */
    MODE_ATT_DEBUG          /* 航向闭环调试（行为同 YAW_HOLD） */
} ctrl_mode_t;

/* ====================== 硬件实例 ====================== */

static chassis_motor_t s_mot[CHASSIS_WHEEL_COUNT] = {
    { CHASSIS_LF_PWM_CHANNEL, CHASSIS_LF_DIR_PIN, CHASSIS_LF_DIR_SIGN },  /* LF */
    { CHASSIS_RF_PWM_CHANNEL, CHASSIS_RF_DIR_PIN, CHASSIS_RF_DIR_SIGN },  /* RF */
    { CHASSIS_LB_PWM_CHANNEL, CHASSIS_LB_DIR_PIN, CHASSIS_LB_DIR_SIGN },  /* LB */
    { CHASSIS_RB_PWM_CHANNEL, CHASSIS_RB_DIR_PIN, CHASSIS_RB_DIR_SIGN },  /* RB */
};

static chassis_encoder_t s_enc[CHASSIS_WHEEL_COUNT] = {
    { CHASSIS_LF_ENC_INDEX, CHASSIS_LF_ENC_CH1, CHASSIS_LF_ENC_CH2, CHASSIS_LF_DIR_SIGN, 0.0f },
    { CHASSIS_RF_ENC_INDEX, CHASSIS_RF_ENC_CH1, CHASSIS_RF_ENC_CH2, CHASSIS_RF_DIR_SIGN, 0.0f },
    { CHASSIS_LB_ENC_INDEX, CHASSIS_LB_ENC_CH1, CHASSIS_LB_ENC_CH2, CHASSIS_LB_DIR_SIGN, 0.0f },
    { CHASSIS_RB_ENC_INDEX, CHASSIS_RB_ENC_CH1, CHASSIS_RB_ENC_CH2, CHASSIS_RB_DIR_SIGN, 0.0f },
};

static chassis_pid_t s_pid[CHASSIS_WHEEL_COUNT];

/* ====================== 运行时状态 ====================== */

/* 可调参数（默认值来自 chassis_config.h） */
static volatile chassis_tune_params_t s_tune = {
    CHASSIS_WHEEL_PID_KP, CHASSIS_WHEEL_PID_KI, CHASSIS_WHEEL_PID_KD,
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

/* 航向积分项 */
static volatile float s_yaw_i  = 0.0f;

/* 里程计反馈 */
static volatile float s_fb_vx  = 0.0f;
static volatile float s_fb_vy  = 0.0f;

/* ====================== 内部工具函数 ====================== */

/** 参数安全限幅，防止异常值进入控制链 */
static chassis_tune_params_t sanitize(chassis_tune_params_t p)
{
    p.wheel_pid_kp         = chassis_clamp_f(p.wheel_pid_kp,         0.0f,  400.0f);
    p.wheel_pid_ki         = chassis_clamp_f(p.wheel_pid_ki,         0.0f,   80.0f);
    p.wheel_pid_kd         = chassis_clamp_f(p.wheel_pid_kd,         0.0f,   40.0f);
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
                                -s_tune.max_yaw_speed_dps,
                                 s_tune.max_yaw_speed_dps);

    norm = sqrtf(c->vx_body_mps * c->vx_body_mps +
                 c->vy_body_mps * c->vy_body_mps);
    if (norm > s_tune.max_linear_speed_mps && norm > 1e-6f) {
        float s = s_tune.max_linear_speed_mps / norm;
        c->vx_body_mps *= s;
        c->vy_body_mps *= s;
    }
}

/** 缓加速斜坡滤波: 每 20ms 周期限制速度变化量，防止轮胎打滑 */
static chassis_body_speed_cmd_t ramp_filter(chassis_body_speed_cmd_t tgt)
{
    const float dv = s_tune.cmd_accel_limit_mps2 * CHASSIS_TASK_DT_20MS_S;
    const float dw = s_tune.cmd_accel_limit_dps2 * CHASSIS_TASK_DT_20MS_S;
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
        chassis_pid_reset(&s_pid[i]);
        chassis_motor_stop(&s_mot[i]);
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
        float pwm = chassis_pid_step(
            &s_pid[i],
            targets[i] * s_mot[i].dir_sign,         /* 方向修正后的目标 */
            wheel_fb_mps[i]);                        /* 本周期真实编码器反馈 */
        chassis_motor_set_pwm(&s_mot[i], pwm);
    }
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

    wz = s_tune.yaw_kp * err + YAW_KI * s_yaw_i;
    wz = chassis_clamp_f(wz, -s_tune.max_yaw_speed_dps, s_tune.max_yaw_speed_dps);

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

    s_tune = sanitize(s_tune);
    chassis_imu_init();

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        chassis_motor_init(&s_mot[i]);
        chassis_encoder_init(&s_enc[i]);
        chassis_pid_init(&s_pid[i],
                         s_tune.wheel_pid_kp,
                         s_tune.wheel_pid_ki,
                         s_tune.wheel_pid_kd,
                         CHASSIS_MOTOR_PWM_MAX);
    }

    s_pose         = (chassis_pose_t){0};
    s_tgt_x_m      = 0.0f;
    s_tgt_y_m      = 0.0f;
    s_tgt_yaw_deg  = 0.0f;
    s_cmd_vx       = 0.0f;
    s_cmd_vy       = 0.0f;
    s_fb_vx        = 0.0f;
    s_fb_vy        = 0.0f;
    s_mode         = MODE_YAW_HOLD;
    s_arrived      = 1U;

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
    float ws[CHASSIS_WHEEL_COUNT];     /* 四轮速度反馈 */
    float vx_raw, vy_raw;
    float yaw_rad, cy, sy;
    chassis_body_speed_cmd_t cmd = {0};
    uint8 i;

    /* 1) 编码器采样 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        chassis_encoder_update(&s_enc[i], CHASSIS_TASK_DT_20MS_S);
        ws[i] = chassis_encoder_get_speed(&s_enc[i]);
    }

    /* 2) 逆运动学 → 车体速度反馈 */
    chassis_mecanum_inverse(ws, &vx_raw, &vy_raw);
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
        float vxg  = s_tune.pos_kp * dx;
        float vyg  = s_tune.pos_kp * dy;
        float norm = sqrtf(vxg * vxg + vyg * vyg);
        if (norm > s_tune.max_linear_speed_mps && norm > 1e-6f) {
            float sc = s_tune.max_linear_speed_mps / norm;
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
        cmd.wz_dps      = chassis_clamp_f(s_tune.yaw_kp * yerr,
                                          -s_tune.max_yaw_speed_dps,
                                           s_tune.max_yaw_speed_dps);
        break;
    }

    case MODE_MOVE_YAW: {
        float yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
        cmd.vx_body_mps = s_cmd_vx;
        cmd.vy_body_mps = s_cmd_vy;
        cmd.wz_dps      = yaw_pi(yerr, 0U);  /* 不补偿最小角速度 */
        break;
    }

    default: {  /* MODE_YAW_HOLD / MODE_ATT_DEBUG */
        float yerr = chassis_normalize_angle_deg(s_tgt_yaw_deg - s_pose.yaw_deg);
        cmd.vx_body_mps = 0.0f;
        cmd.vy_body_mps = 0.0f;
        cmd.wz_dps      = yaw_pi(yerr, 1U);  /* 补偿静摩擦 */
        break;
    }
    }

    apply_speed(cmd, ws);
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
    *out = s_tune;
}

void chassis_ctrl_set_tune_params(const chassis_tune_params_t *in)
{
    chassis_tune_params_t safe;
    uint8 i;

    if (!in) return;

    safe   = sanitize(*in);
    s_tune = safe;

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i) {
        s_pid[i].kp = safe.wheel_pid_kp;
        s_pid[i].ki = safe.wheel_pid_ki;
        s_pid[i].kd = safe.wheel_pid_kd;
    }
}