/*===========================================================================
 * [chassis_ctrl.c] 底盘顶层控制模块实现
 *
 *   将 IMU、编码器、电机、PID、麦轮运动学组装成完整控制链路：
 *     IMU → 航向角 → 里程计 → 导航控制 → 运动学 → PID → 电机
 *
 *   本文件管理"胶水"逻辑，包括：
 *     - 四轮硬件配置实例化（引脚在 chassis_config.h 定义）
 *     - 缓加速/速度斜坡滤波
 *     - 定点移动 + 航向保持模式切换
 *     - 里程计位姿追踪
 *===========================================================================*/

#include "chassis_ctrl.h"
#include "chassis_imu.h"
#include "chassis_encoder.h"
#include "chassis_motor.h"
#include "chassis_pid.h"
#include "chassis_mecanum.h"

#include <math.h>

#define CHASSIS_YAW_HOLD_DEADZONE_DEG   (0.30f)

/* ======================================================================
 *  四轮硬件实例（按 LF / RF / LB / RB 顺序排列）
 *
 *  所有引脚分配来自 chassis_config.h，如需修改请到该文件调整宏定义。
 *  这里只是把宏填入结构体，运行时由 chassis_ctrl_init() 统一初始化。
 * ====================================================================== */

/** 四路电机驱动实例 */
static chassis_motor_t s_motors[CHASSIS_WHEEL_COUNT] =
{
    /* 左前轮 LF: PWM=C11, DIR=C10 */
    { CHASSIS_LF_PWM_CHANNEL, CHASSIS_LF_DIR_PIN, CHASSIS_LF_DIR_SIGN },
    /* 右前轮 RF: PWM=C8,  DIR=C9  */
    { CHASSIS_RF_PWM_CHANNEL, CHASSIS_RF_DIR_PIN, CHASSIS_RF_DIR_SIGN },
    /* 左后轮 LB: PWM=D3,  DIR=D2  */
    { CHASSIS_LB_PWM_CHANNEL, CHASSIS_LB_DIR_PIN, CHASSIS_LB_DIR_SIGN },
    /* 右后轮 RB: PWM=C6,  DIR=C7  */
    { CHASSIS_RB_PWM_CHANNEL, CHASSIS_RB_DIR_PIN, CHASSIS_RB_DIR_SIGN },
};

/** 四路编码器实例 */
static chassis_encoder_t s_encoders[CHASSIS_WHEEL_COUNT] =
{
    /* 左前轮 LF: A相=C0,  B相=C1  */
    { CHASSIS_LF_ENC_INDEX, CHASSIS_LF_ENC_CH1, CHASSIS_LF_ENC_CH2, CHASSIS_LF_DIR_SIGN, 0.0f },
    /* 右前轮 RF: A相=C2,  B相=C24 */
    { CHASSIS_RF_ENC_INDEX, CHASSIS_RF_ENC_CH1, CHASSIS_RF_ENC_CH2, CHASSIS_RF_DIR_SIGN, 0.0f },
    /* 左后轮 LB: A相=C3,  B相=C25 */
    { CHASSIS_LB_ENC_INDEX, CHASSIS_LB_ENC_CH1, CHASSIS_LB_ENC_CH2, CHASSIS_LB_DIR_SIGN, 0.0f },
    /* 右后轮 RB: A相=B18, B相=B19 */
    { CHASSIS_RB_ENC_INDEX, CHASSIS_RB_ENC_CH1, CHASSIS_RB_ENC_CH2, CHASSIS_RB_DIR_SIGN, 0.0f },
};

/** 四路轮速 PID 控制器（chassis_ctrl_init 中初始化增益） */
static chassis_pid_t s_pids[CHASSIS_WHEEL_COUNT];

/** 运行时可调参数（默认值来自 chassis_config.h） */
static volatile chassis_tune_params_t s_tune_params =
{
    CHASSIS_WHEEL_PID_KP,
    CHASSIS_WHEEL_PID_KI,
    CHASSIS_WHEEL_PID_KD,
    CHASSIS_POS_KP,
    CHASSIS_YAW_KP,
    CHASSIS_MAX_LINEAR_SPEED_MPS,
    CHASSIS_MAX_YAW_SPEED_DPS,
    CHASSIS_CMD_ACCEL_LIMIT_MPS2,
    CHASSIS_CMD_ACCEL_LIMIT_DPS2,
};

/* ======================================================================
 *  全局位姿与导航状态变量
 * ====================================================================== */

/** 当前位姿估计值（全局坐标系） */
static volatile chassis_pose_t s_pose = { 0.0f, 0.0f, 0.0f };

/** 上一次实际输出的速度指令（调试用） */
static volatile chassis_body_speed_cmd_t s_last_cmd = { 0.0f, 0.0f, 0.0f };

/** 缓加速滤波器的当前状态 */
static volatile chassis_body_speed_cmd_t s_cmd_filtered = { 0.0f, 0.0f, 0.0f };

/** 目标位置（米），由 move_to_grid() 计算写入 */
static volatile float s_target_x_m     = 0.0f;
static volatile float s_target_y_m     = 0.0f;

/** 目标航向角（度），由 hold_yaw() 写入 */
static volatile float s_target_yaw_deg = 0.0f;

/** 运动模式: 1 = 定点移动模式, 0 = 航向保持模式 */
static volatile uint8 s_move_mode_enabled = 0U;

/** 到达标志: 1 = 已到达（或空闲）, 0 = 移动中 */
static volatile uint8 s_arrived_flag = 1U;

/** 里程计反解的车体速度（车体坐标系） */
static volatile float s_body_vx_mps_feedback = 0.0f;
static volatile float s_body_vy_mps_feedback = 0.0f;

static chassis_tune_params_t ctrl_sanitize_tune_params(chassis_tune_params_t in)
{
    in.wheel_pid_kp          = chassis_clamp_f(in.wheel_pid_kp,          0.0f, 400.0f);
    in.wheel_pid_ki          = chassis_clamp_f(in.wheel_pid_ki,          0.0f, 80.0f);
    in.wheel_pid_kd          = chassis_clamp_f(in.wheel_pid_kd,          0.0f, 40.0f);
    in.pos_kp                = chassis_clamp_f(in.pos_kp,                0.0f, 5.0f);
    in.yaw_kp                = chassis_clamp_f(in.yaw_kp,                0.0f, 10.0f);
    in.max_linear_speed_mps  = chassis_clamp_f(in.max_linear_speed_mps,  0.05f, CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS);
    in.max_yaw_speed_dps     = chassis_clamp_f(in.max_yaw_speed_dps,     10.0f, CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS);
    in.cmd_accel_limit_mps2  = chassis_clamp_f(in.cmd_accel_limit_mps2,  0.10f, 5.00f);
    in.cmd_accel_limit_dps2  = chassis_clamp_f(in.cmd_accel_limit_dps2, 20.0f, 1000.0f);
    return in;
}

/* ======================================================================
 *  内部辅助函数
 * ====================================================================== */

/**
 * @brief  车体速度矢量限幅
 *         (vx, vy) 模长不超过 MAX_LINEAR_SPEED，wz 单独限幅。
 *         避免对角线移动时合成速度超限。
 */
static void ctrl_limit_body_speed(chassis_body_speed_cmd_t *cmd)
{
    float linear_norm;
    float linear_scale;
    float max_yaw_speed_dps = s_tune_params.max_yaw_speed_dps;
    float max_linear_speed_mps = s_tune_params.max_linear_speed_mps;

    /* 角速度独立限幅 */
    cmd->wz_dps = chassis_clamp_f(cmd->wz_dps,
                                  -max_yaw_speed_dps,
                                   max_yaw_speed_dps);

    /* 线速度矢量模长限幅 */
    linear_norm = sqrtf(cmd->vx_body_mps * cmd->vx_body_mps
                      + cmd->vy_body_mps * cmd->vy_body_mps);
    if (linear_norm > max_linear_speed_mps && linear_norm > 1e-6f)
    {
        linear_scale = max_linear_speed_mps / linear_norm;
        cmd->vx_body_mps *= linear_scale;
        cmd->vy_body_mps *= linear_scale;
    }
}

/**
 * @brief  缓加速斜坡滤波器
 *         每 20ms 周期最多允许速度变化 accel_limit × dt，
 *         防止目标突变导致轮胎打滑。
 */
static chassis_body_speed_cmd_t ctrl_filter_body_speed(chassis_body_speed_cmd_t target_cmd)
{
    float delta_linear_max = s_tune_params.cmd_accel_limit_mps2 * CHASSIS_TASK_DT_20MS_S;
    float delta_yaw_max    = s_tune_params.cmd_accel_limit_dps2 * CHASSIS_TASK_DT_20MS_S;
    chassis_body_speed_cmd_t filtered;

    /* 先对目标做矢量限幅 */
    ctrl_limit_body_speed(&target_cmd);

    /* 线性斜坡：每个分量限制单步最大变化量 */
    filtered.vx_body_mps = chassis_clamp_f(
        target_cmd.vx_body_mps,
        s_cmd_filtered.vx_body_mps - delta_linear_max,
        s_cmd_filtered.vx_body_mps + delta_linear_max);

    filtered.vy_body_mps = chassis_clamp_f(
        target_cmd.vy_body_mps,
        s_cmd_filtered.vy_body_mps - delta_linear_max,
        s_cmd_filtered.vy_body_mps + delta_linear_max);

    filtered.wz_dps = chassis_clamp_f(
        target_cmd.wz_dps,
        s_cmd_filtered.wz_dps - delta_yaw_max,
        s_cmd_filtered.wz_dps + delta_yaw_max);

    /* 斜坡后再次限幅，确保不超限 */
    ctrl_limit_body_speed(&filtered);

    /* 保存滤波状态供下一周期使用 */
    s_cmd_filtered = filtered;
    return filtered;
}

/**
 * @brief  紧急停机：清零滤波器和 PID，立即切断全部 PWM
 */
static void ctrl_force_stop(void)
{
    uint8 i;

    s_cmd_filtered = (chassis_body_speed_cmd_t){ 0.0f, 0.0f, 0.0f };
    s_last_cmd     = (chassis_body_speed_cmd_t){ 0.0f, 0.0f, 0.0f };

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        chassis_pid_reset(&s_pids[i]);
        chassis_motor_stop(&s_motors[i]);
    }
}

/**
 * @brief  执行车体速度指令
 *         完整流程：滤波 → 正运动学 → 轮速限幅 → PID → 电机
 */
static void ctrl_apply_body_speed(chassis_body_speed_cmd_t cmd)
{
    chassis_body_speed_cmd_t filtered;
    float wz_radps;
    float wheel_targets[CHASSIS_WHEEL_COUNT];
    float pwm_signed;
    uint8 i;

    /* 步骤 1: 缓加速滤波 */
    filtered = ctrl_filter_body_speed(cmd);
    s_last_cmd = filtered;

    /* 步骤 2: 角速度单位转换 °/s → rad/s */
    wz_radps = filtered.wz_dps * CHASSIS_DEG_TO_RAD_F;

    /* 步骤 3: 麦轮正运动学 → 四轮目标线速度 */
    chassis_mecanum_forward(filtered.vx_body_mps, filtered.vy_body_mps,
                            wz_radps, wheel_targets);

    /* 步骤 4: 四轮等比例限速 */
    chassis_mecanum_clamp_wheels(wheel_targets, CHASSIS_MAX_WHEEL_SPEED_MPS);

    /* 步骤 5: 四轮独立 PID 控制 + 电机 PWM 输出 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        /*
         * 乘以方向修正系数，使 PID 目标方向与编码器反馈方向一致。
         * dir_sign 来自 chassis_config.h 中每轮的 DIR_SIGN 宏。
         */
        float target_mps = wheel_targets[i] * s_motors[i].dir_sign;

        pwm_signed = chassis_pid_step(&s_pids[i], target_mps,
                                      chassis_encoder_get_speed(&s_encoders[i]));

        chassis_motor_set_pwm(&s_motors[i], pwm_signed);
    }
}

/* ========================== 公共 API 实现 ========================== */

void chassis_ctrl_init(void)
{
    uint8 i;

    /* 上电时先应用一次安全限幅，防止默认参数过大。 */
    s_tune_params = ctrl_sanitize_tune_params(s_tune_params);

    /* 初始化 IMU（若失败，5ms 任务会自动跳过 IMU 读取） */
    chassis_imu_init();

    /* 初始化四路电机、编码器和 PID 控制器 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        chassis_motor_init(&s_motors[i]);
        chassis_encoder_init(&s_encoders[i]);
        chassis_pid_init(&s_pids[i],
                         s_tune_params.wheel_pid_kp,
                         s_tune_params.wheel_pid_ki,
                         s_tune_params.wheel_pid_kd,
                         CHASSIS_MOTOR_PWM_MAX);
    }

    /* 重置所有运行状态 */
    s_pose = (chassis_pose_t){ 0.0f, 0.0f, 0.0f };
    s_target_x_m       = 0.0f;
    s_target_y_m       = 0.0f;
    s_target_yaw_deg   = 0.0f;
    s_move_mode_enabled = 0U;
    s_arrived_flag      = 1U;
    s_body_vx_mps_feedback = 0.0f;
    s_body_vy_mps_feedback = 0.0f;
    s_cmd_filtered = (chassis_body_speed_cmd_t){ 0.0f, 0.0f, 0.0f };
    s_last_cmd     = (chassis_body_speed_cmd_t){ 0.0f, 0.0f, 0.0f };

    /* 确保全部电机停止 */
    ctrl_force_stop();
}

void chassis_ctrl_task_5ms(void)
{
    /* IMU 姿态采样与航向角积分 */
    chassis_imu_update_5ms();

    /* 同步航向角到位姿结构体，保证 get_pose() 实时性 */
    s_pose.yaw_deg = chassis_imu_get_yaw_deg();
}

void chassis_ctrl_task_20ms(void)
{
    float yaw_rad, cos_yaw, sin_yaw;
    float wheel_speeds[CHASSIS_WHEEL_COUNT];
    float vx_body_raw, vy_body_raw;
    uint8 i;

    /* --- 步骤 1: 读取全部编码器，更新四轮速度 --- */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        chassis_encoder_update(&s_encoders[i], CHASSIS_TASK_DT_20MS_S);
        wheel_speeds[i] = chassis_encoder_get_speed(&s_encoders[i]);
    }

    /* --- 步骤 2: 麦轮逆运动学 → 车体速度 --- */
    chassis_mecanum_inverse(wheel_speeds, &vx_body_raw, &vy_body_raw);
    s_body_vx_mps_feedback = vx_body_raw * CHASSIS_ODOM_SCALE_X;
    s_body_vy_mps_feedback = vy_body_raw * CHASSIS_ODOM_SCALE_Y;

    /* --- 步骤 3: 里程计积分 → 更新全局位姿 (x, y) --- */
    yaw_rad = s_pose.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    {
        /* 车体速度 → 全局速度（2D 旋转变换） */
        float vx_global = cos_yaw * s_body_vx_mps_feedback - sin_yaw * s_body_vy_mps_feedback;
        float vy_global = sin_yaw * s_body_vx_mps_feedback + cos_yaw * s_body_vy_mps_feedback;

        s_pose.x_m += vx_global * CHASSIS_TASK_DT_20MS_S;
        s_pose.y_m += vy_global * CHASSIS_TASK_DT_20MS_S;
    }

    /* --- 步骤 4: 导航控制 → 输出车体速度指令 --- */
    if (0U != s_move_mode_enabled)
    {
        /* ---- 定点移动模式：同时控制位置和航向 ---- */
        float dx = s_target_x_m - s_pose.x_m;
        float dy = s_target_y_m - s_pose.y_m;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist <= CHASSIS_TARGET_REACHED_EPSILON_M)
        {
            /* 到达目标点 → 切换到停止状态 */
            s_arrived_flag = 1U;
            s_move_mode_enabled = 0U;
            ctrl_force_stop();
            return;
        }

        {
            /* 位置 P 控制器：误差 × Kp → 全局坐标系速度指令 */
            float vx_global_cmd = s_tune_params.pos_kp * dx;
            float vy_global_cmd = s_tune_params.pos_kp * dy;
            float linear_norm = sqrtf(vx_global_cmd * vx_global_cmd + vy_global_cmd * vy_global_cmd);
            if (linear_norm > s_tune_params.max_linear_speed_mps && linear_norm > 1e-6f)
            {
                float scale = s_tune_params.max_linear_speed_mps / linear_norm;
                vx_global_cmd *= scale;
                vy_global_cmd *= scale;
            }

            /* 航向 P 控制器：对准前进方向 */
            {
                float target_yaw = atan2f(dy, dx) * CHASSIS_RAD_TO_DEG_F;
                float yaw_err = chassis_normalize_angle_deg(target_yaw - s_pose.yaw_deg);
                float wz_cmd = chassis_clamp_f(s_tune_params.yaw_kp * yaw_err,
                                               -s_tune_params.max_yaw_speed_dps,
                                                s_tune_params.max_yaw_speed_dps);

                /* 全局速度 → 车体速度（逆旋转变换） */
                chassis_body_speed_cmd_t cmd;
                cmd.vx_body_mps =  cos_yaw * vx_global_cmd + sin_yaw * vy_global_cmd;
                cmd.vy_body_mps = -sin_yaw * vx_global_cmd + cos_yaw * vy_global_cmd;
                cmd.wz_dps = wz_cmd;

                ctrl_apply_body_speed(cmd);
            }
        }
    }
    else
    {
        /* ---- 航向保持模式：原地转向，不平移 ---- */
        float yaw_err = chassis_normalize_angle_deg(s_target_yaw_deg - s_pose.yaw_deg);
        float wz_cmd;

        /* 在零点附近增加小死区，避免静止时高频抖动。 */
        if (fabsf(yaw_err) <= CHASSIS_YAW_HOLD_DEADZONE_DEG)
        {
            wz_cmd = 0.0f;
        }
        else
        {
            wz_cmd = chassis_clamp_f(s_tune_params.yaw_kp * yaw_err,
                                     -s_tune_params.max_yaw_speed_dps,
                                      s_tune_params.max_yaw_speed_dps);
        }

        ctrl_apply_body_speed((chassis_body_speed_cmd_t){ 0.0f, 0.0f, wz_cmd });
    }
}

void chassis_ctrl_move_to_grid(uint8 target_x_grid, uint8 target_y_grid)
{
    /* 边界保护：超出网格范围自动钳位 */
    uint8 x = (target_x_grid > CHASSIS_GRID_MAX_X) ? (uint8)CHASSIS_GRID_MAX_X : target_x_grid;
    uint8 y = (target_y_grid > CHASSIS_GRID_MAX_Y) ? (uint8)CHASSIS_GRID_MAX_Y : target_y_grid;

    /* 网格坐标 → 物理坐标：以左上角为原点，每格 20cm */
    s_target_x_m = (float)x * CHASSIS_GRID_CELL_SIZE_M;
    s_target_y_m = (float)y * CHASSIS_GRID_CELL_SIZE_M;

    s_move_mode_enabled = 1U;
    s_arrived_flag = 0U;
}

void chassis_ctrl_hold_yaw(float target_yaw_deg)
{
    s_target_yaw_deg = chassis_normalize_angle_deg(target_yaw_deg);
    s_move_mode_enabled = 0U;
    s_arrived_flag = 1U;
}

void chassis_ctrl_attitude_debug_start_zero(void)
{
    /* 清理历史速度与 PID 状态，避免从运动态切换到锁角时出现残余冲击。 */
    ctrl_force_stop();
    chassis_ctrl_hold_yaw(0.0f);
}

void chassis_ctrl_attitude_debug_get_state(chassis_attitude_debug_info_t *out_info)
{
    if (0 == out_info)
    {
        return;
    }

    out_info->target_yaw_deg  = s_target_yaw_deg;
    out_info->current_yaw_deg = s_pose.yaw_deg;
    out_info->yaw_err_deg     = chassis_normalize_angle_deg(s_target_yaw_deg - s_pose.yaw_deg);
    out_info->wz_cmd_dps      = s_last_cmd.wz_dps;
}

void chassis_ctrl_attitude_debug_task_5ms(void)
{
    static uint8 s_print_div = 0U;
    chassis_attitude_debug_info_t info;

    s_print_div++;
    if (s_print_div < 20U)
    {
        return;
    }
    s_print_div = 0U;

    chassis_ctrl_attitude_debug_get_state(&info);
    printf("[YawHold] target=%.2f yaw=%.2f err=%.2f wz_cmd=%.2f\r\n",
           info.target_yaw_deg,
           info.current_yaw_deg,
           info.yaw_err_deg,
           info.wz_cmd_dps);
}

uint8 chassis_ctrl_is_arrived(void)
{
    return s_arrived_flag;
}

chassis_pose_t chassis_ctrl_get_pose(void)
{
    chassis_pose_t copy;
    copy.x_m     = s_pose.x_m;
    copy.y_m     = s_pose.y_m;
    copy.yaw_deg = s_pose.yaw_deg;
    return copy;
}

chassis_body_speed_cmd_t chassis_ctrl_get_last_cmd(void)
{
    chassis_body_speed_cmd_t copy;
    copy.vx_body_mps = s_last_cmd.vx_body_mps;
    copy.vy_body_mps = s_last_cmd.vy_body_mps;
    copy.wz_dps      = s_last_cmd.wz_dps;
    return copy;
}

void chassis_ctrl_stop(void)
{
    s_move_mode_enabled = 0U;
    s_arrived_flag = 1U;
    ctrl_force_stop();
}

void chassis_ctrl_set_pose(float x_m, float y_m, float yaw_deg)
{
    s_pose.x_m     = x_m;
    s_pose.y_m     = y_m;
    s_pose.yaw_deg = chassis_normalize_angle_deg(yaw_deg);

    /* 同步航向角到 IMU 模块，避免下一次 5ms 更新覆盖校正值 */
    chassis_imu_set_yaw_deg(yaw_deg);
}

void chassis_ctrl_get_tune_params(chassis_tune_params_t *out_params)
{
    if (0 == out_params)
    {
        return;
    }
    *out_params = s_tune_params;
}

void chassis_ctrl_set_tune_params(const chassis_tune_params_t *in_params)
{
    uint8 i;
    chassis_tune_params_t sanitized;

    if (0 == in_params)
    {
        return;
    }

    sanitized = ctrl_sanitize_tune_params(*in_params);
    s_tune_params = sanitized;

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        s_pids[i].kp = sanitized.wheel_pid_kp;
        s_pids[i].ki = sanitized.wheel_pid_ki;
        s_pids[i].kd = sanitized.wheel_pid_kd;
    }
}
