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

/* 航向闭环参数：用于原地锁角和“移动+航向指令”模式。 */
#define CHASSIS_YAW_HOLD_DEADZONE_DEG   (0.30f)
#define CHASSIS_YAW_HOLD_KI              (0.030f)
#define CHASSIS_YAW_I_LIMIT_DEG_S        (120.0f)

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
static chassis_pid_t s_pids[CHASSIS_WHEEL_COUNT]; /* 四轮 PID 状态与参数 */

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

/** 航向闭环调试强制模式: 1 = 20ms 周期强制走原地航向闭环, 0 = 正常模式 */
static volatile uint8 s_attitude_debug_force_enabled = 0U;

/** 到达标志: 1 = 已到达（或空闲）, 0 = 移动中 */
static volatile uint8 s_arrived_flag = 1U;

/** 里程计反解的车体速度（车体坐标系） */
static volatile float s_body_vx_mps_feedback = 0.0f;
static volatile float s_body_vy_mps_feedback = 0.0f;

/** 外部“移动+航向”指令模式开关: 1=启用, 0=关闭 */
static volatile uint8 s_move_yaw_cmd_mode_enabled = 0U;

/** 外部“移动+航向”指令中的车体系速度目标 */
static volatile float s_cmd_vx_body_mps = 0.0f;
static volatile float s_cmd_vy_body_mps = 0.0f;

/** 航向误差积分（单位：度*秒），用于航向闭环时消除小稳态误差 */
static volatile float s_yaw_err_i_accum = 0.0f;

/**
 * @brief  运行时调参参数安全限幅
 * @param  in  输入参数副本
 * @return 限幅后的参数副本
 * @note   所有外部/菜单写入参数都要先经过本函数，避免失稳参数进入控制链。
 */
static chassis_tune_params_t ctrl_sanitize_tune_params(chassis_tune_params_t in)
{
    /* 对每个可调参数做安全限幅，防止异常参数导致控制失稳。 */
    in.wheel_pid_kp          = chassis_clamp_f(in.wheel_pid_kp,          0.0f, 400.0f); /* 限幅轮速 Kp */
    in.wheel_pid_ki          = chassis_clamp_f(in.wheel_pid_ki,          0.0f, 80.0f);  /* 限幅轮速 Ki */
    in.wheel_pid_kd          = chassis_clamp_f(in.wheel_pid_kd,          0.0f, 40.0f);  /* 限幅轮速 Kd */
    in.pos_kp                = chassis_clamp_f(in.pos_kp,                0.0f, 5.0f);   /* 限幅位置环 Kp */
    in.yaw_kp                = chassis_clamp_f(in.yaw_kp,                0.0f, 10.0f);  /* 限幅航向环 Kp */
    in.max_linear_speed_mps  = chassis_clamp_f(in.max_linear_speed_mps,  0.05f, CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS); /* 限幅线速度上限 */
    in.max_yaw_speed_dps     = chassis_clamp_f(in.max_yaw_speed_dps,     10.0f, CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS);    /* 限幅角速度上限 */
    in.cmd_accel_limit_mps2  = chassis_clamp_f(in.cmd_accel_limit_mps2,  0.10f, 5.00f);  /* 限幅线加速度上限 */
    in.cmd_accel_limit_dps2  = chassis_clamp_f(in.cmd_accel_limit_dps2, 20.0f, 1000.0f); /* 限幅角加速度上限 */
    return in; /* 返回限幅后的参数副本 */
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
    float linear_norm;                               /* 当前线速度矢量模长 */
    float linear_scale;                              /* 线速度缩放比例 */
    float max_yaw_speed_dps = s_tune_params.max_yaw_speed_dps;       /* 角速度上限 */
    float max_linear_speed_mps = s_tune_params.max_linear_speed_mps; /* 线速度上限 */

    /* 角速度独立限幅 */
    cmd->wz_dps = chassis_clamp_f(cmd->wz_dps,
                                  -max_yaw_speed_dps,
                                   max_yaw_speed_dps); /* 对角速度做上下限裁剪 */

    /* 线速度矢量模长限幅 */
    linear_norm = sqrtf(cmd->vx_body_mps * cmd->vx_body_mps
                      + cmd->vy_body_mps * cmd->vy_body_mps); /* 计算线速度模长 */
    if (linear_norm > max_linear_speed_mps && linear_norm > 1e-6f)
    {
        linear_scale = max_linear_speed_mps / linear_norm; /* 计算统一缩放比例 */
        cmd->vx_body_mps *= linear_scale; /* 缩放 vx 分量 */
        cmd->vy_body_mps *= linear_scale; /* 缩放 vy 分量 */
    }
}

/**
 * @brief  缓加速斜坡滤波器
 *         每 20ms 周期最多允许速度变化 accel_limit × dt，
 *         防止目标突变导致轮胎打滑。
 */
static chassis_body_speed_cmd_t ctrl_filter_body_speed(chassis_body_speed_cmd_t target_cmd)
{
    float delta_linear_max = s_tune_params.cmd_accel_limit_mps2 * CHASSIS_TASK_DT_20MS_S; /* 20ms 内线速度最大增量 */
    float delta_yaw_max    = s_tune_params.cmd_accel_limit_dps2 * CHASSIS_TASK_DT_20MS_S; /* 20ms 内角速度最大增量 */
    chassis_body_speed_cmd_t filtered; /* 本周期滤波后的输出 */

    /* 先对目标做矢量限幅 */
    ctrl_limit_body_speed(&target_cmd); /* 先做目标限幅，避免滤波跟踪过大目标 */

    /* 线性斜坡：每个分量限制单步最大变化量 */
    filtered.vx_body_mps = chassis_clamp_f(
        target_cmd.vx_body_mps,
        s_cmd_filtered.vx_body_mps - delta_linear_max,
        s_cmd_filtered.vx_body_mps + delta_linear_max); /* vx 在单周期允许变化范围内爬坡 */

    filtered.vy_body_mps = chassis_clamp_f(
        target_cmd.vy_body_mps,
        s_cmd_filtered.vy_body_mps - delta_linear_max,
        s_cmd_filtered.vy_body_mps + delta_linear_max); /* vy 在单周期允许变化范围内爬坡 */

    filtered.wz_dps = chassis_clamp_f(
        target_cmd.wz_dps,
        s_cmd_filtered.wz_dps - delta_yaw_max,
        s_cmd_filtered.wz_dps + delta_yaw_max); /* wz 在单周期允许变化范围内爬坡 */

    /* 斜坡后再次限幅，确保不超限 */
    ctrl_limit_body_speed(&filtered); /* 二次限幅，保证最终输出合法 */

    /* 保存滤波状态供下一周期使用 */
    s_cmd_filtered = filtered; /* 记录本次输出作为下次滤波初值 */
    return filtered;           /* 返回本周期最终速度命令 */
}

/**
 * @brief  紧急停机：清零滤波器和 PID，立即切断全部 PWM
 */
static void ctrl_force_stop(void)
{
    uint8 i; /* 轮序号：遍历四路电机与 PID */

    s_cmd_filtered = (chassis_body_speed_cmd_t){ 0.0f, 0.0f, 0.0f }; /* 清空滤波状态 */
    s_last_cmd     = (chassis_body_speed_cmd_t){ 0.0f, 0.0f, 0.0f }; /* 清空最近输出 */
    s_yaw_err_i_accum = 0.0f; /* 清空航向积分项，避免模式切换后残余积分冲击 */

    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        chassis_pid_reset(&s_pids[i]);   /* 清空该轮 PID 积分/微分状态 */
        chassis_motor_stop(&s_motors[i]); /* 该轮 PWM 立即清零 */
    }
}

/**
 * @brief  执行车体速度指令
 *         完整流程：滤波 → 正运动学 → 轮速限幅 → PID → 电机
 */
static void ctrl_apply_body_speed(chassis_body_speed_cmd_t cmd)
{
    chassis_body_speed_cmd_t filtered;            /* 缓加速后的车体速度指令 */
    float wz_radps;                               /* 角速度(rad/s)，供运动学计算 */
    float wheel_targets[CHASSIS_WHEEL_COUNT];     /* 四轮目标线速度 */
    float pwm_signed;                             /* 单轮 PID 输出 PWM */
    uint8 i;                                      /* 轮序号 */

    /* 步骤 1: 缓加速滤波 */
    filtered = ctrl_filter_body_speed(cmd); /* 执行缓加速滤波 */
    s_last_cmd = filtered;                   /* 缓存最终下发命令用于调试读取 */

    /* 步骤 2: 角速度单位转换 °/s → rad/s */
    wz_radps = filtered.wz_dps * CHASSIS_DEG_TO_RAD_F; /* 度每秒转换为弧度每秒 */

    /* 步骤 3: 麦轮正运动学 → 四轮目标线速度 */
    chassis_mecanum_forward(filtered.vx_body_mps, filtered.vy_body_mps,
                            wz_radps, wheel_targets); /* 车体速度映射到四轮速度 */

    /* 步骤 4: 四轮等比例限速 */
    chassis_mecanum_clamp_wheels(wheel_targets, CHASSIS_MAX_WHEEL_SPEED_MPS); /* 四轮统一限速 */

    /* 步骤 5: 四轮独立 PID 控制 + 电机 PWM 输出 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        /*
         * 乘以方向修正系数，使 PID 目标方向与编码器反馈方向一致。
         * dir_sign 来自 chassis_config.h 中每轮的 DIR_SIGN 宏。
         */
        float target_mps = wheel_targets[i] * s_motors[i].dir_sign; /* 目标速度乘方向修正 */

        pwm_signed = chassis_pid_step(&s_pids[i], target_mps,
                                      chassis_encoder_get_speed(&s_encoders[i])); /* PID 计算 PWM */

        chassis_motor_set_pwm(&s_motors[i], pwm_signed); /* 输出该轮 PWM */
    }
}

/* ========================== 公共 API 实现 ========================== */

/**
 * @brief  底盘控制模块初始化
 * @note   初始化顺序：
 *         1) 对默认调参做安全限幅
 *         2) 初始化 IMU/电机/编码器/PID
 *         3) 清零位姿、目标、滤波器与模式状态
 *         4) 强制停机，确保上电后无误动作
 */
void chassis_ctrl_init(void)
{
    uint8 i; /* 轮序号：初始化四路硬件 */

    /* 上电时先应用一次安全限幅，防止默认参数过大。 */
    s_tune_params = ctrl_sanitize_tune_params(s_tune_params); /* 对默认参数做启动限幅 */

    /* 初始化 IMU（若失败，5ms 任务会自动跳过 IMU 读取） */
    chassis_imu_init(); /* 初始化 IMU 设备和内部状态 */

    /* 初始化四路电机、编码器和 PID 控制器 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        chassis_motor_init(&s_motors[i]); /* 初始化该轮 PWM/DIR */
        chassis_encoder_init(&s_encoders[i]); /* 初始化该轮编码器 */
        chassis_pid_init(&s_pids[i],
                         s_tune_params.wheel_pid_kp,
                         s_tune_params.wheel_pid_ki,
                         s_tune_params.wheel_pid_kd,
                 CHASSIS_MOTOR_PWM_MAX); /* 初始化该轮 PID 参数和输出限幅 */
    }

    /* 重置所有运行状态 */
    s_pose = (chassis_pose_t){ 0.0f, 0.0f, 0.0f }; /* 清零位姿 */
    s_target_x_m       = 0.0f; /* 清零目标 X */
    s_target_y_m       = 0.0f; /* 清零目标 Y */
    s_target_yaw_deg   = 0.0f; /* 清零目标航向 */
    s_move_mode_enabled = 0U; /* 默认进入航向保持模式 */
    s_attitude_debug_force_enabled = 0U; /* 默认关闭航向调试强制模式 */
    s_move_yaw_cmd_mode_enabled = 0U; /* 默认关闭外部移动+航向指令模式 */
    s_arrived_flag      = 1U; /* 默认处于空闲/到达状态 */
    s_body_vx_mps_feedback = 0.0f; /* 清零反馈 vx */
    s_body_vy_mps_feedback = 0.0f; /* 清零反馈 vy */
    s_cmd_vx_body_mps = 0.0f; /* 清零外部速度指令 vx */
    s_cmd_vy_body_mps = 0.0f; /* 清零外部速度指令 vy */
    /* 确保全部电机停止 */
    ctrl_force_stop(); /* 最终强制停机，确保上电无误动作 */
}

/**
 * @brief  5ms 快速任务
 * @note   仅负责 IMU 更新和航向角同步，不做导航/驱动输出。
 */
void chassis_ctrl_task_5ms(void)
{
    /* IMU 角速度采样与航向角积分 */
    chassis_imu_update_5ms(); /* 更新 IMU 角速度并积分航向角 */

    /* 同步航向角到位姿结构体，保证 get_pose() 实时性 */
    s_pose.yaw_deg = chassis_imu_get_yaw_deg(); /* 将 IMU 航向同步到位姿结构 */
}

/**
 * @brief  20ms 主控制任务
 * @note   本函数统一完成：
 *         1) 轮速采样与里程计积分（全局定位）
 *         2) 点位导航/移动航向指令/原地航向闭环 三种控制分支
 *         3) 速度指令下发到底盘执行链路
 */
void chassis_ctrl_task_20ms(void)
{
    float yaw_rad, cos_yaw, sin_yaw;              /* 当前航向角及其三角函数 */
    float wheel_speeds[CHASSIS_WHEEL_COUNT];      /* 四轮实时速度反馈 */
    float vx_body_raw, vy_body_raw;               /* 麦轮逆解得到的车体速度原始值 */
    chassis_body_speed_cmd_t cmd = {0.0f, 0.0f, 0.0f}; /* 本周期最终下发的车体速度指令 */
    uint8 i;                                      /* 轮序号 */

    /* --- 步骤 1: 读取全部编码器，更新四轮速度 --- */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        chassis_encoder_update(&s_encoders[i], CHASSIS_TASK_DT_20MS_S); /* 按 20ms 周期更新该轮速度 */
        wheel_speeds[i] = chassis_encoder_get_speed(&s_encoders[i]);    /* 读取该轮当前速度 */
    }

    /* --- 步骤 2: 麦轮逆运动学 → 车体速度 --- */
    chassis_mecanum_inverse(wheel_speeds, &vx_body_raw, &vy_body_raw); /* 四轮速度反解车体速度 */
    s_body_vx_mps_feedback = vx_body_raw * CHASSIS_ODOM_SCALE_X; /* 加 X 方向标定系数 */
    s_body_vy_mps_feedback = vy_body_raw * CHASSIS_ODOM_SCALE_Y; /* 加 Y 方向标定系数 */

    /* --- 步骤 3: 里程计积分 → 更新全局位姿 (x, y) --- */
    yaw_rad = s_pose.yaw_deg * CHASSIS_DEG_TO_RAD_F; /* 当前航向角转弧度 */
    cos_yaw = cosf(yaw_rad); /* 计算 cos(yaw) */
    sin_yaw = sinf(yaw_rad); /* 计算 sin(yaw) */

    {
        /* 车体速度 → 全局速度（2D 旋转变换） */
        float vx_global = cos_yaw * s_body_vx_mps_feedback - sin_yaw * s_body_vy_mps_feedback; /* 车体系 vx/vy 旋转到全局 vx */
        float vy_global = sin_yaw * s_body_vx_mps_feedback + cos_yaw * s_body_vy_mps_feedback; /* 车体系 vx/vy 旋转到全局 vy */

        s_pose.x_m += vx_global * CHASSIS_TASK_DT_20MS_S; /* 对全局 vx 积分得到 x */
        s_pose.y_m += vy_global * CHASSIS_TASK_DT_20MS_S; /* 对全局 vy 积分得到 y */
    }

    /* --- 步骤 4: 导航控制（含航向闭环） → 输出车体速度指令 --- */
    /* 分支优先级：点位导航 > 外部移动+航向指令 > 原地航向闭环。 */
    /* 航向调试强制模式打开时，统一走原地航向闭环分支。 */
    if ((0U != s_move_mode_enabled) && (0U == s_attitude_debug_force_enabled))
    {
        /* ---- 定点移动模式：同时控制位置和航向 ---- */
        float dx = s_target_x_m - s_pose.x_m; /* X 轴位置误差 */
        float dy = s_target_y_m - s_pose.y_m; /* Y 轴位置误差 */
        float dist = sqrtf(dx * dx + dy * dy); /* 到目标点的欧氏距离 */

        if (dist <= CHASSIS_TARGET_REACHED_EPSILON_M)
        {
            /* 到达目标点 → 切换到停止状态 */
            s_arrived_flag = 1U;      /* 标记已到达 */
            s_move_mode_enabled = 0U; /* 退出移动模式 */
            ctrl_force_stop();        /* 立即停机 */
            return;                   /* 结束本周期控制 */
        }

        {
            /* 位置 P 控制器：误差 × Kp → 全局坐标系速度指令 */
            float vx_global_cmd = s_tune_params.pos_kp * dx; /* 全局 X 方向速度指令 */
            float vy_global_cmd = s_tune_params.pos_kp * dy; /* 全局 Y 方向速度指令 */
            float linear_norm = sqrtf(vx_global_cmd * vx_global_cmd + vy_global_cmd * vy_global_cmd); /* 全局线速度模长 */
            float target_yaw; /* 朝向目标点的期望航向 */
            float yaw_err;    /* 航向误差（度） */
            float wz_cmd;     /* 航向闭环输出角速度 */

            if (linear_norm > s_tune_params.max_linear_speed_mps && linear_norm > 1e-6f)
            {
                float scale = s_tune_params.max_linear_speed_mps / linear_norm; /* 限速缩放系数 */
                vx_global_cmd *= scale; /* 缩放全局 vx 指令 */
                vy_global_cmd *= scale; /* 缩放全局 vy 指令 */
            }

            /* 航向闭环（点位导航模式）：目标朝向 = 指向目标点，不启用死区。 */
            target_yaw = atan2f(dy, dx) * CHASSIS_RAD_TO_DEG_F;
            yaw_err = chassis_normalize_angle_deg(target_yaw - s_pose.yaw_deg);

            /* 移动态仅使用 P，且清空积分避免拖拽转向响应。 */
            s_yaw_err_i_accum = 0.0f;
            wz_cmd = chassis_clamp_f(s_tune_params.yaw_kp * yaw_err,
                                     -s_tune_params.max_yaw_speed_dps,
                                      s_tune_params.max_yaw_speed_dps);

            /* 全局速度 → 车体速度（逆旋转变换） */
            cmd.vx_body_mps =  cos_yaw * vx_global_cmd + sin_yaw * vy_global_cmd;
            cmd.vy_body_mps = -sin_yaw * vx_global_cmd + cos_yaw * vy_global_cmd;
            cmd.wz_dps = wz_cmd;
        }
    }
    else
    {
        /* ---- 非点位导航分支：统一走航向闭环（可选平移） ---- */
        uint8 move_with_yaw_cmd = ((0U != s_move_yaw_cmd_mode_enabled) && (0U == s_attitude_debug_force_enabled)); /* 1=移动+航向指令, 0=原地航向闭环 */
        float yaw_err = chassis_normalize_angle_deg(s_target_yaw_deg - s_pose.yaw_deg); /* 目标航向与当前航向误差 */
        float wz_cmd; /* 航向闭环输出角速度 */

        /* 航向闭环：零点死区 + PI 控制。 */
        if (fabsf(yaw_err) <= CHASSIS_YAW_HOLD_DEADZONE_DEG)
        {
            /* 误差很小时让积分缓慢回零，抑制长期残余输出。 */
            s_yaw_err_i_accum *= 0.80f;
            wz_cmd = 0.0f;
        }
        else
        {
            s_yaw_err_i_accum += yaw_err * CHASSIS_TASK_DT_20MS_S; /* 误差积分 */
            s_yaw_err_i_accum = chassis_clamp_f(s_yaw_err_i_accum,
                                                -CHASSIS_YAW_I_LIMIT_DEG_S,
                                                 CHASSIS_YAW_I_LIMIT_DEG_S); /* 积分限幅防 wind-up */

            wz_cmd = s_tune_params.yaw_kp * yaw_err + CHASSIS_YAW_HOLD_KI * s_yaw_err_i_accum; /* PI 输出 */
            wz_cmd = chassis_clamp_f(wz_cmd,
                                     -s_tune_params.max_yaw_speed_dps,
                                      s_tune_params.max_yaw_speed_dps); /* 输出限幅 */
        }

        if (move_with_yaw_cmd)
        {
            /* 外部移动+航向指令模式：使用外部下发的平移速度。 */
            cmd.vx_body_mps = s_cmd_vx_body_mps;
            cmd.vy_body_mps = s_cmd_vy_body_mps;
        }
        else
        {
            /* 原地航向闭环模式：平移速度固定为 0。 */
            cmd.vx_body_mps = 0.0f;
            cmd.vy_body_mps = 0.0f;
        }
        cmd.wz_dps      = wz_cmd;
    }

    /* 航向闭环与导航目标统一汇总后，集中下发到底层执行链路。 */
    ctrl_apply_body_speed(cmd);
}

/**
 * @brief  下发网格目标点并启动点位导航
 * @param  target_x_grid 目标网格 X（含边界输入会被自动钳位）
 * @param  target_y_grid 目标网格 Y（含边界输入会被自动钳位）
 * @note   本函数会关闭“移动+航向指令模式”，并清除到达标志。
 */
void chassis_ctrl_move_to_grid(uint8 target_x_grid, uint8 target_y_grid)
{
    /* 边界保护：外圈边界不可进入，目标钳位到可通行内场。 */
    uint8 x = chassis_clamp_grid_x_inner(target_x_grid); /* 限幅后的 X 网格索引（1~14） */
    uint8 y = chassis_clamp_grid_y_inner(target_y_grid); /* 限幅后的 Y 网格索引（1~10） */

    /* 网格坐标 → 物理坐标：按 X/Y 独立步长换算。 */
    s_target_x_m = chassis_grid_x_to_m(x); /* 网格 X 转米 */
    s_target_y_m = chassis_grid_y_to_m(y); /* 网格 Y 转米 */
    s_yaw_err_i_accum = 0.0f; /* 模式切换时清空航向积分，避免历史积分残留 */

    s_move_yaw_cmd_mode_enabled = 0U; /* 点位导航模式会覆盖外部移动+航向指令模式 */
    s_attitude_debug_force_enabled = 0U; /* 进入路径移动前关闭航向调试强制模式 */
    s_move_mode_enabled = 1U; /* 启动移动模式 */
    s_arrived_flag = 0U;      /* 清除到达标志 */
}

/**
 * @brief  下发“移动+航向”复合控制指令
 * @param  vx_body_mps    车体系 X 方向速度目标（m/s）
 * @param  vy_body_mps    车体系 Y 方向速度目标（m/s）
 * @param  target_yaw_deg 目标航向角（度，内部会归一化）
 * @note   本函数会退出点位导航，转入“平移+航向闭环”模式。
 */
void chassis_ctrl_set_move_yaw_cmd(float vx_body_mps, float vy_body_mps, float target_yaw_deg)
{
    /* 写入外部下发的车体系速度与目标航向角。 */
    s_cmd_vx_body_mps = vx_body_mps;
    s_cmd_vy_body_mps = vy_body_mps;
    s_target_yaw_deg = chassis_normalize_angle_deg(target_yaw_deg);
    s_yaw_err_i_accum = 0.0f; /* 模式切换时清空航向积分，避免切入瞬态冲击 */

    /* 切换到“移动+航向指令”模式，并关闭点位导航。 */
    s_move_mode_enabled = 0U;
    s_attitude_debug_force_enabled = 0U;
    s_move_yaw_cmd_mode_enabled = 1U;
    s_arrived_flag = 1U; /* 非点位导航模式不使用到达判定，视为空闲态 */
}

/**
 * @brief  进入原地航向闭环模式
 * @param  target_yaw_deg 目标航向角（度，内部会归一化）
 * @note   平移速度会被置零，仅保留角速度闭环输出。
 */
void chassis_ctrl_hold_yaw(float target_yaw_deg)
{
    s_target_yaw_deg = chassis_normalize_angle_deg(target_yaw_deg); /* 写入并归一化目标角 */
    s_yaw_err_i_accum = 0.0f; /* 切入原地航向闭环前清空积分状态 */
    s_move_yaw_cmd_mode_enabled = 0U; /* 进入原地航向闭环时关闭外部移动+航向指令模式 */
    s_move_mode_enabled = 0U; /* 关闭移动模式，进入航向保持 */
    s_arrived_flag = 1U;      /* 航向保持模式视为空闲 */
}

/**
 * @brief  航向调试入口：清状态并以 0 度为目标启动闭环
 * @note   主要用于串口观察航向误差收敛过程，不参与点位导航。
 */
void chassis_ctrl_attitude_debug_start_zero(void)
{
    /* 清理历史速度与 PID 状态，避免从运动态切换到原地航向闭环时出现残余冲击。 */
    ctrl_force_stop();          /* 清零执行器和 PID 状态 */
    chassis_ctrl_hold_yaw(0.0f); /* 将目标航向固定到 0 度 */
    s_attitude_debug_force_enabled = 1U; /* 打开航向调试强制模式，20ms 仅走原地航向闭环 */
}

/**
 * @brief  获取当前航向调试状态快照
 * @param  out_info 输出结构体指针（不可为空）
 * @note   提供目标角、当前角、误差与最近角速度指令，供调试打印使用。
 */
void chassis_ctrl_attitude_debug_get_state(chassis_attitude_debug_info_t *out_info)
{
    if (0 == out_info)
    {
        return; /* 空指针保护 */
    }

    out_info->target_yaw_deg  = s_target_yaw_deg; /* 当前目标航向 */
    out_info->current_yaw_deg = s_pose.yaw_deg;   /* 当前实际航向 */
    out_info->yaw_err_deg     = chassis_normalize_angle_deg(s_target_yaw_deg - s_pose.yaw_deg); /* 当前航向误差 */
    out_info->wz_cmd_dps      = s_last_cmd.wz_dps; /* 最近一次输出角速度 */
}

/**
 * @brief  航向调试周期打印任务（5ms 调用）
 * @note   内部按 100ms 分频打印一次，避免串口刷屏影响实时性。
 */
void chassis_ctrl_attitude_debug_task_5ms(void)
{
    static uint8 s_print_div = 0U;      /* 打印分频计数：20 * 5ms = 100ms */
    chassis_attitude_debug_info_t info; /* 本次打印快照 */

    /* 每 5ms +1，累计到 20 再打印一次，避免串口刷屏。 */
    s_print_div++;
    if (s_print_div < 20U)
    {
        return; /* 未到打印周期，直接返回 */
    }
    s_print_div = 0U; /* 达到打印周期后清零分频计数 */

    chassis_ctrl_attitude_debug_get_state(&info); /* 获取本次打印快照 */
    printf("[YawClosedLoop] target=%.2f yaw=%.2f err=%.2f wz=%.2f\r\n",
           info.target_yaw_deg,
            info.current_yaw_deg,
            info.yaw_err_deg,
            info.wz_cmd_dps); /* 打印航向闭环核心观测量 */
}

/**
 * @brief  查询点位导航到达标志
 * @return 1=已到达或空闲，0=移动中
 */
uint8 chassis_ctrl_is_arrived(void)
{
    return s_arrived_flag; /* 返回当前到达标志 */
}

/**
 * @brief  获取当前位姿快照
 * @return 当前位姿（x/y/yaw）副本
 */
chassis_pose_t chassis_ctrl_get_pose(void)
{
    chassis_pose_t copy; /* 位姿副本，避免直接暴露全局变量 */
    copy.x_m     = s_pose.x_m;     /* 拷贝 x */
    copy.y_m     = s_pose.y_m;     /* 拷贝 y */
    copy.yaw_deg = s_pose.yaw_deg; /* 拷贝 yaw */
    return copy;                   /* 返回位姿副本 */
}

/**
 * @brief  获取最近一次下发到底层的速度命令
 * @return 车体速度命令副本（vx/vy/wz）
 */
chassis_body_speed_cmd_t chassis_ctrl_get_last_cmd(void)
{
    chassis_body_speed_cmd_t copy; /* 指令副本，避免直接暴露全局变量 */
    copy.vx_body_mps = s_last_cmd.vx_body_mps; /* 拷贝 vx */
    copy.vy_body_mps = s_last_cmd.vy_body_mps; /* 拷贝 vy */
    copy.wz_dps      = s_last_cmd.wz_dps;      /* 拷贝 wz */
    return copy;                               /* 返回指令副本 */
}

/**
 * @brief  底盘停止接口
 * @note   会退出所有运动模式并清零执行链路状态。
 */
void chassis_ctrl_stop(void)
{
    s_move_yaw_cmd_mode_enabled = 0U; /* 停车时关闭外部移动+航向指令模式 */
    s_move_mode_enabled = 0U; /* 关闭移动模式 */
    s_arrived_flag = 1U;      /* 置位空闲状态 */
    ctrl_force_stop();        /* 强制停机 */
}

/**
 * @brief  手动设置位姿（外部校正接口）
 * @param  x_m      校正后的全局 X 坐标（米）
 * @param  y_m      校正后的全局 Y 坐标（米）
 * @param  yaw_deg  校正后的航向角（度）
 * @note   同时同步 IMU 航向，防止下一周期被旧值覆盖。
 */
void chassis_ctrl_set_pose(float x_m, float y_m, float yaw_deg)
{
    s_pose.x_m     = x_m;                               /* 写入校正后的 x */
    s_pose.y_m     = y_m;                               /* 写入校正后的 y */
    s_pose.yaw_deg = chassis_normalize_angle_deg(yaw_deg); /* 写入并归一化校正后的 yaw */

    /* 同步航向角到 IMU 模块，避免下一次 5ms 更新覆盖校正值 */
    chassis_imu_set_yaw_deg(s_pose.yaw_deg); /* 同步 IMU 内部航向状态 */
}

/**
 * @brief  读取当前运行时调参
 * @param  out_params 输出参数结构体（不可为空）
 */
void chassis_ctrl_get_tune_params(chassis_tune_params_t *out_params)
{
    if (0 == out_params)
    {
        return; /* 空指针保护 */
    }
    *out_params = s_tune_params; /* 返回当前生效参数快照 */
}

/**
 * @brief  写入并生效运行时调参
 * @param  in_params 输入参数结构体（不可为空）
 * @note   写入前会统一做安全限幅，随后同步更新四轮 PID 增益。
 */
void chassis_ctrl_set_tune_params(const chassis_tune_params_t *in_params)
{
    uint8 i;                       /* 轮序号：同步更新四路 PID 增益 */
    chassis_tune_params_t sanitized; /* 限幅后的安全参数 */

    if (0 == in_params)
    {
        return; /* 空指针保护 */
    }

    /* 先做限幅，再整体替换运行时参数。 */
    sanitized = ctrl_sanitize_tune_params(*in_params); /* 输入参数限幅 */
    s_tune_params = sanitized;                         /* 覆盖当前参数 */

    /* 同步更新四轮 PID 的三个增益，确保参数立即生效。 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        s_pids[i].kp = sanitized.wheel_pid_kp; /* 更新该轮 Kp */
        s_pids[i].ki = sanitized.wheel_pid_ki; /* 更新该轮 Ki */
        s_pids[i].kd = sanitized.wheel_pid_kd; /* 更新该轮 Kd */
    }
}
