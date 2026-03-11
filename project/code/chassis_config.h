#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*===========================================================================
 * [chassis_config.h] 底盘硬件配置总表（仅头文件，无对应 .c）
 *
 *   集中定义底盘相关的 **所有** 硬件引脚、物理参数、控制参数。
 *   修改引脚或调参时只需编辑本文件，其他模块自动生效。
 *
 * [车轮编号约定]（俯视图，车头朝上）:
 *
 *          ┌── 车头（前方）──┐
 *          │  LF        RF  │    LF = Left-Front  左前轮
 *          │                │    RF = Right-Front 右前轮
 *          │  LB        RB  │    LB = Left-Back   左后轮
 *          └────────────────┘    RB = Right-Back  右后轮
 *
 * [坐标系约定]:
 *   车体坐标系：X 向右为正，Y 向前为正
 *   全局坐标系：X 向右为正，Y 向前为正
 *   航向角 yaw：逆时针为正（从 X 轴到 Y 轴方向）
 *
 * [文件依赖]:
 *   本文件被以下模块包含：
 *     chassis_pid.h / chassis_motor.h / chassis_encoder.h
 *     chassis_imu.h / chassis_mecanum.h / chassis_ctrl.h
 *===========================================================================*/

#include "zf_common_headfile.h"

/* ======================================================================
 *  数学常量
 * ====================================================================== */

/** 圆周率 π */
#define CHASSIS_PI_F                    (3.1415926f)

/** 角度 → 弧度换算系数: rad = deg × DEG_TO_RAD */
#define CHASSIS_DEG_TO_RAD_F            (CHASSIS_PI_F / 180.0f)

/** 弧度 → 角度换算系数: deg = rad × RAD_TO_DEG */
#define CHASSIS_RAD_TO_DEG_F            (180.0f / CHASSIS_PI_F)

/* ======================================================================
 *  控制任务周期
 * ====================================================================== */

/** 5ms 任务周期（秒），用于 IMU 角速度读取和航向角积分 */
#define CHASSIS_TASK_DT_5MS_S           (0.005f)

/** 20ms 任务周期（秒），用于编码器测速、PID 轮控、位置控制 */
#define CHASSIS_TASK_DT_20MS_S          (0.020f)

/* ======================================================================
 *  比赛场地 & 网格参数
 *  场地物理尺寸 3.2m × 2.4m，映射为 16 列 × 12 行的网格
 * ====================================================================== */

/** 每格物理边长（米），即 20cm = 0.20m */
#define CHASSIS_GRID_CELL_SIZE_M        (0.20f)

/** 网格 X 方向最大索引（0 ~ 15，共 16 列） */
#define CHASSIS_GRID_MAX_X              (15U)

/** 网格 Y 方向最大索引（0 ~ 11，共 12 行） */
#define CHASSIS_GRID_MAX_Y              (11U)

/** 到达目标点判定阈值（米），距目标小于此值即认为"已到达" */
#define CHASSIS_TARGET_REACHED_EPSILON_M (0.03f)

/* ======================================================================
 *  速度限幅
 * ====================================================================== */

/** 车体平移最大合成线速度（m/s），矢量模长不超过此值 */
#define CHASSIS_MAX_LINEAR_SPEED_MPS    (0.35f)

/** 车体最大旋转角速度（°/s） */
#define CHASSIS_MAX_YAW_SPEED_DPS       (90.0f)

/** 单个轮子允许的最大线速度（m/s），超出时四轮按比例缩放 */
#define CHASSIS_MAX_WHEEL_SPEED_MPS     (0.60f)

/* ======================================================================
 *  导航控制增益（P 控制器参数）
 * ====================================================================== */

/** 位置环 Kp：值越大，向目标点收敛越快；过大易超调 */
#define CHASSIS_POS_KP                  (0.90f)

/** 航向环 Kp：值越大，朝向对准越快；过大易振荡 */
#define CHASSIS_YAW_KP                  (2.20f)

/* ======================================================================
 *  缓加速参数 — 防止目标速度突变导致轮胎打滑
 * ====================================================================== */

/** 线速度最大加速度（m/s²），每 20ms 允许的最大变化量 */
#define CHASSIS_CMD_ACCEL_LIMIT_MPS2    (1.20f)

/** 角速度最大加速度（°/s²） */
#define CHASSIS_CMD_ACCEL_LIMIT_DPS2    (240.0f)

/* ======================================================================
 *  里程计标定系数 — 补偿轮径/打滑等误差
 *
 *  标定方法：
 *    让车走 1 格(20cm)，用尺子测量实际位移 d_real，
 *    则 SCALE = 0.20 / d_real
 * ====================================================================== */

/** X 方向里程计缩放系数 */
#define CHASSIS_ODOM_SCALE_X            (1.00f)

/** Y 方向里程计缩放系数 */
#define CHASSIS_ODOM_SCALE_Y            (1.00f)

/* ======================================================================
 *  车模物理尺寸 — 已经填入实际测量数据
 * ====================================================================== */

/** 麦克纳姆轮半径（米），测量轮子外径 ÷ 2 */
#define CHASSIS_WHEEL_RADIUS_M          (0.0315f)

/** 前后轴距的 1/2（米），前轮轴心到车体几何中心距离 */
#define CHASSIS_HALF_WHEEL_BASE_M       (0.100f)

/** 左右轮距的 1/2（米），左轮中心到车体几何中心距离 */
#define CHASSIS_HALF_TRACK_WIDTH_M      (0.090f)

/** 麦轮运动学常数 K = 半轴距 + 半轮距，用于正/逆运动学解算 */
#define CHASSIS_MECANUM_K_M             (CHASSIS_HALF_WHEEL_BASE_M + CHASSIS_HALF_TRACK_WIDTH_M)

/* ======================================================================
 *  编码器参数
 * ====================================================================== */

/**
 * 编码器每转脉冲数（count/rev）。
 *   例：256 线编码器 × 4 倍频 = 1024
 *   若逐飞库内部已做 4 倍频，则直接填原始线数。
 *   务必根据实际编码器型号与驱动库确认此值！
 */
#define CHASSIS_ENCODER_COUNTS_PER_REV  (1024.0f)

/* ======================================================================
 *  电机 PWM 参数
 * ====================================================================== */

/** 电机 PWM 载波频率（Hz），DRV8701E 推荐范围 10k~100kHz */
#define CHASSIS_MOTOR_PWM_FREQ_HZ       (17000U)

/** PWM 占空比数值上限，由逐飞库 PWM_DUTY_MAX 决定 */
#define CHASSIS_MOTOR_PWM_MAX           ((float)PWM_DUTY_MAX)

/* ======================================================================
 *  轮速 PID 增益（增量式 PID）
 *
 *  调参建议：
 *    1. 先只给 Kp，Ki = Kd = 0，观察轮速能否跟踪阶跃
 *    2. 加 Ki 消除稳态误差
 *    3. 若有振荡加 Kd 抑制
 * ====================================================================== */

/** 比例增益 Kp — 控制响应速度 */
#define CHASSIS_WHEEL_PID_KP            (120.0f)

/** 积分增益 Ki — 消除稳态误差 */
#define CHASSIS_WHEEL_PID_KI            (8.0f)

/** 微分增益 Kd — 抑制振荡 */
#define CHASSIS_WHEEL_PID_KD            (1.0f)

/* ======================================================================
 *  四轮引脚分配
 *
 *  每个轮子需要 3 组引脚：
 *    PWM — 控制电机转速（占空比越高转越快）
 *    DIR — 控制电机转向（GPIO 高电平=正转，低电平=反转）
 *    ENC — 编码器 A/B 两相，用于测量实际转速
 *
 *  若实车某轮转向相反，将对应 DIR_SIGN 改为 -1.0f 即可，
 *  不需要更改任何代码逻辑。
 * ====================================================================== */

/* ---------- 左前轮 (LF) ---------- */
#define CHASSIS_LF_PWM_CHANNEL      PWM2_MODULE2_CHB_C11        /**< PWM 输出: C11 引脚 */
#define CHASSIS_LF_DIR_PIN          C10                          /**< 方向控制: C10 引脚 */
#define CHASSIS_LF_ENC_INDEX        QTIMER1_ENCODER1             /**< 编码器定时器通道 */
#define CHASSIS_LF_ENC_CH1          QTIMER1_ENCODER1_CH1_C0      /**< 编码器 A 相: C0 引脚 */
#define CHASSIS_LF_ENC_CH2          QTIMER1_ENCODER1_CH2_C1      /**< 编码器 B 相: C1 引脚 */
#define CHASSIS_LF_DIR_SIGN         (1.0f)                       /**< 方向修正: 1.0=正向, -1.0=反向 */

/* ---------- 右前轮 (RF) ---------- */
#define CHASSIS_RF_PWM_CHANNEL      PWM2_MODULE1_CHA_C8          /**< PWM 输出: C8  引脚 */
#define CHASSIS_RF_DIR_PIN          C9                            /**< 方向控制: C9  引脚 */
#define CHASSIS_RF_ENC_INDEX        QTIMER1_ENCODER2              /**< 编码器定时器通道 */
#define CHASSIS_RF_ENC_CH1          QTIMER1_ENCODER2_CH1_C2       /**< 编码器 A 相: C2  引脚 */
#define CHASSIS_RF_ENC_CH2          QTIMER1_ENCODER2_CH2_C24      /**< 编码器 B 相: C24 引脚 */
#define CHASSIS_RF_DIR_SIGN         (1.0f)                        /**< 方向修正 */

/* ---------- 左后轮 (LB) ---------- */
#define CHASSIS_LB_PWM_CHANNEL      PWM2_MODULE3_CHB_D3           /**< PWM 输出: D3  引脚 */
#define CHASSIS_LB_DIR_PIN          D2                             /**< 方向控制: D2  引脚 */
#define CHASSIS_LB_ENC_INDEX        QTIMER2_ENCODER1               /**< 编码器定时器通道 */
#define CHASSIS_LB_ENC_CH1          QTIMER2_ENCODER1_CH1_C3        /**< 编码器 A 相: C3  引脚 */
#define CHASSIS_LB_ENC_CH2          QTIMER2_ENCODER1_CH2_C25       /**< 编码器 B 相: C25 引脚 */
#define CHASSIS_LB_DIR_SIGN         (1.0f)                         /**< 方向修正 */

/* ---------- 右后轮 (RB) ---------- */
#define CHASSIS_RB_PWM_CHANNEL      PWM2_MODULE0_CHA_C6            /**< PWM 输出: C6  引脚 */
#define CHASSIS_RB_DIR_PIN          C7                              /**< 方向控制: C7  引脚 */
#define CHASSIS_RB_ENC_INDEX        QTIMER3_ENCODER2                /**< 编码器定时器通道 */
#define CHASSIS_RB_ENC_CH1          QTIMER3_ENCODER2_CH1_B18        /**< 编码器 A 相: B18 引脚 */
#define CHASSIS_RB_ENC_CH2          QTIMER3_ENCODER2_CH2_B19        /**< 编码器 B 相: B19 引脚 */
#define CHASSIS_RB_DIR_SIGN         (1.0f)                          /**< 方向修正 */

/* ======================================================================
 *  轮子编号枚举 — 统一四轮索引，用于数组下标
 * ====================================================================== */

typedef enum
{
    CHASSIS_WHEEL_LF = 0,   /**< 左前轮 */
    CHASSIS_WHEEL_RF,       /**< 右前轮 */
    CHASSIS_WHEEL_LB,       /**< 左后轮 */
    CHASSIS_WHEEL_RB,       /**< 右后轮 */
    CHASSIS_WHEEL_COUNT     /**< 轮子总数 = 4，用于数组大小 */
} chassis_wheel_index_t;

/* ======================================================================
 *  通用工具函数（static inline，头文件内联，各模块可直接调用）
 * ====================================================================== */

/**
 * @brief  浮点数限幅（钳位）
 * @param  value    输入值
 * @param  min_val  下限
 * @param  max_val  上限
 * @return          限幅后的值，保证在 [min_val, max_val] 范围内
 */
static inline float chassis_clamp_f(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief  角度归一化到 [-180, +180) 范围
 *         避免角度跳变（如从 +179° 突变到 -179°）导致控制突变
 * @param  angle_deg  输入角度（度）
 * @return            归一化后的角度（度）
 */
static inline float chassis_normalize_angle_deg(float angle_deg)
{
    while (angle_deg >  180.0f) { angle_deg -= 360.0f; }
    while (angle_deg < -180.0f) { angle_deg += 360.0f; }
    return angle_deg;
}

#endif /* CHASSIS_CONFIG_H */
