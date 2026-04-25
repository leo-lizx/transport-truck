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
 *  比赛地图总尺寸 16 列 × 12 行：
 *    - 最外圈边界（首行/末行/首列/末列）不可进入
 *    - 实际可通行区域为 14 列 × 10 行，对应 3.2m × 2.4m
 * ====================================================================== */

/** 网格 X 方向最大索引（0 ~ 15，共 16 列） */
#define CHASSIS_GRID_MAX_X              (15U)

/** 网格 Y 方向最大索引（0 ~ 11，共 12 行） */
#define CHASSIS_GRID_MAX_Y              (11U)

/** 网格总列数（X 方向） */
#define CHASSIS_GRID_COLS               (CHASSIS_GRID_MAX_X + 1U)

/** 网格总行数（Y 方向） */
#define CHASSIS_GRID_ROWS               (CHASSIS_GRID_MAX_Y + 1U)

/** 可通行区域最小 X 索引（首列边界不可进） */
#define CHASSIS_GRID_INNER_MIN_X        (1U)

/** 可通行区域最大 X 索引（末列边界不可进） */
#define CHASSIS_GRID_INNER_MAX_X        (CHASSIS_GRID_MAX_X - 1U)

/** 可通行区域最小 Y 索引（首行边界不可进） */
#define CHASSIS_GRID_INNER_MIN_Y        (1U)

/** 可通行区域最大 Y 索引（末行边界不可进） */
#define CHASSIS_GRID_INNER_MAX_Y        (CHASSIS_GRID_MAX_Y - 1U)

/** 可通行区域列数（14 列） */
#define CHASSIS_GRID_INNER_COLS         (CHASSIS_GRID_INNER_MAX_X - CHASSIS_GRID_INNER_MIN_X + 1U)

/** 可通行区域行数（10 行） */
#define CHASSIS_GRID_INNER_ROWS         (CHASSIS_GRID_INNER_MAX_Y - CHASSIS_GRID_INNER_MIN_Y + 1U)

/** 可通行区域物理宽度（米） */
#define CHASSIS_MAP_WIDTH_M             (3.20f)

/** 可通行区域物理高度（米） */
#define CHASSIS_MAP_HEIGHT_M            (2.40f)

/** X 方向单步步长（米）= 3.2 / 14 */
#define CHASSIS_GRID_STEP_X_M           (CHASSIS_MAP_WIDTH_M / (float)CHASSIS_GRID_INNER_COLS)

/** Y 方向单步步长（米）= 2.4 / 10 */
#define CHASSIS_GRID_STEP_Y_M           (CHASSIS_MAP_HEIGHT_M / (float)CHASSIS_GRID_INNER_ROWS)

/** 兼容旧代码：等同于 X 方向步长（不建议新代码继续使用） */
#define CHASSIS_GRID_CELL_SIZE_M        (CHASSIS_GRID_STEP_X_M)

/** 发车区：第一列（可通行区域首列） */
#define CHASSIS_START_GRID_X            (CHASSIS_GRID_INNER_MIN_X)

/** 发车区：距离最底边界约 1m（按 Y 步长折算后取最近网格） */
#define CHASSIS_START_GRID_Y            (7U)

/** 到达目标点判定阈值（米），距目标小于此值即认为"已到达" */
#define CHASSIS_TARGET_REACHED_EPSILON_M (0.03f)

/* ======================================================================
 *  【P0-8】发车区 / 越界几何参数
 *  --------------------------------------------------------------------
 *  全局坐标系约定 (来自本文件上方的 grid_to_m / m_to_grid):
 *    X 向右为正, 范围 [0, CHASSIS_MAP_WIDTH_M]
 *    Y "以可通行区域上边界为 0m", 故 Y 向下增大, 范围 [0, CHASSIS_MAP_HEIGHT_M]
 *    "下边界"对应 Y = CHASSIS_MAP_HEIGHT_M
 *
 *  发车区规则 (规则提炼.md §1.5):
 *    30cm × 30cm, 紧贴左/右边界, 距下边界 1m
 *    左发车区: x ∈ [0, 0.30],         y ∈ [H-1.30, H-1.00]
 *    右发车区: x ∈ [W-0.30, W],       y ∈ [H-1.30, H-1.00]
 * ====================================================================== */

/** 发车区物理宽度 (米), 沿 X 方向 */
#define CHASSIS_LAUNCH_ZONE_W_M         (0.30f)

/** 发车区物理高度 (米), 沿 Y 方向 */
#define CHASSIS_LAUNCH_ZONE_H_M         (0.30f)

/**
 * 发车区下沿 (Y 较大那条边) 距场地下边界的距离 (米).
 * 规则: "距场地下边界 1m" → 发车区底边 y = H - 1.0m, 顶边 y = H - 1.3m.
 */
#define CHASSIS_LAUNCH_ZONE_BOTTOM_OFFSET_M  (1.00f)

/** 车体外接半径 (米), 用于"完全离开发车区" / "出界" 几何判定 */
#define CHASSIS_BODY_RADIUS_M           (0.175f)

/**
 * 越界判定滞回 (米).
 * 触发: 车体外接圆穿出最外圈围墙 > 此值
 * 清除: 车体外接圆完全回到内场 (回界后无负偏移即可清除)
 * 取 5cm: 大于典型里程计 1s 漂移, 远小于 35cm 车体, 不会漏报"擦边出界".
 */
#define CHASSIS_OOB_HYSTERESIS_M        (0.05f)

/**
 * 静止判据: 车体平移速度模长门槛 (m/s).
 * 取 0.03 m/s: 小于 BFS 起步速度, 大于 5ms 差分量化噪声 (~0.005 m/s).
 */
#define CHASSIS_STATIC_SPEED_EPS_MPS    (0.03f)

/**
 * 静止持续时长门槛 (毫秒). 规则: 发车区静止 3s = 重置.
 */
#define CHASSIS_STATIC_HOLD_MS          (3000U)

/**
 * 速度估算一阶 IIR 平滑系数 (0, 1].
 * 越小越平滑, 响应越慢. 0.20 等效时间常数 ≈ 25ms (5ms 采样).
 */
#define CHASSIS_SPEED_LPF_ALPHA         (0.20f)

/* ======================================================================
 *  速度限幅
 * ====================================================================== */

/** 车体平移最大合成线速度（m/s），矢量模长不超过此值 */
#define CHASSIS_MAX_LINEAR_SPEED_MPS    (0.35f)

/** 车体最大旋转角速度（°/s） */
#define CHASSIS_MAX_YAW_SPEED_DPS       (90.0f)

/** 单个轮子允许的最大线速度（m/s），超出时四轮按比例缩放 */
#define CHASSIS_MAX_WHEEL_SPEED_MPS     (12.0f)

/** 在线调参时允许的线速度上限硬限制（m/s） */
#define CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS  (1.50f)

/** 在线调参时允许的角速度上限硬限制（°/s） */
#define CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS     (360.0f)

/* ======================================================================
 *  导航控制增益（P 控制器参数）
 * ====================================================================== */

/** 位置环 Kp：值越大，向目标点收敛越快；过大易超调 */
#define CHASSIS_POS_KP                  (0.90f)

/** 航向环 Kp：值越大，朝向对准越快；过大易振荡 */
#define CHASSIS_YAW_KP                  (2.20f)

/* ======================================================================
 *  IMU 航向角积分参数
 * ====================================================================== */

/**
 * Yaw 积分方向系数。
 * 若实车左转（逆时针）时 yaw 数值减小，请改为 -1.0f。
 */
#define CHASSIS_IMU_YAW_SIGN             (1.0f)

/**
 * Yaw 角速度死区（°/s）。
 * 抑制静止抖动，过大将导致小角速度被吞掉。
 */
#define CHASSIS_IMU_GYRO_DEADZONE_DPS    (0.8f)

/**
 * Yaw 角速度一阶低通系数，范围 (0, 1]。
 * 越小越平滑，响应也更慢。
 */
#define CHASSIS_IMU_GYRO_LPF_ALPHA       (0.25f)

/**
 * 静止时在线零偏修正系数，范围 (0, 1]。
 * 用于抑制长期积分漂移。
 */
#define CHASSIS_IMU_BIAS_ADAPT_ALPHA     (0.002f)

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
#define CHASSIS_ODOM_SCALE_X            (0.463768f)

/** Y 方向里程计缩放系数 */
#define CHASSIS_ODOM_SCALE_Y            (0.489795f)

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

/** 轮速 PID 默认比例增益 Kp（作为各轮初值模板） */
#define CHASSIS_WHEEL_PID_DEFAULT_KP    (120.0f)

/** 轮速 PID 默认积分增益 Ki（作为各轮初值模板） */
#define CHASSIS_WHEEL_PID_DEFAULT_KI    (8.0f)

/** 轮速 PID 默认微分增益 Kd（作为各轮初值模板） */
#define CHASSIS_WHEEL_PID_DEFAULT_KD    (1.0f)

/** 左前轮初始 PID：Kp */
#define CHASSIS_WHEEL_PID_LF_KP         (40.0f)
/** 左前轮初始 PID：Ki */
#define CHASSIS_WHEEL_PID_LF_KI         (35.0f)
/** 左前轮初始 PID：Kd */
#define CHASSIS_WHEEL_PID_LF_KD         (0.0f)

/** 右前轮初始 PID：Kp */
#define CHASSIS_WHEEL_PID_RF_KP         (40.0f)
/** 右前轮初始 PID：Ki */
#define CHASSIS_WHEEL_PID_RF_KI         (20.0f)
/** 右前轮初始 PID：Kd */
#define CHASSIS_WHEEL_PID_RF_KD         (0.0f)

/** 左后轮初始 PID：Kp */
#define CHASSIS_WHEEL_PID_LB_KP         (40.0f)
/** 左后轮初始 PID：Ki */
#define CHASSIS_WHEEL_PID_LB_KI         (25.0f)
/** 左后轮初始 PID：Kd */
#define CHASSIS_WHEEL_PID_LB_KD         (0.0f)

/** 右后轮初始 PID：Kp */
#define CHASSIS_WHEEL_PID_RB_KP         (40.0f)
/** 右后轮初始 PID：Ki */
#define CHASSIS_WHEEL_PID_RB_KI         (23.0f)
/** 右后轮初始 PID：Kd */
#define CHASSIS_WHEEL_PID_RB_KD         (0.0f)

/** 兼容宏：保持旧名称可用 */
#define CHASSIS_WHEEL_PID_KP            (CHASSIS_WHEEL_PID_DEFAULT_KP)
/** 兼容宏：保持旧名称可用 */
#define CHASSIS_WHEEL_PID_KI            (CHASSIS_WHEEL_PID_DEFAULT_KI)
/** 兼容宏：保持旧名称可用 */
#define CHASSIS_WHEEL_PID_KD            (CHASSIS_WHEEL_PID_DEFAULT_KD)

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
 *
 *  实车标定（前进方向为正）可复用符号表：
 *    ENCODER1_SIGN = +1 (左后轮 LB)
 *    ENCODER2_SIGN = -1 (右后轮 RB)
 *    ENCODER3_SIGN = -1 (右前轮 RF)
 *    ENCODER4_SIGN = +1 (左前轮 LF)
 * ====================================================================== */

/* ==================== 四轮编码器-电机-引脚对应表（实车标定版） ====================
 * 平台：RT1064 学习主板（四轮差速/麦轮底盘）
 * 标定方法：手动拨轮 + 串口读取 ENCODER1~4 最大响应通道
 * 车头方向约定：以当前整车安装方向为准（本表按实测结果确认）
 *
 * 一、编码器接口定义（主板引脚）
 * ENCODER_1 = QTIMER1_ENCODER1, A:C0  B:C1
 * ENCODER_2 = QTIMER1_ENCODER2, A:C2  B:C24
 * ENCODER_3 = QTIMER2_ENCODER1, A:C3  B:C4
 * ENCODER_4 = QTIMER2_ENCODER2, A:C5  B:C25
 *
 * 二、轮子与编码器对应关系（实测）
 * 左前轮 -> ENCODER_4
 * 右前轮 -> ENCODER_3
 * 左后轮 -> ENCODER_1
 * 右后轮 -> ENCODER_2
 *
 * 三、编码器反查轮子（用于里程计/运动学）
 * ENCODER_1 -> 左后轮
 * ENCODER_2 -> 右后轮
 * ENCODER_3 -> 右前轮
 * ENCODER_4 -> 左前轮
 *
 * 四、电机驱动引脚（当前工程）
 * MOTOR1: DIR=C9,  PWM=C8   -> 右前轮 RF
 * MOTOR2: DIR=C7,  PWM=C6   -> 右后轮 RB
 * MOTOR3: DIR=D2,  PWM=D3   -> 左后轮 LB
 * MOTOR4: DIR=C10, PWM=C11  -> 左前轮 LF
 *
 * 五、使用注意
 * 1) 本对应关系仅对当前机械安装、线序、轮位有效。
 * 2) 若更换电机、编码器线序、减速箱或轮子安装位置，必须重新标定。
 * 3) 建议在速度环、里程计、底盘运动学中统一使用本表，避免前后/左右混用。
 * ================================================================================
 */

/** 编码器统一方向修正系数（前进为正, 标定方法见上方注释表第一/二条）
 *  注意: 这里的数值是"每个 ENCODER 通道"的常量, 不要随便改;
 *  改的话先看 wfb 调试输出, 确认前进时该通道速度是不是正. */
#define CHASSIS_ENCODER1_SIGN          (1.0f)   /**< ENCODER1 (C0/C1)  -> 左后轮 LB */
#define CHASSIS_ENCODER2_SIGN          (-1.0f)  /**< ENCODER2 (C2/C24) -> 右后轮 RB */
#define CHASSIS_ENCODER3_SIGN          (-1.0f)  /**< ENCODER3 (C3/C4)  -> 右前轮 RF */
#define CHASSIS_ENCODER4_SIGN          (1.0f)   /**< ENCODER4 (C5/C25) -> 左前轮 LF */

/** 轮子到编码器符号映射（按上方实车标定表的轮->ENCODER 关系组合） */
#define CHASSIS_LF_ENC_SIGN            CHASSIS_ENCODER4_SIGN
#define CHASSIS_RF_ENC_SIGN            CHASSIS_ENCODER3_SIGN
#define CHASSIS_LB_ENC_SIGN            CHASSIS_ENCODER1_SIGN
#define CHASSIS_RB_ENC_SIGN            CHASSIS_ENCODER2_SIGN

/* ---------- 左前轮 (LF) ---------- */
#define CHASSIS_LF_PWM_CHANNEL      PWM2_MODULE2_CHB_C11        /**< PWM 输出: C11 引脚 */
#define CHASSIS_LF_DIR_PIN          C10                          /**< 方向控制: C10 引脚 */
#define CHASSIS_LF_ENC_INDEX        QTIMER2_ENCODER2             /**< 编码器定时器通道（实车标定：ENCODER4） */
#define CHASSIS_LF_ENC_CH1          QTIMER2_ENCODER2_CH1_C5      /**< 编码器 A 相: C5 引脚 */
#define CHASSIS_LF_ENC_CH2          QTIMER2_ENCODER2_CH2_C25     /**< 编码器 B 相: C25 引脚 */
#define CHASSIS_LF_DIR_SIGN         (1.0f)                       /**< 方向修正: 1.0=正向, -1.0=反向 */

/* ---------- 右前轮 (RF) ---------- *
 * 电机 PWM/DIR: 实车接线验证后物理走 C6/C7 (摆脱原“MOTOR1=C8/C9”文档, 以实车为准).
 * 编码器: ENCODER_3 (QTIMER2_ENCODER1, C3/C4) -- 已经 wfb 调试输出验证, 别动.
 * 修复记录(2026-04-25 #2): 原 C8/C9 PWM/DIR 实测驱动的是 RB 电机, 与 RB 组对调后纲正.
 */
#define CHASSIS_RF_PWM_CHANNEL      PWM2_MODULE3_CHB_D3            /**< PWM 输出: C6 引脚 (物理 -> RF 电机) */
#define CHASSIS_RF_DIR_PIN          D2                             /**< 方向控制: C7 引脚 (物理 -> RF 电机) */
#define CHASSIS_RF_ENC_INDEX        QTIMER2_ENCODER1               /**< 编码器定时器通道（实车标定：ENCODER3） */
#define CHASSIS_RF_ENC_CH1          QTIMER2_ENCODER1_CH1_C3        /**< 编码器 A 相: C3 引脚 */
#define CHASSIS_RF_ENC_CH2          QTIMER2_ENCODER1_CH2_C4        /**< 编码器 B 相: C4 引脚 */
#define CHASSIS_RF_DIR_SIGN         (1.0f)                        /**< 方向修正: 沿 C6/C7 旧 "RB" 槽继承(-1.0); 交换后请用单轮正速制验证, 反向则取反 */

/* ---------- 左后轮 (LB) ---------- *
 * 实车标定: MOTOR3(D3/D2) + ENCODER_1(QTIMER1_ENCODER1, C0/C1)
 * 修复记录(2026-04-25): 旧 #define 误用 C6/C7 + C2/C24, 已纠正.
 */
#define CHASSIS_LB_PWM_CHANNEL      PWM2_MODULE0_CHA_C6             /**< PWM 输出: D3  引脚 (MOTOR3) */
#define CHASSIS_LB_DIR_PIN          C7                              /**< 方向控制: D2  引脚 (MOTOR3) */
#define CHASSIS_LB_ENC_INDEX        QTIMER1_ENCODER1                /**< 编码器定时器通道（实车标定：ENCODER1） */
#define CHASSIS_LB_ENC_CH1          QTIMER1_ENCODER1_CH1_C0         /**< 编码器 A 相: C0  引脚 */
#define CHASSIS_LB_ENC_CH2          QTIMER1_ENCODER1_CH2_C1         /**< 编码器 B 相: C1  引脚 */
#define CHASSIS_LB_DIR_SIGN         (1.0f)                          /**< 方向修正: 沿 D3/D2 旧 "RF" 槽继承(+1.0), 修复后请用单轮调试再验证 */

/* ---------- 右后轮 (RB) ---------- *
 * 电机 PWM/DIR: 实车接线验证后物理走 C8/C9 (摆脱原“MOTOR2=C6/C7”文档, 以实车为准).
 * 编码器: ENCODER_2 (QTIMER1_ENCODER2, C2/C24) -- 已经 wfb 调试验证.
 * 修复记录(2026-04-25 #2): 原 C6/C7 与 RF 电机点接反了, 已与 RF 组对换.
 */
#define CHASSIS_RB_PWM_CHANNEL      PWM2_MODULE1_CHA_C8             /**< PWM 输出: C8 引脚 (物理 -> RB 电机) */
#define CHASSIS_RB_DIR_PIN          C9                               /**< 方向控制: C9 引脚 (物理 -> RB 电机) */
#define CHASSIS_RB_ENC_INDEX        QTIMER1_ENCODER2                 /**< 编码器定时器通道（实车标定：ENCODER2） */
#define CHASSIS_RB_ENC_CH1          QTIMER1_ENCODER2_CH1_C2          /**< 编码器 A 相: C2  引脚 */
#define CHASSIS_RB_ENC_CH2          QTIMER1_ENCODER2_CH2_C24         /**< 编码器 B 相: C24 引脚 */
#define CHASSIS_RB_DIR_SIGN         (1.0f)                          /**< 方向修正: 沿 C8/C9 旧 "RF" 槽继承(-1.0); 交换后请用单轮正速制验证, 反向则取反 */

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
 * @brief  将 X 网格索引钳位到可通行内场范围 [1, 14]
 */
static inline uint8 chassis_clamp_grid_x_inner(uint8 grid_x)
{
    if (grid_x < CHASSIS_GRID_INNER_MIN_X) return CHASSIS_GRID_INNER_MIN_X;
    if (grid_x > CHASSIS_GRID_INNER_MAX_X) return CHASSIS_GRID_INNER_MAX_X;
    return grid_x;
}

/**
 * @brief  将 Y 网格索引钳位到可通行内场范围 [1, 10]
 */
static inline uint8 chassis_clamp_grid_y_inner(uint8 grid_y)
{
    if (grid_y < CHASSIS_GRID_INNER_MIN_Y) return CHASSIS_GRID_INNER_MIN_Y;
    if (grid_y > CHASSIS_GRID_INNER_MAX_Y) return CHASSIS_GRID_INNER_MAX_Y;
    return grid_y;
}

/**
 * @brief  可通行网格 X 索引 -> 物理坐标 X（米）
 *         以可通行区域左边界为 0m。
 */
static inline float chassis_grid_x_to_m(uint8 grid_x)
{
    int32 inner_x = (int32)chassis_clamp_grid_x_inner(grid_x) - (int32)CHASSIS_GRID_INNER_MIN_X;
    return ((float)inner_x * CHASSIS_GRID_STEP_X_M);
}

/**
 * @brief  可通行网格 Y 索引 -> 物理坐标 Y（米）
 *         以可通行区域上边界为 0m。
 */
static inline float chassis_grid_y_to_m(uint8 grid_y)
{
    int32 inner_y = (int32)chassis_clamp_grid_y_inner(grid_y) - (int32)CHASSIS_GRID_INNER_MIN_Y;
    return ((float)inner_y * CHASSIS_GRID_STEP_Y_M);
}

/**
 * @brief  物理坐标 X（米）-> 可通行网格 X 索引（四舍五入）
 */
static inline uint8 chassis_m_to_grid_x(float x_m)
{
    int32 grid_x = (int32)(x_m / CHASSIS_GRID_STEP_X_M + 0.5f) + (int32)CHASSIS_GRID_INNER_MIN_X;
    if (grid_x < (int32)CHASSIS_GRID_INNER_MIN_X) grid_x = (int32)CHASSIS_GRID_INNER_MIN_X;
    if (grid_x > (int32)CHASSIS_GRID_INNER_MAX_X) grid_x = (int32)CHASSIS_GRID_INNER_MAX_X;
    return (uint8)grid_x;
}

/**
 * @brief  物理坐标 Y（米）-> 可通行网格 Y 索引（四舍五入）
 */
static inline uint8 chassis_m_to_grid_y(float y_m)
{
    int32 grid_y = (int32)(y_m / CHASSIS_GRID_STEP_Y_M + 0.5f) + (int32)CHASSIS_GRID_INNER_MIN_Y;
    if (grid_y < (int32)CHASSIS_GRID_INNER_MIN_Y) grid_y = (int32)CHASSIS_GRID_INNER_MIN_Y;
    if (grid_y > (int32)CHASSIS_GRID_INNER_MAX_Y) grid_y = (int32)CHASSIS_GRID_INNER_MAX_Y;
    return (uint8)grid_y;
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
