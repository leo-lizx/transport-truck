#ifndef _CONFIG_CHASSIS_H_
#define _CONFIG_CHASSIS_H_

#include <stdint.h>

/*
 * RT1064 底盘 参数调节头(configChassis) — 编译期参数单一真相源
 *
 * 集中原散落在 chassis_config.h 的控制参数与时序宏，按 § 分节组织。
 * 引脚集中在 pinMap.h (本文件不含引脚)。
 *
 * 量纲约定 (防数量级混用):
 *   轮速域  = 编码器脉冲/20ms (PID 增益吸收尺度，无物理换算)
 *   几何    = 米 (m)
 *   角速度  = 度/秒 (°/s)
 *   角度    = 度 (°)
 *   时序    = µs / ms / tick
 *   PWM     = 原始单位 (0 ~ PWM_DUTY_MAX = 10000)
 *   电流    = ADC 原始量 (如有)
 *
 * 持久化语义: 编译期宏 = 真相源；
 *   运行时 RAM 镜像 (g_chassis_tune_params) 为可调子集，
 *   flash blob (menu 模块) 带 magic/version/CRC，
 *   version 不符自动写默认覆盖过期 blob。
 *
 * 分节索引:
 *   §A 调度时序与模式
 *   §B 轮速环 PID (四轮独立，Kp 全正号)
 *   §C 位置/航向环 (导航层)
 *   §D IMU / 卡尔曼滤波 / 编码器融合
 *   §E 几何 + 限幅 + 滤波 + 静摩擦前馈 + 里程计
 *   §极性 方向系数单一真相源 (执行器/编码器/IMU *_DIR)
 *   §F 视觉融合 / 发车区&越界 / 故障保护
 */

/* ============================================================
 * §A 调度时序与模式
 * ============================================================ */

/** 5ms 任务周期 (秒)，IMU 角速度读取和航向角积分 */
#define CHASSIS_TASK_DT_5MS_S               (0.005f)

/** 20ms 任务周期 (秒)，编码器测速、PID 轮控、位置控制 */
#define CHASSIS_TASK_DT_20MS_S              (0.020f)

/** 圆周率 π */
#define CHASSIS_PI_F                        (3.1415926f)

/** 角度 → 弧度: rad = deg × DEG_TO_RAD */
#define CHASSIS_DEG_TO_RAD_F                (CHASSIS_PI_F / 180.0f)

/** 弧度 → 角度: deg = rad × RAD_TO_DEG */
#define CHASSIS_RAD_TO_DEG_F                (180.0f / CHASSIS_PI_F)

/** 电机 PWM 载波频率 (Hz)，DRV8701E 推荐 10k~100kHz */
#define CHASSIS_MOTOR_PWM_FREQ_HZ           (17000U)

/** PWM 占空比数值上限 */
#define CHASSIS_MOTOR_PWM_MAX               ((float)PWM_DUTY_MAX)

/* ============================================================
 * §B 轮速环 PID (四轮独立，Kp 全正号)
 *
 *   极性补偿已下沉到 §极性 方向系数，Kp 不兼任极性翻转器
 *   (避免负号反直觉 / 误改即正反馈失控)。
 *
 *   4 轮硬件差异应通过 ENCODER_*_DIR / MOTOR_*_OUTPUT_DIR 标定，
 *   不该让 PID 背锅。四轮 PID 统一: 阶跃响应同步，不扭不抖。
 * ============================================================ */

/** 轮速 PID 默认比例增益 Kp (作为各轮初值模板) */
#define CHASSIS_WHEEL_PID_DEFAULT_KP        (120.0f)

/** 轮速 PID 默认积分增益 Ki */
#define CHASSIS_WHEEL_PID_DEFAULT_KI        (8.0f)

/** 轮速 PID 默认微分增益 Kd */
#define CHASSIS_WHEEL_PID_DEFAULT_KD        (1.0f)

/* ---- 左前轮 LF (Wheel 0) ---- */
#define CHASSIS_WHEEL_PID_LF_KP             (70.0f)
#define CHASSIS_WHEEL_PID_LF_KI             (20.0f)
#define CHASSIS_WHEEL_PID_LF_KD             (0.0f)

/* ---- 右前轮 RF (Wheel 1) ---- */
#define CHASSIS_WHEEL_PID_RF_KP             (70.0f)
#define CHASSIS_WHEEL_PID_RF_KI             (20.0f)
#define CHASSIS_WHEEL_PID_RF_KD             (0.0f)

/* ---- 左后轮 LB (Wheel 2) ---- */
#define CHASSIS_WHEEL_PID_LB_KP             (70.0f)
#define CHASSIS_WHEEL_PID_LB_KI             (20.0f)
#define CHASSIS_WHEEL_PID_LB_KD             (0.0f)

/* ---- 右后轮 RB (Wheel 3) ---- */
#define CHASSIS_WHEEL_PID_RB_KP             (70.0f)
#define CHASSIS_WHEEL_PID_RB_KI             (20.0f)
#define CHASSIS_WHEEL_PID_RB_KD             (0.0f)

/* ---- 兼容宏 (旧名称) ---- */
#define CHASSIS_WHEEL_PID_KP                CHASSIS_WHEEL_PID_DEFAULT_KP
#define CHASSIS_WHEEL_PID_KI                CHASSIS_WHEEL_PID_DEFAULT_KI
#define CHASSIS_WHEEL_PID_KD                CHASSIS_WHEEL_PID_DEFAULT_KD

/* ============================================================
 * §C 位置/航向环 (导航层，Kp 全正号)
 * ============================================================ */

/* ---- 用户常调参数 (优先调下面几个) ---- */

/** 航向响应快慢: 越大越快，过大可能轻微摆动 */
#define CHASSIS_YAW_KP                      (5.0f)

/** 角速度 D 阻尼 (真 D): 只抵抗旋转速度，不放大前馈。
 *  0=无阻尼(快但过冲), 0.5~1.0=适度, >1.5=强阻尼(肉但稳)
 *  自适应: 大误差时乘 |err|/10°, 近 0° 时才满额 */
#define CHASSIS_YAW_RATE_KD                 (1.20f)

/** yaw 积分增益 — 消除静摩擦稳态残差。从 0 起调，+0.3/次 */
#define CHASSIS_YAW_KI                      (1.50f)

/** yaw 积分上限 (°/s) — 最多贡献这么多角速度 */
#define CHASSIS_YAW_I_LIMIT_DPS             (40.0f)

/** yaw 积分生效误差带 (°) — 仅在此误差内累积，防大角度卷绕 */
#define CHASSIS_YAW_I_BAND_DEG              (15.0f)

/** 自适应 D 满额误差 (°) — |err| 超此值 D 满额，低于此值线性衰减 */
#define CHASSIS_YAW_D_FULL_ERR_DEG          (10.0f)

/** 最大旋转速度 (°/s): 限制原地转向和导航修正的最高角速度 */
#define CHASSIS_MAX_YAW_SPEED_DPS           (360.0f)

/** 角速度加减速限制 (°/s²): 同时用于 yaw sqrt 曲线和下游 ramp */
#define CHASSIS_CMD_ACCEL_LIMIT_DPS2        (720.0f)

/** 进入在位锁的角度阈值 (°): 越小锁得越准，越大越不抖 */
#define CHASSIS_YAW_INPOS_ENTER_DEG         (0.50f)

/* ---- 网格 & 场地参数 ----
 *   比赛地图 16×12，最外圈不可进入；可通行 14×10，对应 3.2m×2.4m
 *   全局坐标 X∈[0,3.20m], Y∈[0,2.40m]
 */
#define CHASSIS_GRID_MAX_X                  (15U)
#define CHASSIS_GRID_MAX_Y                  (11U)
#define CHASSIS_GRID_COLS                   (CHASSIS_GRID_MAX_X + 1U)
#define CHASSIS_GRID_ROWS                   (CHASSIS_GRID_MAX_Y + 1U)
#define CHASSIS_GRID_INNER_MIN_X            (1U)
#define CHASSIS_GRID_INNER_MAX_X            (CHASSIS_GRID_MAX_X - 1U)
#define CHASSIS_GRID_INNER_MIN_Y            (1U)
#define CHASSIS_GRID_INNER_MAX_Y            (CHASSIS_GRID_MAX_Y - 1U)
#define CHASSIS_GRID_INNER_COLS             (CHASSIS_GRID_INNER_MAX_X - CHASSIS_GRID_INNER_MIN_X + 1U)
#define CHASSIS_GRID_INNER_ROWS             (CHASSIS_GRID_INNER_MAX_Y - CHASSIS_GRID_INNER_MIN_Y + 1U)

/** 可通行区域物理宽度 (米) */
#define CHASSIS_MAP_WIDTH_M                 (2.80f)

/** 可通行区域物理高度 (米) */
#define CHASSIS_MAP_HEIGHT_M                (2.00f)

/** X 方向单步步长 (米) = 2.80 / 14 */
#define CHASSIS_GRID_STEP_X_M               (CHASSIS_MAP_WIDTH_M / (float)CHASSIS_GRID_INNER_COLS)

/** Y 方向单步步长 (米) = 2.00 / 10 */
#define CHASSIS_GRID_STEP_Y_M               (CHASSIS_MAP_HEIGHT_M / (float)CHASSIS_GRID_INNER_ROWS)

/** 兼容旧代码 (不建议新代码使用) */
#define CHASSIS_GRID_CELL_SIZE_M            CHASSIS_GRID_STEP_X_M

#define CHASSIS_START_GRID_X                CHASSIS_GRID_INNER_MIN_X
#define CHASSIS_START_GRID_Y                (6U)  /* 5×0.20=1.00m，与实测车中心吻合 */

/* ---- 导航：到位/保持/制动 ---- */

/** 到达目标点判定阈值 (米) — 纯位置判断，不叠加速度/ dwell/yaw */
/** P0-修复 2026-06-30: 0.012→0.025。0.012 小于 BRAKE_FLOOR 一拍位移(1.3cm),
 *   物理上无法停住 → 反复过冲震荡。0.025=1/8 格，推箱子精度足够。 */
#define CHASSIS_TARGET_REACHED_EPSILON_M    (0.017f)

/** 到位后 Schmitt 滞后释放阈值 (米) — 10cm 覆盖惯性滑移 */
/** P0-修复 2026-06-30: 0.15→0.30。原值过小，小幅过冲即触发回弹震荡。
 *   配合 BRAKE_FLOOR 降低后，惯性过冲通常 <5cm，30cm 余量充足。 */
#define CHASSIS_POS_HOLD_EXIT_M             (0.30f)

/** 到位后 yaw 容忍带 (°) — |yaw_err| < 该值即硬归零，不咬最后残差
 *  P0-修复 2026-05-12 (走斜线): 1.5°→0.5°  (vyg 投影从 31→10 mm/s)
 *  P0-修复 2026-06-07: 0.5°→0.10°，0.10°每米仅漂 1.7mm */
#define CHASSIS_YAW_GOAL_TOLERANCE_DEG      (0.40f)

/* ---- 导航增益 ---- */

/** 位置环 Kp — 沿程方向增益。建议 1.5~4.0
 *  P0-调参 2026-05-08: 2.50→3.50，加快远场逼近
 *  P0-修复 2026-05-11 (拐点停留+走斜线): 10.50→4.50
 *  (KP=10.5 时 linear_dist=accel/KP²≈0.027m → 冲过头) */
#define CHASSIS_POS_KP                      (6.5f)

/** 位置环沿程积分增益 — 消除静摩擦稳态残差。从 0 起调，+0.05/次 */
#define CHASSIS_POS_KI                      (0.30f)

/** 线速度最大加速度 (m/s²)
 *  P0-调参 2026-05-08: 3.00→5.00，加快爬坡/刹车
 *  P0-修复 2026-06-07: 0.60→3.00，ramp 能跟上 brake_cap */
#define CHASSIS_CMD_ACCEL_LIMIT_MPS2        (3.00f)

/* ---- 位置环自动推导比例常量 (用户一般无需修改) ----
 *   kd_eff = pos_kp * KD_RATIO        (主轴速度阻尼)
 *   kd_hold = pos_kp * HOLD_KD_RATIO  (保持轴轻阻尼)
 *   i_limit = v_max * I_LIMIT_RATIO   (积分最多贡献比例)
 *   i_band  = brake_dist * I_BAND_RATIO (只在减速区积分)
 *   switch_tol   = EPSILON * AXIS_SWITCH_RATIO
 *   hold_max_speed = v_max * HOLD_SPEED_RATIO
 *   brake_dist = v_max²/(2·accel) + EPSILON + BRAKE_MARGIN
 */
#define CHASSIS_POS_KD_RATIO                (0.33f)
#define CHASSIS_POS_HOLD_KD_RATIO           (3.0f)
#define CHASSIS_POS_BRAKE_MARGIN_M          (-0.009f)
#define CHASSIS_POS_I_LIMIT_RATIO           (0.20f)
#define CHASSIS_POS_I_BAND_RATIO            (0.8f)
#define CHASSIS_POS_AXIS_SWITCH_RATIO       (0.6f)
#define CHASSIS_POS_HOLD_SPEED_RATIO        (0.35f)

/** 保持轴死区 (m) — 滤除编码器量化噪声 (~2~5mm)，远小于 EPSILON
 *  P0-修复 2026-06-07: 原无死区，噪声被 sqrt_ctrl 放大 → Y 方向持续微幅震荡 */
#define CHASSIS_POS_HOLD_DEAD_ZONE_M        (0.002f)

/** D 项低通滤波系数 α (一阶 IIR) — 截止 ≈ α/(2π·dt)
 *  0.10 @ 20ms → 截止≈0.8Hz，衰减 odom 高频噪声 */
#define CHASSIS_POS_D_LPF_ALPHA             (0.10f)

/** brake_cap 末段最小有效速度 (m/s) — 克服静摩擦，确保推进到 EPSILON 内
 *  P0-修复 2026-06-30: 0.65→0.15。0.65 刹车距离 7cm > EPSILON 2.5cm → 必定过冲。
 *  0.15 刹车距离 ~4mm < EPSILON，且 MPC 自行规划减速，不需要刹车底速硬兜。
 *  如需应对长距离 sqrt_controller 路径的静摩擦卡死，可保留 0.20~0.25。 */
#define CHASSIS_POS_BRAKE_FLOOR_MPS         (0.85f)

/* ============================================================
 * §D IMU / 卡尔曼滤波 / 编码器融合
 * ============================================================ */

/* ---- Yaw 轴选择 (IMU 侧装/倒装适配) ---- */
#define CHASSIS_IMU_YAW_AXIS_Z              (0)     /* 原始 Z 轴 (IMU 正装) */
#define CHASSIS_IMU_YAW_AXIS_Y              (1)     /* Y 轴 (IMU 侧装，当前实车) */
#define CHASSIS_IMU_YAW_AXIS_X              (2)     /* X 轴 (IMU 另一种侧装) */
#define CHASSIS_IMU_YAW_AXIS                CHASSIS_IMU_YAW_AXIS_Y

/* ---- Yaw 角速度处理 ---- */

/** Yaw 角速度死区 (°/s) — 抑制静止抖动 */
#define CHASSIS_IMU_GYRO_DEADZONE_DPS       (0.1f)

/** Yaw 角速度一阶低通系数 (0,1] — 越小越平滑 */
#define CHASSIS_IMU_GYRO_LPF_ALPHA          (0.1f)

/** 静止时在线零偏慢修正系数 (EMA) */
#define CHASSIS_IMU_BIAS_ADAPT_ALPHA        (0.002f)

/* ---- 静止检测 (滑动窗口方差法) ---- */

/** 滑窗长度 (5ms 一拍，64=320ms) */
#define CHASSIS_IMU_STILL_WINDOW_LEN        (64U)

/** 静止判别阈值: 窗口方差上限 (°/s)²
 *  P0-回调 2026-05-02: 0.020→0.050，平台微震动也能识别静止 */
#define CHASSIS_IMU_STILL_VAR_TH_DPS2       (0.05f)

/** 静止确认时 bias 快速更新系数 — 时间常数 ~10 拍 = 50ms */
#define CHASSIS_IMU_BIAS_FAST_ALPHA         (0.10f)

/* ---- Yaw 卡尔曼滤波 (状态: [angle, bias]ᵀ) ---- */

/** 1=KF 替换原 EMA bias+欧拉积分；0=回退旧实现 */
#define CHASSIS_IMU_USE_KALMAN_YAW          (1)

/** 过程噪声: 角度 (°²/s²) */
#define CHASSIS_IMU_KF_Q_ANGLE_DPS2         (0.0010f)

/** 过程噪声: bias (°²/s²/s) — 反映 bias 随机游走速度
 *  P0-回调 2026-05-02: 8e-6→5e-5，车跑 5min 温升 0.05°/s/min，K 增益翻几倍 */
#define CHASSIS_IMU_KF_Q_BIAS_DPS2_PER_S    (0.0009f)

/** 观测噪声: ZUPT 时陀螺噪声方差 (°²/s²) */
#define CHASSIS_IMU_KF_R_ZUPT_DPS2          (0.000010f)

/** 状态协方差初值 */
#define CHASSIS_IMU_KF_P0_ANGLE_DEG2        (1.0f)
#define CHASSIS_IMU_KF_P0_BIAS_DPS2         (0.10f)

/* ---- 编码器 yaw 融合 (麦轮 O 型 yaw 逆运动学 → KF 弱观测) ---- */

/** 1=启用编码器 yaw 弱观测 */
#define CHASSIS_ODOM_YAW_FUSION_ENABLE      (1)

/** 编码器 yaw 观测噪声方差 (°²) — 大=几乎不信(仅长期纠偏)，小=信任高(打滑污染) */
#define CHASSIS_ODOM_YAW_R_DEG2             (150.0f)

/** odom yaw 与 IMU yaw 偏差超过此值视为打滑/重定位，跳过观测 */
#define CHASSIS_ODOM_YAW_OUTLIER_DEG        (1.20f)

/** Innovation Gating: |wz_odom - wz_imu| 超过此值拒绝 (°/s)
 *  P0-修复 2026-05-02 (抬车空转污染 KF): IMU=0, odom>>0 → 自然拒绝 */
#define CHASSIS_ODOM_YAW_INNOV_GATE_DPS     (5.0f)

/* ============================================================
 * §E 几何 + 限幅 + 滤波 + 静摩擦前馈 + 里程计
 * ============================================================ */

/* ---- 车模物理尺寸 (实测) ---- */

/** 麦克纳姆轮半径 (米) */
#define CHASSIS_WHEEL_RADIUS_M              (0.0315f)

/** 前后轴距的 1/2 (米) */
#define CHASSIS_HALF_WHEEL_BASE_M           (0.100f)

/** 左右轮距的 1/2 (米) */
#define CHASSIS_HALF_TRACK_WIDTH_M          (0.090f)

/** 麦轮运动学常数 K = 半轴距+半轮距 */
#define CHASSIS_MECANUM_K_M                 (CHASSIS_HALF_WHEEL_BASE_M + CHASSIS_HALF_TRACK_WIDTH_M)

/**
 * 麦轮辊子布局: 1=O 型 (实车), 0=X 型
 *   O 型相对 X 型，所有 vx (横移) 项整体翻号；vy/wz 不变。
 *   代码层切换此宏即可自动切换正/逆运动学。
 */
#define CHASSIS_MECANUM_O_TYPE_LAYOUT       (1)

/** 编码器每转脉冲数 (256 线 × 4 倍频) */
#define CHASSIS_ENCODER_COUNTS_PER_REV      (1024.0f)

/* ---- 速度限幅 ---- */

/** 车体平移最大合成线速度 (m/s)
 *  P0-修复 2026-05-19 (轮子停转): 1.20→0.70 (TB6612 过流关断)
 *  0.70: d_stop=0.117m, BRAKE_DIST=0.25m 有裕量 */
#define CHASSIS_MAX_LINEAR_SPEED_MPS        (2.0f)

/** 正常控制路径的单轮速度兑底上限 (m/s) — 等比例缩放保方向 */
#define CHASSIS_WHEEL_SPEED_CAP_MPS         (2.70f)

/** 在线调参时线速度上限硬限制 (m/s) */
#define CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS  (4.0f)

/** 在线调参时角速度上限硬限制 (°/s) */
#define CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS     (360.0f)

/** 单轮 PID 调试模式下调试轮目标速度上限 (m/s) — 仅 debug 使用 */
#define CHASSIS_DEBUG_WHEEL_SPEED_LIMIT_MPS     (5.0f)

/* ---- 发车区 & 越界几何 (P0-8) ---- */
#define CHASSIS_LAUNCH_ZONE_W_M             (0.30f)
#define CHASSIS_LAUNCH_ZONE_H_M             (0.30f)
#define CHASSIS_LAUNCH_ZONE_BOTTOM_OFFSET_M (1.0f)
#define CHASSIS_BODY_RADIUS_M               (0.175f)
#define CHASSIS_OOB_HYSTERESIS_M            (0.05f)

/** 静止判据: 车体平移速度模长门槛 (m/s) */
#define CHASSIS_STATIC_SPEED_EPS_MPS        (0.07f)

/** 静止持续时长门槛 (ms) — 规则: 发车区静止 3s=重置 */
#define CHASSIS_STATIC_HOLD_MS              (3000U)

/** 速度估算一阶 IIR 平滑系数 (0,1] — 0.20 等效时间常数≈25ms */
#define CHASSIS_SPEED_LPF_ALPHA             (0.20f)

/* ---- 里程计标定系数 (补偿轮径/打滑等误差) ----
 *   标定方法: 让车走 1 格(20cm)，用尺子测量实际位移 d_real，
 *   则 SCALE = 0.20 / d_real
 */
#define CHASSIS_ODOM_SCALE_X                (0.3970f)
#define CHASSIS_ODOM_SCALE_Y                (0.4130f)

/* ---- 静摩擦前馈 (Breakaway FF) — 每轮独立可配 ----
 *   实现 (apply_speed): 每轮独立查 g_chassis_tune_params
 *     if |target| > target_eps[i] && |fb| < fb_static_eps[i]:
 *         pwm += sign(target) * pwm_floor[i]
 *   调参口诀:
 *     起步迟/不动 → 加大 FLOOR
 *     起步猛冲    → 减小 FLOOR
 *     起步一抖一抖 → 加大 FB_STATIC_EPS (ff 保持更久)
 *     小目标也冲   → 降低 TARGET_EPS (更晚触发)
 */

/* 默认值 (各轮初值模板) */
#define CHASSIS_WHEEL_BREAKAWAY_DEFAULT_TARGET_EPS_MPS     (0.08f)
#define CHASSIS_WHEEL_BREAKAWAY_DEFAULT_PWM_FLOOR          (1000.0f)
#define CHASSIS_WHEEL_BREAKAWAY_DEFAULT_FB_STATIC_EPS_MPS  (2.80f)

/* LF (Wheel 0)
 * P0-修复 2026-06-07: TARGET_EPS=0+FB_STATIC=2.10→breakaway 永远激活，
 * 位置环收拢时指令小幅正负交替→breakaway 跟跳 2000PWM→把车甩出极限环 */
#define CHASSIS_WHEEL_BREAKAWAY_LF_TARGET_EPS_MPS          (0.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR               (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LF_FB_STATIC_EPS_MPS       (0.90f)

/* RF (Wheel 1) */
#define CHASSIS_WHEEL_BREAKAWAY_RF_TARGET_EPS_MPS          (0.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR               (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_FB_STATIC_EPS_MPS       (0.90f)

/* LB (Wheel 2) */
#define CHASSIS_WHEEL_BREAKAWAY_LB_TARGET_EPS_MPS          (0.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR               (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_FB_STATIC_EPS_MPS       (0.90f)

/* RB (Wheel 3) */
#define CHASSIS_WHEEL_BREAKAWAY_RB_TARGET_EPS_MPS          (0.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR               (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_FB_STATIC_EPS_MPS       (0.90f)

/* 正/负向不对称前馈 (XP=正X移动 / XN=负X移动)
 * P0-修复 2026-06-07: 原 XN 不对称(LB=1200独大)→ -X 偏航力矩→yaw 耦合震荡；
 * 改为对称互换，两侧推力均衡后 yaw 力矩抵消 */
#define CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR_XP            (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR_XP            (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR_XP            (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR_XP            (800.0f)

#define CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR_XN            (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR_XN            (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR_XN            (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR_XN            (800.0f)

/* 兼容宏 (旧名称) */
#define CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS    CHASSIS_WHEEL_BREAKAWAY_DEFAULT_TARGET_EPS_MPS
#define CHASSIS_WHEEL_BREAKAWAY_PWM_FLOOR         CHASSIS_WHEEL_BREAKAWAY_DEFAULT_PWM_FLOOR
#define CHASSIS_WHEEL_BREAKAWAY_FB_STATIC_EPS_MPS CHASSIS_WHEEL_BREAKAWAY_DEFAULT_FB_STATIC_EPS_MPS

/* ============================================================
 * §极性 方向系数单一真相源 ("正方向 = 正" 不变量)
 *
 * 约定: 每轮物理正方向 = 前进 (车身 Y+)，对应正命令 + 正反馈。
 *
 * 三类方向系数各自吸收接线 / 编码器相位 / IMU 安装朝向，
 * 把"正方向=正"归一到上层，使各环 Kp 全用正号。
 *
 * 各 ±1 由**开环实测辨识**(给正命令 → 看执行器方向 / 各反馈符号)，
 * 非固定常量；换硬件 / 重接线须重辨识。
 *
 * 闭环 (yaw/航向环) 另需"传感器正向 = 被控量正向"，
 * 开环单实例辨识不覆盖，须手动转车确认。
 *
 * ★ agent 改/读本段时**主动提醒用户核实接线并请其确认**
 *   (见 hf-hw-mapping / hf-embedded-safety)。
 *
 * 实车标定对照 (从原 chassis_config.h 迁移):
 *   原 LF/RF/LB/RB_DIR_SIGN 全为 +1.0f
 *   原 ENCODER1_SIGN=+1(LB), ENCODER2_SIGN=-1(RB),
 *       ENCODER3_SIGN=-1(RF), ENCODER4_SIGN=+1(LF)
 *   原 IMU_YAW_SIGN=-1 (侧装+左转gyro_y读负)
 * ------------------------------------------------------------ */

/* (1) 执行器输出 MOTOR_*_OUTPUT_DIR: driver 乘命令，使 +命令 → 物理前进
 *     ★ 占位 +1 已按实车标定填写，换接线/换电机线序后必须重新辨识 */
#define MOTOR_LF_OUTPUT_DIR                 (+1.0f)
#define MOTOR_RF_OUTPUT_DIR                 (-1.0f)
#define MOTOR_LB_OUTPUT_DIR                 (+1.0f)
#define MOTOR_RB_OUTPUT_DIR                 (-1.0f)

/* (2) 编码器 ENCODER_*_DIR: 乘原始计数，使 前进 → +速度反馈
 *     ★ 实车已标定值，换编码器线序/减速箱后必须重新辨识 */
#define ENCODER_LF_DIR                      (+1)    /* 实车: ENCODER4_SIGN=+1 */
#define ENCODER_RF_DIR                      (-1)    /* 实车: ENCODER3_SIGN=-1 */
#define ENCODER_LB_DIR                      (+1)    /* 实车: ENCODER1_SIGN=+1 */
#define ENCODER_RB_DIR                      (-1)    /* 实车: ENCODER2_SIGN=-1 */

/* (3) IMU 轴映射符号 — 乘陀螺仪原始值 (Y 轴)
 *     ★ 必须手动转车确认: 左转(CCW)时 yawRate>0 才对。
 *     若反 → 改 -1.0f；开环辨识不覆盖整车轴向 */
#define CHASSIS_IMU_YAW_SIGN                (-1.0f) /* 实车: 左转gyro_y读负→翻转 */

/* (4) 闭环轴向对齐验证 (agent 无法替代，必须交用户):
 *     yaw 环要 IMU yawRate 正向 = 麦轮 wz 正向 (CCW/左转)，
 *     符号在 IMU 驱动 (-gyro_y)。开环辨识只管单轮命令-反馈，
 *     不管整车旋转轴向，必须手动转动车体确认。
 *     改 YAW_SIGN / YAW_AXIS 后重做此验证。
 */

/* ============================================================
 * §F 视觉融合 / 故障保护 / 通信
 *
 *   视觉只给整数格 (单元约 0.20~0.23m)，量化噪声 ±0.10~0.115m。
 *   默认关闭运动中连续融合，改为:
 *     - 到站静止 → 多帧表决 → Snap 到格中心 (事件驱动)
 *     - 运动中一致性监控 → 大幅打滑/搬车时硬重定位
 * ============================================================ */

/* ---- 1) 运动中连续融合 (默认关) ---- */

#ifndef CHASSIS_VISION_FUSION_ENABLE
#define CHASSIS_VISION_FUSION_ENABLE            (0)
#endif

#ifndef CHASSIS_VISION_FUSION_PERIOD_MS
#define CHASSIS_VISION_FUSION_PERIOD_MS         (75U)
#endif

#ifndef CHASSIS_VISION_MAX_OBS_AGE_MS
#define CHASSIS_VISION_MAX_OBS_AGE_MS           (180U)
#endif

#ifndef CHASSIS_VISION_DELAY_COMP_MS
#define CHASSIS_VISION_DELAY_COMP_MS            (40U)
#endif

#ifndef CHASSIS_VISION_IGNORE_ERR_M
#define CHASSIS_VISION_IGNORE_ERR_M             (0.03f)
#endif

#ifndef CHASSIS_VISION_SOFT_ALPHA
#define CHASSIS_VISION_SOFT_ALPHA               (0.20f)
#endif

#ifndef CHASSIS_VISION_ALPHA_STATIC_SCALE
#define CHASSIS_VISION_ALPHA_STATIC_SCALE       (1.50f)
#endif

#ifndef CHASSIS_VISION_STATIC_SPEED_MPS
#define CHASSIS_VISION_STATIC_SPEED_MPS         (0.05f)
#endif

#ifndef CHASSIS_VISION_LARGE_ERR_M
#define CHASSIS_VISION_LARGE_ERR_M              (0.50f)
#endif

#ifndef CHASSIS_VISION_MAX_STEP_M
#define CHASSIS_VISION_MAX_STEP_M               (0.08f)
#endif

/* ---- 2) 到站 Snap (事件驱动；推荐默认开) ---- */

#ifndef CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE
#define CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE    (1)
#endif

#ifndef CHASSIS_VISION_SNAP_VOTE_FRAMES
#define CHASSIS_VISION_SNAP_VOTE_FRAMES         (5U)
#endif

#ifndef CHASSIS_VISION_SNAP_VOTE_MIN
#define CHASSIS_VISION_SNAP_VOTE_MIN            (3U)
#endif

#ifndef CHASSIS_VISION_SNAP_SETTLE_MS
#define CHASSIS_VISION_SNAP_SETTLE_MS           (150U)
#endif

#ifndef CHASSIS_VISION_SNAP_SETTLE_SPEED_MPS
#define CHASSIS_VISION_SNAP_SETTLE_SPEED_MPS    (0.05f)
#endif

#ifndef CHASSIS_VISION_SNAP_MAX_SNAP_AGE_MS
#define CHASSIS_VISION_SNAP_MAX_SNAP_AGE_MS     (200U)
#endif

#ifndef CHASSIS_VISION_SNAP_TIMEOUT_MS
#define CHASSIS_VISION_SNAP_TIMEOUT_MS          (800U)
#endif

#ifndef CHASSIS_VISION_SNAP_MAX_GAP_CELLS
#define CHASSIS_VISION_SNAP_MAX_GAP_CELLS       (1U)
#endif

/* ---- 3) 运动中一致性监控 (安全网) ---- */

#ifndef CHASSIS_VISION_CONSISTENCY_ENABLE
#define CHASSIS_VISION_CONSISTENCY_ENABLE       (1)
#endif

#ifndef CHASSIS_VISION_CONSISTENCY_PERIOD_MS
#define CHASSIS_VISION_CONSISTENCY_PERIOD_MS    (100U)
#endif

#ifndef CHASSIS_VISION_CONSISTENCY_GAP_CELLS
#define CHASSIS_VISION_CONSISTENCY_GAP_CELLS    (2U)
#endif

#ifndef CHASSIS_VISION_CONSISTENCY_HOLD_MS
#define CHASSIS_VISION_CONSISTENCY_HOLD_MS      (600U)
#endif

#ifndef CHASSIS_VISION_CONSISTENCY_COOLDOWN_MS
#define CHASSIS_VISION_CONSISTENCY_COOLDOWN_MS  (1000U)
#endif

#ifndef CHASSIS_VISION_CONSISTENCY_MAX_AGE_MS
#define CHASSIS_VISION_CONSISTENCY_MAX_AGE_MS   (250U)
#endif

/* ============================================================
 * §G MPC 直线行驶 (Model Predictive Control) — 1D 纵向
 *
 *   推箱子场景: 车沿网格轴线走直线, 推箱子时有变负载。
 *   MPC 提前 N 拍预知目标距离, 规划最优速度曲线,
 *   干扰后自动重规划。只接管驱动轴速度, 横向保持和
 *   航向保持沿用现有 hold_axis + yaw_pi。
 *
 *   CHASSIS_MPC_ENABLE=0 回退原 axis-by-axis sqrt_controller。
 * ============================================================ */

/** 1=MPC 接管驱动轴, 0=回退原 axis-by-axis */
#define CHASSIS_MPC_ENABLE                      (1)

/** 预测步数 (20ms/步), N=10 → 200ms 前瞻 */
#define CHASSIS_MPC_HORIZON_N                   (10)

/** QP 决策变量维度 = HORIZON_N */
#define CHASSIS_MPC_H_DIM                       (CHASSIS_MPC_HORIZON_N)

/** 对称矩阵 packed 存储元素数 = N*(N+1)/2 */
#define CHASSIS_MPC_H_PACKED                    ((CHASSIS_MPC_H_DIM) * ((CHASSIS_MPC_H_DIM) + 1) / 2)

/** 位置精度权重 (m⁻²): 大→冲得快/停得猛, 小→柔和但可能不到位 */
#define CHASSIS_MPC_Q_POS                       (8.0f)

/** 速度幅值代价 (s²/m²): 大→巡航速度降低, 小→更接近 v_max */
#define CHASSIS_MPC_R_VEL                       (0.15f)

/** 速度平滑代价 (s²/m²): 大→加减速柔和, 小→响应快但可能 jerk */
#define CHASSIS_MPC_R_SMOOTH                    (0.50f)

/** 终端位置额外权重因子: Q_term = Q_POS * 此值 */
#define CHASSIS_MPC_QF_FACTOR                   (3.0f)

/** MPC 驱动轴最大速度 (m/s), ≤ 全局限速 */
#define CHASSIS_MPC_V_MAX_MPS                   CHASSIS_MAX_LINEAR_SPEED_MPS

/** FISTA 最大迭代数 */
#define CHASSIS_MPC_FISTA_MAX_ITER              (15)

/** FISTA 梯度范数收敛容差 */
#define CHASSIS_MPC_FISTA_TOL                   (1e-4f)

/** FISTA 步长 (0=基于 H 的 Frobenius 范数自动计算) */
#define CHASSIS_MPC_FISTA_STEP                  (0.0f)

/** 距离超此值回退 sqrt_controller (m): 大跳后 MPC 短时域可能不够 */
#define CHASSIS_MPC_FALLBACK_ERR_M              (0.50f)

/** 距离低于此值不用 MPC (m): 近端驻车保持更可靠 */
#define CHASSIS_MPC_MIN_DIST_M                  (0.02f)

#endif /* _CONFIG_CHASSIS_H_ */
