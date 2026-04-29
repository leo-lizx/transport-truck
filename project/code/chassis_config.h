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
#define CHASSIS_MAX_LINEAR_SPEED_MPS    (1.35f)

/** 车体最大旋转角速度（°/s） */
#define CHASSIS_MAX_YAW_SPEED_DPS       (90.0f)

/**
 * 单轮 PID 调试模式下调试轮目标速度上限（m/s）。
 * 仅被 debug_target_speed_clamp() 使用。正常控制路径上不需要这个限幅,
 * 轮端速度由 CHASSIS_MAX_LINEAR_SPEED_MPS + CHASSIS_MAX_YAW_SPEED_DPS 上游限定。
 */
#define CHASSIS_DEBUG_WHEEL_SPEED_LIMIT_MPS  (5.0f)

/**
 * 正常控制路径的单轮速度兑底上限（m/s）。
 * apply_speed() 在麦轮正解后会扫描四轮目标, 只要有一个超过此值,
 * 就把四轮乘同一个缩放系数 (= LIMIT/max_abs), 保证 vx/vy/wz 的比例不变 ->
 * 车体运动方向不偏, 只是整体慢一点。
 *
 * 为什么需要: 单轮 PID 输出一旦撞上 PWM 上限 (±PWM_DUTY_MAX) 会被硬截断,
 * 等价于单独缩某个轮, vx/vy/wz 比例被破坏 -> 直线偏 / 转弯半径跳变。
 * 提前在 m/s 级别给个上限兑底, 能在饱和发生前等比例转化。
 *
 * 取值: 当前 MAX_LINEAR=1.35 + MAX_YAW=90dps 上游限住单轮最坏 ≈1.65 m/s,
 * 兑底设 8 m/s 留 ~5 倍裕量, 平时不触发, 仅在多指令叠加暴冲时生效。
 */
#define CHASSIS_WHEEL_SPEED_CAP_MPS     (8.0f)

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
#define CHASSIS_YAW_KP                  (3.20f)

/* ----- 航向闭环 (yaw_pi) 其余参数：原本散落在 chassis_ctrl.c, 集中到此 ----- */

/**
 * 航向误差死区（度）。
 * |err| <= 该值时输出 wz=0 且 I 项缓慢衰减, 抑制原地小幅抖动。
 * 调大: 更稳但允许残差变大；调小: 跟踪更紧但易抽搐。
 */
#define CHASSIS_YAW_DEADZONE_DEG        (0.30f)

/**
 * 航向 I 增益。
 * 用于消除稳态误差（如轮子对地摩擦不一致导致的偏角）。
 * 先把 KP 调到不振荡, 再缓慢加 KI；过大会反复过冲。
 */
#define CHASSIS_YAW_KI                  (0.030f)

/**
 * 航向积分项幅值上限（°·s）。
 * 防止长期堵转或大误差时积分饱和, 松开后冲过头。
 * 经验值: 30~120, 越保守越小。
 */
#define CHASSIS_YAW_I_LIMIT             (120.0f)

/**
 * 原地航向保持时的最小角速度补偿（°/s）。
 * 用于克服车轮静摩擦：误差脱离死区但 PI 输出还很小时, 强制给一个最小 wz。
 * 离地调试可设为 0；落地按"刚好能起转"来标定。
 */
#define CHASSIS_YAW_MIN_WZ_DPS          (10.0f)

/* ======================================================================
 *  轮速 PID 静摩擦突破 (Stiction breakaway)
 *
 *  问题: 增量式 PID 启动瞬间 output=0, 即便目标速度有十几 cm/s, 每 20ms 拍
 *  只涨 Kp*Δe + Ki*e 这点 PWM, 落到地面后橡胶轮 + 配重的静摩擦动辄要 1500+
 *  PWM 才肯转, 结果 PID 要爬好几秒才能"动起来" -> 看起来像是姿态环不工作。
 *
 *  机制: 当目标速度 |v_tgt| >= TARGET_EPS 但反馈 |v_fb| < SPEED_EPS, 且当前
 *  PID 累积输出 |u| < PWM_MIN 时, 把 u 强行拍到 ±PWM_MIN, 给电机一个突破
 *  静摩擦的初值。一旦轮子转起来 (|v_fb| >= SPEED_EPS) 就交回 PID 自动调速。
 *
 *  调参: 离地空载可调到 0 关闭; 落地若启动迟缓就把 PWM_MIN 往上抬, 直到
 *  地面上轻推一下方向就能立刻有响应。注意 PWM_MIN 不能高过 PWM_DUTY_MAX 的
 *  ~25%, 否则小目标速度会顿挫。
 * ====================================================================== */

/** 静摩擦突破: 目标速度阈值 (m/s), 低于此不触发 (避免 0 速度时上电就转) */
#define CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS  (0.020f)

/** 静摩擦突破: 反馈速度阈值 (m/s), 反馈低于此视为"还没转起来" */
#define CHASSIS_WHEEL_BREAKAWAY_SPEED_EPS_MPS   (0.030f)

/** 静摩擦突破: 强制施加的最小 PWM 幅值, 需大到能克服整车静摩擦
 *  PWM_DUTY_MAX=10000, 2500 ≈ 25% duty, 这是 4 麦轮整车的经验起转点;
 *  若仍迟缓, 按 500 一档往上加, 上限别超 4000 (40%) 否则小角度过冲。 */
#define CHASSIS_WHEEL_BREAKAWAY_PWM_MIN         (2500.0f)

/* ======================================================================
 *  IMU 航向角积分参数
 * ====================================================================== */

/**
 * IMU 偏航角来源轴选择 (P0-修复 2026-04-29 IMU 装反):
 *   原本 IMU 平装, yaw 来自 gyro_z (绕 Z 轴 / 偏航轴);
 *   现在 IMU 不可抗因素侧装/倒装, 原本测 pitch 的 Y 轴变成了真正的偏航轴,
 *   于是 yaw 应该来自 gyro_y (绕 Y 轴 / 原 pitch 轴)。
 *
 * 取值:
 *   0 = CHASSIS_IMU_YAW_AXIS_Z  原始 Z 轴 (IMU 正装, 标准方案)
 *   1 = CHASSIS_IMU_YAW_AXIS_Y  Y 轴      (IMU 侧装, 当前实车)
 *   2 = CHASSIS_IMU_YAW_AXIS_X  X 轴      (IMU 另一种侧装)
 *
 * 切换后若发现 yaw 增减方向跟实车转向反, 改 CHASSIS_IMU_YAW_SIGN 即可。
 */
#define CHASSIS_IMU_YAW_AXIS_Z           (0)
#define CHASSIS_IMU_YAW_AXIS_Y           (1)
#define CHASSIS_IMU_YAW_AXIS_X           (2)
#define CHASSIS_IMU_YAW_AXIS             CHASSIS_IMU_YAW_AXIS_Y

/**
 * Yaw 积分方向系数。
 * 若实车左转（逆时针）时 yaw 数值减小，请改为 -1.0f。
 */
#define CHASSIS_IMU_YAW_SIGN             (-1.0f)

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

/* ----------------------------------------------------------------------
 *  IMU 零偏在线辨识 (P0-改进 2026-04-29):
 *  ------------------------------------------------------------------
 *  问题: 仅靠 deadzone (单拍 |w|<TH) 判定静止, 实际车在轻微震动 / 慢速
 *        漂移时也会被认为静止 -> bias 被错误地往运动方向慢慢拉, 跑久了
 *        即便没人推车, yaw 也会持续漂。
 *
 *  方案: 滑动窗口方差检测静止 (Allan 方差 / VINS 静止判别的简化版)。
 *        每拍把"原始 gyro" 入循环 buffer, 检查最近 N 拍的均值/方差:
 *          - 方差 < STILL_VAR_TH (°/s)^2 -> 真静止
 *            -> 用窗口均值以 BIAS_FAST_ALPHA 快速更新 bias
 *          - 否则保持 bias 不变, 但仍允许 deadzone 抑制小噪声
 *
 *  优点: 通用, 不依赖具体安装轴, 也不依赖加速度计模长辅助。
 * ---------------------------------------------------------------------- */

/** 滑窗长度 (5ms 一拍, 64 = 320ms) - 长一点更稳, 太长则静止判定迟钝 */
#define CHASSIS_IMU_STILL_WINDOW_LEN     (64U)

/** 静止判别阈值: 窗口方差上限 (°/s)^2
 *  典型 IMU 静止噪声 ~0.05 °/s -> 方差 ~0.0025; 留 4~10 倍裕度
 *  阈值过大 -> 把缓慢运动当静止 -> bias 被污染;
 *  阈值过小 -> 永远不识别静止 -> bias 不更新, 长时间漂移 */
#define CHASSIS_IMU_STILL_VAR_TH_DPS2    (0.020f)

/** 静止确认时, bias 快速更新系数 (EMA 拉向窗口均值, 比慢通道快 50 倍)
 *  0.10 -> 时间常数 ~10 拍 = 50ms 完成一次 bias 校准 */
#define CHASSIS_IMU_BIAS_FAST_ALPHA      (0.10f)

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

/* ----------------------------------------------------------------------
 *  麦轮辊子布局: O 型 vs X 型
 *
 *  俯视车顶, 看每个轮上"暴露在外"那条辊子的方向:
 *    X 型: LF 辊 = "\", RF 辊 = "/", LB 辊 = "/", RB 辊 = "\"
 *          四条辊连起来在车顶画出一个 X
 *    O 型: LF 辊 = "/", RF 辊 = "\", LB 辊 = "\", RB 辊 = "/"
 *          四条辊连起来在车顶画出一个 O (菱形)
 *
 *  公式差异: O 型相对 X 型, 所有 vx (横移) 项整体翻号; vy 与 wz 项不变。
 *  代码层只需把这个开关切到 1, 正/逆运动学都会自动切换。
 *
 *  设为 1 = O 型 (当前实车);  设为 0 = X 型 (标准默认)。
 * ---------------------------------------------------------------------- */
#define CHASSIS_MECANUM_O_TYPE_LAYOUT   (1)

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
