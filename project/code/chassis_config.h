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
#define CHASSIS_START_GRID_Y            (6U)    /* 5×0.24=1.20m, 与实测车中心吻合 */

/** 到达目标点判定阈值（米），距目标小于此值即认为"已到达" */
#define CHASSIS_TARGET_REACHED_EPSILON_M  (0.03f)

/**
 * 到达目标点时的速度判据（m/s）.
 * dist < EPSILON 且 ||v_body|| < 该值 才置 s_arrived=1.
 * 防止车以高速穿过目标点瞬间触发到位, 惯性冲出再触发 HOLD_EXIT 反复震荡.
 * ROS Nav2 goal_checker 双判据标准: xy_goal_tolerance + vel_tolerance.
 */
#define CHASSIS_POS_ARRIVED_VEL_MPS       (0.05f)

/**
 * 到位后的 Schmitt 滞后释放阈值（米）.
 * 已到位状态下, 只有被推出此距离才重新开启位置驱动.
 * 放宽到 15cm: 物理动量在 cmd=0 后会让车滑行 5~10cm,
 * 8cm 太紧会反复触发, 15cm 给惯性留余量, 同时仍小于一格 (24cm).
 */
#define CHASSIS_POS_HOLD_EXIT_M           (0.15f)

/**
 * 扰动恢复模式触发距离 (米).
 * 当车被外力推出保持区且 dist < 该值时, 用「弱 KP + 强 KD」缓慢归位,
 * 不再以最大速度冲回 -> 避免回程过冲再震荡.
 * 业内 gain scheduling 标准做法 (Tesla Autopilot lateral controller).
 */
/**
 * 必须 > CHASSIS_POS_HOLD_EXIT_M (0.15m), 否则增益调度永不触发.
 * 逻辑: 被推出保持区时 dist > HOLD_EXIT → 增益调度只在 dist < RECOVERY_DIST 时生效,
 * 若 RECOVERY_DIST < HOLD_EXIT, 恢复全程在 RECOVERY_DIST 外 → 全量 KP → 必震荡.
 */
#define CHASSIS_POS_RECOVERY_DIST_M       (0.35f)

/** 扰动恢复模式 KP 缩放因子: 0.45 = 减少但仍有足够力驱动 */
#define CHASSIS_POS_RECOVERY_KP_SCALE     (0.45f)

/**
 * 轴向独立移动模式 (Manhattan / axis-by-axis).
 * 1 = 先走 X 走完再走 Y, 不走斜线
 * 0 = 走 CTE 直线 (旧默认)
 * 优点: 麦轮 X/Y 解耦控制更稳, 扰动恢复时只动一个轴, 不会对角震荡.
 */
#define CHASSIS_POS_AXIS_BY_AXIS_ENABLE   (1)

/**
 * axis-by-axis 模式: 当前轴误差 < 该值时切换到下一个轴 (米).
 * 这个值决定最终定位精度, 因此保持与 EPSILON 同量级.
 * 切轴后的重新锁回门限不要复用它, 否则 Y 行驶时 X 里程计/打滑漂移
 * 只要超过几厘米就会回切 X, 并把 Y ramp 清零, 表现为抖动前进.
 */
#define CHASSIS_POS_AXIS_SWITCH_TOL_M     (0.03f)

/**
 * axis-by-axis 模式: 已切入 Y 轴后, X 偏差超过该值才允许重新回切 X.
 * 必须明显大于普通编码器噪声和麦轮横向耦合漂移; 最终几厘米残差由终末
 * 2D PD 收敛, 不在 Y 行驶中反复抢轴.
 */
#define CHASSIS_POS_AXIS_RELOCK_TOL_M     (0.15f)

/**
 * 到位后 yaw 容忍带 (°). |yaw_err| < 该值即认为"航向也已到位",
 * wz 直接硬归零, 不再做任何修正.
 *
 * 这是 ROS Nav2 goal_checker 的 yaw_goal_tolerance 思路, 工业 AGV /
 * ArduPilot loiter / PX4 hold 模式都用同一套: 双 tolerance + 死区
 * 硬归零, 而不是让 PI 闭环去咬最后 1° 残差 (必产生极限环).
 */
#define CHASSIS_YAW_GOAL_TOLERANCE_DEG    (1.50f)

/* ======================================================================
 *  OpenART 视觉位姿融合（事件驱动 Snap 为主，连续融合为辅）
 *  --------------------------------------------------------------------
 *  设计选型 (见 视觉位姿融合实现说明.md §算法选型):
 *    - 视觉只给整数格 (单元约 0.23m)，量化噪声 ±0.115m 远大于短距 odom 漂移 (~3cm)。
 *    - 因此默认 *关闭* 运动中连续融合，改为「到站静止 → 多帧表决 → Snap 到格中心」+
 *      「运动中一致性监控 → 大幅打滑/搬车时硬重定位」两类事件触发。
 *    - 连续融合保留作可选，仅在视觉端升级到亚格坐标后才有正收益。
 * ====================================================================== */

/* ----- 1) 运动中连续融合 (默认关；语义专指"运动中"，不含到站 Snap) ---------- */

/** 1=运动中按 PERIOD_MS 周期做软融合；0=运动中纯里程计 (推荐默认) */
#ifndef CHASSIS_VISION_FUSION_ENABLE
#define CHASSIS_VISION_FUSION_ENABLE      (0)
#endif

/** 主循环侧融合调用周期门槛（ms），内部累加 GAME_LOGIC 5ms tick */
#ifndef CHASSIS_VISION_FUSION_PERIOD_MS
#define CHASSIS_VISION_FUSION_PERIOD_MS   (75U)
#endif

/** 观测时间戳过期丢弃（ms），大于此视为不可用 */
#ifndef CHASSIS_VISION_MAX_OBS_AGE_MS
#define CHASSIS_VISION_MAX_OBS_AGE_MS     (180U)
#endif

/** 固定链路+处理延迟补偿（ms），叠加在 obs_age 上做前推：pred += v*(age+delay)/1000 */
#ifndef CHASSIS_VISION_DELAY_COMP_MS
#define CHASSIS_VISION_DELAY_COMP_MS      (40U)
#endif

/** 小于该误差（m）不修正，避免噪声抽动 */
#ifndef CHASSIS_VISION_IGNORE_ERR_M
#define CHASSIS_VISION_IGNORE_ERR_M       (0.03f)
#endif

/** 中等误差下指数平滑系数：pose += alpha*(vision_pred - pose) */
#ifndef CHASSIS_VISION_SOFT_ALPHA
#define CHASSIS_VISION_SOFT_ALPHA         (0.20f)
#endif

/** 车体近乎静止时放大 alpha（乘以该系数后上限 1） */
#ifndef CHASSIS_VISION_ALPHA_STATIC_SCALE
#define CHASSIS_VISION_ALPHA_STATIC_SCALE (1.50f)
#endif

#ifndef CHASSIS_VISION_STATIC_SPEED_MPS
#define CHASSIS_VISION_STATIC_SPEED_MPS   (0.05f)
#endif

/** 大于该误差（m）认为观测可疑：仍可做校正但强制缩小有效 alpha（见实现） */
#ifndef CHASSIS_VISION_LARGE_ERR_M
#define CHASSIS_VISION_LARGE_ERR_M        (0.50f)
#endif

/** 单次融合 XY 最大修正模长（m），抑制突变 */
#ifndef CHASSIS_VISION_MAX_STEP_M
#define CHASSIS_VISION_MAX_STEP_M         (0.08f)
#endif

/* ----- 2) 到站 Snap (事件驱动；推荐默认开) -------------------------------- */

/** 1=底盘 odom 到位后再做"短静止 + 多帧表决 + Snap 到格中心"才放行下一航点 */
#ifndef CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE
#define CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE (1)
#endif

/** 表决窗口最多采样多少帧 (按 frame_id 去重) */
#ifndef CHASSIS_VISION_SNAP_VOTE_FRAMES
#define CHASSIS_VISION_SNAP_VOTE_FRAMES   (5U)
#endif

/** 占比最高的格至少要拿到的票数 (≤ VOTE_FRAMES) */
#ifndef CHASSIS_VISION_SNAP_VOTE_MIN
#define CHASSIS_VISION_SNAP_VOTE_MIN      (3U)
#endif

/** 进入 Snap 前要求"短静止"的持续时间 (ms)，与赛规 3s 静止解耦 */
#ifndef CHASSIS_VISION_SNAP_SETTLE_MS
#define CHASSIS_VISION_SNAP_SETTLE_MS     (150U)
#endif

/** 短静止判据的速度阈 (m/s) */
#ifndef CHASSIS_VISION_SNAP_SETTLE_SPEED_MPS
#define CHASSIS_VISION_SNAP_SETTLE_SPEED_MPS (0.05f)
#endif

/** 表决采样时单帧允许的最大帧龄 (ms) */
#ifndef CHASSIS_VISION_SNAP_MAX_SNAP_AGE_MS
#define CHASSIS_VISION_SNAP_MAX_SNAP_AGE_MS (200U)
#endif

/** 表决总超时 (ms)；超时则放行航点 (避免链路卡滞死锁) */
#ifndef CHASSIS_VISION_SNAP_TIMEOUT_MS
#define CHASSIS_VISION_SNAP_TIMEOUT_MS    (800U)
#endif

/** Snap 与目标格曼哈顿差 ≤ 该值才接受表决；超过认为视觉报错，放行 odom */
#ifndef CHASSIS_VISION_SNAP_MAX_GAP_CELLS
#define CHASSIS_VISION_SNAP_MAX_GAP_CELLS (1U)
#endif

/* ----- 3) 运动中一致性监控 (安全网) -------------------------------------- */

/** 1=每周期对比 odom 格 vs 视觉格，差距持续过大触发硬重定位 */
#ifndef CHASSIS_VISION_CONSISTENCY_ENABLE
#define CHASSIS_VISION_CONSISTENCY_ENABLE (1)
#endif

/** 监控调用周期 (ms)，内部按 5ms tick 累加 */
#ifndef CHASSIS_VISION_CONSISTENCY_PERIOD_MS
#define CHASSIS_VISION_CONSISTENCY_PERIOD_MS (100U)
#endif

/** 曼哈顿差阈值；持续 HOLD_MS 内不下降即触发 */
#ifndef CHASSIS_VISION_CONSISTENCY_GAP_CELLS
#define CHASSIS_VISION_CONSISTENCY_GAP_CELLS (2U)
#endif

/** 持续过大的判定窗 (ms) */
#ifndef CHASSIS_VISION_CONSISTENCY_HOLD_MS
#define CHASSIS_VISION_CONSISTENCY_HOLD_MS (600U)
#endif

/** 触发后冷却 (ms)，期间不再硬重定位，避免抖动 */
#ifndef CHASSIS_VISION_CONSISTENCY_COOLDOWN_MS
#define CHASSIS_VISION_CONSISTENCY_COOLDOWN_MS (1000U)
#endif

/** 监控帧的最大帧龄；过旧不参与判定 */
#ifndef CHASSIS_VISION_CONSISTENCY_MAX_AGE_MS
#define CHASSIS_VISION_CONSISTENCY_MAX_AGE_MS (250U)
#endif

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
#define CHASSIS_MAX_LINEAR_SPEED_MPS    (2.35f)

/** 车体最大旋转角速度（°/s）
 *  P0-修复 2026-04-29 姿态闭环转速慢: 原 90°/s 对应单轮仅 ≈0.4 m/s,
 *  远低于 MAX_LINEAR=1.35, 大量裕度被浪费 -> 提到 180°/s。
 *  P0-调参 2026-05-02 (姿态环中段忽快忽慢):
 *      180°/s 与 KP*err_max(≈90°/s @ KP=1.2) + ramp 不匹配。
 *      回到 150°/s, 单轮 ≈0.5 m/s, 与 PID 实际输出、轮端能力都匹配。 */
#define CHASSIS_MAX_YAW_SPEED_DPS       (150.0f)

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
 * 兑底设 16 m/s 留 ~10 倍裕量, 平时不触发, 仅在多指令叠加暴冲时生效。
 */
#define CHASSIS_WHEEL_SPEED_CAP_MPS     (16.0f)

/** 在线调参时允许的线速度上限硬限制（m/s） */
#define CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS  (3.50f)

/** 在线调参时允许的角速度上限硬限制（°/s） */
#define CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS     (360.0f)

/* ======================================================================
 *  导航控制增益（P 控制器参数）
 * ====================================================================== */

/** 位置环 Kp：沿程方向增益（沿目标方向前进的速度 = Kp × 沿程距离）
 *  物理意义: 决定「冲向目标」的速度, 与轨迹是否直线无关.
 *  建议范围 1.5 ~ 4.0, 偏大冲得快但近点可能超调. */
#define CHASSIS_POS_KP                  (2.50f)

/**
 * 位置环横向增益 Kp_cross（Cross-Track Error 修正增益）
 *   物理意义: 横向偏差→横向修正速度. 设得比 CHASSIS_POS_KP 大
 *   可以更快地把车「推回直线」而不影响前进速度.
 *   建议 = 1.5 ~ 3 倍 CHASSIS_POS_KP. 设为 0 退化为纯 P(弧线).  */
#define CHASSIS_POS_CTE_KP              (5.0f)

/**
 * 位置环 D 项增益 (沿程方向) - 速度阻尼.
 * 业内标准 derivative-on-measurement: D 反馈用 odom 直接给的速度
 * (s_fb_vx/vy), 不做差分以避免噪声放大. 等价于 PD 控制器, 抑制
 * 由电机/麦轮惯性引起的 "冲过头 -> 倒回 -> 再冲" 前后震荡.
 *
 * 物理含义: v_cmd = KP * err - KD * v_meas
 *   KD=0   : 纯 P, 欠阻尼必震荡
 *   KD=KP  : 临界阻尼附近, 最快无超调
 *   KD>KP  : 过阻尼, 收敛变慢但绝不超调
 * 麦轮一般取 KD ≈ KP, 起点用 1.0 ~ 1.5 倍.
 */
#define CHASSIS_POS_KD                  (1.0f)

/** 位置环横向 D 项增益 - 与 CTE_KP 配套 */
#define CHASSIS_POS_CTE_KD              (2.0f)

/**
 * D 项低通滤波系数 α (一阶 IIR, Tesla/Waymo 标准做法).
 * y = (1-α)*y_prev + α*x,  截止频率 ≈ α/(2π·dt).
 * α=0.30 @ dt=20ms -> 截止 ≈ 2.4Hz, 衰减 odom 高频噪声 (~20Hz) 约 -18dB.
 * 增大 α -> 响应更快但噪声更多; 减小 -> 更平滑但阻尼延迟增大.
 */
#define CHASSIS_POS_D_LPF_ALPHA         (0.30f)

/**
 * 位置环最小推进速度 (m/s) - P0-改进 2026-05-02 (终末段龟速爬):
 *   纯 P 在 dist 接近 epsilon 时输出极小 (dist=5cm, KP=0.9 -> v=0.045 m/s),
 *   分到四个麦轮后单轮速度逼近静摩擦门槛, 表现为"间歇前进、慢慢爬完最后几 cm".
 *   做法: 当 dist > epsilon 但 P 输出模长 < V_MIN 时, 把模长抬到 V_MIN,
 *   方向仍由 dx/dy 决定. 工程上取略高于轮端 BREAKAWAY 启动速度.
 *   设为 0 -> 关闭最小推进, 退化为纯 P. */
#define CHASSIS_POS_MIN_DRIVE_SPEED_MPS  (0.02f)

/**
 * 位置环 yaw 跟踪门距 (m) - P0-修复 2026-05-02 (atan2 噪声风暴):
 *   接近目标时 dx/dy 都很小, atan2(dy, dx) 对 odom 噪声极其敏感:
 *     dist=5cm, odom 噪声 1cm -> tgt_yaw 跳 ±10°
 *     dist=1cm, odom 噪声 1cm -> tgt_yaw 跳 ±90° -> wz 直接饱和摆头
 *   工程做法 (ROS Navigation / ArduPilot WP nav 同款): 距离门控
 *     dist > 该值 -> 跟踪 atan2(dy,dx)  (远段对准方位角)
 *     dist < 该值 -> 冻结目标 yaw       (近段只关心位置, 朝向无所谓)
 *   建议 = 5 ~ 10 倍 EPSILON, 保证近场区也比 odom 噪声 (~1cm) 大一个量级. */
#define CHASSIS_POS_YAW_TRACK_DIST_M     (0.20f)

/**
 * 位置环到达后惯性补偿距离 (m) - P0-修复 2026-05-02 (硬刹冲过头):
 *   dist <= EPSILON 时直接 force_stop, 但 ramp 滤波器和电机仍有惯性输出,
 *   实际位置可能超目标 1~3cm. 此处用"软到达": dist 进入 BRAKE_DIST 后开始
 *   按线性把 V_MIN 衰减到 0, 进 EPSILON 时已经基本停下, 不再硬刹.
 *   建议 = 2 ~ 3 倍 V_MIN * 20ms (~5cm). */
#define CHASSIS_POS_BRAKE_DIST_M         (0.10f)

/** 航向环 Kp：值越大，朝向对准越快；过大易振荡
 *  P0-调参 2026-04-29: 取消 YAW_MIN_WZ 阶跃后 wz 连续, 可适度提 KP 加快响应。
 *  P0-调参 2026-05-02 (大角度阶跃响应慢 + 中段"假停"):
 *      原 0.8: err=90° 时 wz=72°/s, 不到 MAX_YAW=150 一半, 中段被 D 项抵后几乎不转。
 *      调到 1.2: err=125° 即到 max, 完整利用 ramp/MAX 限幅, 180°阶跃 ≈1.5s 动作.
 *  P0-改进 2026-05-02 (sqrt_controller 移植):
 *      yaw_pi 改用 ArduPilot 的 sqrt_controller 后, KP 只控"零附近"的小角度增益,
 *      大角度由 sqrt(2·a·err) 自动饱和到 max_yaw_speed, 与 KP 无关.
 *      故 KP 可大胆放到 4~6, 让 0~20° 段也能跑得快, 而不会引入大角度过冲.
 *  P0-回调 2026-05-02 (持续抖动):
 *      KP=4 + KD=0.01 + 死区 1° -> 在 ±1° 内激出高频小振荡. 回到 2.0,
 *      与 KD=0.08 / 死区 2° 配套, 小角度仍比老 0.65 快 3 倍, 不抖. */
#define CHASSIS_YAW_KP                  (1.900f)

/** sqrt_controller 用的最大角加速度 (°/s²) - P0-改进 2026-05-02
 *  物理意义: 终末减速段每秒能掉多少 °/s 的角速度.
 *  设大 -> 减速距离短, 跟踪更紧, 但要求底盘能真正减得下来 (受轮抓地力 / 轮速环带宽限制),
 *          否则会"刹不住"出现超调.
 *  设小 -> 减速段长, 平滑无超调但收敛慢.
 *  与 CHASSIS_CMD_ACCEL_LIMIT_DPS2 (上游 ramp 限幅) 保持同量级, 这里取 1200 留 1.2x 裕量. */
#define CHASSIS_YAW_ACCEL_MAX_DPS2      (1200.0f)

/* ----------------------------------------------------------------------
 *  级联 P-PI + 速度前馈 (P0-改进 2026-05-02 大角度慢/稳态偏差):
 *  ------------------------------------------------------------------
 *  原结构: 单环位置 PID, err -> wz, KP 控全程 -> 大慢小抖两难.
 *  新结构 (ArduPilot AC_AttitudeControl 同款):
 *
 *      err_ang ─[sqrt_ctrl]─> wz_target ─┬───── (前馈) ─────┐
 *                                         │                  ▼
 *                                         └─→ + ─[ PI ]─> + ── wz_cmd
 *                                       wz_meas
 *
 *  外环 (sqrt_ctrl): 算"应该多快转", 与 KP_v 无关, 大角度自动饱和到 max_yaw
 *  内环 (rate PI):    跟踪 wz_target - wz_meas, I 全程累积 -> 自动学到摩擦补偿
 *  前馈 (wz_target):  让大角度时输出直接饱和, 不依赖 KP 增益
 *
 *  好处:
 *    - 大角度: 前馈直接饱和到 max_yaw, 与 KP_pos 无关 -> 110° 不再慢
 *    - 稳态:   I 在整个运动过程中持续学习, 收尾时已有足够能量克服静摩擦 -> 无 10° 偏差
 *    - 整定:   KP_pos 只决定收敛形状, KP_v / KI_v 决定跟踪精度, 解耦清晰
 * ---------------------------------------------------------------------- */

/** 1 = 启用级联 P-PI; 0 = 旧的单环 PID (回退用) */
#define CHASSIS_YAW_USE_CASCADED_CTRL    (1)

/** 内环 (rate loop) 比例增益: wz_err -> wz_correction
 *  小 (0.3~0.5): 弱跟踪, 主要靠前馈, 稳; 大 (1~2): 紧跟踪, 但易激出抖动.
 *  与外环解耦, 只决定"跟随快慢", 不影响轨迹形状. */
#define CHASSIS_YAW_RATE_KP             (0.150f)

/** 内环积分增益: 全程累积, 自动学到摩擦/不平衡转矩
 *  这是治本"稳态偏差"的关键. 小 (0.5): I 学得慢, 残差残留时间长;
 *  大 (5+): 学得快但容易过冲. 1~3 工业典型. */
#define CHASSIS_YAW_RATE_KI             (0.70f)

/** 内环积分上限 (°/s 量纲, 等价于"I 项最多能贡献多少 wz")
 *  设为 max_yaw 的 30~50% 合适, 太大易反向卷绕过冲 */
#define CHASSIS_YAW_RATE_I_LIMIT        (120.0f)

/** 内环积分泄漏率 (per 20ms): 0 = 不泄漏 (纯积分), 0.001~0.01 = 慢泄漏
 *  作用: 长期堵转时防止 I 永久挂账, 避免负载消失瞬间冲过头.
 *  0.005 等效时间常数 ~4s, 工程经验值. */
#define CHASSIS_YAW_RATE_I_LEAK         (0.003f)

/* ----------------------------------------------------------------------
 *  In-Position 滞回锁 (P0-改进 2026-05-02 收尾抖动):
 *  ----------------------------------------------------------------------
 *  问题: err 在死区内 (1.5°) 时, wz_target = KP·err = 3°/s 仍非零, 经轮端
 *        breakaway (PWM=1200 阶跃) 把车体一抖 -> IMU 看到反向 yaw_rate ->
 *        err 翻号 -> wz 翻号 -> 反向阶跃 -> 20Hz 极限环.
 *
 *  解法: 工业伺服 (Yaskawa PSEL / Mitsubishi INP / ArduPilot pos_hold) 通用做法:
 *        Schmitt-trigger 双阈值锁
 *          进入: |err| < IN_POS  AND  |wz_meas| < SETTLE_RATE  -> 锁死, wz=0, I 冻结
 *          释放: |err| > OUT_POS                               -> 解锁, 恢复 PI
 *        OUT_POS > IN_POS 提供滞回, 防 IMU 噪声 / 微动反复触发.
 *
 *  一旦"在位", 整条 sqrt->前馈->PI 全部硬归零, 断开 wz_target -> 轮端 breakaway
 *  的传染路径, 极限环消失.
 * ---------------------------------------------------------------------- */

/** 进入"在位"阈值 (°): 误差必须小于此值才考虑锁死 */
#define CHASSIS_YAW_INPOS_ENTER_DEG     (1.50f)

/** 释放"在位"阈值 (°): 误差超过此值才解锁, 必须 > ENTER 才有滞回
 *  典型 1.5x ~ 2x ENTER, 太小没滞回意义, 太大跟踪精度变差 */
#define CHASSIS_YAW_INPOS_EXIT_DEG      (3.00f)

/** "稳定"判据: 车体角速度低于此值才认为真停下来了 (°/s)
 *  设小: 难锁住; 设大: 正在转就被锁了, 影响动态精度 */
#define CHASSIS_YAW_INPOS_SETTLE_DPS    (25.00f)

/* ----- 航向闭环 (yaw_pi) 其余参数：原本散落在 chassis_ctrl.c, 集中到此 ----- */

/**
 * 航向误差死区（度）。
 * |err| <= 该值时 P/D 项依然计算 (输出连续), 仅接近 0 时 I 项衰减,
 * 用“软死区”避免 wz 阶跃 -> 保证姿态环输出连续。
 * 调大: 允许更大残差但更稳; 调小: 跟踪更紧但易抽搽。
 * (P0-修复 2026-04-29: 原者在死区内直接 return 0 造成 wz 阶跃,
 *  现仅用于控制 I 项衰减, 不再阶跃输出)
 *
 *  P0-回调 2026-05-02 (持续抖动): 1.0° 太紧, IMU 噪声 + 轮端微动直接把车推出
 *  死区, 反复触发 P/D, 表现为持续抖. 回到 2.0°. */
#define CHASSIS_YAW_DEADZONE_DEG        (1.60f)

/**
 * 航向 I 增益。
 * 用于消除稳态误差（如轮子对地摩擦不一致导致的偏角）。
 * 先把 KP 调到不振荡, 再缓慢加 KI；过大会反复过冲。
 */
#define CHASSIS_YAW_KI                  (0.45f)
/**
 * 航向 D 增益 (P0-新增 2026-04-29 抗超调).
 * D = -KD * yaw_rate_dps (derivative-on-measurement, 无 setpoint kick).
 * 物理意义: 阻尼项, 车体转得越快越要"踩刹车", 抑制冲过头.
 *
 * 调参顺序:
 *   1. 先把 KD=0, 调 KP 到刚好不振荡;
 *   2. 加 KD: 0.05 起步, 每次 +0.05;
 *   3. 太大: 高频抖动 / 听到电机嗡嗡响 -> 回退;
 *   4. 太小: 仍有 5%+ 超调 -> 继续加.
 * 经验范围: 0.05 ~ 0.50, 当前 0.10 适合中等惯量 (4 麦轮 + 摄像头云台).
 *
 * P0-修复 2026-05-02 (姿态环超调反弹/大角度振荡):
 *   原 KD=0 完全没有阻尼 -> 车体冲过零点无刹车 -> 反向超调 5~10°.
 *   开到 0.10 提供基础阻尼, 配合下面 I_BAND/I_LIMIT 收窄, 三件套一起改.
 *
 * P0-调参 2026-05-02 (中段"假停"现象):
 *   KD=0.10 在车体 yaw_rate=80°/s 时 d_term = -8°/s, 把 P=72°/s 抵一半,
 *   中段看起来“转不动". 降到 0.05 仍留 ≥1/4 阻尼, 超调 ≤3° 完全可接受.
 *
 * P0-回调 2026-05-02 (持续抖动):
 *   KD=0.01 等于没阻尼, sqrt_controller 高 KP 下小角度发散. 提到 0.08:
 *   yaw_rate=80°/s 时 d_term=-6.4°/s, 比 KP·err (2·5°=10°/s) 小, 中段不"假停";
 *   yaw_rate=10°/s 时 d_term=-0.8°/s, 终末段刚好压住小幅振荡. */
#define CHASSIS_YAW_KD                  (0.10f)

/**
 * 航向条件积分带宽 (°, P0-新增 2026-04-29 防积分饱和).
 * 仅当 |err| < 此值时才累积 I 项, 大误差阶段 I 上锁.
 * 这样大角度阶跃响应不会"先冲过头再回拉", 显著缩短调节时间.
 *
 * 取值: 通常 = 死区*2 ~ 期望稳态精度*5, 当前 5° 对应车体已转到接近目标
 * 才开始消除残差.
 *   - 调小: 稳态更准但中等误差残留时间变长;
 *   - 调大: 收敛更快但可能恢复轻度超调.
 *
 * P0-修复 2026-05-02 (姿态环大角度振荡):
 *   原 20° 太宽, 大误差阶段也在累 I -> 到达目标时 I 已严重饱和 -> 过冲反弹.
 *   收紧到 5°, 让 I 项只在收尾阶段消静差.
 */
#define CHASSIS_YAW_I_BAND_DEG          (5.0f)
/**
 * 航向积分项幅值上限（°·s）。
 * 防止长期堵转或大误差时积分饱和, 松开后冲过头。
 * 经验值: 30~120, 越保守越小。
 *
 * P0-修复 2026-05-02 (姿态卡角不动):
 *   原 120 + KI=0.05 -> i_term 可达 ±6°/s, 足以与 P 项对抗导致车卡角
 *   (P 项 0.8*err 在 err=20° 时也才 16°/s, 反向 I 抵掉后实际驱动很弱).
 *   收到 30°·s 即 i_term 上限 ±1.5°/s, 仅能消静差不再压住 P.
 *
 * P0-回调 2026-05-02 (持续抖动):
 *   200·KI=200·0.45=90°/s, 上一次回调被忘了同步, 这里也是抖动重要源.
 *   恢复 30, i_term 上限 ±13.5°/s, 足够消摩擦差异不会反过来推车抖.
 */
#define CHASSIS_YAW_I_LIMIT             (30.0f)

/**
 * 原地航向保持时的最小角速度补偿（°/s）。
 *
 * P0-重要修复 2026-04-29 (姿态闭环 jump-rotate):
 *   原设计用途: 补偿车轮静摩擦 -> err 出死区但 PI 输出还很小时
 *   强制拍个最小 wz, 让车子能动起来。
 *   问题: 该补偿是个阶跃函数 (wz 从真实 PI 值 一下跳到 ±10),
 *   导致轮端目标速度也阶跃, 与轮端 breakaway 状态机双重阶跃叠加
 *   -> PWM “一段一段”地给 -> 车体一跳一跳地转。
 *
 *   现改为 0 禁用: 静摩擦补偿交给轮端 breakaway 机制处理
 *   (见 chassis_config.h 同名节), 它按目标缩放 PWM + 迟滞防抖,
 *   不会造成 wz 阶跃。
 *
 * 什么时候考虑重启 (调回 ≥5):
 *   - 如果未来改拿位置环, 发现在某些场合 breakaway 不足, 车身微转
 *     仍起不来, 可适度调到 3~5°/s 作为堆叠保险。但一般不需要。
 */
#define CHASSIS_YAW_MIN_WZ_DPS          (0.0f)

/* ======================================================================
 *  轮速 PID 静摩擦前馈 (Stiction feed-forward)
 *
 *  原问题: 增量式 PID 启动瞬间 output=0, 落地后 4 麦轮整车静摩擦动辄
 *  1500+ PWM, PID 要爬好几秒才能动起来。
 *
 *  当前实现 (apply_speed): 加性平滑前馈, 不覆写 PID 内部状态。
 *      ff = sign(target) * FLOOR * smooth_ramp((|target|-EPS)/EPS)
 *      pwm = pid_output + ff
 *  - |target| ≤ EPS:        ff = 0,        全靠 PID
 *  - EPS < |target| < 2EPS: ff 从 0 线性升到 FLOOR (边界连续过零)
 *  - |target| ≥ 2EPS:       ff = ±FLOOR    (静摩擦补偿到位)
 *
 *  仅 2 个调参旋钮:
 *      TARGET_EPS_MPS — 触发阈值, 也是 ramp 分母
 *      PWM_FLOOR      — 饱和幅值
 *
 *  调参: 落地起步迟 -> FLOOR +200; 起步过冲 -> FLOOR -200。
 * ====================================================================== */

/** 静摩擦前馈: 触发阈值 (m/s), 同时用作 smooth_ramp 分母
 *  P0-修复 2026-05-02 (小角度旋转四轮"一个一个动"):
 *      原 0.050: 轮端 target<0.05 时 ff=0, 而 wz 需 ≥15°/s 才能让轮 target 出阈,
 *      也就是 yaw 误差 <12.5° 时四轮没有静摩擦补偿, 各轮各靠 PID 慢慢爬,
 *      谁先克服自己的静摩擦谁先动 -> "一个一个动".
 *      调到 0.020: wz ≥6°/s 即可让 ff 介入, 四轮齐步起步;
 *                  静止 target=0 仍然 ff=0; 死区刚出时 (target≈0.008) ff 仍为 0
 *                  不会引入微抖. */
#define CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS  (0.010f)

/** 静摩擦前馈: 饱和幅值 (PWM 原始单位, PWM_DUTY_MAX=10000)
 *  10% duty 是 4 麦轮整车的经验起转点; 别超 4000 (40%) 否则小目标过冲 */
#define CHASSIS_WHEEL_BREAKAWAY_PWM_FLOOR       (1200.0f)

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
#define CHASSIS_IMU_GYRO_DEADZONE_DPS    (1.8f)

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
 *  阈值过小 -> 永远不识别静止 -> bias 不更新, 长时间漂移
 *
 *  P0-回调 2026-05-02 (多次运动后 yaw 累积误差):
 *  车跑跑停停时, 0.020 太严, 平台微震动就识别不到静止 -> bias 永远不更新
 *  -> 温漂导致 yaw 越走越偏. 放宽到 0.050: 完全静止仍能稳定 ZUPT, 同时
 *  车停在地面手没碰时也能算静止 (典型方差 ~0.01~0.03), bias 持续刷新.
 *  KF 对污染的容忍度高 (Q_BIAS 也提高), 阈值放宽风险可控. */
#define CHASSIS_IMU_STILL_VAR_TH_DPS2    (0.050f)

/** 静止确认时, bias 快速更新系数 (EMA 拉向窗口均值, 比慢通道快 50 倍)
 *  0.10 -> 时间常数 ~10 拍 = 50ms 完成一次 bias 校准 */
#define CHASSIS_IMU_BIAS_FAST_ALPHA      (0.10f)

/* ----------------------------------------------------------------------
 *  Yaw 卡尔曼滤波 (P0-改进 2026-05-02):
 *  ------------------------------------------------------------------
 *  状态: x = [angle, bias]ᵀ  (yaw 角 / gyro 零偏, 单位 deg / dps)
 *  预测: angle' = angle + (gyro - bias) * dt
 *        bias'  = bias                 (随机游走模型)
 *  观测: ZUPT (Zero-rate Update) - 静止时认定真实角速度=0,
 *        故 gyro 本身就是对 bias 的一次观测:  z = gyro,  H = [0, 1]
 *
 *  与原 EMA 方案的差异:
 *    EMA  : 静止时 bias += alpha * (mean - bias),  alpha 固定
 *    KF   : K 增益由 P 矩阵自适应 -> 开机 P 大收敛快, 久静止 P 小抗扰强
 *
 *  为什么不用 acc 做 yaw 观测:
 *    yaw 轴永远与重力对齐, 绕重力轴旋转不改变 acc 投影 -> acc 测不到 yaw.
 *    无 magnetometer / vision 时, KF 的"外部观测"只能来自 ZUPT.
 * ---------------------------------------------------------------------- */

/** 1 = 用 KF 替换原 EMA bias + 欧拉积分; 0 = 维持旧实现 (回退用) */
#define CHASSIS_IMU_USE_KALMAN_YAW       (1)

/** 过程噪声: 角度 (°²/s²), 反映陀螺白噪声. 大 -> 信任观测多, 小 -> 信任预测多
 *  典型 MEMS gyro ARW ≈ 0.1°/√h ≈ 1.7e-3 °/s/√Hz, 200Hz 采样 -> Q_angle ≈ 6e-4 */
#define CHASSIS_IMU_KF_Q_ANGLE_DPS2      (0.0010f)

/** 过程噪声: bias (°²/s²/s), 反映 bias 随机游走速度. 太大 -> bias 抖, 太小 -> bias 不更新
 *  典型 MEMS gyro bias 不稳定度 ≈ 10°/h ≈ 2.8e-3 °/s, Q_bias ≈ (2.8e-3)² ≈ 8e-6
 *
 *  P0-回调 2026-05-02 (多次运动后 yaw 误差累积):
 *  原 8e-6 假设 bias 极慢漂, K 增益小, 就算 ZUPT 触发了 bias 也学不动.
 *  实测车跑 5 分钟后温升明显, 真实 bias 漂移可达 0.05 °/s/min,
 *  对应 Q_bias ≈ (0.05/60)² · 200 ≈ 1.4e-4. 取 5e-5 折中 (兼顾静止稳定),
 *  让 K 增益翻几倍, 静止 1~2 秒就能把 bias 拉准. */
#define CHASSIS_IMU_KF_Q_BIAS_DPS2_PER_S (0.00005f)

/** 观测噪声 (°²/s²): ZUPT 时陀螺噪声方差. 用静止窗口噪声实测约 0.05 dps -> R ≈ 0.0025 */
#define CHASSIS_IMU_KF_R_ZUPT_DPS2       (0.0050f)

/** 状态协方差初值: angle 不知道 (1°²), bias 标定后还有少量残差 (0.1°²/s²) */
#define CHASSIS_IMU_KF_P0_ANGLE_DEG2     (1.0f)
#define CHASSIS_IMU_KF_P0_BIAS_DPS2      (0.10f)

/* ----------------------------------------------------------------------
 *  编码器 yaw 融合 (P0-改进 2026-05-02):
 *  ------------------------------------------------------------------
 *  目的: 用四轮编码器算出的 yaw 角速度积分作为弱观测, 喂进 KF,
 *        长期约束 IMU scale factor 漂移. 短期内打滑 -> R 调大让 KF 不信.
 *
 *  原理 (麦轮 O 型 yaw 逆运动学):
 *      wz_odom = (-vLF + vRF - vLB + vRB) / (4 * K_mecanum)
 *  在 chassis_ctrl 20ms 任务里积分 -> yaw_odom_deg, 喂给
 *  chassis_imu_kf_correct_angle(yaw_odom_deg, R) 完成融合.
 * ---------------------------------------------------------------------- */

/** 1 = 启用编码器 yaw 弱观测; 0 = 关闭 (调试时纯靠 IMU) */
#define CHASSIS_ODOM_YAW_FUSION_ENABLE   (1)

/** 编码器 yaw 观测噪声方差 (°²).
 *  大 -> KF 几乎不信, 仅做长期纠偏 (推荐); 小 -> KF 信任高, 打滑会污染.
 *  100 (= 10° std) 大约 1 分钟才把 1° 真实漂移拉一半, 既能压住 IMU 长期累积,
 *  又不会被瞬间打滑(常 < 5°)拉走. */
#define CHASSIS_ODOM_YAW_R_DEG2          (100.0f)

/** odom yaw 与 IMU yaw 偏差超过此值视为打滑 / 重定位事件,
 *  本拍跳过这次观测 (避免单次大跳跃污染 KF) */
#define CHASSIS_ODOM_YAW_OUTLIER_DEG     (15.0f)

/** Innovation Gating: |wz_odom - wz_imu| 超过此值视为编码器异常 (°/s),
 *  本拍跳过 KF 注入. (P0-修复 2026-05-02 抬车空转污染 KF):
 *
 *  抬起车后用手转车体, 麦轮空转产生虚假 wz_odom -> 错误地把 KF angle 拉走,
 *  导致 err 被消, 系统认为已对齐, 实际车体仍偏几十度. 工业 KF (ArduPilot
 *  EKF / px4 EKF2) 通用门控: 比较两路独立测量, 严重不符即拒收. 这里用
 *  陀螺直接测的 wz_imu 与编码器算的 wz_odom 比对, 抬车场景下两者必然
 *  剧烈不符 (IMU=0, odom>>0), 自然拒绝.
 *
 *  设小 (10): 严苛, 高速转向时也可能误拒 (轮胎打滑常见 wz_odom 偏 5~10°/s)
 *  设大 (50): 宽松, 但抬车空转判别迟钝
 *  20°/s 是工程平衡点, 正常打滑差 < 10, 抬车差 > 50, 留 2x 余量. */
#define CHASSIS_ODOM_YAW_INNOV_GATE_DPS  (20.0f)

/* ======================================================================
 *  缓加速参数 — 防止目标速度突变导致轮胎打滑
 * ====================================================================== */

/**
 * 线速度最大加速度（m/s²），每 20ms 允许的最大变化量.
 * Manhattan 轴切换时 vy 从 0 起步, accel 决定"拐弯后加速时间":
 *   1.2 m/s²: 0→2.35 m/s 需 ~2s, 2m 行程全程在爬坡, 最高仅 1.55 m/s
 *   3.0 m/s²: 0→2.35 m/s 仅 0.78s, 2m 行程可短暂跑满速
 * 麦轮横向移动靠滚轮分力, 不依赖轮端抓地, 比纵向更耐高加速.
 */
#define CHASSIS_CMD_ACCEL_LIMIT_MPS2    (3.00f)

/** 角速度最大加速度（°/s²）
 *  P0-调参 2026-05-02 (大角度阶跃响应慢):
 *      原 240 -> 0→180°/s 要 750ms, 这段时间车几乎没转 -> 看起来"假停".
 *      提到 720, 0→150°/s 仅 ≈210ms, 与 PID 响应节奏匹配。
 *      不担心轮胎打滑: yaw 加速是转动, 单轮线加速在 ramp 下还是受
 *      cmd_accel_limit_mps2 限制。 */
#define CHASSIS_CMD_ACCEL_LIMIT_DPS2    (720.0f)

/* ======================================================================
 *  里程计标定系数 — 补偿轮径/打滑等误差
 *
 *  标定方法：
 *    让车走 1 格(20cm)，用尺子测量实际位移 d_real，
 *    则 SCALE = 0.20 / d_real
 * ====================================================================== */

/** X 方向里程计缩放系数 */
#define CHASSIS_ODOM_SCALE_X            (0.417809f)

/** Y 方向里程计缩放系数 */
//#define CHASSIS_ODOM_SCALE_Y            (0.447301336986f)
#define CHASSIS_ODOM_SCALE_Y            (0.41f)

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
 * MOTOR1: DIR=C9,  PWM=C8   -> 右后轮 RB
 * MOTOR2: DIR=C7,  PWM=C6   -> 左后轮 LB
 * MOTOR3: DIR=D2,  PWM=D3   -> 右前轮 RF
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
 * 电机 PWM/DIR: 实车接线验证后物理走 D3/D2, 以当前宏定义为准.
 * 编码器: ENCODER_3 (QTIMER2_ENCODER1, C3/C4) -- 已经 wfb 调试输出验证, 别动.
 */
#define CHASSIS_RF_PWM_CHANNEL      PWM2_MODULE3_CHB_D3            /**< PWM 输出: D3 引脚 (物理 -> RF 电机) */
#define CHASSIS_RF_DIR_PIN          D2                             /**< 方向控制: D2 引脚 (物理 -> RF 电机) */
#define CHASSIS_RF_ENC_INDEX        QTIMER2_ENCODER1               /**< 编码器定时器通道（实车标定：ENCODER3） */
#define CHASSIS_RF_ENC_CH1          QTIMER2_ENCODER1_CH1_C3        /**< 编码器 A 相: C3 引脚 */
#define CHASSIS_RF_ENC_CH2          QTIMER2_ENCODER1_CH2_C4        /**< 编码器 B 相: C4 引脚 */
#define CHASSIS_RF_DIR_SIGN         (1.0f)                        /**< 方向修正: 1.0=正向, -1.0=反向 */

/* ---------- 左后轮 (LB) ---------- *
 * 实车标定: MOTOR2(C6/C7) + ENCODER_1(QTIMER1_ENCODER1, C0/C1)
 */
#define CHASSIS_LB_PWM_CHANNEL      PWM2_MODULE0_CHA_C6             /**< PWM 输出: C6  引脚 (MOTOR2) */
#define CHASSIS_LB_DIR_PIN          C7                              /**< 方向控制: C7  引脚 (MOTOR2) */
#define CHASSIS_LB_ENC_INDEX        QTIMER1_ENCODER1                /**< 编码器定时器通道（实车标定：ENCODER1） */
#define CHASSIS_LB_ENC_CH1          QTIMER1_ENCODER1_CH1_C0         /**< 编码器 A 相: C0  引脚 */
#define CHASSIS_LB_ENC_CH2          QTIMER1_ENCODER1_CH2_C1         /**< 编码器 B 相: C1  引脚 */
#define CHASSIS_LB_DIR_SIGN         (1.0f)                          /**< 方向修正: 1.0=正向, -1.0=反向 */

/* ---------- 右后轮 (RB) ---------- *
 * 电机 PWM/DIR: 实车接线验证后物理走 C8/C9, 以当前宏定义为准.
 * 编码器: ENCODER_2 (QTIMER1_ENCODER2, C2/C24) -- 已经 wfb 调试验证.
 */
#define CHASSIS_RB_PWM_CHANNEL      PWM2_MODULE1_CHA_C8             /**< PWM 输出: C8 引脚 (物理 -> RB 电机) */
#define CHASSIS_RB_DIR_PIN          C9                               /**< 方向控制: C9 引脚 (物理 -> RB 电机) */
#define CHASSIS_RB_ENC_INDEX        QTIMER1_ENCODER2                 /**< 编码器定时器通道（实车标定：ENCODER2） */
#define CHASSIS_RB_ENC_CH1          QTIMER1_ENCODER2_CH1_C2          /**< 编码器 A 相: C2  引脚 */
#define CHASSIS_RB_ENC_CH2          QTIMER1_ENCODER2_CH2_C24         /**< 编码器 B 相: C24 引脚 */
#define CHASSIS_RB_DIR_SIGN         (1.0f)                          /**< 方向修正: 1.0=正向, -1.0=反向 */

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
