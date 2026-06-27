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
 *  用户常调参数区
 *
 *  优先只调下面 5 个角度环参数。其它 yaw 内部参数已固定或自动推导,
 *  不需要日常修改。
 * ====================================================================== */

/** 航向响应快慢: 越大越快, 过大可能轻微摆动 */
#define CHASSIS_YAW_KP                  (4.20f)

/** 角速度 P-only 阻尼: 越大越稳, 过大可能发闷.
 * P0-调 2026-06-07: 0.05 几乎没阻尼, yaw 来回摆 → 车一直抖.
 * 提到 0.25 加 5 倍阻尼, 抑制 yaw 震荡. */
#define CHASSIS_YAW_RATE_KP             (0.35f)

/** 最大旋转速度 (°/s): 限制原地转向和导航修正的最高角速度 */
#define CHASSIS_MAX_YAW_SPEED_DPS       (170.0f)

/** 角速度加减速限制 (°/s²): 同时用于 yaw sqrt 曲线和下游 ramp */
#define CHASSIS_CMD_ACCEL_LIMIT_DPS2    (720.0f)

/** 进入在位锁的角度阈值 (°): 越小锁得越准, 越大越不抖 */
#define CHASSIS_YAW_INPOS_ENTER_DEG     (0.30f)

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
#define CHASSIS_MAP_WIDTH_M             (2.80f)

/** 可通行区域物理高度（米） */
#define CHASSIS_MAP_HEIGHT_M            (2.00f)

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

/**
 * 到达目标点判定阈值（米）.
 * POINT_NAV 当前采用最简单的纯位置判断:
 *   dist = sqrt(dx² + dy²) <= EPSILON 立即置 s_arrived=1.
 * 不再叠加速度稳定帧 / dwell 等待 / yaw 条件, 避免车已在误差范围内但上层迟迟不切点.
 */
#define CHASSIS_TARGET_REACHED_EPSILON_M  (0.025f)

/**
 * 到位后的 Schmitt 滞后释放阈值（米）.
 * 已到位状态下, 只有被推出此距离才重新开启位置驱动.
 * 10cm 足够覆盖停车后几厘米的惯性滑移, 又不会让位置闭环在大范围内失效.
 */
#define CHASSIS_POS_HOLD_EXIT_M           (0.15f)

/* 注:
 *   原扰动恢复 (RECOVERY_*)、轴保持速度上限 (AXIS_HOLD_MAX_SPEED)、
 *   轴切换/回切门限 (AXIS_SWITCH_TOL / AXIS_RELOCK_TOL)、
 *   以及曼哈顿开关 (AXIS_BY_AXIS_ENABLE) 已于「位置环参数精简」中删除:
 *     - 曼哈顿轴模式成为唯一实现, 不再有 CTE 直线分支;
 *     - 减速统一由 brake cap (sqrt 限速) 负责, 不再单独做 gain scheduling / recovery;
 *     - 轴保持速度、切轴门限改为按 max_speed / EPSILON 的比例自动推导 (见下方比例常量).
 */

/**
 * 到位后 yaw 容忍带 (°). |yaw_err| < 该值即认为"航向也已到位",
 * wz 直接硬归零, 不再做任何修正.
 *
 * 这是 ROS Nav2 goal_checker 的 yaw_goal_tolerance 思路, 工业 AGV /
 * ArduPilot loiter / PX4 hold 模式都用同一套: 双 tolerance + 死区
 * 硬归零, 而不是让 PI 闭环去咬最后 1° 残差 (必产生极限环).
 *
 * P0-修复 2026-05-12 (走斜线根因): 1.5° → 0.5°.
 *   POINT_NAV 中该值是 "动态走行中" 的 yaw 死区, 不是到位判据.
 *   1.5° 太宽: vxg=1.2 m/s 投影出 vyg = 1.2*sin(1.5°) = 31 mm/s,
 *   1m 走行累积 26mm 偏移, CTE 保持环上限 0.06 m/s 追不上 -> 走斜线.
 *   收到 0.5°: vyg 言上限 10 mm/s, 1m 仅 8mm 偏, CTE 可轻松修复.
 *   超出容忍带后由 yaw sqrt_ctrl + P-only 阻尼实时拉回.
 * P0-修复 2026-06-07: 0.5°→0.2°. 0.5°下每米漂 8.7mm, 与保持轴 1cm 死区
 *   叠加形成盲区 → 小 yaw 偏差无人管 → 车走不直. 0.2°每米仅漂 3.5mm. */
#define CHASSIS_YAW_GOAL_TOLERANCE_DEG    (0.10f)

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
#define CHASSIS_LAUNCH_ZONE_BOTTOM_OFFSET_M  (1.0f)

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
#define CHASSIS_STATIC_SPEED_EPS_MPS    (0.07f)

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
/* P0-修复 2026-05-19 (轮子停转): 1.20 → 0.70.
 *   1.20 m/s 时单轮峰值电流超 TB6612 过流阈值 → 驱动器关断 → 轮停.
 *   麦轮斜走单轮最坏 = 1.20×√2≈1.70 m/s, 冲击更大.
 *   0.70 m/s: d_stop=0.70²/(2×2.10)=0.117m, BRAKE_DIST=0.25m ✓ 有裕量. */
#define CHASSIS_MAX_LINEAR_SPEED_MPS    (2.10f)

/* CHASSIS_MAX_YAW_SPEED_DPS 已移到文件顶部“用户常调参数区”。 */

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
#define CHASSIS_WHEEL_SPEED_CAP_MPS     (2.70f)

/** 在线调参时允许的线速度上限硬限制（m/s） */
#define CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS  (4.0f)

/** 在线调参时允许的角速度上限硬限制（°/s） */
#define CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS     (360.0f)

/* ======================================================================
 *  导航控制增益（P 控制器参数）
 * ====================================================================== */

/** 位置环 Kp：沿程方向增益（沿目标方向前进的速度 = Kp × 沿程距离）
 *  物理意义: 决定「冲向目标」的速度, 与轨迹是否直线无关.
 *  建议范围 1.5 ~ 4.0, 偏大冲得快但近点可能超调.
 *  P0-调参 2026-05-08 (收敛太慢): 2.50 → 3.50, 加快远场逼近速度. */
/* P0-修复 2026-05-11 (拐点停留+走斜线): 10.50 → 4.50.
 * KP=10.5 时 linear_dist = accel/KP² = 3.0/110.25 ≈ 0.027m, 意味着
 * 只有 2.7cm 内是线性段, 稍远就切 sqrt 段大速度, 车高速冲入 EPSILON
 * 后 brake_cap 来不及刹停, 穿越后反弹. 恢复 4.5 使 linear_dist≈0.15m,
 * 近场仍保持 P 线性响应, 配合 BRAKE_DIST=0.25 可以平滑停车. */
#define CHASSIS_POS_KP                  (8.5f)

/** 位置环沿程方向积分增益 (m/s per m·s).
 * 消除静摩擦/坡面等引起的稳态位置残差.
 * 建议从 0 开始调, 每次 +0.05; 过大时车到位后缓慢漂移/越界. 设 0 即关闭. */
#define CHASSIS_POS_KI                  (0.17f)

/* ----------------------------------------------------------------------
 *  位置环自动推导 / 内部固定常量 (用户一般无需修改)
 *  --------------------------------------------------------------------
 *  「位置环参数精简」后, 以下量不再各自暴露成独立可调 #define, 而是由
 *  pos_kp / max_linear_speed_mps / cmd_accel_limit_mps2 (均为运行时可调) 与
 *  EPSILON / HOLD_EXIT 按固定比例在 chassis_ctrl.c 中自动推导:
 *
 *    kd_eff          = pos_kp * KD_RATIO            (主轴速度阻尼, 麦轮 ≈ KP/3)
 *    kd_hold         = pos_kp * HOLD_KD_RATIO       (保持轴轻阻尼)
 *    brake_dist      = v_max²/(2·accel) + EPSILON + BRAKE_MARGIN
 *    i_limit         = v_max * I_LIMIT_RATIO        (积分最多贡献一半速度)
 *    i_band          = brake_dist * I_BAND_RATIO    (只在减速区内积分)
 *    switch_tol      = EPSILON * AXIS_SWITCH_RATIO  (切轴精度略严于到位精度)
 *    hold_max_speed  = v_max * HOLD_SPEED_RATIO     (非驱动轴保持速度上限)
 *  另有两个与轮端特性绑定、对所有麦轮底盘通用的固定值:
 *    d_lpf_alpha     = D_LPF_ALPHA  (D 项一阶 IIR 系数)
 *    brake_floor     = BRAKE_FLOOR  (克服静摩擦的末段最小速度)
 * ---------------------------------------------------------------------- */

/** 主轴速度阻尼比例: kd_eff = pos_kp × 该值. 原 0.30, 提高以增强刹车阻尼,
 *  配合更高的 KI 使用, 防止积分推力导致过冲震荡. 0.50 约 2/3 临界阻尼. */
#define CHASSIS_POS_KD_RATIO            (0.30f)

/** 保持轴 (非驱动轴) 阻尼比例: kd_hold = pos_kp × 该值. 保持轴限速很低, 取轻阻尼. */
#define CHASSIS_POS_HOLD_KD_RATIO       (2.0f)

/** brake_dist 安全裕量 (m): brake_dist = d_stop + EPSILON + 该值. */
#define CHASSIS_POS_BRAKE_MARGIN_M      (-0.006f)

/** 积分输出上限比例: i_limit = max_linear_speed × 该值.
 *  原 0.5 (贡献一半速度), 降低以抑制积分过冲 → 震荡. */
#define CHASSIS_POS_I_LIMIT_RATIO       (0.20f)

/** 条件积分带宽比例: i_band = brake_dist × 该值 (只在减速区内累积积分).
 *  原 0.8, 缩小以推迟积分介入, 近端才发力, 避免远距离积分卷绕 → 震荡. */
#define CHASSIS_POS_I_BAND_RATIO        (0.8f)

/** 切轴门限比例: switch_tol = EPSILON × 该值 (切轴精度略严于到位精度). */
#define CHASSIS_POS_AXIS_SWITCH_RATIO   (0.6f)

/** 非驱动轴保持速度上限比例: hold_max_speed = max_linear_speed × 该值. */
#define CHASSIS_POS_HOLD_SPEED_RATIO    (0.25f)

/**
 * 保持轴死区 (m): |hold_error| ≤ 该值时保持轴不输出修正力.
 * 1cm 足以滤除编码器量化噪声 (~2~5mm) 和 odom 积分短时抖动,
 * 同时远小于 EPSILON=8cm, 不会让保持轴长期漂移累积.
 * P0-修复 2026-06-07: 原保持轴无死区, 编码器噪声被 sqrt_controller
 * 放大成 0.06~0.12 m/s 的修正脉冲 → Y 方向持续微幅震荡.
 */
#define CHASSIS_POS_HOLD_DEAD_ZONE_M     (0.002f)

/**
 * D 项低通滤波系数 α (一阶 IIR, Tesla/Waymo 标准做法).
 * y = (1-α)*y_prev + α*x, 截止频率 ≈ α/(2π·dt).
 * 0.15 @ dt=20ms -> 截止 ≈ 1.2Hz, 衰减 odom 高频噪声, 对所有麦轮底盘通用. */
#define CHASSIS_POS_D_LPF_ALPHA         (0.10f)

/**
 * brake_cap 末段最小有效速度 (m/s).
 * 保证 sqrt 减速曲线末段输出仍足以克服静摩擦, 车始终能推进到 EPSILON 内.
 * 只与轮端静摩擦特性相关, 与 0.05 的轮端 ff 起步阈值同量级. */
#define CHASSIS_POS_BRAKE_FLOOR_MPS      (0.450f)

/* ----------------------------------------------------------------------
 *  航向环实现: sqrt_ctrl + P-only 速率阻尼
 *
 *  常调参数已集中到文件顶部:
 *    CHASSIS_YAW_KP / CHASSIS_YAW_RATE_KP / CHASSIS_MAX_YAW_SPEED_DPS
 *    CHASSIS_CMD_ACCEL_LIMIT_DPS2 / CHASSIS_YAW_INPOS_ENTER_DEG
 *
 *  内部固定规则:
 *    - sqrt_ctrl 的角加速度与下游 ramp 共用 CHASSIS_CMD_ACCEL_LIMIT_DPS2
 *    - In-position 释放阈值 = ENTER_DEG * 2.0
 *    - In-position 角速度稳定判据 = 25.0°/s
 *    - rate 内环无 I 项, 只做 P-only 阻尼
 * ---------------------------------------------------------------------- */

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

/* ======================================================================
 *  静摩擦前馈 (Breakaway feed-forward) — 每轮独立可配
 *
 *  原问题: 增量式 PID 启动瞬间 output=0, 落地后 4 麦轮整车静摩擦动辄
 *  1500+ PWM, PID 要爬好几秒才能动起来. 且四轮电机/减速箱/轮胎磨损不同,
 *  静摩擦差异可达 ±30%, 全局统一 FLOOR 无法兼顾"弱轮起不来"和"强轮过冲".
 *
 *  实现 (apply_speed): 每轮独立查 g_chassis_tune_params 中的值
 *      if |target| > target_eps[i] && |fb| < fb_static_eps[i]:
 *          pwm += sign(target) * pwm_floor[i]
 *
 *  每轮 3 个调参旋钮:
 *      TARGET_EPS_MPS   — 触发阈值 (m/s), 同时用作 smooth_ramp 分母
 *      PWM_FLOOR        — 饱和幅值 (PWM_DUTY_MAX=10000)
 *      FB_STATIC_EPS_MPS — "反馈视为静止"门槛 (m/s), 超过此值撤 ff
 *
 *  调参口诀:
 *      该轮起步迟 / 不动  -> 加大该轮 FLOOR
 *      该轮起步猛冲 / 过冲 -> 减小该轮 FLOOR
 *      该轮起步一抖一抖    -> 加大该轮 FB_STATIC_EPS (让 ff 保持更久)
 *      该轮小目标也冲      -> 降低该轮 TARGET_EPS (更晚触发 ff)
 * ====================================================================== */

/* ---- 默认值 (作为各轮初值模板, 也供旧代码兼容引用) ---- */

/** 静摩擦前馈: 触发阈值 (m/s)
 *  P0-修复 2026-05-02: 原 0.050 太高 -> 小角度旋转四轮"一个一个动".
 *  0.008 -> wz>=2.5dps 即可触发 ff, 四轮齐步起步. */
#define CHASSIS_WHEEL_BREAKAWAY_DEFAULT_TARGET_EPS_MPS     (0.08f)

/** 静摩擦前馈: 饱和幅值 (PWM 原始单位, PWM_DUTY_MAX=10000)
 *  10% duty 是 4 麦轮整车的经验起转点; 别超 4000 (40%) 否则小目标过冲 */
#define CHASSIS_WHEEL_BREAKAWAY_DEFAULT_PWM_FLOOR          (1000.0f)

/** 静摩擦前馈"反馈视为静止"门槛 (m/s):
 *  仅当 target>EPS 且 |fb|<该值时才注入 ff, 一旦轮真正起转 ff=0 交给 PID.
 *  设大 -> ff 保持更久 -> 起步更果断但可能过冲;
 *  设小 -> ff 撤得快 -> 可能一抖一抖 (ff 反复开关). */
#define CHASSIS_WHEEL_BREAKAWAY_DEFAULT_FB_STATIC_EPS_MPS  (2.80f)

/* ---- 左前轮 LF (Wheel 0) ---- */
/* P0-修复 2026-06-07: TARGET_EPS=0+FB_STATIC=2.10 → breakaway 永远激活.
 * 位置环收拢时指令小幅正负交替 → breakaway 跟跳 2000PWM → 把车甩出极限环.
 * TARGET_EPS→0.03: 极小目标不触发; FB_STATIC→0.10: 轮一转就撤, PID 接管. */
#define CHASSIS_WHEEL_BREAKAWAY_LF_TARGET_EPS_MPS     (0.03f)
#define CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LF_FB_STATIC_EPS_MPS  (0.90f)

/* ---- 右前轮 RF (Wheel 1) ---- */
#define CHASSIS_WHEEL_BREAKAWAY_RF_TARGET_EPS_MPS     (0.03f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_FB_STATIC_EPS_MPS  (0.90f )

/* ---- 左后轮 LB (Wheel 2) ---- */
#define CHASSIS_WHEEL_BREAKAWAY_LB_TARGET_EPS_MPS     (0.03f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_FB_STATIC_EPS_MPS  (0.90f)

/* ---- 右后轮 RB (Wheel 3) ---- */
#define CHASSIS_WHEEL_BREAKAWAY_RB_TARGET_EPS_MPS     (0.03f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_FB_STATIC_EPS_MPS  (0.90f)

/* ---- X 正向 (vx>0) 静摩擦前馈 PWM 幅值, 每轮独立 ---- */
#define CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR_XP          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR_XP          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR_XP          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR_XP          (800.0f)

/* ---- X 负向 (vx<0) 静摩擦前馈 PWM 幅值, 每轮独立 ---- */
/* P0-修复 2026-06-07: 原 XN 不对称 (LB=1200 独大) → -X 起步时左侧推力 > 右侧
 * → 偏航力矩 → yaw 环介入 → Y 轴耦合震荡. 改为与 XP 对称互换 (方向反了所以左右对调):
 *   XP: LF=1000 RF=1200 LB=1000 RB=1400  (右侧强)
 *   XN: LF=1200 RF=1000 LB=1400 RB=1000  (左侧强, 与 XP 对称)
 * 两侧推力均衡后 yaw 力矩抵消, 不再干扰 Y 保持轴. */
#define CHASSIS_WHEEL_BREAKAWAY_LF_PWM_FLOOR_XN          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RF_PWM_FLOOR_XN          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_LB_PWM_FLOOR_XN          (800.0f)
#define CHASSIS_WHEEL_BREAKAWAY_RB_PWM_FLOOR_XN          (800.0f)

/* 兼容宏: 保持旧名称可用 (指向默认值, 新代码请用 g_chassis_tune_params 中的值) */
#define CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS    CHASSIS_WHEEL_BREAKAWAY_DEFAULT_TARGET_EPS_MPS
#define CHASSIS_WHEEL_BREAKAWAY_PWM_FLOOR         CHASSIS_WHEEL_BREAKAWAY_DEFAULT_PWM_FLOOR
#define CHASSIS_WHEEL_BREAKAWAY_FB_STATIC_EPS_MPS CHASSIS_WHEEL_BREAKAWAY_DEFAULT_FB_STATIC_EPS_MPS

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
 *  阈值过小 -> 永远不识别静止 -> bias 不更新, 长时间漂移
 *
 *  P0-回调 2026-05-02 (多次运动后 yaw 累积误差):
 *  车跑跑停停时, 0.020 太严, 平台微震动就识别不到静止 -> bias 永远不更新
 *  -> 温漂导致 yaw 越走越偏. 放宽到 0.050: 完全静止仍能稳定 ZUPT, 同时
 *  车停在地面手没碰时也能算静止 (典型方差 ~0.01~0.03), bias 持续刷新.
 *  KF 对污染的容忍度高 (Q_BIAS 也提高), 阈值放宽风险可控. */
#define CHASSIS_IMU_STILL_VAR_TH_DPS2    (0.020f)

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
#define CHASSIS_ODOM_YAW_R_DEG2          (40.0f)

/** odom yaw 与 IMU yaw 偏差超过此值视为打滑 / 重定位事件,
 *  本拍跳过这次观测 (避免单次大跳跃污染 KF) */
#define CHASSIS_ODOM_YAW_OUTLIER_DEG     (3.0f)

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
 * P0-调参 2026-05-08 (收敛太慢): 3.00 → 5.00, 加快速度命令爬坡/刹车响应.
 * P0-修复 2026-06-07: 0.60 太低, ramp 每拍只能变 0.012 m/s,
 * brake cap 要求减速但 ramp 跟不上 → 近目标冲过头 → 回程方向反转时尤其严重.
 * 提到 3.00: dv=0.06/拍, 从 0.5→0.1 仅需 7 拍=140ms, brake cap 能生效. */
#define CHASSIS_CMD_ACCEL_LIMIT_MPS2    (3.00f)

/** 角速度最大加速度（°/s²）
 *  P0-调参 2026-05-02 (大角度阶跃响应慢):
 *      原 240 -> 0→180°/s 要 750ms, 这段时间车几乎没转 -> 看起来"假停".
 *      提到 720, 0→150°/s 仅 ≈210ms, 与 PID 响应节奏匹配。
 *      不担心轮胎打滑: yaw 加速是转动, 单轮线加速在 ramp 下还是受
 *      cmd_accel_limit_mps2 限制。 */
/* CHASSIS_CMD_ACCEL_LIMIT_DPS2 已移到文件顶部“用户常调参数区”。 */

/* ======================================================================
 *  里程计标定系数 — 补偿轮径/打滑等误差
 *
 *  标定方法：
 *    让车走 1 格(20cm)，用尺子测量实际位移 d_real，
 *    则 SCALE = 0.20 / d_real
 * ====================================================================== */

/** X 方向里程计缩放系数 */
//#define CHASSIS_ODOM_SCALE_X            (0.468539f)
#define CHASSIS_ODOM_SCALE_X            (0.3930f)

/** Y 方向里程计缩放系数 */
#define CHASSIS_ODOM_SCALE_Y            (0.4130f)

//#define CHASSIS_ODOM_SCALE_Y            (0.43617f)

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

/* 4 轮 PID 统一: 旧版 Ki 在 20~35 之间各异 + Kd=0,
 * 4 轮阶跃响应不同步 -> 行进中持续轻微抖, 转弯时车体扭.
 * 改成 Kp=40 / Ki=10 / Kd=1.5 (Kd≈Kp/27, 增量 PID 适度阻尼).
 * 4 轮硬件差异应通过 ENC_SIGN/DIR_SIGN 标定, 不该让 PID 背锅. */
#define CHASSIS_WHEEL_PID_LF_KP         (70.0f)
#define CHASSIS_WHEEL_PID_LF_KI         (20.0f)
#define CHASSIS_WHEEL_PID_LF_KD         (0.0f)

#define CHASSIS_WHEEL_PID_RF_KP         (70.0f)
#define CHASSIS_WHEEL_PID_RF_KI         (20.0f)
#define CHASSIS_WHEEL_PID_RF_KD         (0.0f)

#define CHASSIS_WHEEL_PID_LB_KP         (70.0f)
#define CHASSIS_WHEEL_PID_LB_KI         (20.0f)
#define CHASSIS_WHEEL_PID_LB_KD         (0.0f)

#define CHASSIS_WHEEL_PID_RB_KP         (70.0f)
#define CHASSIS_WHEEL_PID_RB_KI         (20.0f)
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
#define CHASSIS_RF_DIR_SIGN         (-1.0f)                        /**< 方向修正: 1.0=正向, -1.0=反向 */

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
#define CHASSIS_RB_DIR_SIGN         (-1.0f)                          /**< 方向修正: 1.0=正向, -1.0=反向 */

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
 * @brief  可通行网格 X 索引 -> 车体中心物理坐标 X（米）
 *         以可通行区域左边界为 0m, 整数格返回对应格中心。
 */
static inline float chassis_grid_x_to_m(uint8 grid_x)
{
    int32 inner_x = (int32)chassis_clamp_grid_x_inner(grid_x) - (int32)CHASSIS_GRID_INNER_MIN_X;
    return (((float)inner_x + 0.5f) * CHASSIS_GRID_STEP_X_M);
}

/**
 * @brief  可通行网格 Y 索引 -> 车体中心物理坐标 Y（米）
 *         以可通行区域上边界为 0m, 整数格返回对应格中心。
 */
static inline float chassis_grid_y_to_m(uint8 grid_y)
{
    int32 inner_y = (int32)chassis_clamp_grid_y_inner(grid_y) - (int32)CHASSIS_GRID_INNER_MIN_Y;
    return (((float)inner_y + 0.5f) * CHASSIS_GRID_STEP_Y_M);
}

/**
 * @brief  车体中心物理坐标 X（米）-> 可通行网格 X 索引
 */
static inline uint8 chassis_m_to_grid_x(float x_m)
{
    int32 grid_x = (int32)(x_m / CHASSIS_GRID_STEP_X_M) + (int32)CHASSIS_GRID_INNER_MIN_X;
    if (grid_x < (int32)CHASSIS_GRID_INNER_MIN_X) grid_x = (int32)CHASSIS_GRID_INNER_MIN_X;
    if (grid_x > (int32)CHASSIS_GRID_INNER_MAX_X) grid_x = (int32)CHASSIS_GRID_INNER_MAX_X;
    return (uint8)grid_x;
}

/**
 * @brief  车体中心物理坐标 Y（米）-> 可通行网格 Y 索引
 */
static inline uint8 chassis_m_to_grid_y(float y_m)
{
    int32 grid_y = (int32)(y_m / CHASSIS_GRID_STEP_Y_M) + (int32)CHASSIS_GRID_INNER_MIN_Y;
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

