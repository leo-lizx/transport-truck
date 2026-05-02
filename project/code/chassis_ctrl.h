#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

/*===========================================================================
 * [chassis_ctrl.h] 底盘顶层控制模块
 *
 *   整合 IMU 航向、编码器测速、麦轮运动学、PID 轮控、里程计定位、
 *   定点移动导航等功能，对外提供简洁统一的 API。
 *
 *   本模块是底盘子系统的唯一对外入口，上层代码（如 app_game_logic）
 *   不需要直接调用 motor/encoder/imu/mecanum 等底层模块。
 *
 *   典型调用流程：
 *     main.c 中调用: chassis_ctrl_init()
 *     5ms PIT 中断:  chassis_ctrl_task_5ms()
 *     20ms PIT 中断: chassis_ctrl_task_20ms()
 *     业务层:        chassis_ctrl_move_to_grid(x, y)
 *                    chassis_ctrl_set_move_yaw_cmd(vx, vy, yaw)
 *                    while (!chassis_ctrl_is_arrived()) { ... }
 *
 *   内部控制流程（20ms 周期内依次执行）：
 *     ① 读取编码器 → 计算四轮实际速度
 *     ② 麦轮逆运动学 → 估算车体速度
 *     ③ 里程计积分 → 更新全局位姿 (x, y)
 *     ④ 导航 P 控制器 → 计算目标车体速度
 *     ⑤ 缓加速滤波 → 麦轮正运动学 → PID → 电机输出
 *
 * [模块依赖]:
 *   chassis_config.h  — 所有引脚和参数配置
 *   chassis_imu.h     — IMU 航向采样
 *   chassis_encoder.h — 编码器测速
 *   chassis_motor.h   — 电机驱动
 *   chassis_pid.h     — PID 控制器
 *   chassis_mecanum.h — 麦轮运动学
 *===========================================================================*/

#include "chassis_config.h"

/* ========================== 公共数据类型 ========================== */

/** 车体位姿：全局坐标系下的位置和航向角 */
typedef struct
{
    float x_m;       /**< 全局 X 坐标（米），向右为正 */
    float y_m;       /**< 全局 Y 坐标（米），向前为正 */
    float yaw_deg;   /**< 航向角（度），逆时针为正，范围 [-180, +180] */
} chassis_pose_t;

/** 车体速度指令：车体坐标系下的三自由度速度 */
typedef struct
{
    float vx_body_mps;   /**< X 方向速度（m/s），向右为正 */
    float vy_body_mps;   /**< Y 方向速度（m/s），向前为正 */
    float wz_dps;        /**< 转向角速度（°/s），逆时针为正 */
} chassis_body_speed_cmd_t;

/** 航向闭环调试信息（用于 0 度航向保持调试打印） */
typedef struct
{
    float target_yaw_deg;   /**< 目标航向角（度） */
    float current_yaw_deg;  /**< 当前航向角（度） */
    float yaw_err_deg;      /**< 航向误差（度） */
    float wz_cmd_dps;       /**< 当前角速度指令（°/s） */
} chassis_attitude_debug_info_t;

/** 运行时可调参数（用于按键菜单在线调参） */
#define CHASSIS_CTRL_TUNE_WHEEL_COUNT   (4U)

typedef struct
{
    float wheel_pid_kp[CHASSIS_CTRL_TUNE_WHEEL_COUNT];
    float wheel_pid_ki[CHASSIS_CTRL_TUNE_WHEEL_COUNT];
    float wheel_pid_kd[CHASSIS_CTRL_TUNE_WHEEL_COUNT];

    float pos_kp;
    float yaw_kp;

    float max_linear_speed_mps;
    float max_yaw_speed_dps;

    float cmd_accel_limit_mps2;
    float cmd_accel_limit_dps2;
} chassis_tune_params_t;

/* 全局运行时调参参数（单一数据源，默认值在 chassis_ctrl.c 初始化） */
extern volatile chassis_tune_params_t g_chassis_tune_params;

/* ========================== 公共 API ========================== */

/**
 * @brief  初始化底盘控制子系统
 *         包括：IMU、4 路编码器、4 路电机 PWM/DIR、4 路 PID
 *         初始化后电机处于停止状态
 */
void chassis_ctrl_init(void);

/**
 * @brief  5ms 高频任务：读取 IMU 陀螺仪并积分航向角
 *         在 5ms PIT 中断中调用
 */
void chassis_ctrl_task_5ms(void);

/**
 * @brief  20ms 低频任务：编码器 → 里程计 → 导航控制 → PID → 电机
 *         在 20ms PIT 中断中调用
 */
void chassis_ctrl_task_20ms(void);

/**
 * @brief  下发网格坐标目标，底盘自动移动到该位置
 *         竞赛地图采用 12×16，总外框边界不可进入。
 *         可通行索引范围: x=[CHASSIS_GRID_INNER_MIN_X, CHASSIS_GRID_INNER_MAX_X]
 *                       y=[CHASSIS_GRID_INNER_MIN_Y, CHASSIS_GRID_INNER_MAX_Y]
 *         超出范围自动钳位到可通行内场
 * @param  target_x_grid  目标 X 网格索引
 * @param  target_y_grid  目标 Y 网格索引
 */
void chassis_ctrl_move_to_grid(uint8 target_x_grid, uint8 target_y_grid);

/**
 * @brief  以米为单位下发位置目标，并锁定全程航向角（不跟踪 atan2）
 * @param  x_m          目标 X 坐标（米）
 * @param  y_m          目标 Y 坐标（米）
 * @param  hold_yaw_deg 全程保持的航向角（度）
 */
void chassis_ctrl_move_to_m(float x_m, float y_m, float hold_yaw_deg);

/**
 * @brief  下发“移动 + 航向”指令（车体系速度 + 航向闭环目标角）
 *         该模式下不走点位导航，直接按速度指令运动，同时执行航向闭环。
 * @param  vx_body_mps     车体系 X 方向速度（m/s，向右为正）
 * @param  vy_body_mps     车体系 Y 方向速度（m/s，向前为正）
 * @param  target_yaw_deg  目标航向角（度）
 */
void chassis_ctrl_set_move_yaw_cmd(float vx_body_mps, float vy_body_mps, float target_yaw_deg);

/**
 * @brief  切换到航向保持模式：停止位置移动，仅保持指定航向角
 * @param  target_yaw_deg  目标航向角（度）
 */
void chassis_ctrl_hold_yaw(float target_yaw_deg);

/**
 * @brief  启动单轮 PID 调试模式
 *         仅指定轮子参与速度闭环，其余轮子目标速度固定为 0。
 * @param  wheel_index      调试轮子索引（0:LF 1:RF 2:LB 3:RB）
 * @param  target_speed_mps 该轮目标速度（m/s）
 */
void chassis_ctrl_start_single_wheel_pid_debug(uint8 wheel_index,
                                               float target_speed_mps);

/**
 * @brief  更新单轮 PID 调试目标速度（m/s）
 * @param  target_speed_mps 新目标速度（m/s）
 */
void chassis_ctrl_set_single_wheel_pid_debug_target(float target_speed_mps);

/**
 * @brief  退出单轮 PID 调试模式并停车
 */
void chassis_ctrl_stop_single_wheel_pid_debug(void);

/**
 * @brief  读取 4 路轮速反馈快照 (LPF 后, 单位 m/s).
 *         用于诊断接线: 调单轮 PID 时一并打印 4 路反馈, 手转任一物理轮观察哪个 index 在动,
 *         即可反推出该物理轮接的是哪个软件 wheel index, 进而修正 chassis_config.h 的 ENC 引脚定义.
 * @param  out_wheel_fb_mps 4 元素输出数组 (LF/RF/LB/RB), 必须非空.
 */
void chassis_ctrl_get_wheel_feedback_snapshot(float out_wheel_fb_mps[4]);

/**
 * @brief  进入航向调试模式：固定 0 度航向保持，停止平移
 *         推荐在 main 初始化完成后调用一次。
 */
void chassis_ctrl_attitude_debug_start_zero(void);

/**
 * @brief  读取航向闭环调试信息（线程安全的结构体拷贝）
 * @param  out_info  输出信息结构体指针，传空则忽略
 */
void chassis_ctrl_attitude_debug_get_state(chassis_attitude_debug_info_t *out_info);

/**
 * @brief  读取四轮 PID 输出快照 (前进符号域, 未乘 dir_sign)
 * @param  out_pwm  长度 4 的输出数组, 顺序: LF / RF / LB / RB
 */
void chassis_ctrl_attitude_debug_get_wheel_pwm(float out_pwm[4]);

/**
 * @brief  航向闭环调试任务（建议主循环每 5ms 调用一次）
 *         内部 100ms 打印一次目标角/当前角/误差/角速度指令。
 */
void chassis_ctrl_attitude_debug_task_5ms(void);

/**
 * @brief  查询是否已到达目标点
 * @return 1 = 已到达或空闲，0 = 移动中
 */
uint8 chassis_ctrl_is_arrived(void);

/**
 * @brief  获取当前位姿副本（结构体拷贝）
 * @return 当前位姿 {x_m, y_m, yaw_deg}
 */
chassis_pose_t chassis_ctrl_get_pose(void);

/**
 * @brief  获取最近一次输出的车体速度指令（调试用）
 * @return 车体速度指令
 */
chassis_body_speed_cmd_t chassis_ctrl_get_last_cmd(void);

/**
 * @brief  紧急停止：立即清零所有控制状态并切断 PWM 输出
 */
void chassis_ctrl_stop(void);

/**
 * @brief  外部校正位姿（如视觉重定位矫正里程计累积误差）
 * @param  x_m      校正后 X 坐标（米）
 * @param  y_m      校正后 Y 坐标（米）
 * @param  yaw_deg  校正后航向角（度）
 */
void chassis_ctrl_set_pose(float x_m, float y_m, float yaw_deg);

/**
 * @brief  获取当前运行时调参参数
 * @param  out_params  输出参数结构体指针
 */
void chassis_ctrl_get_tune_params(chassis_tune_params_t *out_params);

/**
 * @brief  设置运行时调参参数（将自动限幅并立即生效）
 * @param  in_params  输入参数结构体指针
 */
void chassis_ctrl_set_tune_params(const chassis_tune_params_t *in_params);

/* ==================================================================
 * 【P0-8】发车区 / 越界几何判定 API
 * ----------------------------------------------------------------
 * 全部为 *只读查询*, 内部仅消费 chassis_ctrl_get_pose() 的 seq-lock 快照,
 * 不下发任何控制指令, 状态机如何响应由 app_game_logic 决定.
 *
 * 调用约束: 仅在主循环侧调用 (app_game_logic 5ms tick).
 * 速度估算: 由 chassis_zone_tick() 周期更新, 必须先于查询函数调用.
 * ================================================================== */

/** 发车区编号. 规则: 场地左右各一个发车区. */
typedef enum {
    LAUNCH_ZONE_LEFT  = 0,   /**< 左发车区 (x ∈ [0, 0.30]) */
    LAUNCH_ZONE_RIGHT,       /**< 右发车区 (x ∈ [W-0.30, W]) */
    LAUNCH_ZONE_ANY          /**< 任一发车区 (左 || 右) */
} LaunchZone_e;

/**
 * @brief  几何判定周期 tick (建议 5ms 调用一次, 与 app_game_logic 对齐).
 *         内部刷新位姿快照 + 平滑速度估算 + 越界滞回标志位.
 *         若不调用, chassis_zone_is_static() / chassis_zone_is_out_of_bounds()
 *         的结果将停留在初始态, 但其余基于位置的查询仍可用.
 */
void chassis_zone_tick(void);

/**
 * @brief  车体几何中心是否在指定发车区内.
 * @param  zone  LAUNCH_ZONE_LEFT / RIGHT / ANY
 * @return 1 = 在区内, 0 = 不在.
 */
uint8 chassis_zone_is_in_launch(LaunchZone_e zone);

/**
 * @brief  车体外接圆 (半径 CHASSIS_BODY_RADIUS_M) 是否完全离开发车区.
 *         判据: 车体外接圆与发车区矩形不相交.
 *         规则: "完全离开发车区" 即视为发车成功.
 * @param  zone  LAUNCH_ZONE_LEFT / RIGHT / ANY (ANY 表示同时离开两个发车区)
 * @return 1 = 完全离开, 0 = 仍有重叠.
 */
uint8 chassis_zone_is_fully_outside_launch(LaunchZone_e zone);

/**
 * @brief  车体外接圆是否越过最外圈围墙 (含 CHASSIS_OOB_HYSTERESIS_M 滞回).
 *         规则: 越过最外圈 = 比赛立即结束.
 * @return 1 = 已判定出界, 0 = 在场内.
 *         状态由 chassis_zone_tick() 维护, 一旦置位需通过 chassis_zone_clear_oob()
 *         手动清除 (避免业务侧自动复位).
 */
uint8 chassis_zone_is_out_of_bounds(void);

/**
 * @brief  清除越界标志 (用于复位流程或调试).
 *         典型业务侧不应调用 — 出界=结束.
 */
void chassis_zone_clear_oob(void);

/**
 * @brief  车体是否处于静止状态: 平滑速度模长 < CHASSIS_STATIC_SPEED_EPS_MPS
 *         且持续时间 ≥ CHASSIS_STATIC_HOLD_MS.
 * @return 1 = 静止持续达标, 0 = 仍在动 / 未达时长.
 *         必须周期调用 chassis_zone_tick() 才能正确累计.
 */
uint8 chassis_zone_is_static(void);

#endif /* CHASSIS_CTRL_H */
