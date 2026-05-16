#ifndef CHASSIS_ZONE_H
#define CHASSIS_ZONE_H

/*===========================================================================
 * [chassis_zone.h] 底盘几何区域判定 / 越界 / 静止检测
 *
 *   职责:
 *     · 发车区 (left/right) 进出判定
 *     · 越过最外圈围墙 (Out-Of-Bounds) 一次性滞回置位
 *     · 车体静止持续时间累计 (赛规要求"完全停稳"判据)
 *     · 提供软限位 guard 接口给 chassis_ctrl 主任务调用 (P0-3)
 *
 *   调用节拍:
 *     · chassis_zone_tick()                — 5ms 周期, 由主循环调用
 *     · chassis_zone_apply_soft_limit_guard()
 *                                          — 在 chassis_ctrl_task_20ms() 内
 *                                            POINT_NAV 之外的模式后调用
 *     · 其余 chassis_zone_is_*()           — 任意主循环上下文随时查询
 *
 *   线程安全:
 *     · 全部 file-static 仅由主循环单线程访问, 无锁
 *     · 通过 chassis_ctrl_get_pose() 走 seq-lock 拿位姿副本 (P0-3)
 *     · 软限位读 app_link_get_map_snapshot() 拿一致地图副本
 *
 *   依赖:
 *     · chassis_config.h    — 物理尺寸 / 阈值 / 滞回参数
 *     · chassis_ctrl.h      — chassis_pose_t / chassis_body_speed_cmd_t / pose 查询
 *     · app_link.h          — 地图快照 (软限位 guard 需要)
 *
 *   重构来源 (2026-05-13):
 *     原代码位于 chassis_ctrl.c L62-82, L462-540, L2048-2141; 整体迁移, 行为等价.
 *===========================================================================*/

#include "chassis_config.h"

/* 前向声明: chassis_body_speed_cmd_t 的实际定义见 chassis_ctrl.h */
struct chassis_body_speed_cmd_s;

/*============================== 数据类型 ==============================*/

/** 发车区编号. 规则: 场地左右各一个发车区. */
typedef enum {
    LAUNCH_ZONE_LEFT  = 0,   /**< 左发车区 (x ∈ [0, CHASSIS_LAUNCH_ZONE_W_M]) */
    LAUNCH_ZONE_RIGHT,       /**< 右发车区 (x ∈ [W-CHASSIS_LAUNCH_ZONE_W_M, W]) */
    LAUNCH_ZONE_ANY          /**< 任一发车区 (左 || 右) */
} LaunchZone_e;

/*============================== 公共 API ==============================*/

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
 *         判据: 车体外接圆与发车区矩形不相交 (即赛规"完全离开").
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

/**
 * @brief  软限位保护: 预测下一步位置, 若即将撞墙则减速并施加微转向.
 *         仅供 chassis_ctrl_task_20ms() 在非 POINT_NAV 模式下调用.
 *
 *         逻辑:
 *           1) 以前瞻时间预测车体中心落点
 *           2) 若前方落点位于墙体, 则线速度强制缩放 (BRAKE_SCALE)
 *           3) 对比前左/前右两个试探点, 向更空旷一侧施加角速度偏置
 *
 *         调用约束: PIT_IRQn 上下文; 内部走 seq-lock 取一致位姿 + 一致地图副本.
 *
 * @param  cmd  车体速度指令 (输入兼输出, in-place 修改)
 */
void chassis_zone_apply_soft_limit_guard(struct chassis_body_speed_cmd_s *cmd);

#endif /* CHASSIS_ZONE_H */
