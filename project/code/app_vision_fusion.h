/*********************************************************************************************************************
 * app_vision_fusion — OpenART 视觉位姿与里程计融合（事件驱动 Snap 为主，连续融合可选）
 *
 * 提供三类能力：
 *   1) app_vision_fusion_task            : 运动中的连续软融合 (默认编译为空)。
 *   2) app_vision_fusion_consistency_tick: 运动中的一致性监控 (大幅打滑/搬车时硬重定位)。
 *   3) app_vision_fusion_snap_*          : 到站静止后的「多帧表决 → Snap 到格中心」状态机。
 *
 * 行为开关与调参参数集中在 chassis_config.h 中 CHASSIS_VISION_* 宏。
 *********************************************************************************************************************/
#ifndef APP_VISION_FUSION_H_
#define APP_VISION_FUSION_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Snap 状态机对外可见状态。 */
typedef enum
{
    APP_VISION_SNAP_IDLE    = 0,    /**< 当前未发起 Snap (尚未调用 request, 或刚被取消) */
    APP_VISION_SNAP_PENDING,        /**< 表决进行中, 业务侧应继续等待               */
    APP_VISION_SNAP_DONE,           /**< 表决通过且已写入 chassis_ctrl_set_pose      */
    APP_VISION_SNAP_TIMEOUT,        /**< 超过 SNAP_TIMEOUT_MS 仍未达成多数表决       */
    APP_VISION_SNAP_REJECT          /**< 表决格与目标差距 > MAX_GAP_CELLS，视为视觉错觉 */
} app_vision_snap_state_e;

/**
 * @brief  运动中连续软融合 (默认编译为空)。
 *         仅当 CHASSIS_VISION_FUSION_ENABLE==1 时生效；推荐保持默认 0。
 * @param  allow_fuse  1=允许融合 (链路在线 && 当前阶段允许)；0=本拍跳过且重置周期累加器
 */
void app_vision_fusion_task(uint8 allow_fuse);

/**
 * @brief  运动中一致性监控；与 task 同侧调用。
 *         读取 odom 当前格 vs 最新视觉格的曼哈顿差，
 *         若差距 ≥ CHASSIS_VISION_CONSISTENCY_GAP_CELLS 持续 ≥ HOLD_MS，
 *         触发一次 chassis_ctrl_set_pose() 硬重定位，并进入冷却期。
 * @param  allow  与 task 一致的开关；0 时本拍不累计也不触发，并清除窗口。
 */
void app_vision_fusion_consistency_tick(uint8 allow);

/**
 * @brief  发起一次到站 Snap 表决 (在 odom 已到位 + 短静止后调用)。
 *         多次重复调用会被聚合到同一次表决；目标点变化时自动重启表决。
 *         运行节奏与调用方一致 (推荐每 5ms 一次)。
 * @param  target_x_m / target_y_m  当前导航目标的米坐标 (用于 MAX_GAP_CELLS 判断)
 */
void app_vision_fusion_snap_request(float target_x_m, float target_y_m);

/**
 * @brief  读取 Snap 状态机当前状态。
 *         状态转 DONE/TIMEOUT/REJECT 后, 在下一次 *新* 的 snap_request 调用时自动归 IDLE。
 */
app_vision_snap_state_e app_vision_fusion_snap_state(void);

/**
 * @brief  显式取消当前 Snap (例如链路掉线/航点切换/紧急停止)，状态归 IDLE。
 */
void app_vision_fusion_snap_cancel(void);

/**
 * @brief  Snap 与一致性监控的诊断计数 (主循环侧只读，统计长期表现)。
 */
typedef struct
{
    uint32 snap_done;          /**< 表决通过累计 */
    uint32 snap_timeout;       /**< 超时累计 */
    uint32 snap_reject;        /**< 视觉与目标差距过大累计 */
    uint32 consistency_fire;   /**< 一致性硬重定位触发次数 */
} app_vision_fusion_stats_t;

void app_vision_fusion_get_stats(app_vision_fusion_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_FUSION_H_ */
