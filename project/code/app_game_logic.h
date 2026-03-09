#ifndef _APP_GAME_LOGIC_H_
#define _APP_GAME_LOGIC_H_

#include "zf_common_headfile.h"
#include "algo_bfs_scout.h"
#include "chassis_pose_ctrl_call_example.h"

// ==========================================
// 底盘硬件调用层（已对接到 code/chassis_pose_ctrl_call_example.*）
// 说明：
// 1. HAL_CHASSIS_MOVE_TO: 下发网格坐标目标点。
// 2. HAL_CHASSIS_IS_ARRIVED: 查询是否已到达目标点。
// 3. HAL_VISION_GET_BOX_CLASS_ID: 视觉分类结果仍为占位，后续接入串口协议。
// ==========================================

/*
 * 宏名称: HAL_CHASSIS_MOVE_TO(x, y)
 * 功能说明:
 * 1) 向底盘控制链路下发“网格坐标目标点”。
 * 2) 该宏不会阻塞等待底盘到达，仅负责触发目标更新。
 * 参数说明:
 * 1) x: 目标网格 X 坐标（建议范围 0~15）。
 * 2) y: 目标网格 Y 坐标（建议范围 0~11）。
 * 返回值:
 * 1) 无返回值。
 * 使用建议:
 * 1) 调用一次后，配合 HAL_CHASSIS_IS_ARRIVED() 轮询到点状态。
 * 2) 若中途重复调用，会覆盖上一目标点，属于预期行为。
 */
#define HAL_CHASSIS_MOVE_TO(x, y)      app_control_pipeline_move_to_grid((uint8)(x), (uint8)(y))

/*
 * 宏名称: HAL_CHASSIS_IS_ARRIVED()
 * 功能说明:
 * 1) 查询当前底盘是否已到达最近一次下发的目标网格点。
 * 2) 返回值由底盘控制层统一维护，可用于状态机切换。
 * 参数说明:
 * 1) 无参数。
 * 返回值:
 * 1) 1: 已到达目标点。
 * 2) 0: 尚未到达目标点。
 * 使用建议:
 * 1) 建议在任务调度循环中周期调用，避免在中断中做复杂状态判断。
 */
#define HAL_CHASSIS_IS_ARRIVED()       (app_control_pipeline_is_arrived())

/*
 * 宏名称: HAL_VISION_GET_BOX_CLASS_ID()
 * 功能说明:
 * 1) 获取前向视觉识别到的箱子类别编号。
 * 2) 当前为占位实现，固定返回 0，表示“未识别到有效类别”。
 * 参数说明:
 * 1) 无参数。
 * 返回值:
 * 1) 0: 无有效识别结果/背景。
 * 2) >0: 预留为后续真实类别 ID。
 * 后续接入建议:
 * 1) 对接 OpenART 串口协议后，在此宏映射到实际解析结果变量。
 */
#define HAL_VISION_GET_BOX_CLASS_ID()  (0)
// ==========================================

// 游戏阶段枚举
typedef enum {
    STAGE_UNKNOWN = 0,        // 0. 未知阶段（地图未加载）
    STAGE_PENDING_SCOUT,      // 1. 待侦查确认（前往最近的一个箱子）
    STAGE_OBSERVE_ALL,        // 2. 遍历侦查模式（若判定为二/三阶段，需看遍所有箱子）
    STAGE_1_BASIC_EXEC,       // 3. 第一阶段：基础推箱执行
    STAGE_2_CLASS_EXEC,       // 4. 第二阶段：分类推箱执行
    STAGE_3_STRATEGY_EXEC     // 5. 第三阶段：含炸弹推箱执行
} GameStage_e;

// 地图全局变量 (由副镜头串口解析后写入此数组)
extern uint8 g_game_map[MAP_ROWS][MAP_COLS];

// 业务调度函数
void Game_Logic_Task_Run(void);

#endif