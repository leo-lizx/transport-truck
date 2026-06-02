#ifndef _APP_GAME_LOGIC_H_
#define _APP_GAME_LOGIC_H_

#include "zf_common_headfile.h"
#include "chassis_config.h"
#include "algo_sokoban_solver.h"
#include "chassis_ctrl.h"

#ifdef MAP_ROWS
#undef MAP_ROWS
#endif
#define MAP_ROWS (12)

#ifdef MAP_COLS
#undef MAP_COLS
#endif
#define MAP_COLS (16)

// ==========================================
// 底盘硬件调用层（直接对接 chassis_ctrl.*）
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
 * 1) x: 目标网格 X 坐标（建议范围 1~14，最外圈边界不可进）。
 * 2) y: 目标网格 Y 坐标（建议范围 1~10，最外圈边界不可进）。
 * 返回值:
 * 1) 无返回值。
 * 使用建议:
 * 1) 调用一次后，配合 HAL_CHASSIS_IS_ARRIVED() 轮询到点状态。
 * 2) 若中途重复调用，会覆盖上一目标点，属于预期行为。
 */
#define HAL_CHASSIS_MOVE_TO(x, y)      chassis_ctrl_move_to_grid((uint8)(x), (uint8)(y))

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
#define HAL_CHASSIS_IS_ARRIVED()       (chassis_ctrl_is_arrived())

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

// 游戏主流程状态机枚举
typedef enum {
    STAGE_WAIT_START = 0,           // 发车区等待
    STAGE_RECOGNIZE_MAP,            // 识别地图与箱子
    STAGE_PLAN_PATH,                // 寻路/推箱策略计算
    STAGE_EXECUTE_ACTION,           // 执行推箱动作
    STAGE_LEVEL_JUDGE,              // 一关完成后判断是否进入下一关
    STAGE_DEADLOCK_RESET,           // 死局恢复：回发车区静止3秒后重置
    STAGE_DONE,                     // 全流程完成
    STAGE_PAUSE_ON_LINK_LOSS        // 【P0-2】视觉链路超时刹停, 链路恢复后自动续跑
} GameStage_e;

// 地图全局变量 (由副镜头串口解析后写入此数组)
extern uint8 g_game_map[MAP_ROWS][MAP_COLS];

// 业务调度函数
void Game_Logic_Task_Run(void);

/* ==================================================================
 * 【P0-2】链路状态查询接口
 * Game_Link_Is_Alive():
 *   1: 视觉端心跳 / MAP 帧在 LINK_LOSS_MS 内仍可见
 *   0: 链路已超时, 状态机已进入 STAGE_PAUSE_ON_LINK_LOSS
 * 用途: 菜单 / IPS 显示、调试上位机查询。
 * ================================================================== */
uint8 Game_Link_Is_Alive(void);

/* ==================================================================
 * 【P0-8】比赛失败原因枚举 + 查询接口
 *   GAME_FAIL_NONE          : 无失败 (正常运行 / 正常完赛)
 *   GAME_FAIL_OUT_OF_BOUNDS : 车体越过最外圈围墙, 比赛立即终止
 * 用途: 调试上位机 / 菜单显示原因; 状态机决定是否切 STAGE_DONE.
 * 注意: 触发后 Game_Logic_Task_Run 会强制 chassis_ctrl_stop() 并切 STAGE_DONE,
 *       业务侧不会再回到 RECOGNIZE_MAP/PLAN_PATH 等任何执行态.
 * ================================================================== */
typedef enum {
    GAME_FAIL_NONE = 0,
    GAME_FAIL_OUT_OF_BOUNDS
} GameFailureReason_e;

GameFailureReason_e Game_Get_Failure_Reason(void);

/* ==================================================================
 * 【B17】识别 tour 进度查询接口 (转发自 App_Recognize_Get_Debug)
 *   用途: 菜单 / IPS / 上位机显示当前识别到第几个物体、多数票占比等.
 *   线程安全: 与 App_Recognize_Tick 同线程(主循环), 直接读 BSS 即可.
 * ================================================================== */
#include "app_recognize.h"
void Game_Get_Recognize_Debug(AppRecognizeDebug_t *out);

#endif