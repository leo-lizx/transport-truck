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
// 3. 分类识别已改由 app_recognize 读取 OpenART2 BOX_CLASS 快照。
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

/* Legacy placeholder. New code reads OpenART2 BOX_CLASS via app_recognize/app_link. */
#define HAL_VISION_GET_BOX_CLASS_ID()  (0)
// ==========================================

/* Formal launch debug baseline: match hardcoded-map mode and lock yaw to 0 degrees. */
#define APP_GAME_LAUNCH_FACE_YAW_DEG   (0.0f)

// 游戏主流程状态机枚举
typedef enum {
    STAGE_WAIT_START = 0,           // 首次从贴边位发车；关间返回并停在 (1,5)
    STAGE_LAUNCH_EXIT,              // 首次贴边位/关间 (1,5) 驶向触发点 (2,5)
    STAGE_WAIT_MAP_REFRESH,         // 已到 (2,5)，停车等待上位机刷新地图
    STAGE_RECOGNIZE_MAP,            // 识别地图与箱子
    STAGE_PLAN_PATH,                // 寻路/推箱策略计算
    STAGE_EXECUTE_ACTION,           // 执行推箱动作
    STAGE_LEVEL_JUDGE,              // 仅确认本关箱子已全部完成
    STAGE_DEADLOCK_RESET,           // 异常停车：自动死局返航/重置暂时关闭
    STAGE_DONE,                     // 全流程完成
    STAGE_PAUSE_ON_LINK_LOSS,       // 【P0-2】视觉链路超时刹停
    STAGE_WAIT_RECOVERY_MAP         // 断链恢复/部分批次/推完校验：原地等待新鲜稳定地图
} GameStage_e;

/* Race-flow diagnostics for the IPS status line. */
typedef struct
{
    GameStage_e stage;
    uint8 map_accepted;       /* A valid post-departure map entered game logic. */
    uint8 solve_succeeded;    /* A push-box/bomb solution is ready to execute. */
    uint8 waypoint_issued;    /* The current execution waypoint was sent. */
    uint16 waypoint_index;    /* Zero-based index in the current waypoint segment. */
    uint16 waypoint_count;
} GameRuntimeStatus_t;

// 地图全局变量 (由副镜头串口解析后写入此数组)
extern uint8 g_game_map[MAP_ROWS][MAP_COLS];

// 业务调度函数
void Game_Logic_Task_Run(void);

/* 利用主循环等待下一拍的空档推进一个有界规划单元；仅主线程调用。
 * STAGE_PLAN_PATH 前台求解或第一关 EXECUTE 后台滚动求解推进时返回 1。 */
uint8 Game_Logic_Idle_Plan_Step(void);

/* 正式比赛状态初始化：上电已在贴边发车位，从第一关发车准备开始。 */
void Game_Logic_Init(void);

/* ==================================================================
 * 【P0-2】链路状态查询接口
 * Game_Link_Is_Alive():
 *   1: 当前所监控的视觉链路在线
 *      - 地图链路: OpenART1 MAP/heartbeat 在线
 *      - 图案采样阶段: OpenART1 + OpenART2 均在线
 *   0: 所需链路已超时
 *      - 收图/恢复阶段进入 STAGE_PAUSE_ON_LINK_LOSS
 *      - 规划/执行阶段只记录离线，不中断已有航点
 * 用途: 菜单 / IPS 显示、调试上位机查询。
 * ================================================================== */
uint8 Game_Link_Is_Alive(void);

/* Read-only race-flow status used by the differential IPS refresh. */
void Game_Get_Runtime_Status(GameRuntimeStatus_t *out);

/* Copy the latest map accepted by the five-frame freeze gate.
 * Returns 0 before the first map is frozen; generation increments once per
 * accepted map so the IPS can ignore live UART frames and redraw only once. */
uint8 Game_Get_Frozen_Map(uint8 out[MAP_ROWS][MAP_COLS], uint32 *generation);

/* ==================================================================
 * 【B17】识别 tour 进度查询接口 (转发自 App_Recognize_Get_Debug)
 *   用途: 菜单 / IPS / 上位机显示当前识别到第几个物体和有效采样数等.
 *   线程安全: 与 App_Recognize_Tick 同线程(主循环), 直接读 BSS 即可.
 * ================================================================== */
#include "app_recognize.h"
void Game_Get_Recognize_Debug(AppRecognizeDebug_t *out);

#endif
