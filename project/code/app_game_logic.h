#ifndef _APP_GAME_LOGIC_H_
#define _APP_GAME_LOGIC_H_

#include "zf_common_headfile.h"
#include "algo_bfs_scout.h"

// ==========================================
// 💡 以下为底层硬件接口占位符，需您后续根据驱动填充
// ==========================================
// 占位宏1：控制车模前往指定网格坐标
#define HAL_CHASSIS_MOVE_TO(x, y)      /* [TODO: 调用底盘移动函数] */
// 占位宏2：检查车模是否已经到达目标网格 (1:到达, 0:未到达)
#define HAL_CHASSIS_IS_ARRIVED()       (1) /* [TODO: 返回底盘到达标志位] */
// 占位宏3：获取前向摄像头(主镜头)的图像分类结果 (0:无图片/背景, >0:具体的图片ID)
#define HAL_VISION_GET_BOX_CLASS_ID()  (0) /* [TODO: 返回前向摄像头识别结果] */
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