#ifndef _APP_RECOGNIZE_CLEAR_H_
#define _APP_RECOGNIZE_CLEAR_H_

#include "algo_sokoban_solver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 识别巡航路径规划:
 *   1) 优先只导航到物体相邻观察格;
 *   2) 无路时允许最多推动少量非目标箱子清障;
 *   3) 不推动当前待识别物体, 不占用目标格, 不把箱子推入墙角.
 *
 * 输出为逐格动作, 调用方可统一转成底盘航点执行。模块只规划, 不修改输入地图。
 */
#define APP_RECOG_CLEAR_MAX_PUSHES (2U)

typedef struct
{
    SokoActionSeq_t actions;
    Point_t         observe;
    Point_t         player_end;
    uint8           push_count;
} AppRecogClearPlan_t;

uint8 App_Recog_Clear_Plan(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player,
                           Point_t protected_object,
                           AppRecogClearPlan_t *out);

#ifdef __cplusplus
}
#endif

#endif /* _APP_RECOGNIZE_CLEAR_H_ */
