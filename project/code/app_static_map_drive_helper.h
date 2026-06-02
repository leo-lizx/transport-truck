/*===========================================================================
 * [app_static_map_drive_helper.h] 静态地图推箱启动辅助接口
 *
 * 处理手写/视觉地图字符转换、半格发车点入口候选枚举、推箱方案择优、
 * 完整航点数组生成等逻辑。主流程只关心最终的 AppStaticMapDrivePlan_t。
 *===========================================================================*/

#ifndef APP_STATIC_MAP_DRIVE_HELPER_H_
#define APP_STATIC_MAP_DRIVE_HELPER_H_

#include "algo_sokoban_solver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 静态地图驱动的规划结果:
 * solution     : 完整推箱求解结果，保留每个箱子的原始动作段。
 * waypoints    : 已展开的完整网格航点数组，STATIC_MAP_DRIVE 可直接顺序下发。
 * entry        : 实际选用的整数入口格。
 * entry_dist_m : 半格发车点到入口格中心的距离。
 * score_m      : 入口距离 + 推箱动作估算距离，用于候选入口择优。
 */
typedef struct
{
    SokoFullSolution_t solution;
    SokoWaypointPath_t waypoints;
    Point_t entry;
    float entry_dist_m;
    float score_m;
} AppStaticMapDrivePlan_t;

/*
 * 函数: App_StaticMapDrive_CharToMap
 * 功能: ASCII 地图字符转换为 MAP_* 枚举，未知字符按空地处理，便于手写地图容错。
 * 支持: '#'=墙, '-'=空, '.'=目标, '$'=箱, '*'=炸弹, '@'=车位/空地。
 */
uint8 App_StaticMapDrive_CharToMap(char ch);

/*
 * 函数: App_StaticMapDrive_MapToChar
 * 功能: MAP_* 枚举转换回 ASCII 字符，用于日志打印和地图调试。
 */
char App_StaticMapDrive_MapToChar(uint8 v);

/*
 * 函数: App_StaticMapDrive_LoadCharMap
 * 功能: 将 MAP_ROWS 行、每行 MAP_COLS 字符的手写地图加载到枚举地图数组。
 * 注意: src 每行应额外包含字符串结尾 '\0'，但转换时只读取 MAP_COLS 个字符。
 */
void App_StaticMapDrive_LoadCharMap(const char src[MAP_ROWS][MAP_COLS + 1U],
                                    uint8 dst[MAP_ROWS][MAP_COLS]);

/*
 * 函数: App_StaticMapDrive_SolveFromLaunch
 * 功能: 从可能处于半格位置的发车点出发，枚举可进入的整数入口格并选择最优推箱方案。
 * 返回: 1=求解成功；0=无解或参数无效。
 */
uint8 App_StaticMapDrive_SolveFromLaunch(const uint8 map[MAP_ROWS][MAP_COLS],
                                         float start_x_grid,
                                         float start_y_grid,
                                         Point_t preferred_entry,
                                         AppStaticMapDrivePlan_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_STATIC_MAP_DRIVE_HELPER_H_ */
