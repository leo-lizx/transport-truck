#ifndef _road_H_
#define _road_H_

#include "zf_common_headfile.h" // 包含逐飞通用头文件，提供基础数据类型如 uint8, uint16 等

// 地图尺寸定义 (根据赛题16行12列)
#define MAP_ROWS    16
#define MAP_COLS    12

// 地图元素枚举定义
typedef enum {
    MAP_EMPTY   = 0, // 空地
    MAP_WALL    = 1, // 墙体
    MAP_TARGET  = 2, // 目的地
    MAP_BOX     = 3, // 箱子
    MAP_BOMB    = 4  // 炸弹 (第三阶段使用)
} MapElement_e;

// 移动方向枚举
typedef enum {
    ACT_NONE  = 0,
    ACT_UP    = 1,
    ACT_DOWN  = 2,
    ACT_LEFT  = 3,
    ACT_RIGHT = 4,
    ACT_PUSH  = 5  // 代表这是一个推箱子动作(可选，也可融入普通方向中)
} Action_e;

// 坐标结构体
typedef struct {
    int8 x; // 列号 (0-11)
    int8 y; // 行号 (0-15)
} Point_t;

// 寻路结果结构体
typedef struct {
    Action_e path[200];  // 保存路径动作，假设最长不超过200步
    uint16   step_count; // 总步数
    uint8    is_success; // 是否成功找到路径: 1成功, 0失败
} PathResult_t;

/**
 * @brief  推箱子单箱子最短路径解算 (BFS)
 * @param  map         16x12全局二维地图 (注意：必须是降维后的单箱地图)
 * @param  player_pos  车模(玩家)当前坐标
 * @param  box_pos     当前需要推的箱子坐标
 * @param  target_pos  该箱子对应的目的地坐标
 * @param  result      解算结果存放指针
 * @return uint8       1: 成功, 0: 失败
 */
uint8 Algo_Sokoban_Solve(const uint8 map[MAP_ROWS][MAP_COLS], 
                         Point_t player_pos, 
                         Point_t box_pos, 
                         Point_t target_pos, 
                         PathResult_t *result);

#endif