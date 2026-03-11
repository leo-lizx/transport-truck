#ifndef _ALGO_BFS_SCOUT_H_
#define _ALGO_BFS_SCOUT_H_

#include "zf_common_headfile.h"

#define MAP_ROWS    12      // 12 行（Y 方向，对应 2.4m / 0.20m）
#define MAP_COLS    16      // 16 列（X 方向，对应 3.2m / 0.20m）

// 地图元素定义
typedef enum {
    MAP_EMPTY   = 0, // 空地
    MAP_WALL    = 1, // 墙体
    MAP_TARGET  = 2, // 目的地 (数字点，侦查阶段可直接穿过)
    MAP_BOX     = 3, // 箱子 (图片点)
    MAP_BOMB    = 4  // 炸弹
} MapElement_e;

typedef struct {
    int8 x;
    int8 y;
} Point_t;

// 观察点结构体（包含坐标和需要车头朝向的方向）
typedef struct {
    Point_t pos;        // 观察点的网格坐标
    uint8   is_valid;   // 是否有效
} ObservePoint_t;

// 导航路径结果
typedef struct {
    Point_t path[200];  // 存放沿途经过的坐标点队列
    uint16  step_count; // 路径总步数
} NavPath_t;

// 函数声明
ObservePoint_t Algo_Find_Nearest_Box_Observe_Point(const uint8 map[MAP_ROWS][MAP_COLS], Point_t player_pos, Point_t *target_box_pos);
uint8 Algo_Nav_BFS(const uint8 map[MAP_ROWS][MAP_COLS], Point_t start, Point_t end, NavPath_t *result_path);

#endif