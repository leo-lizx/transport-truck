#ifndef _ALGO_SOKOBAN_SOLVER_H_
#define _ALGO_SOKOBAN_SOLVER_H_

/*===========================================================================
 * [algo_sokoban_solver.h] 推箱子求解器
 *
 *   基于逐飞演示车模思路实现的"地图分解法"：
 *     1. 将 N 个箱子拆解为 N 个独立的"单箱推送子问题"
 *     2. 每个子问题用 BFS（状态 = 玩家坐标 + 箱子坐标）快速求解
 *     3. 子地图中其余未完成箱子视为墙壁，已完成箱子视为空地
 *     4. 解算结果为方向动作序列，可转换为车模导航路点
 *
 *   三阶段支持：
 *     Stage1 — 任意箱配任意目标（贪心分配，最近优先）
 *     Stage2 — 指定箱→目标映射（分类模式）
 *     Stage3 — 炸弹墙体求解辅助函数
 *
 *   内存占用（优化后）：
 *     单箱状态空间 = 12×16×12×16 = 36864
 *     位图+4bit前驱记录约 32KB（替代原 108KB 队列方案）
 *===========================================================================*/

#include "zf_common_typedef.h"

/* ======================================================================
 *  地图基础定义（原 algo_bfs_scout.h）
 * ====================================================================== */

#ifdef MAP_ROWS
#undef MAP_ROWS
#endif
#define MAP_ROWS    (12)   // 12 行总地图（含上下边界）

#ifdef MAP_COLS
#undef MAP_COLS
#endif
#define MAP_COLS    (16)   // 16 列总地图（含左右边界）

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

/* ======================================================================
 *  常量
 * ====================================================================== */

#define SOKOBAN_MAX_BOXES       8       // 地图中最大箱子数量
#define SOKOBAN_MAX_ACTIONS     500     // 单次推箱最大步数
#define SOKOBAN_MAX_WAYPOINTS   200     // 路点数组上限

/* ======================================================================
 *  数据类型
 * ====================================================================== */

/** 玩家移动/推箱方向 */
typedef enum {
    SOKO_ACT_UP    = 0,     // row - 1
    SOKO_ACT_DOWN  = 1,     // row + 1
    SOKO_ACT_LEFT  = 2,     // col - 1
    SOKO_ACT_RIGHT = 3,     // col + 1
    SOKO_ACT_NONE  = 0xFF
} SokoAction_e;

/** 单个子问题的动作序列 */
typedef struct {
    SokoAction_e actions[SOKOBAN_MAX_ACTIONS];
    uint16       count;         // 有效动作数量
} SokoActionSeq_t;

/** 路点路径（仅保留转弯点，供车模逐点导航） */
typedef struct {
    Point_t  points[SOKOBAN_MAX_WAYPOINTS];
    uint16   count;
} SokoWaypointPath_t;

/** 完整解算结果（覆盖全部箱子） */
typedef struct {
    SokoActionSeq_t  sub_solutions[SOKOBAN_MAX_BOXES];       // 每个子问题的动作序列
    Point_t          player_end_pos[SOKOBAN_MAX_BOXES];      // 每个子问题结束后玩家坐标
    uint8            total_boxes;                             // 箱子总数
    uint8            is_solved;                               // 1=全部解算成功
} SokoFullSolution_t;

/* ======================================================================
 *  API 函数
 * ====================================================================== */

/**
 * @brief 寻找距离车模最近的箱子的"侧面观察点" (使用BFS曼哈顿扩散)
 * @param map 地图矩阵
 * @param player_pos 车模当前坐标
 * @param target_box_pos (输出参数) 记录最终选定的箱子坐标
 * @return ObservePoint_t 最佳的空地观察坐标
 */
ObservePoint_t Algo_Find_Nearest_Box_Observe_Point(const uint8 map[MAP_ROWS][MAP_COLS],
                                                   Point_t player_pos,
                                                   Point_t *target_box_pos);

/**
 * @brief 点到点纯寻路算法 (不推箱，专用于侦查阶段的快速跑位)
 * @param map 地图矩阵
 * @param start 起点坐标
 * @param end 终点坐标
 * @param result_path 结构体指针，用于接收解算出的路径队列
 * @return uint8 1: 成功找到路径, 0: 无法到达目标点
 */
uint8 Algo_Nav_BFS(const uint8 map[MAP_ROWS][MAP_COLS],
                   Point_t start,
                   Point_t end,
                   NavPath_t *result_path);

/**
 * @brief  第一阶段求解 — 任意箱→任意目标（基础模式）
 *
 * 使用"地图分解 + 贪心分配 + 单箱 BFS":
 *   1. 提取地图中所有箱子和目标
 *   2. 贪心选取最近的箱子，为其分配最近的目标
 *   3. 构建子地图（其余箱子→墙，其余目标→空地）
 *   4. BFS 解算单箱推送路径
 *   5. 循环直到所有箱子完成
 *
 * @param  map         当前地图
 * @param  player_pos  玩家初始坐标
 * @param  result      [out] 完整解算结果
 * @return 1=成功, 0=无解
 */
uint8 Sokoban_Solve_Stage1(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player_pos,
                           SokoFullSolution_t *result);

/**
 * @brief  第二阶段求解 — 指定箱→目标映射（分类模式）
 *
 * @param  map                当前地图
 * @param  player_pos         玩家初始坐标
 * @param  box_to_target_idx  映射数组: box_to_target_idx[i] 表示第 i 个箱子
 *                            应推到第 box_to_target_idx[i] 个目标
 * @param  box_count          箱子数量
 * @param  result             [out] 完整解算结果
 * @return 1=成功, 0=无解
 */
uint8 Sokoban_Solve_Stage2(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player_pos,
                           const uint8 box_to_target_idx[],
                           uint8 box_count,
                           SokoFullSolution_t *result);

/**
 * @brief  检测当前地图是否存在明显死局（如箱子被卡在非目标角落）
 *
 * @param  map           当前地图
 * @param  dead_box_pos  [out] 死局箱子坐标，可为 NULL
 * @return 1=检测到死局, 0=未检测到
 */
uint8 Sokoban_Is_Deadlock(const uint8 map[MAP_ROWS][MAP_COLS],
                          Point_t *dead_box_pos);

/**
 * @brief  第三阶段辅助 — 寻找最优炸弹爆破墙体
 *
 * 遍历所有内部墙体，逐个假设被炸掉(3×3范围清除)，
 * 根据“破局收益”评分选最优墙体：
 *   1) blocked_target 可达性
 *   2) 爆炸后可达目标数量
 *   3) 清除墙体数量与路径代价
 *
 * @param  map             当前地图
 * @param  player_pos      玩家坐标
 * @param  blocked_target  当前不可达的目标位置
 * @param  bomb_wall_pos   [out] 应推炸弹到此墙体
 * @return 1=找到, 0=无法解决
 */
uint8 Sokoban_Find_Bomb_Wall(const uint8 map[MAP_ROWS][MAP_COLS],
                             Point_t player_pos,
                             Point_t blocked_target,
                             Point_t *bomb_wall_pos);

/**
 * @brief  将动作序列转换为路点路径（仅保留转弯点）
 *
 * 同方向的连续移动合并为一个路点（区间终点），
 * 方向改变时产生新路点。车模按路点列表逐点移动即可。
 *
 * @param  actions     动作数组
 * @param  count       动作数量
 * @param  start_pos   起始坐标
 * @param  wp_path     [out] 路点路径
 * @return 路点数量
 */
uint16 Sokoban_Actions_To_Waypoints(const SokoAction_e *actions,
                                    uint16 count,
                                    Point_t start_pos,
                                    SokoWaypointPath_t *wp_path);

/**
 * @brief  在地图上模拟炸弹爆炸，清除以 (wall_pos) 为中心的 3×3 内墙
 *
 * @param  map       [in/out] 地图会被原地修改
 * @param  wall_pos  爆炸中心
 */
void Sokoban_Apply_Bomb_Explosion(uint8 map[MAP_ROWS][MAP_COLS],
                                  Point_t wall_pos);

/**
 * @brief  求解"推炸弹到指定墙体"的路径（单箱 BFS，炸弹视为可推物体）
 *
 * @param  map        当前地图
 * @param  player_pos 玩家坐标
 * @param  bomb_pos   炸弹坐标
 * @param  wall_pos   目标墙体坐标
 * @param  sol        [out] 动作序列
 * @return 1=成功, 0=无解
 */
uint8 Sokoban_Solve_Push_Bomb(const uint8 map[MAP_ROWS][MAP_COLS],
                             Point_t player_pos,
                             Point_t bomb_pos,
                             Point_t wall_pos,
                             SokoActionSeq_t *sol);

#endif /* _ALGO_SOKOBAN_SOLVER_H_ */
