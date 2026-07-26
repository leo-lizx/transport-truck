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
 *     Stage1 — 任意箱配任意目标（连续计时时间成本全局搜索）
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
    MAP_TARGET  = 2, // 目的地地面格（玩家可行走，箱子可推入并继续通过）
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
#define SOKOBAN_PUSH_BITMAP_BYTES ((SOKOBAN_MAX_ACTIONS + 7U) / 8U)
#define ALGO_NAV_DISTANCE_UNREACHABLE (0xFFU)
#define ALGO_NAV_TIME_COST_UNREACHABLE (0xFFFFU)

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
    uint8        push_bitmap[SOKOBAN_PUSH_BITMAP_BYTES]; /* 1=该步实际推动箱子/炸弹 */
    uint16       count;         // 有效动作数量
} SokoActionSeq_t;

/** 航点类型：普通空走航点不需要视觉 Snap；关键航点需要。 */
typedef enum {
    SOKO_WP_WALK = 0,
    SOKO_WP_CRITICAL
} SokoWaypointKind_e;

/** 路点路径（仅保留转弯点，供车模逐点导航） */
typedef struct {
    Point_t  points[SOKOBAN_MAX_WAYPOINTS];
    uint8    kinds[SOKOBAN_MAX_WAYPOINTS];
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
 * @brief 从起点执行一次完整 BFS 扩散，生成全图可达标记
 * @param map 地图矩阵
 * @param start 起点坐标
 * @param reach [out] 可达标记，1=可达，0=不可达；不需要时可传 NULL
 * @param distance_steps [out] 起点到各格的最短步数，不可达为
 *                       ALGO_NAV_DISTANCE_UNREACHABLE；
 *                       不需要时可传 NULL
 * @return 1=扩散成功, 0=起点越界或不可通行
 *
 * 与 Algo_Nav_BFS 共享静态队列，不可重入，不得在 ISR 或嵌套调用中使用。
 */
uint8 Algo_Nav_BFS_Flood(const uint8 map[MAP_ROWS][MAP_COLS],
                         Point_t start,
                         uint8 reach[MAP_ROWS][MAP_COLS],
                         uint8 distance_steps[MAP_ROWS][MAP_COLS]);

/**
 * @brief 以“移动格数 + 方向段停站”为代价的方向状态寻路。
 *
 * 状态为 (网格, 上一步平移方向)，代价单位为 100ms：每移动一格 1，
 * 每开始一个新方向段额外 4。initial_direction 可传 SOKO_ACT_NONE，表示
 * 从停车状态开始；result_path / end_direction / cost_units 均允许传 NULL。
 *
 * 与普通导航 BFS、单箱求解共享文件级 scratch，不可重入。
 */
uint8 Algo_Nav_Time_Path(const uint8 map[MAP_ROWS][MAP_COLS],
                         Point_t start,
                         Point_t end,
                         SokoAction_e initial_direction,
                         NavPath_t *result_path,
                         uint16 *cost_units,
                         SokoAction_e *end_direction);

/** 一次方向状态扩散，返回到每个网格的最小时间代价及对应到达方向。 */
uint8 Algo_Nav_Time_Flood(const uint8 map[MAP_ROWS][MAP_COLS],
                          Point_t start,
                          SokoAction_e initial_direction,
                          uint16 cost_units[MAP_ROWS][MAP_COLS],
                          uint8 arrival_direction[MAP_ROWS][MAP_COLS]);

/** 基于 Algo_Nav_BFS_Flood 结果查询单点是否可达 */
uint8 Algo_Nav_Is_Reachable(const uint8 reach[MAP_ROWS][MAP_COLS],
                            Point_t target);

/**
 * @brief  第一阶段同步求解 — 任意箱→任意目标（PC/自测兼容入口）
 *
 * 使用时间成本全局搜索 + 单箱推宏 A*；正式 5ms 主循环应使用下面的
 * Begin/Step 分时接口，避免在单次 Game_Logic_Task_Run 中同步完成全部规划。
 *
 * @param  map         当前地图
 * @param  player_pos  玩家初始坐标
 * @param  result      [out] 完整解算结果
 * @return 1=成功, 0=无解
 */
uint8 Sokoban_Solve_Stage1(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player_pos,
                           SokoFullSolution_t *result);

/** 第一关分时搜索状态。 */
typedef enum {
    SOKO_SEARCH_IDLE = 0,
    SOKO_SEARCH_RUNNING,
    SOKO_SEARCH_SOLVED,
    SOKO_SEARCH_FAILED
} SokoSearchStatus_e;

/**
 * @brief 启动第一关分时全局搜索。
 *
 * 搜索目标为“动作时间 + 航点停站/Snap + 完成后按底盘 X/Y 轴执行的返库时间”。
 * 本模块不可重入；Begin 后只能由同一主循环周期调用 Step，直至终态或 Cancel。
 */
uint8 Sokoban_Stage1_Search_Begin(const uint8 map[MAP_ROWS][MAP_COLS],
                                 Point_t player_pos,
                                 Point_t home_pos);

/** 每次最多推进 max_work_units 个内部工作单元，适合拆到 5ms tick 中运行。 */
SokoSearchStatus_e Sokoban_Stage1_Search_Step(uint8 max_work_units,
                                              SokoFullSolution_t *result);

/** 取消尚未结束的第一关分时搜索并释放上下文。 */
void Sokoban_Stage1_Search_Cancel(void);

/** 启动第二/三关固定映射的分时搜索，目标同样包含完成后的返库时间。 */
uint8 Sokoban_Stage2_Search_Begin(const uint8 map[MAP_ROWS][MAP_COLS],
                                 Point_t player_pos,
                                 const uint8 box_to_target_idx[],
                                 uint8 box_count,
                                 Point_t home_pos);

/** 第二/三关分时搜索；每拍最多推进 max_work_units 个内部工作单元。 */
SokoSearchStatus_e Sokoban_Stage2_Search_Step(uint8 max_work_units,
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

/** 根据动作中的 push_bitmap 生成带关键性标记的航点。 */
uint16 Sokoban_Seq_To_Waypoints(const SokoActionSeq_t *seq,
                                Point_t start_pos,
                                SokoWaypointPath_t *wp_path);

/**
 * @brief 关卡完成后的单命令返库航点。
 *
 * 通关后不再按虚拟墙和炸弹绕行，只生成一个指向车库的关键航点，供底盘
 * POINT_NAV 在同一命令内依次完成 X/Y 轴运动；到达后的视觉 Snap 由底盘配置决定。
 */
uint8 Sokoban_Build_Return_Waypoints(const uint8 map[MAP_ROWS][MAP_COLS],
                                     Point_t player_pos,
                                     Point_t home_pos,
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

/**
 * @brief  第三阶段顶层规划 — 多炸弹联合 (炸弹, 墙体) 最优可行解搜索
 *
 * 旧 `Sokoban_Find_Bomb_Wall` + `find_bomb_pos_on_map()` 假设全图仅 1 颗炸弹:
 * 墙体打分与"用哪颗炸弹"完全解耦, 且固定取扫描序第一颗炸弹去推。
 * 多炸弹时常出现"选中的墙最优但第一颗炸弹推不过去 → 整盘判失败"的误杀。
 *
 * 本函数做联合搜索, 一次性给出可直接执行的完整炸弹计划:
 *   1. 提取全部炸弹;
 *   2. 遍历每一面内部墙体 W，模拟 3×3 爆破并检查目标可达性;
 *   3. 对每个 (B, W) 组合实际求解推炸弹动作，不以曼哈顿距离替代可行性;
 *   4. 按“不可达目标罚时 + 推炸弹执行时间 + 爆破后关键目标导航时间”取最小值，
 *      清墙数量仅作为同成本时的次级判据。
 *
 * 其余炸弹在求解 B→W 时仍视为障碍 (符合"一次只引爆一颗"的物理), 多颗炸弹
 * 由上层在每次爆炸后重新规划 (STAGE_EXECUTE → STAGE_PLAN_PATH) 逐颗消化。
 *
 * @param  map             当前地图
 * @param  player_pos      玩家坐标
 * @param  blocked_target  当前不可达目标 (用于破局门控); {x<0,y<0} 表示无特定
 *                         目标, 退化为"最大化可达目标 + 清墙数"的通用破局
 * @param  out_bomb_pos    [out] 选中的炸弹坐标
 * @param  out_wall_pos    [out] 选中的爆破墙体坐标
 * @param  out_seq         [out] 把该炸弹推到该墙体的动作序列
 * @return 1=找到可行炸弹计划, 0=无可行解
 */
uint8 Sokoban_Plan_Bomb(const uint8 map[MAP_ROWS][MAP_COLS],
                        Point_t player_pos,
                        Point_t blocked_target,
                        Point_t *out_bomb_pos,
                        Point_t *out_wall_pos,
                        SokoActionSeq_t *out_seq);

/** 启动/推进炸弹联合搜索；Step 每拍最多推进 max_work_units 个内部工作单元。 */
uint8 Sokoban_Bomb_Search_Begin(const uint8 map[MAP_ROWS][MAP_COLS],
                                Point_t player_pos,
                                Point_t blocked_target);
SokoSearchStatus_e Sokoban_Bomb_Search_Step(uint8 max_work_units,
                                            Point_t *out_bomb_pos,
                                            Point_t *out_wall_pos,
                                            SokoActionSeq_t *out_seq);
void Sokoban_Bomb_Search_Cancel(void);

/* ======================================================================
 *  顶层迭代求解（推箱 + 多炸弹一气呵成）
 * ====================================================================== */

#ifdef SOKOBAN_PC_VALIDATION

/** 解算段类型 */
typedef enum {
    SOKO_PHASE_PUSH = 0,    /* 推箱段: movable=箱子起点(可为空) */
    SOKO_PHASE_BOMB = 1     /* 推炸弹段: 把 movable(炸弹) 推到 wall 后引爆 3×3 */
} SokoPhaseKind_e;

/** 单个解算段（按时序排列） */
typedef struct {
    SokoPhaseKind_e kind;
    SokoActionSeq_t seq;            /* 该段方向动作序列 */
    Point_t         player_start;   /* 该段起始玩家格（供 Sokoban_Actions_To_Waypoints） */
    Point_t         movable;        /* 被推物体起点（箱子/炸弹）；push 段可为 {-1,-1} */
    Point_t         wall;           /* 仅 BOMB 段：引爆墙体（= movable 终点） */
} SokoPhase_t;

/* 段数上限：最多 N 颗炸弹段 + N 个推箱段 */
#define SOKOBAN_MAX_PHASES   (SOKOBAN_MAX_BOXES * 2)

/** 完整解算结果（含炸弹段与推箱段） */
typedef struct {
    SokoPhase_t phases[SOKOBAN_MAX_PHASES];
    uint8       count;
    uint8       is_solved;          /* 1 = 全部箱子可入目标 */
} SokoPlan_t;

/**
 * @brief  顶层迭代求解：推箱失败→炸墙开路→爆炸→重规划，循环至通关或无解
 *
 * 与固件 `app_game_logic` 的 stage_plan/stage_execute 循环等价（纯函数版）：
 *   每轮：无箱→成功；否则整体推箱 (Stage2/Stage1)；失败则按
 *   死局→不可达目标→通用破局 选一颗炸弹炸墙，模拟"推到墙+3×3 爆破"后重算。
 *
 * 解决「单次 Stage3 只放 1 颗炸弹」无法应对**需 ≥2 颗炸弹**的关卡
 * （例如：先炸开夹死某箱子的墙，再炸通另一目标的走廊）。
 *
 * @param  map                当前地图（不被修改）
 * @param  player_pos         玩家起点
 * @param  box_to_target_idx  箱→目标映射（NULL ⇒ Stage1 任意配对，自测/第 1 关用）
 * @param  box_count          mapping 非空时用于校验箱子数
 * @param  out_plan           [out] 有序解算段（炸弹段 + 推箱段）
 * @return 1=可通关, 0=无解（应进入死局复位）
 */
uint8 Sokoban_Solve_Full(const uint8 map[MAP_ROWS][MAP_COLS],
                         Point_t player_pos,
                         const uint8 *box_to_target_idx,
                         uint8 box_count,
                         SokoPlan_t *out_plan);

#endif /* SOKOBAN_PC_VALIDATION */

#endif /* _ALGO_SOKOBAN_SOLVER_H_ */
