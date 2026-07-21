#include "algo_sokoban_solver.h"
#include "chassis_config.h"
#include <string.h>
#include <stdlib.h>     /* abs() */
#include <math.h>       /* sqrtf() */

/*===========================================================================
 *  [algo_sokoban_solver.c] 推箱子求解 + 导航 BFS + 炸弹策略
 *
 *  @owner  rt1064-main
 *  @periph none                  纯算法 (BFS 导航 + 推箱子求解器 + 死局检测)
 *---------------------------------------------------------------------------
 *  分区索引:
 *    § 1. 内部常量 / 静态数组 / 推宏 A* 状态                [L  10±]
 *    § 2. 导航 BFS (Algo_Nav_BFS, 原 algo_bfs_scout)              [L  62±]
 *    § 3. 观察点查找 (Algo_Find_Nearest_Box_Observe_Point)         [L  78±]
 *    § 4. 单箱推宏 A*（箱格+上次推向，导航 BFS 连接）       [L 220±]
 *    § 5. Stage1/2 多箱分解与死局检测 (extract/simulate/deadlock)[L 425±]
 *    § 6. 炸弹策略 (Find_Bomb_Wall / Apply_Bomb_Explosion / Push_Bomb)[L 685±]
 *    § 7. 动作序列 → 航点路径 (Sokoban_Actions_To_Waypoints)     [L 789±]
 *===========================================================================*/

/*===========================================================================
 *  内部常量
 *===========================================================================*/

/** 方向偏移：UP, DOWN, LEFT, RIGHT */
static const int8 s_dr[4] = { -1,  1,  0,  0 };
static const int8 s_dc[4] = {  0,  0, -1,  1 };

/**
 * 单箱宏状态只记录“箱子格 + 上一次推箱方向”。玩家在一次推箱后必然位于
 * 箱子反方向相邻格，故无需把玩家全坐标并入状态。
 *
 * 旧动作层 BFS 有 192*192=36864 个状态、约 31.5KB BSS；宏状态 A* 仅
 * 192*4=768 个状态，行走段按需调用导航 BFS，既降低内存也把转弯停站成本
 * 纳入搜索目标。
 */
#define SB_RC                   (MAP_ROWS * MAP_COLS)       /* 192 */
#define SB_MACRO_STATE_COUNT    (SB_RC * 4U)                /* 768 */
#define SB_MACRO_BITMAP_BYTES   ((SB_MACRO_STATE_COUNT + 7U) / 8U)
#define SB_MACRO_PREV_NONE      (0xFFFFU)
#define SB_MACRO_COST_INF       (0xFFFFFFFFUL)

/* 与正式执行层一致的规划标尺（毫秒量级，不用于精确物理仿真）。 */
#define SOKO_COST_MOVE_MS       (100UL)
#define SOKO_COST_WAYPOINT_MS   (400UL)
#define SOKO_COST_SNAP_MS       (200UL)

static uint32 sb_macro_cost[SB_MACRO_STATE_COUNT];
static uint16 sb_macro_prev[SB_MACRO_STATE_COUNT];
static uint8  sb_macro_open[SB_MACRO_BITMAP_BYTES];
static uint8  sb_macro_closed[SB_MACRO_BITMAP_BYTES];
static uint16 sb_macro_reverse[SB_MACRO_STATE_COUNT];
static uint8  sb_macro_base_map[MAP_ROWS][MAP_COLS];
static uint8  sb_macro_walk_map[MAP_ROWS][MAP_COLS];
static NavPath_t sb_macro_walk_path;

/** 子地图临时缓冲 */
static uint8  sb_sub_map[MAP_ROWS][MAP_COLS];
/* Stage1/2 贪心只会串行执行，复用同一份 BFS 距离缓冲，避免为 Stage2 额外增加 BSS。 */
static uint8  sb_nav_distance[MAP_ROWS][MAP_COLS];

/*===========================================================================
 *  原 algo_bfs_scout.c 实现（合并到本文件）
 *===========================================================================*/

/**
 * 定义静态大数组防止单片机栈溢出
 * (16x12 = 192个节点，内存占用极小)
 */
static Point_t bfs_queue[MAP_ROWS * MAP_COLS];
static Point_t parent_map[MAP_ROWS][MAP_COLS];
static uint8   bfs_visited[MAP_ROWS][MAP_COLS];

/* P0-4: Algo_Nav_BFS 反推路径用临时缓冲, 由栈迁至 BSS (~400B)。
 *       与 bfs_queue/parent_map/bfs_visited 共享同一条非可重入约束:
 *       仅允许主循环线程同时调用 1 次, 严禁 ISR / 嵌套 / 递归调用。 */
static Point_t s_nav_temp_path[200];

/** 判断坐标是否位于可通行内场（最外圈边界不可进入） */
static inline uint8 map_is_inner_cell(int8 y, int8 x)
{
    if (x < (int8)CHASSIS_GRID_INNER_MIN_X || x > (int8)CHASSIS_GRID_INNER_MAX_X) return 0;
    if (y < (int8)CHASSIS_GRID_INNER_MIN_Y || y > (int8)CHASSIS_GRID_INNER_MAX_Y) return 0;
    return 1;
}

/** 判断坐标是否可通行（空地/目标可通过，箱子/炸弹/墙不可通过） */
static uint8 algo_is_nav_passable(const uint8 map[MAP_ROWS][MAP_COLS], int8 y, int8 x)
{
    if (!map_is_inner_cell(y, x)) return 0;
    if (map[y][x] == MAP_EMPTY || map[y][x] == MAP_TARGET) return 1;
    return 0;
}

/* ==========================================================================
 *  § 3. 观察点查找 (原 BFS 扫描路径上距离箱子最近的可推点)
 * ========================================================================== */

ObservePoint_t Algo_Find_Nearest_Box_Observe_Point(const uint8 map[MAP_ROWS][MAP_COLS],
                                                   Point_t player_pos,
                                                   Point_t *target_box_pos)
{
    ObservePoint_t result = {{0, 0}, 0};
    int16 head = 0, tail = 0;

    if (0 == target_box_pos) {
        return result;
    }

    /* 起点异常时先钳位到内场，避免访问边界或越界下标。 */
    if (player_pos.x < (int8)CHASSIS_GRID_INNER_MIN_X) player_pos.x = (int8)CHASSIS_GRID_INNER_MIN_X;
    if (player_pos.x > (int8)CHASSIS_GRID_INNER_MAX_X) player_pos.x = (int8)CHASSIS_GRID_INNER_MAX_X;
    if (player_pos.y < (int8)CHASSIS_GRID_INNER_MIN_Y) player_pos.y = (int8)CHASSIS_GRID_INNER_MIN_Y;
    if (player_pos.y > (int8)CHASSIS_GRID_INNER_MAX_Y) player_pos.y = (int8)CHASSIS_GRID_INNER_MAX_Y;

    memset(bfs_visited, 0, sizeof(bfs_visited));

    /* 玩家起点入队 */
    bfs_queue[tail++] = player_pos;
    bfs_visited[player_pos.y][player_pos.x] = 1;

    while (head < tail) {
        Point_t current = bfs_queue[head++];

        /* 1. 检查当前网格四周是否有箱子 */
        for (int i = 0; i < 4; i++) {
            int8 ny = current.y + s_dr[i];
            int8 nx = current.x + s_dc[i];

            if (map_is_inner_cell(ny, nx)) {
                if (map[ny][nx] == MAP_BOX) {
                    result.pos = current;
                    result.is_valid = 1;
                    target_box_pos->x = nx;
                    target_box_pos->y = ny;
                    return result;
                }
            }
        }

        /* 2. 没看到箱子，继续向四周可通行区域扩散 */
        for (int i = 0; i < 4; i++) {
            int8 ny = current.y + s_dr[i];
            int8 nx = current.x + s_dc[i];

            if (algo_is_nav_passable(map, ny, nx) && !bfs_visited[ny][nx]) {
                bfs_visited[ny][nx] = 1;
                bfs_queue[tail].x = nx;
                bfs_queue[tail].y = ny;
                tail++;
            }
        }
    }
    return result;
}

uint8 Algo_Nav_BFS(const uint8 map[MAP_ROWS][MAP_COLS],
                   Point_t start,
                   Point_t end,
                   NavPath_t *result_path)
{
    int16 head = 0, tail = 0;
    uint8 found = 0;

    if (0 == result_path) {
        return 0;
    }

    /* 起终点必须在内场且可通行。 */
    if (!map_is_inner_cell(start.y, start.x) || !map_is_inner_cell(end.y, end.x)) {
        return 0;
    }
    if (!algo_is_nav_passable(map, start.y, start.x) || !algo_is_nav_passable(map, end.y, end.x)) {
        return 0;
    }

    result_path->step_count = 0;
    if (start.x == end.x && start.y == end.y) {
        return 1;
    }

    memset(bfs_visited, 0, sizeof(bfs_visited));
    for (int i = 0; i < MAP_ROWS; i++) {
        for (int j = 0; j < MAP_COLS; j++) {
            parent_map[i][j].x = -1;
            parent_map[i][j].y = -1;
        }
    }

    bfs_queue[tail++] = start;
    bfs_visited[start.y][start.x] = 1;

    while (head < tail) {
        Point_t current = bfs_queue[head++];

        if (current.x == end.x && current.y == end.y) {
            found = 1;
            break;
        }

        for (int i = 0; i < 4; i++) {
            int8 ny = current.y + s_dr[i];
            int8 nx = current.x + s_dc[i];

            if (algo_is_nav_passable(map, ny, nx) && !bfs_visited[ny][nx]) {
                bfs_visited[ny][nx] = 1;
                parent_map[ny][nx] = current;

                bfs_queue[tail].x = nx;
                bfs_queue[tail].y = ny;
                tail++;
            }
        }
    }

    if (found) {
        /* P0-4: temp_path 由栈数组 (Point_t[200]=400B) 迁至文件级 static s_nav_temp_path,
         *       与 bfs_queue/parent_map/bfs_visited 同属本函数私有 BSS, 非可重入。 */
        uint16 step = 0;
        Point_t curr = end;

        while (curr.x != start.x || curr.y != start.y) {
            s_nav_temp_path[step++] = curr;
            curr = parent_map[curr.y][curr.x];
            if (step >= (uint16)(sizeof(s_nav_temp_path) / sizeof(s_nav_temp_path[0]))) {
                /* 路径异常超长 (>200), 防越界直接判失败; 正常 12x16 地图最大步数 <192 */
                return 0;
            }
        }

        result_path->step_count = step;
        for (int i = 0; i < step; i++) {
            result_path->path[i] = s_nav_temp_path[step - 1 - i];
        }
        return 1;
    }

    return 0;
}

uint8 Algo_Nav_BFS_Flood(const uint8 map[MAP_ROWS][MAP_COLS],
                         Point_t start,
                         uint8 reach[MAP_ROWS][MAP_COLS],
                         uint8 distance_steps[MAP_ROWS][MAP_COLS])
{
    int16 head = 0;
    int16 tail = 0;

    if (!reach && !distance_steps) return 0;

    memset(bfs_visited, 0, sizeof(bfs_visited));
    if (reach) {
        memset(reach, 0, (size_t)(MAP_ROWS * MAP_COLS));
    }
    if (distance_steps) {
        memset(distance_steps, ALGO_NAV_DISTANCE_UNREACHABLE,
               (size_t)(MAP_ROWS * MAP_COLS));
    }
    if (!map_is_inner_cell(start.y, start.x) ||
        !algo_is_nav_passable(map, start.y, start.x)) {
        return 0;
    }

    bfs_queue[tail++] = start;
    bfs_visited[start.y][start.x] = 1U;
    if (reach) reach[start.y][start.x] = 1U;
    if (distance_steps) distance_steps[start.y][start.x] = 0U;

    while (head < tail) {
        Point_t current = bfs_queue[head++];

        for (int i = 0; i < 4; i++) {
            int8 ny = current.y + s_dr[i];
            int8 nx = current.x + s_dc[i];

            if (algo_is_nav_passable(map, ny, nx) && !bfs_visited[ny][nx]) {
                bfs_visited[ny][nx] = 1U;
                if (reach) reach[ny][nx] = 1U;
                if (distance_steps) {
                    distance_steps[ny][nx] =
                        (uint8)(distance_steps[current.y][current.x] + 1U);
                }
                bfs_queue[tail].x = nx;
                bfs_queue[tail].y = ny;
                tail++;
            }
        }
    }
    return 1;
}

uint8 Algo_Nav_Is_Reachable(const uint8 reach[MAP_ROWS][MAP_COLS],
                            Point_t target)
{
    if (!reach || !map_is_inner_cell(target.y, target.x)) return 0;
    return reach[target.y][target.x] ? 1U : 0U;
}

/** 玩家到箱子四邻接可站立格的最短距离；箱子本身不可通行，不能直接查其坐标。 */
static uint8 nav_distance_to_box(const uint8 distance_steps[MAP_ROWS][MAP_COLS],
                                 Point_t box)
{
    uint8 best = ALGO_NAV_DISTANCE_UNREACHABLE;

    for (int d = 0; d < 4; d++) {
        int8 y = (int8)(box.y + s_dr[d]);
        int8 x = (int8)(box.x + s_dc[d]);
        if (map_is_inner_cell(y, x) && distance_steps[y][x] < best) {
            best = distance_steps[y][x];
        }
    }
    return best;
}

/*===========================================================================
 *  内联辅助函数
 *===========================================================================*/

/* ==========================================================================
 *  § 4. 单箱推宏 A* — 状态=(箱子格, 上次推向)，边=绕行到箱后并推一步
 * ========================================================================== */

static inline uint8 sb_is_free(const uint8 map[MAP_ROWS][MAP_COLS], int8 r, int8 c)
{
    if (!map_is_inner_cell(r, c)) return 0U;
    return (map[r][c] == MAP_EMPTY || map[r][c] == MAP_TARGET) ? 1U : 0U;
}

static inline uint8 sb_bm_test(const uint8 *bm, uint16 idx)
{
    return (uint8)((bm[idx >> 3] >> (idx & 7U)) & 1U);
}

static inline void sb_bm_set(uint8 *bm, uint16 idx)
{
    bm[idx >> 3] |= (uint8)(1U << (idx & 7U));
}

static inline void sb_bm_clear(uint8 *bm, uint16 idx)
{
    bm[idx >> 3] &= (uint8)~(uint8)(1U << (idx & 7U));
}

static inline uint16 sb_macro_encode(Point_t box, uint8 push_dir)
{
    return (uint16)(((uint16)(box.y * MAP_COLS + box.x) * 4U) + push_dir);
}

static inline Point_t sb_macro_box(uint16 state)
{
    Point_t box;
    uint16 cell = (uint16)(state / 4U);
    box.y = (int8)(cell / MAP_COLS);
    box.x = (int8)(cell % MAP_COLS);
    return box;
}

static inline void soko_push_bit_set(SokoActionSeq_t *seq, uint16 index)
{
    seq->push_bitmap[index >> 3] |= (uint8)(1U << (index & 7U));
}

static inline uint8 soko_push_bit_test(const SokoActionSeq_t *seq, uint16 index)
{
    return (uint8)((seq->push_bitmap[index >> 3] >> (index & 7U)) & 1U);
}

static uint8 soko_action_from_points(Point_t from, Point_t to, uint8 *dir)
{
    if (!dir) return 0U;
    for (uint8 d = 0U; d < 4U; ++d) {
        if ((int8)(from.y + s_dr[d]) == to.y &&
            (int8)(from.x + s_dc[d]) == to.x) {
            *dir = d;
            return 1U;
        }
    }
    return 0U;
}

static uint8 soko_seq_append(SokoActionSeq_t *seq, uint8 dir, uint8 is_push)
{
    uint16 index;
    if (!seq || dir >= 4U || seq->count >= SOKOBAN_MAX_ACTIONS) return 0U;
    index = seq->count++;
    seq->actions[index] = (SokoAction_e)dir;
    if (is_push) soko_push_bit_set(seq, index);
    return 1U;
}

/** 计算一次“走到箱后并推一步”的执行时间标尺。 */
static uint32 sb_macro_edge_cost(const NavPath_t *walk,
                                 Point_t walk_start,
                                 uint8 previous_push_dir,
                                 uint8 push_dir)
{
    Point_t prev = walk_start;
    uint8 last_dir = previous_push_dir;
    uint8 had_walk = 0U;
    uint32 cost = 0UL;

    for (uint16 i = 0U; i < walk->step_count; ++i) {
        uint8 dir;
        if (!soko_action_from_points(prev, walk->path[i], &dir)) return SB_MACRO_COST_INF;
        cost += SOKO_COST_MOVE_MS;
        if (last_dir >= 4U || dir != last_dir) cost += SOKO_COST_WAYPOINT_MS;
        last_dir = dir;
        prev = walk->path[i];
        had_walk = 1U;
    }

    cost += SOKO_COST_MOVE_MS;
    if (last_dir >= 4U || push_dir != last_dir) cost += SOKO_COST_WAYPOINT_MS;

    /* 连续同向推仍属于同一个关键航段，只计一次 Snap。 */
    if (had_walk || previous_push_dir >= 4U || push_dir != previous_push_dir) {
        cost += SOKO_COST_SNAP_MS;
    }
    return cost;
}

/** 检查一条推宏边并返回其导航路径与代价。 */
static uint8 sb_macro_make_edge(const uint8 sub_map[MAP_ROWS][MAP_COLS],
                                Point_t player,
                                Point_t box,
                                Point_t target,
                                uint8 push_dir,
                                uint8 previous_push_dir,
                                uint8 block_other_targets,
                                uint32 *edge_cost)
{
    Point_t stand;
    Point_t box_next;

    stand.y = (int8)(box.y - s_dr[push_dir]);
    stand.x = (int8)(box.x - s_dc[push_dir]);
    box_next.y = (int8)(box.y + s_dr[push_dir]);
    box_next.x = (int8)(box.x + s_dc[push_dir]);

    if (!sb_is_free(sub_map, stand.y, stand.x) ||
        !sb_is_free(sub_map, box_next.y, box_next.x)) {
        return 0U;
    }
    if (block_other_targets &&
        sub_map[box_next.y][box_next.x] == MAP_TARGET &&
        !(box_next.x == target.x && box_next.y == target.y)) {
        return 0U;
    }

    memcpy(sb_macro_walk_map, sub_map, sizeof(sb_macro_walk_map));
    sb_macro_walk_map[box.y][box.x] = MAP_WALL;
    if (!Algo_Nav_BFS(sb_macro_walk_map, player, stand, &sb_macro_walk_path)) return 0U;

    if (edge_cost) {
        *edge_cost = sb_macro_edge_cost(&sb_macro_walk_path,
                                        player,
                                        previous_push_dir,
                                        push_dir);
    }
    return 1U;
}

static uint8 sokoban_bfs_single(const uint8 sub_map[MAP_ROWS][MAP_COLS],
                                Point_t player, Point_t box, Point_t target,
                                SokoActionSeq_t *sol,
                                uint8 block_other_targets)
{
    uint16 goal_state = SB_MACRO_PREV_NONE;

    if (!sol) return 0U;
    if (!map_is_inner_cell(player.y, player.x) ||
        !map_is_inner_cell(box.y, box.x) ||
        !map_is_inner_cell(target.y, target.x)) {
        return 0U;
    }

    sol->count = 0U;
    memset(sol->push_bitmap, 0, sizeof(sol->push_bitmap));
    if (box.y == target.y && box.x == target.x) return 1U;

    memcpy(sb_macro_base_map, sub_map, sizeof(sb_macro_base_map));
    sb_macro_base_map[box.y][box.x] = MAP_EMPTY;

    for (uint16 i = 0U; i < SB_MACRO_STATE_COUNT; ++i) {
        sb_macro_cost[i] = SB_MACRO_COST_INF;
        sb_macro_prev[i] = SB_MACRO_PREV_NONE;
    }
    memset(sb_macro_open, 0, sizeof(sb_macro_open));
    memset(sb_macro_closed, 0, sizeof(sb_macro_closed));

    /* 由任意初始玩家位置产生第一推。 */
    for (uint8 d = 0U; d < 4U; ++d) {
        uint32 edge_cost;
        Point_t next_box;
        uint16 state;
        if (!sb_macro_make_edge(sb_macro_base_map, player, box, target, d,
                                (uint8)SOKO_ACT_NONE,
                                block_other_targets, &edge_cost)) {
            continue;
        }
        next_box.y = (int8)(box.y + s_dr[d]);
        next_box.x = (int8)(box.x + s_dc[d]);
        state = sb_macro_encode(next_box, d);
        if (edge_cost < sb_macro_cost[state]) {
            sb_macro_cost[state] = edge_cost;
            sb_macro_prev[state] = SB_MACRO_PREV_NONE;
            sb_bm_set(sb_macro_open, state);
        }
    }

    while (1) {
        uint16 current = SB_MACRO_PREV_NONE;
        uint32 best_f = SB_MACRO_COST_INF;

        for (uint16 state = 0U; state < SB_MACRO_STATE_COUNT; ++state) {
            Point_t state_box;
            uint32 heuristic;
            uint32 f;
            if (!sb_bm_test(sb_macro_open, state)) continue;
            state_box = sb_macro_box(state);
            heuristic = (uint32)(abs(state_box.x - target.x) +
                                 abs(state_box.y - target.y)) * SOKO_COST_MOVE_MS;
            f = sb_macro_cost[state] + heuristic;
            if (f < best_f) {
                best_f = f;
                current = state;
            }
        }
        if (current == SB_MACRO_PREV_NONE) return 0U;

        sb_bm_clear(sb_macro_open, current);
        sb_bm_set(sb_macro_closed, current);

        {
            Point_t current_box = sb_macro_box(current);
            uint8 previous_dir = (uint8)(current & 3U);
            Point_t current_player;
            current_player.y = (int8)(current_box.y - s_dr[previous_dir]);
            current_player.x = (int8)(current_box.x - s_dc[previous_dir]);

            if (current_box.x == target.x && current_box.y == target.y) {
                goal_state = current;
                break;
            }

            for (uint8 d = 0U; d < 4U; ++d) {
                uint32 edge_cost;
                uint32 next_cost;
                Point_t next_box;
                uint16 next_state;
                if (!sb_macro_make_edge(sb_macro_base_map, current_player, current_box,
                                        target, d, previous_dir,
                                        block_other_targets, &edge_cost)) {
                    continue;
                }
                next_box.y = (int8)(current_box.y + s_dr[d]);
                next_box.x = (int8)(current_box.x + s_dc[d]);
                next_state = sb_macro_encode(next_box, d);
                if (sb_bm_test(sb_macro_closed, next_state)) continue;
                next_cost = sb_macro_cost[current] + edge_cost;
                if (next_cost < sb_macro_cost[next_state]) {
                    sb_macro_cost[next_state] = next_cost;
                    sb_macro_prev[next_state] = current;
                    sb_bm_set(sb_macro_open, next_state);
                }
            }
        }
    }

    /* 反转推宏链，再逐段重建完整行走+推箱动作。 */
    {
        uint16 macro_count = 0U;
        uint16 state = goal_state;
        Point_t cur_player = player;
        Point_t cur_box = box;

        while (state != SB_MACRO_PREV_NONE) {
            if (macro_count >= SB_MACRO_STATE_COUNT) return 0U;
            sb_macro_reverse[macro_count++] = state;
            state = sb_macro_prev[state];
        }

        for (uint16 ri = macro_count; ri > 0U; --ri) {
            uint16 push_state = sb_macro_reverse[ri - 1U];
            uint8 push_dir = (uint8)(push_state & 3U);
            Point_t stand;
            Point_t walk_prev = cur_player;
            stand.y = (int8)(cur_box.y - s_dr[push_dir]);
            stand.x = (int8)(cur_box.x - s_dc[push_dir]);

            memcpy(sb_macro_walk_map, sb_macro_base_map, sizeof(sb_macro_walk_map));
            sb_macro_walk_map[cur_box.y][cur_box.x] = MAP_WALL;
            if (!Algo_Nav_BFS(sb_macro_walk_map, cur_player, stand,
                              &sb_macro_walk_path)) {
                return 0U;
            }
            for (uint16 wi = 0U; wi < sb_macro_walk_path.step_count; ++wi) {
                uint8 walk_dir;
                if (!soko_action_from_points(walk_prev,
                                             sb_macro_walk_path.path[wi],
                                             &walk_dir) ||
                    !soko_seq_append(sol, walk_dir, 0U)) {
                    return 0U;
                }
                walk_prev = sb_macro_walk_path.path[wi];
            }
            if (!soko_seq_append(sol, push_dir, 1U)) return 0U;

            cur_player = cur_box;
            cur_box.y = (int8)(cur_box.y + s_dr[push_dir]);
            cur_box.x = (int8)(cur_box.x + s_dc[push_dir]);
        }
    }
    return 1U;
}

/*===========================================================================
 *  辅助：在地图上提取指定类型元素坐标
 *===========================================================================*/
/* ==========================================================================
 *  § 5. Stage1/2 多箱分解 + 死局检测 (extract / simulate / deadlock)
 * ========================================================================== */

static uint8 extract_elements(const uint8 map[MAP_ROWS][MAP_COLS],
                              MapElement_e type,
                              Point_t out[], uint8 max_count)
{
    uint8 n = 0;
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (map[r][c] == (uint8)type && n < max_count) {
                out[n].y = r;
                out[n].x = c;
                n++;
            }
        }
    }
    return n;
}

/*===========================================================================
 *  辅助：模拟动作序列，获取玩家最终坐标
 *===========================================================================*/
static Point_t simulate_actions(const SokoActionSeq_t *sol,
                                Point_t player, Point_t box)
{
    int8 pr = player.y, pc = player.x;
    int8 br = box.y,    bc = box.x;

    for (uint16 i = 0; i < sol->count; i++) {
        int8 d  = (int8)sol->actions[i];
        int8 npr = pr + s_dr[d];
        int8 npc = pc + s_dc[d];

        if (npr == br && npc == bc) {
            br += s_dr[d];
            bc += s_dc[d];
        }
        pr = npr;
        pc = npc;
    }

    Point_t end;
    end.y = pr;
    end.x = pc;
    return end;
}

/*===========================================================================
 *  第三阶段辅助：死局检测 / 炸弹收益评估
 *===========================================================================*/

/** 判断格子是否属于“阻挡物”（越界、墙、箱子、炸弹均视为阻挡） */
static uint8 is_blocker_cell(const uint8 map[MAP_ROWS][MAP_COLS], int8 r, int8 c)
{
    if (!map_is_inner_cell(r, c)) return 1;
    return (map[r][c] == MAP_WALL || map[r][c] == MAP_BOX || map[r][c] == MAP_BOMB) ? 1 : 0;
}

/** O3.2: 箱子在当前静态地图上是否还存在"可被立即推动"的方向。
 *  方向 d 可推的充要条件:
 *    - 箱子去向格 box+Δd 可通行(空地/目标);
 *    - 玩家站位格 box-Δd 可通行(玩家需站在反侧才能向 d 推)。
 *  四个方向都不满足 → 该箱当前一步也动不了 → 死局。
 *  注意: 与既有角落判定一致, 把其他箱子/炸弹也视为阻挡(sb_is_free 仅放行空地/目标),
 *  因此与角落判定属同一保守口径——可能因"其他箱子之后会移走"而偏保守, 但只在
 *  推箱规划已失败后用于挑选破局方向, 退化路径安全。 */
static uint8 box_has_pushable_direction(const uint8 map[MAP_ROWS][MAP_COLS],
                                        int8 br, int8 bc)
{
    for (int d = 0; d < 4; d++) {
        int8 dest_r  = br + s_dr[d];
        int8 dest_c  = bc + s_dc[d];
        int8 stand_r = br - s_dr[d];
        int8 stand_c = bc - s_dc[d];
        if (sb_is_free(map, dest_r, dest_c) && sb_is_free(map, stand_r, stand_c)) {
            return 1;
        }
    }
    return 0;
}

/** O3.1: 边线冻结死局。贴内场外边界的箱子永远无法离开所贴的那条行/列:
 *    - 贴上/下边界: 朝边界外推不可行(去向格在界外), 反向推又需玩家站界外, 故纵向永久不可动,
 *      只能在该行内左右滑 → 该行若无任何目标, 箱子永远到不了目标, 死局;
 *    - 贴左/右边界: 同理横向永久不可动, 该列无目标即死局。
 *  这是 O3.2 之外、可移动箱子也可能成立的独立死局(箱子能滑动但永远脱不开此线)。 */
static uint8 box_frozen_on_border_line(const uint8 map[MAP_ROWS][MAP_COLS],
                                       int8 br, int8 bc)
{
    if (br == (int8)CHASSIS_GRID_INNER_MIN_Y || br == (int8)CHASSIS_GRID_INNER_MAX_Y) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (map[br][c] == MAP_TARGET) return 0;
        }
        return 1;
    }
    if (bc == (int8)CHASSIS_GRID_INNER_MIN_X || bc == (int8)CHASSIS_GRID_INNER_MAX_X) {
        for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
            if (map[r][bc] == MAP_TARGET) return 0;
        }
        return 1;
    }
    return 0;
}

/** 死局检测:
 *   第一遍 — 角落死局(箱子卡在两垂直阻挡夹角), 保持历史判定与返回的箱子完全不变;
 *   第二遍 — 仅当无角落死局时, 再补 O3.2(无可推方向) / O3.1(边线冻结)两类。
 *  分两遍是为了保证"原本能判出角落死局"的场景结果与返回箱子逐字节不变,
 *  新增类型只在原本判为"无死局"时才可能额外命中。 */
uint8 Sokoban_Is_Deadlock(const uint8 map[MAP_ROWS][MAP_COLS],
                          Point_t *dead_box_pos)
{
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (map[r][c] != MAP_BOX) continue;

            {
                uint8 up    = is_blocker_cell(map, r - 1, c);
                uint8 down  = is_blocker_cell(map, r + 1, c);
                uint8 left  = is_blocker_cell(map, r, c - 1);
                uint8 right = is_blocker_cell(map, r, c + 1);

                if ((up && left) || (up && right) || (down && left) || (down && right)) {
                    if (dead_box_pos) {
                        dead_box_pos->x = c;
                        dead_box_pos->y = r;
                    }
                    return 1;
                }
            }
        }
    }

    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (map[r][c] != MAP_BOX) continue;

            if (!box_has_pushable_direction(map, r, c) ||
                box_frozen_on_border_line(map, r, c)) {
                if (dead_box_pos) {
                    dead_box_pos->x = c;
                    dead_box_pos->y = r;
                }
                return 1;
            }
        }
    }
    return 0;
}

/*===========================================================================
 *  辅助：构建子地图
 *
 *  参数:
 *    base_map        原始地图
 *    boxes/targets   箱子/目标坐标数组
 *    box_count       箱子总数
 *    solved_flags    已完成标记 (1=已推到位并消失)
 *    cur_box_idx     当前要推送的箱子索引
 *    cur_target_idx  当前目标索引
 *    out_map         输出子地图
 *===========================================================================*/
static void build_sub_map(const uint8 base_map[MAP_ROWS][MAP_COLS],
                          const Point_t boxes[],
                          const Point_t targets[],
                          uint8 box_count, uint8 target_count,
                          const uint8 solved_flags[],
                          const uint8 target_used_flags[],
                          uint8 cur_box_idx, uint8 cur_target_idx,
                          uint8 out_map[MAP_ROWS][MAP_COLS])
{
    memcpy(out_map, base_map, MAP_ROWS * MAP_COLS);

    /* 处理箱子 */
    for (uint8 i = 0; i < box_count; i++) {
        if (i == cur_box_idx) {
            /* 当前箱子: 从地图移除 (位置由 BFS 状态跟踪) */
            out_map[boxes[i].y][boxes[i].x] = MAP_EMPTY;
        } else if (solved_flags[i]) {
            /* 已完成箱子: 消失 → 空地 */
            out_map[boxes[i].y][boxes[i].x] = MAP_EMPTY;
        } else {
            /* 其余未完成箱子: 视为墙壁 */
            out_map[boxes[i].y][boxes[i].x] = MAP_WALL;
        }
    }

    /* Unused targets stay as MAP_TARGET; box BFS blocks entering them. */
    for (uint8 i = 0; i < target_count; i++) {
        if (i == cur_target_idx) continue;   /* 当前目标保留 */
        if (target_used_flags != 0 &&
            target_used_flags[i] &&
            out_map[targets[i].y][targets[i].x] == MAP_TARGET) {
            out_map[targets[i].y][targets[i].x] = MAP_EMPTY;
        }
    }

    /* 炸弹保持原样 — sb_is_free 自动将其视为障碍 */
}

/*===========================================================================
 *  公开 API: 第一阶段求解
 *===========================================================================*/
static uint8 sokoban_solve_stage1_greedy(const uint8 map[MAP_ROWS][MAP_COLS],
                                         Point_t player_pos,
                                         SokoFullSolution_t *result)
{
    Point_t boxes[SOKOBAN_MAX_BOXES];
    Point_t targets[SOKOBAN_MAX_BOXES];
    uint8   solved[SOKOBAN_MAX_BOXES]  = {0};
    uint8   t_used[SOKOBAN_MAX_BOXES]  = {0};

    result->is_solved   = 0;
    result->total_boxes = 0;

    uint8 box_n    = extract_elements(map, MAP_BOX,    boxes,   SOKOBAN_MAX_BOXES);
    uint8 target_n = extract_elements(map, MAP_TARGET, targets, SOKOBAN_MAX_BOXES);

    if (box_n == 0 || box_n != target_n) return 0;

    result->total_boxes = box_n;
    Point_t cur_player = player_pos;

    /* 逐个解算：仍按“玩家可接近距离、箱到目标距离”排序，但最近候选无解时
     * 继续尝试同轮其他箱/目标，避免一个局部死配对把整张可解地图误判无解。 */
    for (uint8 done = 0; done < box_n; done++) {
        int8 chosen_b = -1;
        int8 chosen_t = -1;
        uint8 box_tried[SOKOBAN_MAX_BOXES] = {0};
        SokoActionSeq_t *sol = &result->sub_solutions[done];

        /* 从当前玩家一次扩散，得到所有未完成箱子的实际绕障接近距离。 */
        memcpy(sb_sub_map, map, sizeof(sb_sub_map));
        for (uint8 i = 0; i < box_n; i++) {
            if (solved[i]) sb_sub_map[boxes[i].y][boxes[i].x] = MAP_EMPTY;
        }
        if (!Algo_Nav_BFS_Flood(sb_sub_map, cur_player, 0, sb_nav_distance)) return 0;

        for (uint8 box_try = 0U; box_try < box_n; ++box_try) {
            int8 best_b = -1;
            uint16 best_box_d = 0xFFFFU;
            uint8 target_tried[SOKOBAN_MAX_BOXES] = {0};

            for (uint8 i = 0U; i < box_n; ++i) {
                uint16 d;
                if (solved[i] || box_tried[i]) continue;
                d = nav_distance_to_box(sb_nav_distance, boxes[i]);
                if ((d < best_box_d) && (d < ALGO_NAV_DISTANCE_UNREACHABLE)) {
                    best_box_d = d;
                    best_b = (int8)i;
                }
            }
            if (best_b < 0) break;
            box_tried[(uint8)best_b] = 1U;

            for (uint8 target_try = 0U; target_try < target_n; ++target_try) {
                int8 best_t = -1;
                int16 best_target_d = 32767;

                for (uint8 i = 0U; i < target_n; ++i) {
                    int16 d;
                    if (t_used[i] || target_tried[i]) continue;
                    d = (int16)(abs(targets[i].x - boxes[best_b].x)
                              + abs(targets[i].y - boxes[best_b].y));
                    if (d < best_target_d) {
                        best_target_d = d;
                        best_t = (int8)i;
                    }
                }
                if (best_t < 0) break;
                target_tried[(uint8)best_t] = 1U;

                build_sub_map(map, boxes, targets, box_n, target_n,
                              solved, t_used, (uint8)best_b, (uint8)best_t, sb_sub_map);
                if (sokoban_bfs_single(sb_sub_map, cur_player,
                                       boxes[best_b], targets[best_t], sol, 1U)) {
                    chosen_b = best_b;
                    chosen_t = best_t;
                    break;
                }
            }
            if (chosen_b >= 0) break;
        }
        if (chosen_b < 0 || chosen_t < 0) return 0;

        cur_player = simulate_actions(sol, cur_player, boxes[chosen_b]);
        result->player_end_pos[done] = cur_player;

        solved[(uint8)chosen_b] = 1U;
        t_used[(uint8)chosen_t] = 1U;
    }

    result->is_solved = 1;
    return 1;
}

/*===========================================================================
 *  公开 API: 第二阶段求解
 *===========================================================================*/
static uint8 sokoban_solve_stage2_greedy(const uint8 map[MAP_ROWS][MAP_COLS],
                                         Point_t player_pos,
                                         const uint8 box_to_target_idx[],
                                         uint8 box_count_in,
                                         SokoFullSolution_t *result)
{
    Point_t boxes[SOKOBAN_MAX_BOXES];
    Point_t targets[SOKOBAN_MAX_BOXES];
    uint8   solved[SOKOBAN_MAX_BOXES] = {0};
    uint8   t_used[SOKOBAN_MAX_BOXES] = {0};

    result->is_solved   = 0;
    result->total_boxes = 0;

    uint8 box_n    = extract_elements(map, MAP_BOX,    boxes,   SOKOBAN_MAX_BOXES);
    uint8 target_n = extract_elements(map, MAP_TARGET, targets, SOKOBAN_MAX_BOXES);

    if (box_n != box_count_in || box_n == 0) return 0;

    result->total_boxes = box_n;
    Point_t cur_player = player_pos;

    /* 按离当前玩家最近的箱子优先顺序依次求解
     * (与 Stage1 一致改用 BFS 绕障步数: 曼哈顿距离在有墙/箱阻挡时会选中
     *  "直线近但绕路远"的箱子, 导致整体路径变长甚至选中暂不可达的箱子) */
    for (uint8 done = 0; done < box_n; done++) {
        int8 chosen_b = -1;
        uint8 box_tried[SOKOBAN_MAX_BOXES] = {0};
        SokoActionSeq_t *sol = &result->sub_solutions[done];

        memcpy(sb_sub_map, map, sizeof(sb_sub_map));
        for (uint8 i = 0; i < box_n; i++) {
            if (solved[i]) sb_sub_map[boxes[i].y][boxes[i].x] = MAP_EMPTY;
        }
        if (!Algo_Nav_BFS_Flood(sb_sub_map, cur_player, 0, sb_nav_distance)) return 0;

        /* 固定映射不允许换目标，但最近箱无解时可先推下一只映射合法的箱。 */
        for (uint8 box_try = 0U; box_try < box_n; ++box_try) {
            int8 best_b = -1;
            uint16 best_box_d = 0xFFFFU;
            uint8 ti;

            for (uint8 i = 0U; i < box_n; ++i) {
                uint16 d;
                if (solved[i] || box_tried[i]) continue;
                d = nav_distance_to_box(sb_nav_distance, boxes[i]);
                if ((d < best_box_d) && (d < ALGO_NAV_DISTANCE_UNREACHABLE)) {
                    best_box_d = d;
                    best_b = (int8)i;
                }
            }
            if (best_b < 0) break;
            box_tried[(uint8)best_b] = 1U;

            ti = box_to_target_idx[(uint8)best_b];
            if (ti >= target_n) return 0;
            build_sub_map(map, boxes, targets, box_n, target_n,
                          solved, t_used, (uint8)best_b, ti, sb_sub_map);
            if (sokoban_bfs_single(sb_sub_map, cur_player,
                                   boxes[best_b], targets[ti], sol, 1U)) {
                chosen_b = best_b;
                break;
            }
        }
        if (chosen_b < 0) return 0;

        cur_player = simulate_actions(sol, cur_player, boxes[chosen_b]);
        result->player_end_pos[done] = cur_player;
        solved[(uint8)chosen_b] = 1U;
        t_used[box_to_target_idx[(uint8)chosen_b]] = 1U;
    }

    result->is_solved = 1;
    return 1;
}

/* ==========================================================================
 *  Stage1/2 稳健优化层
 *
 *  策略:
 *    1) 先跑旧贪心, 得到一个可行上界;
 *    2) 3 箱精确搜索, 5 箱分支限界搜索;
 *    3) 搜索失败/超限时保留贪心解, 不让规划层退化为无解。
 * ========================================================================== */

#define SOKO_OPT_EXACT_BOX_LIMIT       (3U)
#define SOKO_OPT_BRANCH_BOX_LIMIT      (5U)
#define SOKO_OPT_STAGE1_NODE_LIMIT_5   (1024UL)
#define SOKO_OPT_INF_COST              (0xFFFFU)
#define SOKO_STAGE1_PAIR_LIMIT         (256UL)

typedef struct {
    uint8 b;
    uint8 t;
    int16 h;
} SokoOptCandidate_t;

typedef struct {
    uint8 initialized;
    uint8 solved_mask;
    uint8 target_mask;
    uint8 candidate_count;
    uint8 next_candidate;
    Point_t player;
    uint32 cost;
    SokoOptCandidate_t candidates[SOKOBAN_MAX_BOXES * SOKOBAN_MAX_BOXES];
} SokoStage1Frame_t;

typedef struct {
    const uint8 (*map)[MAP_COLS];
    uint8 map_copy[MAP_ROWS][MAP_COLS];
    Point_t boxes[SOKOBAN_MAX_BOXES];
    Point_t targets[SOKOBAN_MAX_BOXES];
    uint8 box_n;
    uint8 target_n;
    const uint8 *mapping;
    uint8 fixed_mapping;

    uint32 node_count;
    uint32 node_limit;
    uint8  hit_limit;
    uint8  best_valid;
    uint32 best_cost;

    SokoActionSeq_t cur_seq[SOKOBAN_MAX_BOXES];
    Point_t         cur_end[SOKOBAN_MAX_BOXES];
    SokoFullSolution_t *out;

    SokoSearchStatus_e stage1_status;
    Point_t stage1_home;
    uint8 stage1_depth;
    SokoStage1Frame_t stage1_frames[SOKOBAN_MAX_BOXES + 1U];
} SokoOptContext_t;

static SokoOptContext_t s_soko_opt;

static int16 soko_abs_i16(int16 v)
{
    return (v < 0) ? (int16)-v : v;
}

static uint16 soko_solution_cost(const SokoFullSolution_t *sol)
{
    uint16 cost = 0U;
    if (!sol || !sol->is_solved) return SOKO_OPT_INF_COST;
    for (uint8 i = 0U; i < sol->total_boxes; ++i) {
        if ((uint16)(SOKO_OPT_INF_COST - cost) < sol->sub_solutions[i].count) {
            return SOKO_OPT_INF_COST;
        }
        cost = (uint16)(cost + sol->sub_solutions[i].count);
    }
    return cost;
}

static uint8 soko_mapping_is_valid(const uint8 mapping[], uint8 box_n, uint8 target_n)
{
    uint8 used = 0U;
    if (!mapping) return 0U;

    for (uint8 i = 0U; i < box_n; ++i) {
        uint8 ti = mapping[i];
        uint8 bit;
        if (ti >= target_n || ti >= 8U) return 0U;
        bit = (uint8)(1U << ti);
        if (used & bit) return 0U;
        used = (uint8)(used | bit);
    }
    return 1U;
}

static void soko_flags_from_mask(uint8 mask, uint8 flags[], uint8 n)
{
    for (uint8 i = 0U; i < n; ++i) {
        flags[i] = (uint8)((mask & (uint8)(1U << i)) ? 1U : 0U);
    }
}

static int16 soko_pair_heuristic(Point_t player, Point_t box, Point_t target)
{
    int16 d1 = (int16)(soko_abs_i16((int16)(box.x - player.x))
                     + soko_abs_i16((int16)(box.y - player.y)));
    int16 d2 = (int16)(soko_abs_i16((int16)(target.x - box.x))
                     + soko_abs_i16((int16)(target.y - box.y)));
    return (int16)(d1 + d2);
}

static void soko_sort_candidates(SokoOptCandidate_t cand[], uint8 n)
{
    for (uint8 i = 1U; i < n; ++i) {
        SokoOptCandidate_t key = cand[i];
        int8 j = (int8)i - 1;
        while (j >= 0 && cand[j].h > key.h) {
            cand[j + 1] = cand[j];
            --j;
        }
        cand[j + 1] = key;
    }
}

static void soko_opt_save_best(uint8 depth, uint16 cost)
{
    s_soko_opt.out->total_boxes = s_soko_opt.box_n;
    s_soko_opt.out->is_solved = 1U;
    for (uint8 i = 0U; i < depth; ++i) {
        s_soko_opt.out->sub_solutions[i] = s_soko_opt.cur_seq[i];
        s_soko_opt.out->player_end_pos[i] = s_soko_opt.cur_end[i];
    }
    s_soko_opt.best_cost = cost;
    s_soko_opt.best_valid = 1U;
}

static uint8 soko_opt_solve_pair(uint8 solved_mask,
                                 uint8 used_target_mask,
                                 uint8 box_idx,
                                 uint8 target_idx,
                                 Point_t cur_player,
                                 SokoActionSeq_t *seq,
                                 Point_t *end_player)
{
    uint8 solved_flags[SOKOBAN_MAX_BOXES];
    uint8 target_used_flags[SOKOBAN_MAX_BOXES];

    soko_flags_from_mask(solved_mask, solved_flags, s_soko_opt.box_n);
    soko_flags_from_mask(used_target_mask, target_used_flags, s_soko_opt.target_n);
    build_sub_map(s_soko_opt.map,
                  s_soko_opt.boxes,
                  s_soko_opt.targets,
                  s_soko_opt.box_n,
                  s_soko_opt.target_n,
                  solved_flags,
                  target_used_flags,
                  box_idx,
                  target_idx,
                  sb_sub_map);

    if (!sokoban_bfs_single(sb_sub_map,
                            cur_player,
                            s_soko_opt.boxes[box_idx],
                            s_soko_opt.targets[target_idx],
                            seq,
                            1U)) {
        return 0U;
    }

    *end_player = simulate_actions(seq, cur_player, s_soko_opt.boxes[box_idx]);
    return 1U;
}

static void soko_opt_dfs(uint8 depth,
                         uint8 solved_mask,
                         uint8 used_target_mask,
                         Point_t cur_player,
                         uint16 cost)
{
    SokoOptCandidate_t cand[SOKOBAN_MAX_BOXES * SOKOBAN_MAX_BOXES];
    uint8 cand_n = 0U;

    if (s_soko_opt.best_valid && cost >= s_soko_opt.best_cost) return;
    if (s_soko_opt.hit_limit) return;
    if (depth >= s_soko_opt.box_n) {
        soko_opt_save_best(depth, cost);
        return;
    }

    for (uint8 bi = 0U; bi < s_soko_opt.box_n; ++bi) {
        if (solved_mask & (uint8)(1U << bi)) continue;

        if (s_soko_opt.fixed_mapping) {
            uint8 ti = s_soko_opt.mapping[bi];
            cand[cand_n].b = bi;
            cand[cand_n].t = ti;
            cand[cand_n].h = soko_pair_heuristic(cur_player,
                                                  s_soko_opt.boxes[bi],
                                                  s_soko_opt.targets[ti]);
            ++cand_n;
        } else {
            for (uint8 ti = 0U; ti < s_soko_opt.target_n; ++ti) {
                if (used_target_mask & (uint8)(1U << ti)) continue;
                cand[cand_n].b = bi;
                cand[cand_n].t = ti;
                cand[cand_n].h = soko_pair_heuristic(cur_player,
                                                      s_soko_opt.boxes[bi],
                                                      s_soko_opt.targets[ti]);
                ++cand_n;
            }
        }
    }

    soko_sort_candidates(cand, cand_n);

    for (uint8 ci = 0U; ci < cand_n; ++ci) {
        uint8 bi = cand[ci].b;
        uint8 ti = cand[ci].t;
        uint16 next_cost;

        if (s_soko_opt.hit_limit) return;
        if (s_soko_opt.node_limit != 0UL) {
            if (s_soko_opt.node_count >= s_soko_opt.node_limit) {
                s_soko_opt.hit_limit = 1U;
                return;
            }
            ++s_soko_opt.node_count;
        }
        if (!soko_opt_solve_pair(solved_mask,
                                 used_target_mask,
                                 bi,
                                 ti,
                                 cur_player,
                                 &s_soko_opt.cur_seq[depth],
                                 &s_soko_opt.cur_end[depth])) {
            continue;
        }

        if ((uint16)(SOKO_OPT_INF_COST - cost) < s_soko_opt.cur_seq[depth].count) {
            continue;
        }
        next_cost = (uint16)(cost + s_soko_opt.cur_seq[depth].count);
        if (s_soko_opt.best_valid && next_cost >= s_soko_opt.best_cost) {
            continue;
        }

        soko_opt_dfs((uint8)(depth + 1U),
                     (uint8)(solved_mask | (uint8)(1U << bi)),
                     (uint8)(used_target_mask | (uint8)(1U << ti)),
                     s_soko_opt.cur_end[depth],
                     next_cost);
    }
}

static uint8 soko_opt_prepare(const uint8 map[MAP_ROWS][MAP_COLS],
                              const uint8 mapping[],
                              uint8 fixed_mapping,
                              SokoFullSolution_t *result,
                              uint8 fallback_ok)
{
    memset(&s_soko_opt, 0, sizeof(s_soko_opt));
    s_soko_opt.map = map;
    s_soko_opt.box_n = extract_elements(map, MAP_BOX,
                                        s_soko_opt.boxes,
                                        SOKOBAN_MAX_BOXES);
    s_soko_opt.target_n = extract_elements(map, MAP_TARGET,
                                           s_soko_opt.targets,
                                           SOKOBAN_MAX_BOXES);
    s_soko_opt.mapping = mapping;
    s_soko_opt.fixed_mapping = fixed_mapping;
    s_soko_opt.out = result;
    s_soko_opt.best_valid = fallback_ok ? 1U : 0U;
    s_soko_opt.best_cost = fallback_ok ? soko_solution_cost(result) : SOKO_OPT_INF_COST;

    if (s_soko_opt.box_n == 0U || s_soko_opt.box_n != s_soko_opt.target_n) {
        return 0U;
    }
    if (fixed_mapping && !soko_mapping_is_valid(mapping,
                                                s_soko_opt.box_n,
                                                s_soko_opt.target_n)) {
        return 0U;
    }
    if (s_soko_opt.box_n > SOKO_OPT_BRANCH_BOX_LIMIT) {
        return 0U;
    }

    if (!fixed_mapping && s_soko_opt.box_n > SOKO_OPT_EXACT_BOX_LIMIT) {
        s_soko_opt.node_limit = SOKO_OPT_STAGE1_NODE_LIMIT_5;
    } else {
        s_soko_opt.node_limit = 0UL;    /* 3 箱 Stage1 / 5 箱 Stage2 全量精确搜索 */
    }
    return 1U;
}

/** 动作段的正式执行成本：每格行驶、每个方向航点停站、含推箱航段 Snap。 */
static uint32 soko_seq_time_cost(const SokoActionSeq_t *seq)
{
    uint32 cost = 0UL;
    uint16 segment_start = 0U;

    if (!seq) return SB_MACRO_COST_INF;
    for (uint16 i = 0U; i < seq->count; ++i) {
        uint8 segment_end;
        cost += SOKO_COST_MOVE_MS;
        segment_end = (uint8)((i + 1U >= seq->count) ||
                              (seq->actions[i] != seq->actions[i + 1U]));
        if (!segment_end) continue;

        cost += SOKO_COST_WAYPOINT_MS;
        for (uint16 j = segment_start; j <= i; ++j) {
            if (soko_push_bit_test(seq, j)) {
                cost += SOKO_COST_SNAP_MS;
                break;
            }
        }
        segment_start = (uint16)(i + 1U);
    }
    return cost;
}

static uint32 soko_stage1_return_cost(Point_t player, Point_t home)
{
    int16 dx = (int16)home.x - (int16)player.x;
    int16 dy = (int16)home.y - (int16)player.y;
    uint32 cost = SOKO_COST_SNAP_MS;  /* 回库终点始终保留一次关键校正 */

    if (dx != 0 || dy != 0) {
        float distance_cells = sqrtf((float)(dx * dx + dy * dy));
        cost += (uint32)(distance_cells * (float)SOKO_COST_MOVE_MS + 0.5f);
        cost += SOKO_COST_WAYPOINT_MS; /* 直达库位仅产生一个停车航点 */
    }
    return cost;
}

static void soko_stage1_prepare_frame(SokoStage1Frame_t *frame)
{
    if (!frame || frame->initialized) return;
    frame->initialized = 1U;
    frame->candidate_count = 0U;
    frame->next_candidate = 0U;

    memcpy(sb_sub_map, s_soko_opt.map_copy, sizeof(sb_sub_map));
    for (uint8 bi = 0U; bi < s_soko_opt.box_n; ++bi) {
        if (frame->solved_mask & (uint8)(1U << bi)) {
            sb_sub_map[s_soko_opt.boxes[bi].y][s_soko_opt.boxes[bi].x] = MAP_EMPTY;
        }
    }
    if (!Algo_Nav_BFS_Flood(sb_sub_map, frame->player, 0, sb_nav_distance)) return;

    for (uint8 bi = 0U; bi < s_soko_opt.box_n; ++bi) {
        uint8 box_distance;
        if (frame->solved_mask & (uint8)(1U << bi)) continue;
        box_distance = nav_distance_to_box(sb_nav_distance, s_soko_opt.boxes[bi]);
        if (box_distance >= ALGO_NAV_DISTANCE_UNREACHABLE) continue;

        for (uint8 ti = 0U; ti < s_soko_opt.target_n; ++ti) {
            int16 target_distance;
            SokoOptCandidate_t *candidate;
            if (frame->target_mask & (uint8)(1U << ti)) continue;
            candidate = &frame->candidates[frame->candidate_count++];
            target_distance = (int16)(abs(s_soko_opt.targets[ti].x -
                                           s_soko_opt.boxes[bi].x) +
                                      abs(s_soko_opt.targets[ti].y -
                                           s_soko_opt.boxes[bi].y));
            candidate->b = bi;
            candidate->t = ti;
            /* 先按玩家真实绕障接近距离选箱，再按箱到目标距离选配对。 */
            candidate->h = (int16)((int16)box_distance * 32 + target_distance);
        }
    }
    soko_sort_candidates(frame->candidates, frame->candidate_count);
}

static void soko_stage1_save_best(uint32 cost)
{
    if (!s_soko_opt.out) return;
    s_soko_opt.out->total_boxes = s_soko_opt.box_n;
    s_soko_opt.out->is_solved = 1U;
    for (uint8 i = 0U; i < s_soko_opt.box_n; ++i) {
        s_soko_opt.out->sub_solutions[i] = s_soko_opt.cur_seq[i];
        s_soko_opt.out->player_end_pos[i] = s_soko_opt.cur_end[i];
    }
    s_soko_opt.best_cost = cost;
    s_soko_opt.best_valid = 1U;
}

uint8 Sokoban_Stage1_Search_Begin(const uint8 map[MAP_ROWS][MAP_COLS],
                                 Point_t player_pos,
                                 Point_t home_pos)
{
    SokoStage1Frame_t *root;
    if (!map || !map_is_inner_cell(player_pos.y, player_pos.x) ||
        !map_is_inner_cell(home_pos.y, home_pos.x)) {
        return 0U;
    }

    memset(&s_soko_opt, 0, sizeof(s_soko_opt));
    memcpy(s_soko_opt.map_copy, map, sizeof(s_soko_opt.map_copy));
    s_soko_opt.map = s_soko_opt.map_copy;
    s_soko_opt.box_n = extract_elements(map, MAP_BOX,
                                        s_soko_opt.boxes,
                                        SOKOBAN_MAX_BOXES);
    s_soko_opt.target_n = extract_elements(map, MAP_TARGET,
                                           s_soko_opt.targets,
                                           SOKOBAN_MAX_BOXES);
    if (s_soko_opt.box_n == 0U ||
        s_soko_opt.box_n != s_soko_opt.target_n ||
        s_soko_opt.box_n > SOKOBAN_MAX_BOXES) {
        s_soko_opt.stage1_status = SOKO_SEARCH_FAILED;
        return 0U;
    }

    s_soko_opt.fixed_mapping = 0U;
    s_soko_opt.stage1_home = home_pos;
    s_soko_opt.stage1_depth = 0U;
    s_soko_opt.best_cost = SB_MACRO_COST_INF;
    s_soko_opt.node_limit = (s_soko_opt.box_n <= SOKO_OPT_EXACT_BOX_LIMIT)
                          ? 0UL : SOKO_STAGE1_PAIR_LIMIT;
    s_soko_opt.stage1_status = SOKO_SEARCH_RUNNING;

    root = &s_soko_opt.stage1_frames[0];
    root->solved_mask = 0U;
    root->target_mask = 0U;
    root->player = player_pos;
    root->cost = 0UL;
    root->initialized = 0U;
    return 1U;
}

SokoSearchStatus_e Sokoban_Stage1_Search_Step(uint8 max_pair_evals,
                                              SokoFullSolution_t *result)
{
    uint8 evaluated = 0U;
    if (s_soko_opt.stage1_status != SOKO_SEARCH_RUNNING) {
        return s_soko_opt.stage1_status;
    }
    if (!result) {
        s_soko_opt.stage1_status = SOKO_SEARCH_FAILED;
        return s_soko_opt.stage1_status;
    }
    if (!s_soko_opt.out) {
        memset(result, 0, sizeof(*result));
        s_soko_opt.out = result;
    } else if (s_soko_opt.out != result) {
        s_soko_opt.stage1_status = SOKO_SEARCH_FAILED;
        return s_soko_opt.stage1_status;
    }
    if (max_pair_evals == 0U) max_pair_evals = 1U;

    while (evaluated < max_pair_evals) {
        SokoStage1Frame_t *frame =
            &s_soko_opt.stage1_frames[s_soko_opt.stage1_depth];

        if (s_soko_opt.stage1_depth >= s_soko_opt.box_n) {
            uint32 return_cost = soko_stage1_return_cost(frame->player,
                                                         s_soko_opt.stage1_home);
            if (return_cost != SB_MACRO_COST_INF &&
                frame->cost <= (SB_MACRO_COST_INF - return_cost) &&
                (!s_soko_opt.best_valid ||
                 frame->cost + return_cost < s_soko_opt.best_cost)) {
                soko_stage1_save_best(frame->cost + return_cost);
            }
            --s_soko_opt.stage1_depth;
            continue;
        }

        soko_stage1_prepare_frame(frame);
        if (frame->next_candidate >= frame->candidate_count ||
            (s_soko_opt.best_valid && frame->cost >= s_soko_opt.best_cost)) {
            if (s_soko_opt.stage1_depth == 0U) {
                s_soko_opt.stage1_status = s_soko_opt.best_valid
                                          ? SOKO_SEARCH_SOLVED
                                          : SOKO_SEARCH_FAILED;
                return s_soko_opt.stage1_status;
            }
            --s_soko_opt.stage1_depth;
            continue;
        }

        if (s_soko_opt.node_limit != 0UL &&
            s_soko_opt.node_count >= s_soko_opt.node_limit) {
            /* 多箱简单图通常很早即可得到完整分支；若 256 次预算内恰好尚未走到叶子，
             * 只在这个罕见终点调用已有贪心解，避免把可解地图误报成死局。 */
            if (!s_soko_opt.best_valid &&
                sokoban_solve_stage1_greedy(s_soko_opt.map_copy,
                                            s_soko_opt.stage1_frames[0].player,
                                            result)) {
                s_soko_opt.best_valid = 1U;
            }
            s_soko_opt.stage1_status = s_soko_opt.best_valid
                                      ? SOKO_SEARCH_SOLVED
                                      : SOKO_SEARCH_FAILED;
            return s_soko_opt.stage1_status;
        }

        {
            SokoOptCandidate_t candidate =
                frame->candidates[frame->next_candidate++];
            uint8 depth = s_soko_opt.stage1_depth;
            uint32 seq_cost;
            uint32 next_cost;
            SokoStage1Frame_t *child;

            ++s_soko_opt.node_count;
            ++evaluated;
            if (!soko_opt_solve_pair(frame->solved_mask,
                                     frame->target_mask,
                                     candidate.b,
                                     candidate.t,
                                     frame->player,
                                     &s_soko_opt.cur_seq[depth],
                                     &s_soko_opt.cur_end[depth])) {
                continue;
            }
            seq_cost = soko_seq_time_cost(&s_soko_opt.cur_seq[depth]);
            if (seq_cost == SB_MACRO_COST_INF ||
                frame->cost > (SB_MACRO_COST_INF - seq_cost)) {
                continue;
            }
            next_cost = frame->cost + seq_cost;
            if (s_soko_opt.best_valid && next_cost >= s_soko_opt.best_cost) continue;

            child = &s_soko_opt.stage1_frames[depth + 1U];
            child->initialized = 0U;
            child->solved_mask = (uint8)(frame->solved_mask |
                                         (uint8)(1U << candidate.b));
            child->target_mask = (uint8)(frame->target_mask |
                                         (uint8)(1U << candidate.t));
            child->player = s_soko_opt.cur_end[depth];
            child->cost = next_cost;
            s_soko_opt.stage1_depth = (uint8)(depth + 1U);
        }
    }
    return SOKO_SEARCH_RUNNING;
}

void Sokoban_Stage1_Search_Cancel(void)
{
    s_soko_opt.stage1_status = SOKO_SEARCH_IDLE;
    s_soko_opt.out = 0;
}

uint8 Sokoban_Solve_Stage1(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player_pos,
                           SokoFullSolution_t *result)
{
    SokoSearchStatus_e status;
    if (!result || !Sokoban_Stage1_Search_Begin(map, player_pos, player_pos)) return 0U;
    do {
        status = Sokoban_Stage1_Search_Step(255U, result);
    } while (status == SOKO_SEARCH_RUNNING);

    if (status == SOKO_SEARCH_SOLVED) return 1U;
    /* 非正式同步调用保留原贪心兜底；正式游戏使用分时接口，不阻塞 5ms tick。 */
    return sokoban_solve_stage1_greedy(map, player_pos, result);
}

uint8 Sokoban_Solve_Stage2(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player_pos,
                           const uint8 box_to_target_idx[],
                           uint8 box_count_in,
                           SokoFullSolution_t *result)
{
    uint8 fallback_ok;
    Point_t check_boxes[SOKOBAN_MAX_BOXES];
    Point_t check_targets[SOKOBAN_MAX_BOXES];
    uint8 check_box_n;
    uint8 check_target_n;

    if (result == 0 || box_to_target_idx == 0) return 0U;
    result->is_solved = 0U;
    result->total_boxes = 0U;

    check_box_n = extract_elements(map, MAP_BOX, check_boxes, SOKOBAN_MAX_BOXES);
    check_target_n = extract_elements(map, MAP_TARGET, check_targets, SOKOBAN_MAX_BOXES);
    if (check_box_n == 0U ||
        check_box_n != box_count_in ||
        check_box_n != check_target_n ||
        !soko_mapping_is_valid(box_to_target_idx, check_box_n, check_target_n)) {
        return 0U;
    }

    fallback_ok = sokoban_solve_stage2_greedy(map, player_pos,
                                             box_to_target_idx,
                                             box_count_in,
                                             result);

    if (!soko_opt_prepare(map, box_to_target_idx, 1U, result, fallback_ok)) {
        return fallback_ok;
    }
    if (s_soko_opt.box_n != box_count_in) {
        return fallback_ok;
    }

    soko_opt_dfs(0U, 0U, 0U, player_pos, 0U);
    return s_soko_opt.best_valid ? 1U : fallback_ok;
}

/* ==========================================================================
 *  § 6. 炸弹策略 — 寻炸墙 / 应用爆炸 / 推炸弹策略求解
 * ========================================================================== */

uint8 Sokoban_Find_Bomb_Wall(const uint8 map[MAP_ROWS][MAP_COLS],
                             Point_t player_pos,
                             Point_t blocked_target,
                             Point_t *bomb_wall_pos)
{
    static uint8   tmp_map[MAP_ROWS][MAP_COLS];
    static NavPath_t tmp_path;

    int32 best_score = -2147483647;
    uint8 found    = 0;

    if (!bomb_wall_pos) return 0;

    /* 遍历所有内部墙体（最外圈不可炸） */
    for (int8 r = 1; r < MAP_ROWS - 1; r++) {
        for (int8 c = 1; c < MAP_COLS - 1; c++) {
            if (map[r][c] != MAP_WALL) continue;

            uint8 cleared_walls = 0;
            uint8 reachable_targets = 0;
            uint16 blocked_len = 0;
            int32 score;

            /* 假设在 (r, c) 引爆炸弹，3×3 范围清除内墙 */
            memcpy(tmp_map, map, sizeof(tmp_map));
            for (int8 dr = -1; dr <= 1; dr++) {
                for (int8 dc = -1; dc <= 1; dc++) {
                    int8 rr = r + dr, cc = c + dc;
                    if (rr >= 1 && rr < MAP_ROWS - 1 &&
                        cc >= 1 && cc < MAP_COLS - 1) {
                        if (tmp_map[rr][cc] == MAP_WALL) {
                            tmp_map[rr][cc] = MAP_EMPTY;
                            cleared_walls++;
                        }
                    }
                }
            }

            /* 统计爆炸后可达目标数量 */
            for (int8 tr = (int8)CHASSIS_GRID_INNER_MIN_Y; tr <= (int8)CHASSIS_GRID_INNER_MAX_Y; tr++) {
                for (int8 tc = (int8)CHASSIS_GRID_INNER_MIN_X; tc <= (int8)CHASSIS_GRID_INNER_MAX_X; tc++) {
                    if (tmp_map[tr][tc] != MAP_TARGET) continue;
                    {
                        Point_t tp = {tc, tr};
                        if (Algo_Nav_BFS(tmp_map, player_pos, tp, &tmp_path)) {
                            reachable_targets++;
                        }
                    }
                }
            }

            /* 优先考虑 blocked_target 破局能力（若给定） */
            if (blocked_target.x >= 0 && blocked_target.y >= 0) {
                if (!Algo_Nav_BFS(tmp_map, player_pos, blocked_target, &tmp_path)) {
                    continue; /* 该墙无法破当前卡点，直接跳过 */
                }
                blocked_len = tmp_path.step_count;
            }

            /* B13: 评分 — 优先解锁不可达目标 (×200), 其次清墙数 (×20),
             *       同时强惩罚长路径 (二次项防止权重 1000 压制路径项) */
            score = (int32)reachable_targets * 200
                  + (int32)cleared_walls * 20
                  - ((int32)blocked_len * (int32)blocked_len) / 50;

            if (!found || score > best_score) {
                best_score = score;
                bomb_wall_pos->x = c;
                bomb_wall_pos->y = r;
                found = 1;
            }
        }
    }
    return found;
}

/*===========================================================================
 *  公开 API: 应用炸弹爆炸到地图
 *===========================================================================*/
void Sokoban_Apply_Bomb_Explosion(uint8 map[MAP_ROWS][MAP_COLS],
                                  Point_t wall_pos)
{
    for (int8 dr = -1; dr <= 1; dr++) {
        for (int8 dc = -1; dc <= 1; dc++) {
            int8 r = wall_pos.y + dr;
            int8 c = wall_pos.x + dc;
            if (r >= 1 && r < MAP_ROWS - 1 &&
                c >= 1 && c < MAP_COLS - 1) {
                if (map[r][c] == MAP_WALL)
                    map[r][c] = MAP_EMPTY;
            }
        }
    }
}

/*===========================================================================
 *  公开 API: 动作序列 → 路点路径
 *
 *  只在方向改变或序列末尾生成路点（即转弯点压缩），
 *  同方向连续移动只保留最终位置。
 *===========================================================================*/
/* ==========================================================================
 *  § 7. 动作序列 → 航点路径 (供底盘 chassis_ctrl_move_to_grid 逐点走)
 * ========================================================================== */

uint16 Sokoban_Actions_To_Waypoints(const SokoAction_e *actions,
                                    uint16 count,
                                    Point_t start_pos,
                                    SokoWaypointPath_t *wp_path)
{
    if (!wp_path) return 0U;
    wp_path->count = 0;
    if (!actions || count == 0U) return 0U;

    int8 cx = start_pos.x;
    int8 cy = start_pos.y;

    for (uint16 i = 0; i < count; i++) {
        int8 d = (int8)actions[i];
        cx += s_dc[d];
        cy += s_dr[d];

        /* 方向改变 或 最后一步 → 记录路点 */
        if (i == count - 1 || actions[i] != actions[i + 1]) {
            if (wp_path->count < SOKOBAN_MAX_WAYPOINTS) {
                wp_path->points[wp_path->count].x = cx;
                wp_path->points[wp_path->count].y = cy;
                wp_path->kinds[wp_path->count] = (uint8)SOKO_WP_WALK;
                wp_path->count++;
            }
        }
    }
    return wp_path->count;
}

uint16 Sokoban_Seq_To_Waypoints(const SokoActionSeq_t *seq,
                                Point_t start_pos,
                                SokoWaypointPath_t *wp_path)
{
    int8 cx;
    int8 cy;
    uint8 segment_has_push = 0U;

    if (!wp_path) return 0U;
    wp_path->count = 0U;
    if (!seq || seq->count == 0U) return 0U;

    cx = start_pos.x;
    cy = start_pos.y;
    for (uint16 i = 0U; i < seq->count; ++i) {
        uint8 dir = (uint8)seq->actions[i];
        if (dir >= 4U) return 0U;
        cx = (int8)(cx + s_dc[dir]);
        cy = (int8)(cy + s_dr[dir]);
        if (soko_push_bit_test(seq, i)) segment_has_push = 1U;

        if (i + 1U >= seq->count || seq->actions[i] != seq->actions[i + 1U]) {
            if (wp_path->count >= SOKOBAN_MAX_WAYPOINTS) return 0U;
            wp_path->points[wp_path->count].x = cx;
            wp_path->points[wp_path->count].y = cy;
            wp_path->kinds[wp_path->count] = segment_has_push
                                           ? (uint8)SOKO_WP_CRITICAL
                                           : (uint8)SOKO_WP_WALK;
            ++wp_path->count;
            segment_has_push = 0U;
        }
    }
    return wp_path->count;
}

uint8 Sokoban_Build_Return_Waypoints(const uint8 map[MAP_ROWS][MAP_COLS],
                                     Point_t player_pos,
                                     Point_t home_pos,
                                     SokoWaypointPath_t *wp_path)
{
    (void)map;
    (void)player_pos;

    if (!wp_path || !map_is_inner_cell(home_pos.y, home_pos.x)) return 0U;

    /* 通关后虚拟墙/炸弹不再约束车体：直接向库位下发一个关键航点。 */
    wp_path->points[0] = home_pos;
    wp_path->kinds[0] = (uint8)SOKO_WP_CRITICAL;
    wp_path->count = 1U;
    return 1U;
}

/*===========================================================================
 *  公开 API: 推炸弹到指定墙体
 *
 *  复用单箱 BFS 求解器: 炸弹视为可推物体，目标为墙体位置。
 *  构建临时子地图：炸弹位置清空（由 BFS 状态跟踪），其余保持原样。
 *===========================================================================*/
uint8 Sokoban_Solve_Push_Bomb(const uint8 map[MAP_ROWS][MAP_COLS],
                              Point_t player_pos,
                              Point_t bomb_pos,
                              Point_t wall_pos,
                              SokoActionSeq_t *sol)
{
    /* 构建临时地图: 炸弹位置清空, 目标墙体位置改为 MAP_TARGET */
    memcpy(sb_sub_map, map, MAP_ROWS * MAP_COLS);
    sb_sub_map[bomb_pos.y][bomb_pos.x] = MAP_EMPTY;
    sb_sub_map[wall_pos.y][wall_pos.x] = MAP_TARGET;

    return sokoban_bfs_single(sb_sub_map, player_pos, bomb_pos, wall_pos, sol, 0U);
}

/*===========================================================================
 *  公开 API: 多炸弹联合 (炸弹, 墙体) 规划
 *
 *  与旧 Sokoban_Find_Bomb_Wall 的关键区别:
 *    - 墙体打分阶段就把"该墙能否被某颗炸弹推到"纳入硬约束 (可行性);
 *    - 在多颗炸弹中按到墙曼哈顿距离从近到远挑选, 选到第一颗可推的即可;
 *    - P0-2: 打分阶段将全部炸弹从 tmp_map 中清除 (被选中的炸弹推走后原格即空,
 *      其余炸弹虽暂留但本阶段只做一次可达性快照, 清空所有炸弹比保留全部
 *      更接近爆炸后真实状态)。注意这仅影响评分, 可行性检查仍使用原始地图。
 *
 *  为控制单片机上的一次性规划耗时, 仅当某面墙的得分"有望刷新当前最优"时,
 *  才执行 (较昂贵的) 单箱推炸弹可行性 BFS, 失败则不更新最优, 继续下一面墙。
 *===========================================================================*/
uint8 Sokoban_Plan_Bomb(const uint8 map[MAP_ROWS][MAP_COLS],
                        Point_t player_pos,
                        Point_t blocked_target,
                        Point_t *out_bomb_pos,
                        Point_t *out_wall_pos,
                        SokoActionSeq_t *out_seq)
{
    static uint8     tmp_map[MAP_ROWS][MAP_COLS];
    static uint8     reach[MAP_ROWS][MAP_COLS];
    static NavPath_t tmp_path;
    static SokoActionSeq_t try_seq;

    Point_t bombs[SOKOBAN_MAX_BOXES];
    uint8   bomb_n;
    int32   best_score = -2147483647;
    uint8   found = 0;

    if (!out_bomb_pos || !out_wall_pos || !out_seq) return 0;

    bomb_n = extract_elements(map, MAP_BOMB, bombs, SOKOBAN_MAX_BOXES);
    if (bomb_n == 0) return 0;

    /* 遍历所有内部墙体（最外圈不可炸） */
    for (int8 r = 1; r < MAP_ROWS - 1; r++) {
        for (int8 c = 1; c < MAP_COLS - 1; c++) {
            uint8  cleared_walls = 0;
            uint8  reachable_targets = 0;
            uint16 blocked_len = 0;
            int32  score;
            Point_t wall;

            if (map[r][c] != MAP_WALL) continue;

            wall.x = c;
            wall.y = r;

            /* 假设在 (r, c) 引爆炸弹，3×3 范围清除内墙 */
            memcpy(tmp_map, map, sizeof(tmp_map));
            for (int8 dr = -1; dr <= 1; dr++) {
                for (int8 dc = -1; dc <= 1; dc++) {
                    int8 rr = r + dr, cc = c + dc;
                    if (rr >= 1 && rr < MAP_ROWS - 1 &&
                        cc >= 1 && cc < MAP_COLS - 1) {
                        if (tmp_map[rr][cc] == MAP_WALL) {
                            tmp_map[rr][cc] = MAP_EMPTY;
                            cleared_walls++;
                        }
                    }
                }
            }

            /* P0-2: 打分阶段将所有炸弹从 tmp_map 中清除。
             * 被选中的炸弹推走后原格即空; 其余炸弹虽暂留但本阶段只做一次
             * 可达性快照, 清空所有炸弹比保留全部更接近爆炸后真实状态。
             * 注意: 这仅影响评分, 可行性检查 (Sokoban_Solve_Push_Bomb)
             * 仍使用原始地图 (仅清空被选炸弹 + 目标墙)。 */
            for (uint8 bi = 0; bi < bomb_n; bi++) {
                if (tmp_map[bombs[bi].y][bombs[bi].x] == MAP_BOMB) {
                    tmp_map[bombs[bi].y][bombs[bi].x] = MAP_EMPTY;
                }
            }

            /* 一次扩散后查询全部目标 */
            if (!Algo_Nav_BFS_Flood(tmp_map, player_pos, reach, 0)) continue;
            for (int8 tr = (int8)CHASSIS_GRID_INNER_MIN_Y; tr <= (int8)CHASSIS_GRID_INNER_MAX_Y; tr++) {
                for (int8 tc = (int8)CHASSIS_GRID_INNER_MIN_X; tc <= (int8)CHASSIS_GRID_INNER_MAX_X; tc++) {
                    if (tmp_map[tr][tc] != MAP_TARGET) continue;
                    {
                        Point_t tp = {tc, tr};
                        if (Algo_Nav_Is_Reachable(reach, tp)) {
                            reachable_targets++;
                        }
                    }
                }
            }

            /* 破局门控: 给定 blocked_target 时, 该墙必须能恢复其可达性 */
            if (blocked_target.x >= 0 && blocked_target.y >= 0) {
                if (!Algo_Nav_Is_Reachable(reach, blocked_target)) {
                    continue;
                }
                /* 评分仍需真实最短路长度，仅为该单一目标保留一次点到点 BFS。 */
                if (!Algo_Nav_BFS(tmp_map, player_pos, blocked_target, &tmp_path)) continue;
                blocked_len = tmp_path.step_count;
            }

            score = (int32)reachable_targets * 200
                  + (int32)cleared_walls * 20
                  - ((int32)blocked_len * (int32)blocked_len) / 50;

            /* 仅当本墙有望刷新最优时才付出可行性 BFS 代价 */
            if (found && score <= best_score) continue;

            /* 可行性: 在多颗炸弹中按到墙曼哈顿距离由近到远, 取第一颗可推到 W 的 */
            {
                uint8 tried[SOKOBAN_MAX_BOXES] = {0};
                int8  chosen = -1;
                uint8 k;

                for (k = 0; k < bomb_n; k++) {
                    int8  bi = -1;
                    int16 bd = 32767;
                    uint8 i;
                    for (i = 0; i < bomb_n; i++) {
                        int16 d;
                        if (tried[i]) continue;
                        d = (int16)(abs(bombs[i].x - c) + abs(bombs[i].y - r));
                        if (d < bd) { bd = d; bi = (int8)i; }
                    }
                    if (bi < 0) break;
                    tried[bi] = 1;

                    if (Sokoban_Solve_Push_Bomb(map, player_pos,
                                                bombs[bi], wall, &try_seq)) {
                        chosen = bi;
                        break;
                    }
                }

                if (chosen < 0) continue;   /* 没有任何炸弹能被推到这面墙 */

                best_score    = score;
                *out_wall_pos = wall;
                *out_bomb_pos = bombs[chosen];
                *out_seq      = try_seq;
                found = 1;
            }
        }
    }

    return found;
}

/*===========================================================================
 *  顶层迭代求解 (推箱 + 多炸弹)
 *===========================================================================*/

#ifdef SOKOBAN_PC_VALIDATION

/** 在地图上找离 ref 曼哈顿最近的目标 */
static uint8 sf_nearest_target(const uint8 map[MAP_ROWS][MAP_COLS],
                               Point_t ref, Point_t *out)
{
    int16 best = 32767;
    uint8 found = 0;
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (map[r][c] != MAP_TARGET) continue;
            {
                int16 d = (int16)(abs((int)c - (int)ref.x) + abs((int)r - (int)ref.y));
                if (d < best) { best = d; out->x = c; out->y = r; found = 1; }
            }
        }
    }
    return found;
}

/** 找首个玩家走不到的目标 (炸弹破局门控用) */
static uint8 sf_first_unreachable(const uint8 map[MAP_ROWS][MAP_COLS],
                                  Point_t player, Point_t *out)
{
    static NavPath_t np;
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (map[r][c] != MAP_TARGET) continue;
            {
                Point_t t = {c, r};
                if (!Algo_Nav_BFS(map, player, t, &np)) { *out = t; return 1; }
            }
        }
    }
    return 0;
}

uint8 Sokoban_Solve_Full(const uint8 map[MAP_ROWS][MAP_COLS],
                         Point_t player_pos,
                         const uint8 *box_to_target_idx,
                         uint8 box_count,
                         SokoPlan_t *out_plan)
{
    static uint8              sf_work[MAP_ROWS][MAP_COLS];
    static SokoFullSolution_t sf_push;
    static SokoActionSeq_t    sf_bseq;

    Point_t player = player_pos;
    uint8   round;

    if (!out_plan) return 0;
    out_plan->count     = 0;
    out_plan->is_solved = 0;

    memcpy(sf_work, map, sizeof(sf_work));

    /* 轮次上限: 每轮最多消化 1 颗炸弹, 加首轮直接推箱, 留 2 余量 */
    for (round = 0; round < (uint8)(SOKOBAN_MAX_BOXES + 2); round++) {
        Point_t boxes_tmp[SOKOBAN_MAX_BOXES];
        uint8   box_n;
        uint8   push_ok;

        box_n = extract_elements(sf_work, MAP_BOX, boxes_tmp, SOKOBAN_MAX_BOXES);
        if (box_n == 0U) {
            out_plan->is_solved = 1;
            return 1;
        }

        /* 1) 整体推箱尝试 */
        if (box_to_target_idx != NULL) {
            push_ok = Sokoban_Solve_Stage2(sf_work, player, box_to_target_idx,
                                           box_count, &sf_push);
        } else {
            push_ok = Sokoban_Solve_Stage1(sf_work, player, &sf_push);
        }

        if (push_ok && sf_push.is_solved) {
            Point_t cur = player;
            uint8 i;
            for (i = 0U; i < sf_push.total_boxes; i++) {
                SokoPhase_t *ph;
                if (out_plan->count >= (uint8)SOKOBAN_MAX_PHASES) return 0;
                ph = &out_plan->phases[out_plan->count++];
                ph->kind         = SOKO_PHASE_PUSH;
                ph->seq          = sf_push.sub_solutions[i];
                ph->player_start = cur;
                ph->movable.x    = -1; ph->movable.y = -1;
                ph->wall.x       = -1; ph->wall.y    = -1;
                cur = sf_push.player_end_pos[i];
            }
            out_plan->is_solved = 1;
            return 1;
        }

        /* 2) 推箱失败 → 选一颗炸弹炸墙 (镜像 stage_plan_handler 决策链) */
        {
            Point_t bomb, wall, dead, bt;
            uint8   planned = 0;

            if (out_plan->count >= (uint8)SOKOBAN_MAX_PHASES) return 0;

            if (Sokoban_Is_Deadlock(sf_work, &dead)) {
                if (sf_nearest_target(sf_work, dead, &bt)) {
                    if (Sokoban_Plan_Bomb(sf_work, player, bt, &bomb, &wall, &sf_bseq)) {
                        planned = 1;
                    }
                }
            }
            if (!planned) {
                if (sf_first_unreachable(sf_work, player, &bt)) {
                    if (Sokoban_Plan_Bomb(sf_work, player, bt, &bomb, &wall, &sf_bseq)) {
                        planned = 1;
                    }
                }
            }
            if (!planned) {
                Point_t bombs_tmp[SOKOBAN_MAX_BOXES];
                if (extract_elements(sf_work, MAP_BOMB, bombs_tmp, SOKOBAN_MAX_BOXES) > 0U) {
                    Point_t none = {-1, -1};
                    if (Sokoban_Plan_Bomb(sf_work, player, none, &bomb, &wall, &sf_bseq)) {
                        planned = 1;
                    }
                }
            }
            if (!planned) return 0;   /* 无可行炸弹 → 死局复位 */

            /* 追加炸弹段 */
            {
                SokoPhase_t *ph = &out_plan->phases[out_plan->count++];
                ph->kind         = SOKO_PHASE_BOMB;
                ph->seq          = sf_bseq;
                ph->player_start = player;
                ph->movable      = bomb;
                ph->wall         = wall;
            }

            /* 模拟"推炸弹到墙 + 3×3 爆破", 更新 work 与 player */
            player = simulate_actions(&sf_bseq, player, bomb);
            sf_work[bomb.y][bomb.x] = MAP_EMPTY;
            Sokoban_Apply_Bomb_Explosion(sf_work, wall);
            sf_work[wall.y][wall.x] = MAP_EMPTY;
        }
    }

    return 0;   /* 轮次预算耗尽 */
}

#endif /* SOKOBAN_PC_VALIDATION */
