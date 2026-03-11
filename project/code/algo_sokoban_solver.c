#include "algo_sokoban_solver.h"
#include <string.h>
#include <stdlib.h>     /* abs() */

/*===========================================================================
 *  内部常量
 *===========================================================================*/

/** 方向偏移：UP, DOWN, LEFT, RIGHT */
static const int8 s_dr[4] = { -1,  1,  0,  0 };
static const int8 s_dc[4] = {  0,  0, -1,  1 };

/** 单箱 BFS 状态空间大小 = MAP_ROWS × MAP_COLS × MAP_ROWS × MAP_COLS */
#define SB_RC           (MAP_ROWS * MAP_COLS)               /* 192   */
#define SB_STATE_COUNT  ((uint32)SB_RC * (uint32)SB_RC)     /* 36864 */

/*===========================================================================
 *  静态大数组 — 放在全局/静态区，避免栈溢出
 *
 *  sb_came_from[i]:
 *      0       = 未访问
 *      1..4    = 到达该状态的动作编号 + 1 (UP+1, DOWN+1, LEFT+1, RIGHT+1)
 *      5       = 起始状态标记
 *
 *  sb_queue[i]: BFS 队列, 存放状态扁平索引 (uint16, 最大 36863)
 *
 *  总内存: 36864 + 36864×2 = 110,592 字节 ≈ 108 KB
 *===========================================================================*/
static uint8  sb_came_from[SB_STATE_COUNT];
static uint16 sb_queue[SB_STATE_COUNT];

/** 子地图临时缓冲 */
static uint8  sb_sub_map[MAP_ROWS][MAP_COLS];

/*===========================================================================
 *  内联辅助函数
 *===========================================================================*/

/** 状态 → 扁平索引 */
static inline uint16 sb_encode(int8 pr, int8 pc, int8 br, int8 bc)
{
    return (uint16)(((uint16)(pr * MAP_COLS + pc)) * SB_RC
                  + (uint16)(br * MAP_COLS + bc));
}

/** 扁平索引 → 状态分量 */
static inline void sb_decode(uint16 idx, int8 *pr, int8 *pc, int8 *br, int8 *bc)
{
    uint16 player_part = idx / SB_RC;
    uint16 box_part    = idx % SB_RC;
    *pr = (int8)(player_part / MAP_COLS);
    *pc = (int8)(player_part % MAP_COLS);
    *br = (int8)(box_part    / MAP_COLS);
    *bc = (int8)(box_part    % MAP_COLS);
}

/** 检查 (r, c) 是否在地图内且可通行（空地 / 目标点） */
static inline uint8 sb_is_free(const uint8 map[MAP_ROWS][MAP_COLS], int8 r, int8 c)
{
    if (r < 0 || r >= MAP_ROWS || c < 0 || c >= MAP_COLS) return 0;
    return (map[r][c] == MAP_EMPTY || map[r][c] == MAP_TARGET) ? 1 : 0;
}

/*===========================================================================
 *  核心：单箱 BFS 求解
 *
 *  状态 = (玩家行, 玩家列, 箱子行, 箱子列)
 *  转移 = 玩家尝试 4 方向移动:
 *        - 若移入箱子位置 → 推箱（箱子目标格必须可通行）
 *        - 否则 → 普通行走（目标格必须可通行且非箱子位置）
 *  目标 = 箱子坐标到达 target
 *===========================================================================*/
static uint8 sokoban_bfs_single(const uint8 sub_map[MAP_ROWS][MAP_COLS],
                                Point_t player, Point_t box, Point_t target,
                                SokoActionSeq_t *sol)
{
    uint32 head = 0, tail = 0;
    uint16 start_idx, goal_idx = 0;

    sol->count = 0;
    memset(sb_came_from, 0, SB_STATE_COUNT);

    /* 起点已是终点 */
    if (box.y == target.y && box.x == target.x) return 1;

    start_idx = sb_encode(player.y, player.x, box.y, box.x);
    sb_came_from[start_idx] = 5;            /* 标记为起始 */
    sb_queue[tail++] = start_idx;

    while (head < tail) {
        uint16 cur_idx = sb_queue[head++];
        int8 pr, pc, br, bc;
        sb_decode(cur_idx, &pr, &pc, &br, &bc);

        for (int d = 0; d < 4; d++) {
            int8 npr = pr + s_dr[d];
            int8 npc = pc + s_dc[d];
            int8 nbr, nbc;

            if (npr == br && npc == bc) {
                /* ---------- 推箱 ---------- */
                nbr = br + s_dr[d];
                nbc = bc + s_dc[d];
                if (!sb_is_free(sub_map, nbr, nbc)) continue;
            } else {
                /* ---------- 普通行走 ---------- */
                if (!sb_is_free(sub_map, npr, npc)) continue;
                nbr = br;
                nbc = bc;
            }

            uint16 nidx = sb_encode(npr, npc, nbr, nbc);
            if (sb_came_from[nidx] != 0) continue;   /* 已访问 */

            sb_came_from[nidx] = (uint8)(d + 1);     /* 记录到达动作 */
            sb_queue[tail++] = nidx;

            /* 箱子到达目标 → 成功 */
            if (nbr == target.y && nbc == target.x) {
                goal_idx = nidx;
                goto found;
            }

            if (tail >= SB_STATE_COUNT) return 0;     /* 队列溢出保护 */
        }
    }
    return 0;   /* 无解 */

found:
    /* -------- 从终态反向回溯提取动作序列 -------- */
    {
        SokoAction_e rev_buf[SOKOBAN_MAX_ACTIONS];
        uint16 steps = 0;
        uint16 idx   = goal_idx;

        while (sb_came_from[idx] != 5) {        /* 5 = 起始标记 */
            uint8 act = sb_came_from[idx] - 1;  /* 0..3 */
            if (steps >= SOKOBAN_MAX_ACTIONS) return 0;
            rev_buf[steps++] = (SokoAction_e)act;

            /* 还原前驱状态 */
            int8 pr2, pc2, br2, bc2;
            sb_decode(idx, &pr2, &pc2, &br2, &bc2);

            int8 prev_pr = pr2 - s_dr[act];
            int8 prev_pc = pc2 - s_dc[act];

            int8 prev_br, prev_bc;
            /* 是否发生过推箱: 玩家当前位置 = 箱子当前位置 - 方向偏移 */
            if (pr2 == br2 - s_dr[act] && pc2 == bc2 - s_dc[act]) {
                /* 推箱: 箱子之前在玩家现在的位置 */
                prev_br = pr2;
                prev_bc = pc2;
            } else {
                /* 普通行走: 箱子未动 */
                prev_br = br2;
                prev_bc = bc2;
            }

            idx = sb_encode(prev_pr, prev_pc, prev_br, prev_bc);
        }

        /* 倒序 → 正序 */
        sol->count = steps;
        for (uint16 i = 0; i < steps; i++) {
            sol->actions[i] = rev_buf[steps - 1 - i];
        }
    }
    return 1;
}

/*===========================================================================
 *  辅助：在地图上提取指定类型元素坐标
 *===========================================================================*/
static uint8 extract_elements(const uint8 map[MAP_ROWS][MAP_COLS],
                              MapElement_e type,
                              Point_t out[], uint8 max_count)
{
    uint8 n = 0;
    for (int8 r = 0; r < MAP_ROWS; r++) {
        for (int8 c = 0; c < MAP_COLS; c++) {
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

    /* 处理目标: 仅保留当前任务的目标，其余变空地 */
    for (uint8 i = 0; i < target_count; i++) {
        if (i == cur_target_idx) continue;   /* 当前目标保留 */
        if (out_map[targets[i].y][targets[i].x] == MAP_TARGET) {
            out_map[targets[i].y][targets[i].x] = MAP_EMPTY;
        }
    }

    /* 炸弹保持原样 — sb_is_free 自动将其视为障碍 */
}

/*===========================================================================
 *  公开 API: 第一阶段求解
 *===========================================================================*/
uint8 Sokoban_Solve_Stage1(const uint8 map[MAP_ROWS][MAP_COLS],
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

    /* 逐个解算 — 贪心: 先推离当前玩家最近的箱子 */
    for (uint8 done = 0; done < box_n; done++) {

        /* 1. 选最近未完成箱子 */
        int8  best_b = -1;
        int16 best_d = 32767;
        for (uint8 i = 0; i < box_n; i++) {
            if (solved[i]) continue;
            int16 d = (int16)(abs(boxes[i].x - cur_player.x)
                            + abs(boxes[i].y - cur_player.y));
            if (d < best_d) { best_d = d; best_b = (int8)i; }
        }
        if (best_b < 0) return 0;

        /* 2. 为该箱子分配最近未使用目标 */
        int8  best_t = -1;
        best_d = 32767;
        for (uint8 i = 0; i < target_n; i++) {
            if (t_used[i]) continue;
            int16 d = (int16)(abs(targets[i].x - boxes[best_b].x)
                            + abs(targets[i].y - boxes[best_b].y));
            if (d < best_d) { best_d = d; best_t = (int8)i; }
        }
        if (best_t < 0) return 0;

        /* 3. 构建子地图 */
        build_sub_map(map, boxes, targets, box_n, target_n,
                      solved, (uint8)best_b, (uint8)best_t, sb_sub_map);

        /* 4. BFS 解算 */
        SokoActionSeq_t *sol = &result->sub_solutions[done];
        if (!sokoban_bfs_single(sb_sub_map, cur_player,
                                boxes[best_b], targets[best_t], sol)) {
            return 0;
        }

        /* 5. 模拟得到结束位置 */
        cur_player = simulate_actions(sol, cur_player, boxes[best_b]);
        result->player_end_pos[done] = cur_player;

        solved[best_b] = 1;
        t_used[best_t] = 1;
    }

    result->is_solved = 1;
    return 1;
}

/*===========================================================================
 *  公开 API: 第二阶段求解
 *===========================================================================*/
uint8 Sokoban_Solve_Stage2(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player_pos,
                           const uint8 box_to_target_idx[],
                           uint8 box_count_in,
                           SokoFullSolution_t *result)
{
    Point_t boxes[SOKOBAN_MAX_BOXES];
    Point_t targets[SOKOBAN_MAX_BOXES];
    uint8   solved[SOKOBAN_MAX_BOXES] = {0};

    result->is_solved   = 0;
    result->total_boxes = 0;

    uint8 box_n    = extract_elements(map, MAP_BOX,    boxes,   SOKOBAN_MAX_BOXES);
    uint8 target_n = extract_elements(map, MAP_TARGET, targets, SOKOBAN_MAX_BOXES);

    if (box_n != box_count_in || box_n == 0) return 0;

    result->total_boxes = box_n;
    Point_t cur_player = player_pos;

    /* 按离当前玩家最近的箱子优先顺序依次求解 */
    for (uint8 done = 0; done < box_n; done++) {
        int8  best_b = -1;
        int16 best_d = 32767;
        for (uint8 i = 0; i < box_n; i++) {
            if (solved[i]) continue;
            int16 d = (int16)(abs(boxes[i].x - cur_player.x)
                            + abs(boxes[i].y - cur_player.y));
            if (d < best_d) { best_d = d; best_b = (int8)i; }
        }
        if (best_b < 0) return 0;

        uint8 ti = box_to_target_idx[best_b];
        if (ti >= target_n) return 0;

        build_sub_map(map, boxes, targets, box_n, target_n,
                      solved, (uint8)best_b, ti, sb_sub_map);

        SokoActionSeq_t *sol = &result->sub_solutions[done];
        if (!sokoban_bfs_single(sb_sub_map, cur_player,
                                boxes[best_b], targets[ti], sol)) {
            return 0;
        }

        cur_player = simulate_actions(sol, cur_player, boxes[best_b]);
        result->player_end_pos[done] = cur_player;
        solved[best_b] = 1;
    }

    result->is_solved = 1;
    return 1;
}

/*===========================================================================
 *  公开 API: 炸弹墙体搜索
 *===========================================================================*/
uint8 Sokoban_Find_Bomb_Wall(const uint8 map[MAP_ROWS][MAP_COLS],
                             Point_t player_pos,
                             Point_t blocked_target,
                             Point_t *bomb_wall_pos)
{
    static uint8   tmp_map[MAP_ROWS][MAP_COLS];
    static NavPath_t tmp_path;

    int16 best_len = 32767;
    uint8 found    = 0;

    /* 遍历所有内部墙体（最外圈不可炸） */
    for (int8 r = 1; r < MAP_ROWS - 1; r++) {
        for (int8 c = 1; c < MAP_COLS - 1; c++) {
            if (map[r][c] != MAP_WALL) continue;

            /* 假设在 (r, c) 引爆炸弹，3×3 范围清除内墙 */
            memcpy(tmp_map, map, sizeof(tmp_map));
            for (int8 dr = -1; dr <= 1; dr++) {
                for (int8 dc = -1; dc <= 1; dc++) {
                    int8 rr = r + dr, cc = c + dc;
                    if (rr >= 1 && rr < MAP_ROWS - 1 &&
                        cc >= 1 && cc < MAP_COLS - 1) {
                        if (tmp_map[rr][cc] == MAP_WALL)
                            tmp_map[rr][cc] = MAP_EMPTY;
                    }
                }
            }

            /* 检查 blocked_target 是否变得可达 */
            if (Algo_Nav_BFS(tmp_map, player_pos, blocked_target, &tmp_path)) {
                if ((int16)tmp_path.step_count < best_len) {
                    best_len = (int16)tmp_path.step_count;
                    bomb_wall_pos->x = c;
                    bomb_wall_pos->y = r;
                    found = 1;
                }
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
uint16 Sokoban_Actions_To_Waypoints(const SokoAction_e *actions,
                                    uint16 count,
                                    Point_t start_pos,
                                    SokoWaypointPath_t *wp_path)
{
    wp_path->count = 0;
    if (count == 0) return 0;

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
                wp_path->count++;
            }
        }
    }
    return wp_path->count;
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

    return sokoban_bfs_single(sb_sub_map, player_pos, bomb_pos, wall_pos, sol);
}
