/*===========================================================================
 * [app_static_map_drive_helper.c] 静态地图推箱启动辅助实现
 *
 * 由于车可能从半格物理位置出发，本模块会把起点附近可进入的整数格全部
 * 作为候选入口，分别调用推箱求解器，再按总估算距离选出最适合执行的方案。
 *===========================================================================*/

#include "app_static_map_drive_helper.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

/* 推箱求解结果体较大，放到静态区避免占用主循环栈空间。 */
static SokoFullSolution_t s_candidate_solution;

/*
 * 函数: App_StaticMapDrive_CharToMap
 * 功能: 将手写/视觉 ASCII 地图字符转换成 MAP_* 枚举。
 * 参数: ch - 地图字符，支持 '#', '-', '.', '$', '*', '@'。
 * 返回: 对应 MAP_* 单元类型；未知字符按 MAP_EMPTY 处理。
 * 说明: '@' 表示视觉端标出的车位，本模块不把车写进地图，因此按空地处理。
 */
uint8 App_StaticMapDrive_CharToMap(char ch)
{
    switch (ch)
    {
    case '#': return MAP_WALL;
    case '-': return MAP_EMPTY;
    case '.': return MAP_TARGET;
    case '$': return MAP_BOX;
    case '*': return MAP_BOMB;
    case '@': return MAP_EMPTY;
    default:  return MAP_EMPTY;
    }
}

/*
 * 函数: App_StaticMapDrive_MapToChar
 * 功能: 将 MAP_* 枚举转换回 ASCII 地图字符。
 * 参数: v - MAP_EMPTY/MAP_WALL/MAP_TARGET/MAP_BOX/MAP_BOMB 等枚举值。
 * 返回: 对应字符；未知值按 '-' 空地输出。
 * 用途: 打印求解输入地图，确认接收到的地图内容是否符合预期。
 */
char App_StaticMapDrive_MapToChar(uint8 v)
{
    switch (v)
    {
    case MAP_WALL:   return '#';
    case MAP_TARGET: return '.';
    case MAP_BOX:    return '$';
    case MAP_BOMB:   return '*';
    case MAP_EMPTY:
    default:         return '-';
    }
}

/*
 * 函数: App_StaticMapDrive_LoadCharMap
 * 功能: 将 MAP_ROWS 行 ASCII 字符串地图加载为 MAP_* 二维数组。
 * 参数:
 *   src - 每行 MAP_COLS 个地图字符，末尾额外 1 字节用于 '\0'。
 *   dst - 输出的 MAP_* 二维数组。
 * 返回: 无。
 * 说明: 不检查 src 是否为 NULL；调用方应传入编译期固定地图或已校验地图。
 */
void App_StaticMapDrive_LoadCharMap(const char src[MAP_ROWS][MAP_COLS + 1U],
                                    uint8 dst[MAP_ROWS][MAP_COLS])
{
    uint8 r, c;
    /* 每行末尾预留 '\0' 仅用于字符串书写，这里只拷贝实际地图列。 */
    for (r = 0U; r < MAP_ROWS; r++)
    {
        for (c = 0U; c < MAP_COLS; c++)
        {
            dst[r][c] = App_StaticMapDrive_CharToMap(src[r][c]);
        }
    }
}

/*
 * 函数: app_smd_is_passable_entry
 * 功能: 判断某个整数格是否可作为半格起点进入规划的入口格。
 * 参数:
 *   map - MAP_* 地图。
 *   p   - 候选入口格。
 * 返回: 1=在内场且为空地/目标点；0=越界或被墙/箱子/炸弹占用。
 */
static uint8 app_smd_is_passable_entry(const uint8 map[MAP_ROWS][MAP_COLS], Point_t p)
{
    /* 入口格必须在内场，并且不能是墙、箱子或炸弹。 */
    if (p.x < (int8)CHASSIS_GRID_INNER_MIN_X || p.x > (int8)CHASSIS_GRID_INNER_MAX_X) return 0U;
    if (p.y < (int8)CHASSIS_GRID_INNER_MIN_Y || p.y > (int8)CHASSIS_GRID_INNER_MAX_Y) return 0U;
    return ((map[(uint8)p.y][(uint8)p.x] == MAP_EMPTY) ||
            (map[(uint8)p.y][(uint8)p.x] == MAP_TARGET)) ? 1U : 0U;
}

/*
 * 函数: app_smd_add_start_candidate
 * 功能: 将合法且未重复的入口格加入候选数组。
 * 参数:
 *   map        - MAP_* 地图。
 *   candidates - 候选入口数组，最大由调用方保证。
 *   count      - 当前候选数量，成功加入后自增。
 *   p          - 待加入入口格。
 * 返回: 1=成功加入；0=不可通行或重复。
 */
static uint8 app_smd_add_start_candidate(const uint8 map[MAP_ROWS][MAP_COLS],
                                         Point_t candidates[],
                                         uint8 *count,
                                         Point_t p)
{
    uint8 i;
    if (!app_smd_is_passable_entry(map, p)) return 0U;
    /* 同一个入口格可能由 X/Y 候选组合重复生成，写入前去重。 */
    for (i = 0U; i < *count; i++)
    {
        if ((candidates[i].x == p.x) && (candidates[i].y == p.y)) return 0U;
    }
    candidates[*count] = p;
    (*count)++;
    return 1U;
}

/*
 * 函数: app_smd_axis_candidates
 * 功能: 将一个浮点网格坐标展开为 1~2 个相邻整数候选。
 * 参数:
 *   grid - 浮点网格坐标，例如 5.0 或 5.5。
 *   out  - 输出候选整数坐标，长度至少为 2。
 * 返回: 输出候选数量。
 * 示例: 5.0 -> {5}; 5.5 -> {5, 6}。
 */
static uint8 app_smd_axis_candidates(float grid, int8 out[2])
{
    int8 base = (int8)grid;
    float frac = grid - (float)base;
    if (frac < 0.0f) frac = -frac;
    out[0] = base;
    if (frac < 0.001f)
    {
        /* 已经落在整数格中心附近，只需要一个候选入口。 */
        return 1U;
    }
    /* 例如 5.5 会同时尝试 5 和 6 两个邻近入口格。 */
    out[1] = (int8)(base + 1);
    return 2U;
}

/*
 * 函数: app_smd_build_start_candidates
 * 功能: 根据浮点发车网格坐标生成最多 4 个可用入口格。
 * 参数:
 *   map          - MAP_* 地图。
 *   start_x_grid - 发车点 X 浮点网格坐标。
 *   start_y_grid - 发车点 Y 浮点网格坐标。
 *   preferred    - 同分优先的入口格。
 *   candidates   - 输出候选入口数组，容量至少为 4。
 * 返回: 候选数量。
 * 说明: 对 X/Y 两个方向分别展开候选，再组合、过滤、去重。
 */
static uint8 app_smd_build_start_candidates(const uint8 map[MAP_ROWS][MAP_COLS],
                                            float start_x_grid,
                                            float start_y_grid,
                                            Point_t preferred,
                                            Point_t candidates[4])
{
    int8 xs[2];
    int8 ys[2];
    uint8 xn = app_smd_axis_candidates(start_x_grid, xs);
    uint8 yn = app_smd_axis_candidates(start_y_grid, ys);
    uint8 count = 0U;
    uint8 xi, yi, i;

    for (yi = 0U; yi < yn; yi++)
    {
        for (xi = 0U; xi < xn; xi++)
        {
            Point_t p;
            p.x = xs[xi];
            p.y = ys[yi];
            (void)app_smd_add_start_candidate(map, candidates, &count, p);
        }
    }

    /* preferred_entry 通常来自底盘发车区默认格，放到候选首位便于同分时优先尝试。 */
    for (i = 0U; i < count; i++)
    {
        if ((candidates[i].x == preferred.x) && (candidates[i].y == preferred.y))
        {
            Point_t tmp = candidates[0];
            candidates[0] = candidates[i];
            candidates[i] = tmp;
            break;
        }
    }

    return count;
}

/* 将 X 浮点网格坐标换算为米制坐标，用于候选入口距离评分。 */
static float app_smd_grid_to_m_x(float grid)
{
    return (grid - 0.5f) * CHASSIS_GRID_STEP_X_M;
}

/* 将 Y 浮点网格坐标换算为米制坐标，用于候选入口距离评分。 */
static float app_smd_grid_to_m_y(float grid)
{
    return (grid - 0.5f) * CHASSIS_GRID_STEP_Y_M;
}

/*
 * 函数: app_smd_entry_distance_m
 * 功能: 计算发车浮点位置到候选入口格中心的直线距离。
 * 参数:
 *   start_x_grid/start_y_grid - 发车点浮点网格坐标。
 *   entry                     - 候选入口整数格。
 * 返回: 米制距离。
 */
static float app_smd_entry_distance_m(float start_x_grid, float start_y_grid, Point_t entry)
{
    /* 对半格发车点到整数入口格的物理距离做评分，避免只看推箱步数。 */
    float sx = app_smd_grid_to_m_x(start_x_grid);
    float sy = app_smd_grid_to_m_y(start_y_grid);
    float ex = app_smd_grid_to_m_x((float)entry.x);
    float ey = app_smd_grid_to_m_y((float)entry.y);
    float dx = ex - sx;
    float dy = ey - sy;
    return sqrtf(dx * dx + dy * dy);
}

/*
 * 函数: app_smd_action_distance_m
 * 功能: 估算一段推箱动作序列对应的物理移动距离。
 * 参数: seq - 求解器输出的动作序列。
 * 返回: 米制估算距离。
 * 说明: X/Y 格距可能不同，因此按动作方向分别加 CHASSIS_GRID_STEP_X/Y_M。
 */
static float app_smd_action_distance_m(const SokoActionSeq_t *seq)
{
    uint16 i;
    float dist = 0.0f;
    /* X/Y 网格物理间距不同，按动作方向分别累计真实距离。 */
    for (i = 0U; i < seq->count; i++)
    {
        if ((seq->actions[i] == SOKO_ACT_LEFT) || (seq->actions[i] == SOKO_ACT_RIGHT))
        {
            dist += CHASSIS_GRID_STEP_X_M;
        }
        else
        {
            dist += CHASSIS_GRID_STEP_Y_M;
        }
    }
    return dist;
}

/*
 * 函数: app_smd_solution_distance_m
 * 功能: 估算完整推箱方案中所有箱子动作段的总物理距离。
 * 参数: solution - 完整推箱求解结果。
 * 返回: 米制估算距离。
 * 用途: 与入口距离相加作为候选入口评分。
 */
static float app_smd_solution_distance_m(const SokoFullSolution_t *solution)
{
    uint8 i;
    float dist = 0.0f;
    for (i = 0U; i < solution->total_boxes; i++)
    {
        dist += app_smd_action_distance_m(&solution->sub_solutions[i]);
    }
    return dist;
}

/*
 * 函数: app_smd_action_delta
 * 功能: 将推箱动作枚举转换成网格坐标增量。
 * 参数:
 *   act - 动作枚举。
 *   dx  - 输出 X 增量。
 *   dy  - 输出 Y 增量。
 * 返回: 1=动作有效；0=未知动作，增量置 0。
 */
static uint8 app_smd_action_delta(SokoAction_e act, int8 *dx, int8 *dy)
{
    switch (act)
    {
    case SOKO_ACT_UP:    *dx =  0; *dy = -1; return 1U;
    case SOKO_ACT_DOWN:  *dx =  0; *dy =  1; return 1U;
    case SOKO_ACT_LEFT:  *dx = -1; *dy =  0; return 1U;
    case SOKO_ACT_RIGHT: *dx =  1; *dy =  0; return 1U;
    default:             *dx =  0; *dy =  0; return 0U;
    }
}

/*
 * 函数: app_smd_actions_to_waypoints_from_entry
 * 功能: 从入口格开始，把动作序列压缩成底盘航点序列。
 * 参数:
 *   actions - 求解器动作数组。
 *   count   - 动作数量。
 *   entry   - 执行起点入口格。
 *   wp_path - 输出航点路径。
 * 返回: 输出航点数量。
 * 压缩规则: 连续同方向动作只保留该连续段末端作为航点，减少底盘频繁切点。
 */
static uint16 app_smd_actions_to_waypoints_from_entry(const SokoAction_e *actions,
                                                      uint16 count,
                                                      Point_t entry,
                                                      SokoWaypointPath_t *wp_path)
{
    uint16 i;
    int8 cx = entry.x;
    int8 cy = entry.y;

    wp_path->count = 0U;
    if (wp_path->count < SOKOBAN_MAX_WAYPOINTS)
    {
        /* 首航点就是选中的入口格，让车先从发车物理点收敛到可规划网格中心。 */
        wp_path->points[wp_path->count] = entry;
        wp_path->count++;
    }

    for (i = 0U; i < count; i++)
    {
        int8 dx, dy;
        if (!app_smd_action_delta(actions[i], &dx, &dy)) break;
        cx = (int8)(cx + dx);
        cy = (int8)(cy + dy);

        if ((i == (uint16)(count - 1U)) || (actions[i] != actions[i + 1U]))
        {
            /* 连续同方向动作合并成一个航点，减少上层导航切点次数。 */
            if (wp_path->count < SOKOBAN_MAX_WAYPOINTS)
            {
                wp_path->points[wp_path->count].x = cx;
                wp_path->points[wp_path->count].y = cy;
                wp_path->count++;
            }
        }
    }
    return wp_path->count;
}

/*
 * 函数: App_StaticMapDrive_SolveFromLaunch
 * 功能: 从半格/浮点发车位置求出可执行的静态地图推箱方案。
 * 参数:
 *   map             - MAP_* 地图。
 *   start_x_grid    - 发车点 X 浮点网格坐标。
 *   start_y_grid    - 发车点 Y 浮点网格坐标。
 *   preferred_entry - 同分优先入口格，通常是底盘默认发车格。
 *   out             - 输出规划结果。
 * 返回: 1=找到可执行方案；0=无合法入口或求解失败。
 * 算法:
 *   1. 根据浮点起点生成最多 4 个整数入口候选。
 *   2. 对每个候选入口调用 Sokoban_Solve_Stage1。
 *   3. 用“入口距离 + 推箱动作估算距离”打分，选择最短方案。
 *   4. 为首个箱子生成第一段航点，后续段由主状态机继续展开。
 */
uint8 App_StaticMapDrive_SolveFromLaunch(const uint8 map[MAP_ROWS][MAP_COLS],
                                         float start_x_grid,
                                         float start_y_grid,
                                         Point_t preferred_entry,
                                         AppStaticMapDrivePlan_t *out)
{
    Point_t candidates[4];
    uint8 cand_count;
    uint8 i;
    uint8 best_valid = 0U;
    float best_score = 0.0f;

    if (out == NULL) { return 0U; }

    memset(out, 0, sizeof(*out));
    cand_count = app_smd_build_start_candidates(map,
                                                start_x_grid,
                                                start_y_grid,
                                                preferred_entry,
                                                candidates);

    /* 对每个合法入口格分别求解，最终按“进入入口距离 + 推箱估算距离”择优。 */
    for (i = 0U; i < cand_count; i++)
    {
        Point_t start = candidates[i];
        float entry_dist = app_smd_entry_distance_m(start_x_grid, start_y_grid, start);
        float score;

        memset(&s_candidate_solution, 0, sizeof(s_candidate_solution));
        if (!Sokoban_Solve_Stage1(map, start, &s_candidate_solution))
        {
            continue;
        }
        if (!s_candidate_solution.is_solved || s_candidate_solution.total_boxes == 0U)
        {
            continue;
        }

        score = entry_dist + app_smd_solution_distance_m(&s_candidate_solution);
        if (!best_valid || (score < best_score))
        {
            out->solution = s_candidate_solution;
            out->entry = start;
            out->entry_dist_m = entry_dist;
            out->score_m = score;
            best_score = score;
            best_valid = 1U;
        }
    }

    if (best_valid != 0U)
    {
        /* 只预生成第一段航点；后续箱子的航点由主流程在每段结束后继续展开。 */
        (void)app_smd_actions_to_waypoints_from_entry(out->solution.sub_solutions[0].actions,
                                                      out->solution.sub_solutions[0].count,
                                                      out->entry,
                                                      &out->first_waypoints);
    }

    return best_valid;
}
