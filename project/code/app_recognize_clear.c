/*********************************************************************************************************************
 * 文件名称   : app_recognize_clear.c
 * 模块功能   : 识别巡航导航/轻量推箱清障规划
 *
 * @owner     rt1064-main
 * @periph    none                纯算法 (导航 + 避障规划)
 *
 * 设计边界:
 *   - 先尝试纯导航, 只有观察格不可达时才推箱;
 *   - 最多 APP_RECOG_CLEAR_MAX_PUSHES 次推箱, 便于控制搜索量和实车风险;
 *   - 每次推箱前, 玩家必须能在当前地图上走到箱子背后;
 *   - 当前待识别物体受保护, 不会在观察前被移动;
 *   - 不把清障箱推上目标格或静态墙角.
 *
 * 后续若要支持更深搜索, 只需增大最大推箱数或替换本模块, 识别状态机接口无需变化。
 *********************************************************************************************************************/

#include "app_recognize_clear.h"
#include "chassis_config.h"
#include <string.h>

static const int8 s_dr[4] = {-1, 1, 0, 0};
static const int8 s_dc[4] = {0, 0, -1, 1};

static const SokoAction_e s_action[4] = {
    SOKO_ACT_UP, SOKO_ACT_DOWN, SOKO_ACT_LEFT, SOKO_ACT_RIGHT
};

/* 避免在递归栈上放地图和 NavPath。 */
static uint8     s_map_stack[APP_RECOG_CLEAR_MAX_PUSHES + 1U][MAP_ROWS][MAP_COLS];
static NavPath_t s_walk_path[APP_RECOG_CLEAR_MAX_PUSHES + 1U];
static NavPath_t s_observe_path[APP_RECOG_CLEAR_MAX_PUSHES + 1U];
static SokoActionSeq_t s_work_actions;
static Point_t   s_goal_observe;
static Point_t   s_goal_player;
static uint8     s_goal_pushes;

static uint8 clear_is_inner(int8 y, int8 x)
{
    return (uint8)(x >= (int8)CHASSIS_GRID_INNER_MIN_X &&
                   x <= (int8)CHASSIS_GRID_INNER_MAX_X &&
                   y >= (int8)CHASSIS_GRID_INNER_MIN_Y &&
                   y <= (int8)CHASSIS_GRID_INNER_MAX_Y);
}

static uint8 clear_is_walkable(const uint8 map[MAP_ROWS][MAP_COLS], int8 y, int8 x)
{
    if (!clear_is_inner(y, x)) return 0U;
    return (uint8)(map[y][x] == MAP_EMPTY || map[y][x] == MAP_TARGET);
}

static uint8 clear_is_wall_or_edge(const uint8 map[MAP_ROWS][MAP_COLS], int8 y, int8 x)
{
    if (!clear_is_inner(y, x)) return 1U;
    return (uint8)(map[y][x] == MAP_WALL);
}

static uint8 clear_is_static_corner(const uint8 map[MAP_ROWS][MAP_COLS], int8 y, int8 x)
{
    uint8 up    = clear_is_wall_or_edge(map, (int8)(y - 1), x);
    uint8 down  = clear_is_wall_or_edge(map, (int8)(y + 1), x);
    uint8 left  = clear_is_wall_or_edge(map, y, (int8)(x - 1));
    uint8 right = clear_is_wall_or_edge(map, y, (int8)(x + 1));
    return (uint8)((up && left) || (up && right) ||
                   (down && left) || (down && right));
}

static uint8 append_action(SokoAction_e action)
{
    if (s_work_actions.count >= (uint16)SOKOBAN_MAX_ACTIONS) return 0U;
    s_work_actions.actions[s_work_actions.count++] = action;
    return 1U;
}

static uint8 append_nav_path(Point_t start, const NavPath_t *path)
{
    Point_t prev = start;
    uint16 i;

    for (i = 0U; i < path->step_count; ++i)
    {
        Point_t next = path->path[i];
        SokoAction_e action;
        int8 dx = (int8)(next.x - prev.x);
        int8 dy = (int8)(next.y - prev.y);

        if      (dx ==  0 && dy == -1) action = SOKO_ACT_UP;
        else if (dx ==  0 && dy ==  1) action = SOKO_ACT_DOWN;
        else if (dx == -1 && dy ==  0) action = SOKO_ACT_LEFT;
        else if (dx ==  1 && dy ==  0) action = SOKO_ACT_RIGHT;
        else return 0U;

        if (!append_action(action)) return 0U;
        prev = next;
    }
    return 1U;
}

static uint8 find_best_observe_path(const uint8 map[MAP_ROWS][MAP_COLS],
                                    Point_t player,
                                    Point_t object,
                                    uint8 depth,
                                    Point_t *observe)
{
    NavPath_t *best = &s_observe_path[depth];
    uint16 best_len = 0xFFFFU;
    uint8 found = 0U;
    uint8 d;

    for (d = 0U; d < 4U; ++d)
    {
        Point_t candidate;
        candidate.y = (int8)(object.y + s_dr[d]);
        candidate.x = (int8)(object.x + s_dc[d]);
        if (!clear_is_walkable(map, candidate.y, candidate.x)) continue;

        if (Algo_Nav_BFS(map, player, candidate, &s_walk_path[depth]) &&
            s_walk_path[depth].step_count < best_len)
        {
            best_len = s_walk_path[depth].step_count;
            *best = s_walk_path[depth];
            *observe = candidate;
            found = 1U;
        }
    }
    return found;
}

static uint8 clear_search(Point_t player,
                          Point_t protected_object,
                          uint8 depth,
                          uint8 max_pushes)
{
    uint8 (*map)[MAP_COLS] = s_map_stack[depth];
    Point_t observe;
    uint16 action_mark;
    int8 y, x;
    uint8 d;

    /* 任意深度都先检查是否已经打通观察路线。 */
    if (find_best_observe_path((const uint8 (*)[MAP_COLS])map,
                               player, protected_object, depth, &observe))
    {
        action_mark = s_work_actions.count;
        if (append_nav_path(player, &s_observe_path[depth]))
        {
            s_goal_observe = observe;
            s_goal_player = observe;
            s_goal_pushes = depth;
            return 1U;
        }
        s_work_actions.count = action_mark;
        return 0U;
    }

    if (depth >= max_pushes) return 0U;

    for (y = (int8)CHASSIS_GRID_INNER_MIN_Y;
         y <= (int8)CHASSIS_GRID_INNER_MAX_Y; ++y)
    {
        for (x = (int8)CHASSIS_GRID_INNER_MIN_X;
             x <= (int8)CHASSIS_GRID_INNER_MAX_X; ++x)
        {
            if (map[y][x] != MAP_BOX) continue;
            if (x == protected_object.x && y == protected_object.y) continue;

            for (d = 0U; d < 4U; ++d)
            {
                Point_t stand;
                Point_t destination;
                Point_t player_after_push;

                stand.x = (int8)(x - s_dc[d]);
                stand.y = (int8)(y - s_dr[d]);
                destination.x = (int8)(x + s_dc[d]);
                destination.y = (int8)(y + s_dr[d]);

                /* 清障不占目标格, 避免破坏目标编号和底图语义。 */
                if (!clear_is_inner(destination.y, destination.x) ||
                    map[destination.y][destination.x] != MAP_EMPTY)
                {
                    continue;
                }
                if (!clear_is_walkable((const uint8 (*)[MAP_COLS])map,
                                       stand.y, stand.x))
                {
                    continue;
                }
                if (clear_is_static_corner((const uint8 (*)[MAP_COLS])map,
                                           destination.y, destination.x))
                {
                    continue;
                }
                if (!Algo_Nav_BFS((const uint8 (*)[MAP_COLS])map,
                                  player, stand, &s_walk_path[depth]))
                {
                    continue;
                }

                action_mark = s_work_actions.count;
                if (!append_nav_path(player, &s_walk_path[depth]) ||
                    !append_action(s_action[d]))
                {
                    s_work_actions.count = action_mark;
                    continue;
                }

                memcpy(s_map_stack[depth + 1U], map, MAP_ROWS * MAP_COLS);
                s_map_stack[depth + 1U][y][x] = MAP_EMPTY;
                s_map_stack[depth + 1U][destination.y][destination.x] = MAP_BOX;
                player_after_push.x = x;
                player_after_push.y = y;

                if (clear_search(player_after_push, protected_object,
                                 (uint8)(depth + 1U), max_pushes))
                {
                    return 1U;
                }
                s_work_actions.count = action_mark;
            }
        }
    }
    return 0U;
}

uint8 App_Recog_Clear_Plan(const uint8 map[MAP_ROWS][MAP_COLS],
                           Point_t player,
                           Point_t protected_object,
                           AppRecogClearPlan_t *out)
{
    uint8 push_limit;

    if (map == 0 || out == 0) return 0U;
    memset(out, 0, sizeof(*out));
    memcpy(s_map_stack[0], map, MAP_ROWS * MAP_COLS);

    /* 迭代加深: 0 次推箱优先, 然后 1 次、2 次。 */
    for (push_limit = 0U;
         push_limit <= (uint8)APP_RECOG_CLEAR_MAX_PUSHES;
         ++push_limit)
    {
        s_work_actions.count = 0U;
        memcpy(s_map_stack[0], map, MAP_ROWS * MAP_COLS);
        if (clear_search(player, protected_object, 0U, push_limit))
        {
            out->actions = s_work_actions;
            out->observe = s_goal_observe;
            out->player_end = s_goal_player;
            out->push_count = s_goal_pushes;
            return 1U;
        }
    }
    return 0U;
}
