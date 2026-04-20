#include "app_game_logic.h"

uint8 g_game_map[MAP_ROWS][MAP_COLS];
Point_t g_player_pos = {(int8)CHASSIS_START_GRID_X, (int8)CHASSIS_START_GRID_Y};

#define GAME_LOGIC_TASK_PERIOD_MS      (5U)
#define DEADLOCK_RESET_HOLD_MS         (3000U)
#define DEADLOCK_RESET_HOLD_TICKS      (DEADLOCK_RESET_HOLD_MS / GAME_LOGIC_TASK_PERIOD_MS)

typedef enum {
    EXEC_NONE = 0,
    EXEC_PUSH_BOX,
    EXEC_PUSH_BOMB
} ExecMode_e;

static GameStage_e current_stage = STAGE_WAIT_START;
static uint8 is_navigating = 0;

static SokoFullSolution_t    g_soko_solution;
static SokoWaypointPath_t    g_soko_waypoints;
static uint8                 g_soko_sub_idx = 0;
static uint16                g_soko_wp_idx = 0;
static uint8                 g_soko_exec_init = 0;

static Point_t               g_bomb_pos;
static Point_t               g_bomb_wall_pos;
static SokoActionSeq_t       g_bomb_action_seq;
static SokoWaypointPath_t    g_bomb_waypoints;
static uint16                g_bomb_wp_idx = 0;

static uint8                 g_box_to_target[SOKOBAN_MAX_BOXES] = {0};
static uint8                 g_current_level = 1;
static uint16                g_reset_hold_ticks = 0;
static ExecMode_e            g_exec_mode = EXEC_NONE;

static void reset_exec_context(void)
{
    g_soko_solution.is_solved = 0;
    g_soko_solution.total_boxes = 0;
    g_soko_waypoints.count = 0;
    g_soko_sub_idx = 0;
    g_soko_wp_idx = 0;
    g_soko_exec_init = 0;

    g_bomb_waypoints.count = 0;
    g_bomb_wp_idx = 0;
    g_exec_mode = EXEC_NONE;
    is_navigating = 0;
}

static void goto_stage(GameStage_e next)
{
    current_stage = next;
}

static uint8 get_map_box_count(void)
{
    uint8 count = 0;
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_BOX) {
                count++;
            }
        }
    }
    return count;
}

static uint8 map_has_bomb(void)
{
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_BOMB) {
                return 1;
            }
        }
    }
    return 0;
}

static Point_t find_bomb_pos_on_map(void)
{
    Point_t bomb = {-1, -1};
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_BOMB) {
                bomb.x = c;
                bomb.y = r;
                return bomb;
            }
        }
    }
    return bomb;
}

static Point_t choose_nearest_target(Point_t ref)
{
    Point_t best = {-1, -1};
    int16 best_d = 32767;

    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] != MAP_TARGET) continue;

            int16 dx = (int16)c - (int16)ref.x;
            int16 dy = (int16)r - (int16)ref.y;
            if (dx < 0) dx = (int16)-dx;
            if (dy < 0) dy = (int16)-dy;

            if ((int16)(dx + dy) < best_d) {
                best_d = (int16)(dx + dy);
                best.x = c;
                best.y = r;
            }
        }
    }
    return best;
}

static void sync_player_pos(void)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();
    g_player_pos.x = (int8)chassis_m_to_grid_x(pose.x_m);
    g_player_pos.y = (int8)chassis_m_to_grid_y(pose.y_m);
}

static uint8 exec_waypoints_common(const SokoWaypointPath_t *wp, uint16 *wp_idx)
{
    if (!is_navigating) {
        if (*wp_idx < wp->count) {
            HAL_CHASSIS_MOVE_TO(wp->points[*wp_idx].x, wp->points[*wp_idx].y);
            is_navigating = 1;
        }
        return 0;
    }

    if (!HAL_CHASSIS_IS_ARRIVED()) return 0;

    is_navigating = 0;
    (*wp_idx)++;

    if (*wp_idx < wp->count) {
        HAL_CHASSIS_MOVE_TO(wp->points[*wp_idx].x, wp->points[*wp_idx].y);
        is_navigating = 1;
        return 0;
    }

    return 1;
}

static uint8 exec_push_box_solution(void)
{
    if (g_soko_wp_idx >= g_soko_waypoints.count) {
        g_soko_sub_idx++;
        if (g_soko_sub_idx >= g_soko_solution.total_boxes) {
            return 1;
        }

        Sokoban_Actions_To_Waypoints(
            g_soko_solution.sub_solutions[g_soko_sub_idx].actions,
            g_soko_solution.sub_solutions[g_soko_sub_idx].count,
            g_soko_solution.player_end_pos[g_soko_sub_idx - 1],
            &g_soko_waypoints);
        g_soko_wp_idx = 0;
        is_navigating = 0;
    }

    return exec_waypoints_common(&g_soko_waypoints, &g_soko_wp_idx);
}

static uint8 find_first_unreachable_target(Point_t *blocked_target)
{
    NavPath_t nav_tmp;

    if (!blocked_target) return 0;

    blocked_target->x = -1;
    blocked_target->y = -1;

    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_TARGET) {
                Point_t tp = {c, r};
                if (!Algo_Nav_BFS(g_game_map, g_player_pos, tp, &nav_tmp)) {
                    *blocked_target = tp;
                    return 1;
                }
            }
        }
    }

    return 0;
}

static uint8 build_bomb_plan(Point_t blocked_target)
{
    g_bomb_pos = find_bomb_pos_on_map();
    if (g_bomb_pos.x < 0) return 0;

    if (!Sokoban_Find_Bomb_Wall(g_game_map, g_player_pos,
                                blocked_target, &g_bomb_wall_pos)) {
        return 0;
    }

    if (!Sokoban_Solve_Push_Bomb(g_game_map, g_player_pos,
                                 g_bomb_pos, g_bomb_wall_pos,
                                 &g_bomb_action_seq)) {
        return 0;
    }

    Sokoban_Actions_To_Waypoints(g_bomb_action_seq.actions,
                                 g_bomb_action_seq.count,
                                 g_player_pos,
                                 &g_bomb_waypoints);
    g_bomb_wp_idx = 0;
    g_exec_mode = EXEC_PUSH_BOMB;
    is_navigating = 0;
    return 1;
}

static uint8 build_push_box_plan(void)
{
    uint8 box_count = get_map_box_count();

    if (box_count == 0) {
        return 1;
    }

    if (map_has_bomb()) {
        if (!Sokoban_Solve_Stage2(g_game_map, g_player_pos,
                                  g_box_to_target,
                                  box_count,
                                  &g_soko_solution)) {
            return 0;
        }
    } else {
        if (!Sokoban_Solve_Stage1(g_game_map, g_player_pos,
                                  &g_soko_solution)) {
            return 0;
        }
    }

    if (!g_soko_solution.is_solved || g_soko_solution.total_boxes == 0) {
        return 0;
    }

    g_soko_sub_idx = 0;
    Sokoban_Actions_To_Waypoints(g_soko_solution.sub_solutions[0].actions,
                                 g_soko_solution.sub_solutions[0].count,
                                 g_player_pos,
                                 &g_soko_waypoints);
    g_soko_wp_idx = 0;
    g_exec_mode = EXEC_PUSH_BOX;
    is_navigating = 0;
    return 1;
}

static uint8 should_enter_next_level(void)
{
    /* TODO: 结合裁判系统/传感器触发下一关。
     * 当前默认仅跑单关，返回 0。 */
    return 0;
}

static void stage_wait_start_handler(void)
{
    if (!is_navigating) {
        HAL_CHASSIS_MOVE_TO(CHASSIS_START_GRID_X, CHASSIS_START_GRID_Y);
        is_navigating = 1;
        return;
    }

    if (!HAL_CHASSIS_IS_ARRIVED()) return;

    is_navigating = 0;
    reset_exec_context();
    goto_stage(STAGE_RECOGNIZE_MAP);
}

static void stage_recognize_handler(void)
{
    /* TODO: 在这里接入视觉识别与 box->target 映射更新。
     * 当前框架下识别完成后直接进入规划状态。 */
    goto_stage(STAGE_PLAN_PATH);
}

static void stage_plan_handler(void)
{
    Point_t dead_box;
    Point_t blocked_target;

    if (g_soko_exec_init) {
        return;
    }

    g_soko_exec_init = 1;

    if (get_map_box_count() == 0) {
        goto_stage(STAGE_LEVEL_JUDGE);
        return;
    }

    if (build_push_box_plan()) {
        goto_stage(STAGE_EXECUTE_ACTION);
        return;
    }

    dead_box.x = -1;
    dead_box.y = -1;
    blocked_target.x = -1;
    blocked_target.y = -1;

    if (Sokoban_Is_Deadlock(g_game_map, &dead_box)) {
        blocked_target = choose_nearest_target(dead_box);
        if (blocked_target.x >= 0 && build_bomb_plan(blocked_target)) {
            goto_stage(STAGE_EXECUTE_ACTION);
            return;
        }
        goto_stage(STAGE_DEADLOCK_RESET);
        return;
    }

    if (find_first_unreachable_target(&blocked_target) &&
        build_bomb_plan(blocked_target)) {
        goto_stage(STAGE_EXECUTE_ACTION);
        return;
    }

    goto_stage(STAGE_DEADLOCK_RESET);
}

static void stage_execute_handler(void)
{
    uint8 done = 0;

    if (g_exec_mode == EXEC_PUSH_BOX) {
        done = exec_push_box_solution();
    } else if (g_exec_mode == EXEC_PUSH_BOMB) {
        done = exec_waypoints_common(&g_bomb_waypoints, &g_bomb_wp_idx);
        if (done) {
            Sokoban_Apply_Bomb_Explosion(g_game_map, g_bomb_wall_pos);
            g_game_map[g_bomb_pos.y][g_bomb_pos.x] = MAP_EMPTY;
            reset_exec_context();
            goto_stage(STAGE_PLAN_PATH);
            return;
        }
    } else {
        goto_stage(STAGE_PLAN_PATH);
        return;
    }

    if (done) {
        reset_exec_context();
        goto_stage(STAGE_LEVEL_JUDGE);
    }
}

static void stage_level_judge_handler(void)
{
    if (get_map_box_count() != 0) {
        goto_stage(STAGE_PLAN_PATH);
        return;
    }

    if (should_enter_next_level()) {
        g_current_level++;
        reset_exec_context();
        goto_stage(STAGE_WAIT_START);
        return;
    }

    goto_stage(STAGE_DONE);
}

static void stage_deadlock_reset_handler(void)
{
    if (!is_navigating) {
        HAL_CHASSIS_MOVE_TO(CHASSIS_START_GRID_X, CHASSIS_START_GRID_Y);
        is_navigating = 1;
        g_reset_hold_ticks = 0;
        return;
    }

    if (!HAL_CHASSIS_IS_ARRIVED()) return;

    is_navigating = 0;

    if (g_reset_hold_ticks < DEADLOCK_RESET_HOLD_TICKS) {
        g_reset_hold_ticks++;
        return;
    }

    g_reset_hold_ticks = 0;
    reset_exec_context();
    goto_stage(STAGE_WAIT_START);
}

static void stage_done_handler(void)
{
    /* 比赛流程完成，维持静止即可。 */
}

void Game_Logic_Task_Run(void)
{
    sync_player_pos();

    switch (current_stage) {
        case STAGE_WAIT_START:
            stage_wait_start_handler();
            break;

        case STAGE_RECOGNIZE_MAP:
            stage_recognize_handler();
            break;

        case STAGE_PLAN_PATH:
            stage_plan_handler();
            break;

        case STAGE_EXECUTE_ACTION:
            stage_execute_handler();
            break;

        case STAGE_LEVEL_JUDGE:
            stage_level_judge_handler();
            break;

        case STAGE_DEADLOCK_RESET:
            stage_deadlock_reset_handler();
            break;

        case STAGE_DONE:
            stage_done_handler();
            break;

        default:
            reset_exec_context();
            goto_stage(STAGE_WAIT_START);
            break;
    }
}
