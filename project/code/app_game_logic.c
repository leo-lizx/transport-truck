#include "app_game_logic.h"
#include "app_link.h"      /* P0-2: 读取 g_link_last_hb_ms 判断链路是否在线; P0-3: 拷贝 seq-lock 地图快照 */
#include "app_vision_fusion.h"

/*
 * P0-3 说明:
 *   g_game_map 现已降级为 "主循环侧的稳定地图快照", 唯一写者是
 *   Game_Logic_Task_Run() 入口处的 app_link_get_map_snapshot();
 *   LPUART1 ISR 不再直接写它. 业务层各 stage handler 继续裸读 g_game_map 即可,
 *   不会再读到半更新地图.
 */
uint8 g_game_map[MAP_ROWS][MAP_COLS];
Point_t g_player_pos = {(int8)CHASSIS_START_GRID_X, (int8)CHASSIS_START_GRID_Y};

#define GAME_LOGIC_TASK_PERIOD_MS      (5U)
#define DEADLOCK_RESET_HOLD_MS         (3000U)
#define DEADLOCK_RESET_HOLD_TICKS      (DEADLOCK_RESET_HOLD_MS / GAME_LOGIC_TASK_PERIOD_MS)

/* ==================================================================
 * 【P0-2】视觉链路超时回退参数
 *   LINK_LOSS_MS  : 触发"掉线"判定的静默时长门槛
 *   LINK_OK_MS    : 触发"恢复"判定的静默时长门槛 (< LINK_LOSS_MS, 形成迟滞防抖)
 *   视觉端策略 = MAP 帧约 16ms/帧 + 心跳 100ms/次, 200ms 给出 ~2 个心跳余量
 * ================================================================== */
#define LINK_LOSS_MS                   (200U)
#define LINK_OK_MS                     (100U)

typedef enum {
    EXEC_NONE = 0,
    EXEC_PUSH_BOX,
    EXEC_PUSH_BOMB
} ExecMode_e;

static GameStage_e current_stage = STAGE_WAIT_START;
static uint8 is_navigating = 0;

/* ----- 【P0-2】链路状态相关静态变量 -------------------------------- */
static uint8        s_link_alive       = 0U;     /* 当前链路状态: 1=在线 0=离线/未启动 */
static uint8        s_link_ever_alive  = 0U;     /* 是否曾经在线过 (开机直接没数据时, 保持 WAIT 而非 LOSS) */
static GameStage_e  s_stage_resume     = STAGE_WAIT_START; /* LOSS 触发时保存原状态, 恢复时回到该状态 */

/* ----- 【P0-8】OOB / 发车 / 死局静止 相关静态变量 ------------------
 * s_failure_reason   : 比赛失败原因, 触发后状态机锁死在 STAGE_DONE
 * s_wait_start_phase : WAIT_START 子相位
 *                       0 = 还未到达起点 (沿用历史 MOVE_TO 行为)
 *                       1 = 已到起点, 等待车被 "完全离开发车区" (人推 / 系统识别)
 * s_default_launch_zone : 当前默认发车区 (左). 后续菜单可改, 默认 LAUNCH_ZONE_LEFT.
 * --------------------------------------------------------------- */
static GameFailureReason_e s_failure_reason     = GAME_FAIL_NONE;
static uint8               s_wait_start_phase   = 0U;
static LaunchZone_e        s_default_launch_zone = LAUNCH_ZONE_LEFT;

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
static ExecMode_e            g_exec_mode = EXEC_NONE;

/* ==========================================================================
 *  § 1. 执行上下文 / stage 跳转 / 地图查询工具 (全部需主循环单线程调用)
 * ========================================================================== */

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

/*
 * 航点到位判定: odom 到位 + (可选) 视觉到站 Snap 状态机。
 *
 * 行为:
 *   1) chassis_ctrl_is_arrived() == 0  → 直接返回 0 (并取消可能未结束的 Snap, 防跨航点串扰)。
 *   2) 启用 Snap 时:
 *        - 链路掉线 → 取消 Snap 并放行 (避免靠不上视觉时卡死);
 *        - 否则把当前导航目标米坐标喂给 app_vision_fusion_snap_request(), 状态机内部处理短静止+表决+Snap;
 *        - 状态 ∈ {DONE, TIMEOUT, REJECT} 视为放行 (REJECT 时按 odom 兜底);
 *        - 状态 == PENDING / IDLE 视为继续等待。
 *   3) 关闭 Snap → 直接 odom 到位即放行。
 */
/* ==========================================================================
 *  § 2. 航点执行公用逻辑 (chassis_nav_arrived / exec_waypoints_common)
 * ========================================================================== */

static uint8 chassis_nav_arrived_for_waypoint(void)
{
    if (chassis_ctrl_is_arrived() == 0U)
    {
        app_vision_fusion_snap_cancel();
        return 0U;
    }

#if CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE
    if (s_link_alive == 0U)
    {
        app_vision_fusion_snap_cancel();
        return 1U;
    }
    {
        float tx_m = 0.0f;
        float ty_m = 0.0f;
        app_vision_snap_state_e st;
        chassis_ctrl_get_point_nav_target_m(&tx_m, &ty_m);
        app_vision_fusion_snap_request(tx_m, ty_m);
        st = app_vision_fusion_snap_state();
        if ((st == APP_VISION_SNAP_DONE)    ||
            (st == APP_VISION_SNAP_TIMEOUT) ||
            (st == APP_VISION_SNAP_REJECT))
        {
            return 1U;
        }
        return 0U;
    }
#else
    return 1U;
#endif
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

    if (!chassis_nav_arrived_for_waypoint()) return 0;

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

/* ==========================================================================
 *  § 3. 规划层 — 炸弹计划 / 推箱计划 / 关卡推进判定
 * ========================================================================== */

static uint8 find_first_unreachable_target(Point_t *blocked_target)
{
    /* P0-4: NavPath_t (Point_t[200]+uint16 ~402B) \u7531\u6808\u8fc1\u81f3\u6587\u4ef6\u7ea7 BSS\u3002
     *       \u672c\u51fd\u6570\u4ec5\u5728 STAGE_DEADLOCK_RESET (\u4e3b\u5faa\u73af\u7ebf\u7a0b) \u8c03\u7528, \u65e0\u9012\u5f52\u65e0 ISR\u3002 */
    static NavPath_t nav_tmp;

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

/* ==========================================================================
 *  § 4. Stage 处理器 — WAIT_START / RECOGNIZE / PLAN / EXECUTE / JUDGE
 *                       / DEADLOCK_RESET / DONE / PAUSE_ON_LINK_LOSS
 * ========================================================================== */

static void stage_wait_start_handler(void)
{
    /* 【P0-8】WAIT_START 两段式:
     *   phase 0: 主控主动 MOVE_TO 起点 (车被人放偏时复位到发车区中心)
     *   phase 1: 等待车被 "完全离开发车区" → 视为发车成功 → 进入 RECOGNIZE_MAP
     * 与赛规一致: "完全离开发车区" 即视为发车成功 (规则提炼.md §3 要点 1)
     */
    if (s_wait_start_phase == 0U) {
        if (!is_navigating) {
            HAL_CHASSIS_MOVE_TO(CHASSIS_START_GRID_X, CHASSIS_START_GRID_Y);
            is_navigating = 1;
            return;
        }
        if (!chassis_nav_arrived_for_waypoint()) return;

        is_navigating = 0;
        s_wait_start_phase = 1U;
        return;
    }

    /* phase 1: 持续判定是否已 "完全离开发车区" */
    if (chassis_zone_is_fully_outside_launch(s_default_launch_zone)) {
        s_wait_start_phase = 0U;     /* 重置子相位, 供后续 LEVEL_JUDGE 复用 */
        reset_exec_context();
        goto_stage(STAGE_RECOGNIZE_MAP);
    }
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
    /* 【P0-8】死局重置: 改为 "在发车区内 + 静止 ≥3s" 几何判据,
     * 不再依赖 wall-clock tick, 与 chassis_zone_is_static() (PIT 5ms tick) 同源.
     * (规则: 返回发车区静止 3s = 系统重置)
     */
    if (!is_navigating) {
        HAL_CHASSIS_MOVE_TO(CHASSIS_START_GRID_X, CHASSIS_START_GRID_Y);
        is_navigating = 1;
        return;
    }

    if (!chassis_nav_arrived_for_waypoint()) return;

    is_navigating = 0;

    /* 必须 "在发车区内" + chassis_zone_is_static() (内部已含 3s 持续判定).
     * chassis_zone_is_static 由 chassis_zone_tick() 周期更新, 在 Run() 入口已调一次. */
    if (!chassis_zone_is_in_launch(s_default_launch_zone)) {
        return;     /* 还未真正回到发车区, 继续等里程计/视觉拉车进入 */
    }
    if (!chassis_zone_is_static()) {
        return;     /* 还未静止满 3s */
    }

    reset_exec_context();
    s_wait_start_phase = 0U;     /* 重置 WAIT_START 子相位, 下一关重新走 "复位→等离开" */
    goto_stage(STAGE_WAIT_START);
}

static void stage_done_handler(void)
{
    /* 比赛流程完成，维持静止即可。 */
}

/* ==================================================================
 * 【P0-2】视觉链路监控 + 超时回退
 * ----------------------------------------------------------------
 * update_link_state():
 *   每个调度 tick 在 Game_Logic_Task_Run 入口被调用一次
 *   - 用 (now - g_link_last_hb_ms) 与 LINK_LOSS_MS / LINK_OK_MS 做迟滞判定
 *   - 状态翻转时:
 *       OK  -> LOSS : 保存 current_stage 到 s_stage_resume, 切到 PAUSE, 立即 chassis_ctrl_stop()
 *       LOSS-> OK   : current_stage 还原为 s_stage_resume, 状态机自然续跑
 *
 * stage_pause_on_link_loss_handler():
 *   PAUSE 状态下不做任何业务逻辑, 只是周期性确保电机维持在停车状态
 *   (chassis_ctrl_stop 已是幂等, 但为避免反复清 PID 积分, 这里只在
 *    LOSS 触发瞬间调用一次, handler 内不再重复调.)
 * ================================================================== */

/* ==========================================================================
 *  § 5. 外部查询 API + 链路守护 + 越界检测 + 任务主入口
 * ========================================================================== */

uint8 Game_Link_Is_Alive(void)
{
    return s_link_alive;
}

/* ==================================================================
 * 【P0-8】对外查询: 比赛失败原因
 * ================================================================== */
GameFailureReason_e Game_Get_Failure_Reason(void)
{
    return s_failure_reason;
}

/*
 * 越界检测 — 主循环侧调用, 仅在以下条件满足时 *判定+触发*:
 *   1) 链路在线 (避免 PAUSE 期间陈旧位姿误触发)
 *   2) 当前不在 WAIT_START / PAUSE_ON_LINK_LOSS / DONE
 *   3) 尚未失败过 (s_failure_reason==NONE)
 * 一旦触发: 立即 chassis_ctrl_stop() + 切 STAGE_DONE + 锁失败原因
 */
static void check_out_of_bounds(void)
{
    if (s_failure_reason != GAME_FAIL_NONE) {
        return;     /* 已失败, 状态机锁死在 DONE, 不重复判 */
    }
    if (!s_link_alive) {
        return;     /* PAUSE 优先, 链路掉线期间不判 OOB */
    }
    if (current_stage == STAGE_WAIT_START ||
        current_stage == STAGE_PAUSE_ON_LINK_LOSS ||
        current_stage == STAGE_DONE) {
        return;
    }

    if (chassis_zone_is_out_of_bounds()) {
        s_failure_reason = GAME_FAIL_OUT_OF_BOUNDS;
        chassis_ctrl_stop();
        is_navigating = 0U;
        goto_stage(STAGE_DONE);
    }
}

static void update_link_state(void)
{
    /* 32 位字段在 M7 上读取原子, 无需临界区                                       */
    uint32 last_ok = g_link_last_hb_ms;
    uint32 now_ms  = app_link_get_ms();    /* 与 g_link_last_hb_ms 同源时基, 步长一致  */
    uint32 silence_ms = (now_ms >= last_ok) ? (now_ms - last_ok) : 0U;

    if (!s_link_ever_alive) {
        /* 上电后还没收到过任何帧: 不进入 LOSS 状态, 让用户看到的是 WAIT_START */
        if (last_ok != 0U) {
            s_link_ever_alive  = 1U;
            s_link_alive       = 1U;
        }
        return;
    }

    if (s_link_alive) {
        /* 在线 -> 检查是否需要触发 LOSS                                          */
        if (silence_ms > LINK_LOSS_MS) {
            s_link_alive = 0U;
            if (current_stage != STAGE_PAUSE_ON_LINK_LOSS) {
                s_stage_resume = current_stage;     /* 保存恢复点                  */
                current_stage  = STAGE_PAUSE_ON_LINK_LOSS;
                chassis_ctrl_stop();                /* 立即刹停 (force_stop)        */
                is_navigating = 0U;
            }
        }
    } else {
        /* 离线 -> 检查是否恢复 (用 LINK_OK_MS 做迟滞)                            */
        if (silence_ms < LINK_OK_MS) {
            s_link_alive = 1U;
            if (current_stage == STAGE_PAUSE_ON_LINK_LOSS) {
                /* 安全策略: 链路恢复后强制重新识别地图, 避免基于陈旧地图直接执行  */
                current_stage = STAGE_RECOGNIZE_MAP;
                /* s_stage_resume 已不再使用, 但保留供调试观察 */
                (void)s_stage_resume;
            }
        }
    }
}

static void stage_pause_on_link_loss_handler(void)
{
    /* 视觉链路掉线期间维持静止, 不读 g_game_map, 不下发新目标.
     * 链路恢复由 update_link_state() 自动切回 STAGE_RECOGNIZE_MAP.
     * 此处刻意保持空, 避免反复调用 chassis_ctrl_stop() 把 PID 积分清得过频.
     */
}

void Game_Logic_Task_Run(void)
{
    /* P0-2: 链路监控总闸 — 必须先于状态分发                                      */
    update_link_state();

    /* P0-8: 几何判定 tick (速度估算 / 静止累计 / OOB 滞回), 必须先于 check_out_of_bounds */
    chassis_zone_tick();

    /* P0-8: 越界总闸 — 在链路监控之后, 在状态分发之前. 触发即锁 STAGE_DONE. */
    check_out_of_bounds();

    /* P0-3: 链路在线时刷新 g_game_map 私有快照 (seq-lock 拷贝).
     *       链路 LOSS 期间冻结上次快照, 配合 P0-2 恢复策略 (强制 RECOGNIZE_MAP) 自洽.
     *       上电首帧到达前 s_link_alive=0, g_game_map 维持 BSS 0 = MAP_EMPTY, 业务侧无副作用. */
    if (s_link_alive)
    {
        app_link_get_map_snapshot(g_game_map);
    }

    /* OpenART 低频位姿融合 + 一致性监控:
     * 必须在 sync_player_pos 之前把视觉触发的硬重定位写回 odom, 否则本拍的 g_player_pos 仍是漂移值。
     * - app_vision_fusion_task           : 运动中连续软融合 (默认编译为空)。
     * - app_vision_fusion_consistency_tick: 大幅打滑/搬车时硬重定位 (安全网)。
     * 链路掉线或处于 STAGE_DONE/PAUSE 时, 全部"不允许"，避免基于陈旧或冻结状态做判定。
     */
    {
        uint8 allow_continuous = (uint8)((s_link_alive != 0U) && (current_stage != STAGE_DONE));
        uint8 allow_consistency = (uint8)((s_link_alive != 0U) &&
                                          (current_stage != STAGE_DONE) &&
                                          (current_stage != STAGE_PAUSE_ON_LINK_LOSS));
        app_vision_fusion_task(allow_continuous);
        app_vision_fusion_consistency_tick(allow_consistency);
    }

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

        case STAGE_PAUSE_ON_LINK_LOSS:
            stage_pause_on_link_loss_handler();
            break;

        default:
            reset_exec_context();
            goto_stage(STAGE_WAIT_START);
            break;
    }
}
