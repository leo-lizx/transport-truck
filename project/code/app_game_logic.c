#include "app_game_logic.h"
#include "app_link.h"      /* P0-2: 读取 g_link_last_hb_ms 判断链路是否在线; P0-3: 拷贝 seq-lock 地图快照 */
#include "app_vision_fusion.h"
#include "app_recognize.h"

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

/* ----- 【B1+B12】地图快照冻结标志 ----------------------------------
 * s_map_freeze: 1 = 主循环入口不再 app_link_get_map_snapshot 覆盖 g_game_map
 * 触发场景:
 *   1) 炸弹爆炸后 (主控直接 Sokoban_Apply_Bomb_Explosion 修改地图,
 *      视觉端不一定能在 1 帧内同步, 防止陈旧帧覆盖)
 *   2) RECOGNIZE_MAP 识别 tour 期间 (s_items[] 已按当时地图缓存,
 *      期间地图变更会导致访问错误格)
 * 清除场景:
 *   1) STAGE_WAIT_START 入口 (新一关或复位, 兜底)
 *   2) STAGE_DONE 入口 (比赛结束)
 *   3) RECOGNIZE_MAP 结束 (DONE_OK / DONE_NO_NEED / FAIL 都清除)
 *   4) DEADLOCK_RESET 完成 (重新开始, 视觉权威)
 * 注意: 设置后视觉端发的所有更新会被忽略, 直到清除.
 * --------------------------------------------------------------- */
static uint8               s_map_freeze         = 0U;

/* ----- 【B3b】航点执行 watchdog ------------------------------------
 * 单航点最长允许执行时长 = 5s @ 5ms/tick = 1000 tick.
 * 超时后先重发一次 MOVE_TO; 第二次仍超时切 STAGE_DEADLOCK_RESET.
 * --------------------------------------------------------------- */
#define WAYPOINT_TIMEOUT_TICKS         (1000U)
static uint16              s_wp_timeout_ticks   = 0U;
static uint8               s_wp_retry_count     = 0U;

/* ----- 【B8】WAIT_START phase 0 自动复位超时 -----------------------
 * 10s 仍到不了发车区 → 跳过自动复位进 phase 1, 让操作员手动放车.
 * --------------------------------------------------------------- */
#define WAIT_START_PHASE0_TIMEOUT_TICKS  (2000U)
static uint16              s_wait_phase0_ticks  = 0U;

/* ----- 自动发车横移 (贴墙摆位 → 单轴驶出发车区) --------------------
 * 摆位约定: 左发车区车左侧贴左墙黄线 / 右发车区车右侧贴右墙黄线.
 * phase 1 由被动"等人推车"改为主动朝场内横移, 横移到位/几何判定
 * "完全离开发车区"即停, 进入识别. 全程锁 yaw, 仅单轴 X 平移.
 *   LAUNCH_EXIT_MARGIN_M       : 目标点超出"完全离开"阈值的余量
 *   LAUNCH_DRIVE_TIMEOUT_TICKS : 4s @5ms 横移超时兜底 → 转 DEADLOCK_RESET
 * --------------------------------------------------------------- */
#define LAUNCH_EXIT_MARGIN_M         (0.02f)
#define LAUNCH_DRIVE_TIMEOUT_TICKS   (800U)
static uint8               s_launch_drive_issued = 0U;
static uint16              s_launch_drive_ticks  = 0U;
static uint8               s_launch_retry_count  = 0U;

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
    s_wp_timeout_ticks = 0U;        /* B3b: 航点 watchdog 计数清零 */
    s_wp_retry_count   = 0U;
    s_wait_phase0_ticks = 0U;       /* Issue A: WAIT_START phase 0 计时跨入口清零 */
    s_launch_drive_issued = 0U;     /* 自动发车横移: 跨入口清零 */
    s_launch_drive_ticks  = 0U;
    s_launch_retry_count  = 0U;
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
    /* B10: 跨格滞回 — 防 odom 噪声让 g_player_pos 在两格间反复跳动.
     * 仅当米坐标距当前格中心 > 70% 步长时才允许跨格. */
    int8 nx = (int8)chassis_m_to_grid_x(pose.x_m);
    int8 ny = (int8)chassis_m_to_grid_y(pose.y_m);
    if (nx != g_player_pos.x) {
        float center_x = chassis_grid_x_to_m((uint8)g_player_pos.x);
        float dx_abs = pose.x_m - center_x;
        if (dx_abs < 0.0f) dx_abs = -dx_abs;
        if (dx_abs > 0.70f * CHASSIS_GRID_STEP_X_M) {
            g_player_pos.x = nx;
        }
    }
    if (ny != g_player_pos.y) {
        float center_y = chassis_grid_y_to_m((uint8)g_player_pos.y);
        float dy_abs = pose.y_m - center_y;
        if (dy_abs < 0.0f) dy_abs = -dy_abs;
        if (dy_abs > 0.70f * CHASSIS_GRID_STEP_Y_M) {
            g_player_pos.y = ny;
        }
    }
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
            s_wp_timeout_ticks = 0U;        /* B3b: 派发新航点, watchdog 重置 */
        }
        return 0;
    }

    /* B3b: 航点 watchdog — 超时 5s 先重发, 再超时切 DEADLOCK_RESET */
    s_wp_timeout_ticks++;
    if (!chassis_nav_arrived_for_waypoint()) {
        if (s_wp_timeout_ticks > WAYPOINT_TIMEOUT_TICKS) {
            if (s_wp_retry_count == 0U) {
                /* 第 1 次超时: 刹停后重发同一航点 (可能是 Snap 表决卡住) */
                ++s_wp_retry_count;
                chassis_ctrl_stop();
                is_navigating = 0U;
                s_wp_timeout_ticks = 0U;
                return 0;
            }
            /* 第 2 次仍超时 → 进入死局复位流程 */
            s_wp_retry_count   = 0U;
            s_wp_timeout_ticks = 0U;
            chassis_ctrl_stop();
            is_navigating = 0U;
            goto_stage(STAGE_DEADLOCK_RESET);
            return 0;
        }
        return 0;
    }

    is_navigating = 0;
    s_wp_retry_count   = 0U;       /* 到位 → watchdog 全部清零 */
    s_wp_timeout_ticks = 0U;
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
            g_player_pos,                              /* B5: 用实时格而非 BFS 预测格 */
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
    /* 多炸弹联合规划: 一次性给出 (炸弹, 墙体, 推炸弹动作序列).
     * 取代旧的 "固定推扫描序第一颗炸弹 + 墙体独立打分" 三步式, 后者在
     * 多炸弹时常因 "最优墙体的第一颗炸弹推不过去" 而误判无解。 */
    if (!Sokoban_Plan_Bomb(g_game_map, g_player_pos, blocked_target,
                           &g_bomb_pos, &g_bomb_wall_pos,
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

    /* B2: 第 1 关 = 任意箱→任意目标 (Stage1 贪心)
     * 第 2 关 = 必须按数字配对 (Stage2, 用 g_box_to_target[])
     * 第 3 关 = 含炸弹 (Stage2 + 炸弹辅助)
     * 判据用 g_current_level 而非 map_has_bomb(): 第 2 关无炸弹也必须按映射配对.
     */
    if (g_current_level >= 2U) {
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

/**
 * 计算自动发车横移的目标车体中心 X (米).
 *   左发车区: 朝右(+X)移到 "发车区右沿 + R + 余量", 刚好让外接圆脱离矩形.
 *   右发车区: 朝左(-X)移到 "发车区左沿 - R - 余量", 镜像对称.
 * 实际停车以 chassis_zone_is_fully_outside_launch() 几何判定为准 (与车宽无关),
 * 此目标点只是给位置环一个"够远"的单轴终点.
 */
static float launch_drive_target_x_m(LaunchZone_e zone)
{
    float margin = CHASSIS_BODY_RADIUS_M + LAUNCH_EXIT_MARGIN_M;
    if (zone == LAUNCH_ZONE_RIGHT) {
        /* 右发车区左沿 = W - 发车区宽; 目标在其左侧 margin 处 ≈ 2.705m */
        return (CHASSIS_MAP_WIDTH_M - CHASSIS_LAUNCH_ZONE_W_M) - margin;
    }
    /* 左发车区右沿 = 发车区宽; 目标在其右侧 margin 处 ≈ 0.495m */
    return CHASSIS_LAUNCH_ZONE_W_M + margin;
}

static void stage_wait_start_handler(void)
{
    /* 【P0-8 + 自动发车】WAIT_START 两段式:
     *   phase 0: 主控主动 MOVE_TO 起点 (车被人放偏时复位到发车区中心)
     *   phase 1: 朝场内单轴横移, 自动驶出发车区 → 进入 RECOGNIZE_MAP
     *
     * 摆位约定 (规则提炼 + 用户确认):
     *   左发车区: 车左侧贴发车区左侧黄线 (x=0 墙线), 发车向右 (+X)
     *   右发车区: 车右侧贴发车区右侧黄线 (x=W 墙线), 发车向左 (-X)
     * 由 s_default_launch_zone 决定方向, 逻辑左右对称.
     *
     * B1+B12: 进入 WAIT_START 总是解冻地图 (兜底), 防上一关地图冻结状态残留.
     */
    s_map_freeze = 0U;

    if (s_wait_start_phase == 0U) {
        s_wait_phase0_ticks++;          /* B8: phase 0 超时计时 */
        if (!is_navigating) {
            /* 起点按发车区对称: 左区→首列(贴左墙), 右区→末列(贴右墙) */
            uint8 start_gx = (s_default_launch_zone == LAUNCH_ZONE_RIGHT)
                           ? (uint8)CHASSIS_GRID_INNER_MAX_X
                           : (uint8)CHASSIS_START_GRID_X;
            HAL_CHASSIS_MOVE_TO(start_gx, CHASSIS_START_GRID_Y);
            is_navigating = 1;
            return;
        }
        /* B8: 10s 仍到不了 → 跳过自动复位, 让操作员手动放车 */
        if (s_wait_phase0_ticks > WAIT_START_PHASE0_TIMEOUT_TICKS) {
            chassis_ctrl_stop();
            is_navigating = 0;
            s_wait_phase0_ticks = 0U;
            s_launch_drive_issued = 0U;
            s_launch_drive_ticks  = 0U;
            s_wait_start_phase  = 1U;
            return;
        }
        if (!chassis_nav_arrived_for_waypoint()) return;

        is_navigating = 0;
        s_wait_phase0_ticks = 0U;
        s_launch_drive_issued = 0U;
        s_launch_drive_ticks  = 0U;
        s_wait_start_phase  = 1U;
        return;
    }

    /* phase 1: 自动横移驶出发车区 (主动发车) */

    /* 已完全离开发车区 → 发车成功, 进入识别 */
    if (chassis_zone_is_fully_outside_launch(s_default_launch_zone)) {
        chassis_ctrl_stop();
        s_launch_drive_issued = 0U;
        s_launch_drive_ticks  = 0U;
        s_wait_start_phase = 0U;     /* 重置子相位, 供后续 LEVEL_JUDGE 复用 */
        s_map_freeze = 1U;           /* B12: 进 RECOGNIZE 前锁定地图, 防 s_items[] 错位 */
        memset(g_box_to_target, 0, sizeof(g_box_to_target));   /* Issue C: 清陈旧映射 */
        reset_exec_context();
        App_Recognize_Reset();       /* 进入 RECOGNIZE 前清识别 tour 状态 */
        goto_stage(STAGE_RECOGNIZE_MAP);
        return;
    }

    /* 首次进入: 朝场内下发一次单轴横移目标 (锁当前 yaw, Y 不动) */
    if (!s_launch_drive_issued) {
        chassis_pose_t pose = chassis_ctrl_get_pose();
        chassis_ctrl_move_to_m(launch_drive_target_x_m(s_default_launch_zone),
                               pose.y_m, pose.yaw_deg);
        s_launch_drive_issued = 1U;
        s_launch_drive_ticks  = 0U;
        return;
    }

    /* 横移超时兜底: 重发一次; 仍超时则转死局复位 (避免卡死) */
    s_launch_drive_ticks++;
    if (s_launch_drive_ticks >= LAUNCH_DRIVE_TIMEOUT_TICKS) {
        chassis_ctrl_stop();
        s_launch_drive_issued = 0U;
        s_launch_drive_ticks  = 0U;
        if (s_launch_retry_count == 0U) {
            s_launch_retry_count = 1U;   /* 允许重发一次 */
        } else {
            s_launch_retry_count = 0U;
            s_wait_start_phase = 0U;
            reset_exec_context();
            goto_stage(STAGE_DEADLOCK_RESET);
        }
    }
}

static void stage_recognize_handler(void)
{
    /* ============================================================
     * 识别 tour 子状态机驱动 (app_recognize.c):
     *   - Stage1 简单贪心模式 (level=1 且无炸弹) → DONE_NO_NEED 直接放行
     *   - Stage2/3: 遍历每个箱子和目标的观察点, 多数票投出 class_id,
     *               配对成 box→target 映射写入 g_box_to_target[]
     *   - 不可达或视觉持续无识别 → DEADLOCK_RESET 复位重试
     * ============================================================ */
    AppRecognizeStatus_e r = App_Recognize_Tick(g_game_map, g_player_pos,
                                                map_has_bomb(),
                                                g_current_level,
                                                g_box_to_target);
    switch (r)
    {
        case APP_RECOG_RUNNING:
            return;
        case APP_RECOG_DONE_OK:
        case APP_RECOG_DONE_NO_NEED:
            s_map_freeze = 0U;             /* B12: 识别结束, 解冻地图 */
            reset_exec_context();
            goto_stage(STAGE_PLAN_PATH);
            return;
        case APP_RECOG_FAIL:
        default:
            s_map_freeze = 0U;             /* B12: 识别失败也解冻 */
            reset_exec_context();
            goto_stage(STAGE_DEADLOCK_RESET);
            return;
    }
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

    /* 通用炸弹兜底 (多炸弹关键补强):
     * 推箱规划失败, 但既无角落死局、目标对玩家也都"可走到" —— 典型成因是
     * 炸弹/内墙堵在 *箱子* 的推进通道上 (find_first_unreachable_target 只看
     * 玩家→目标可达, 看不到箱子被堵)。此时以"最大化破局收益"为目标 (无特定
     * blocked_target) 再尝试炸墙开路; Sokoban_Plan_Bomb 内部会确保选中的墙
     * 确有炸弹可推, 找不到则照常进入死局复位。 */
    if (map_has_bomb()) {
        Point_t no_target = { -1, -1 };
        if (build_bomb_plan(no_target)) {
            goto_stage(STAGE_EXECUTE_ACTION);
            return;
        }
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
            g_game_map[g_bomb_pos.y][g_bomb_pos.x]           = MAP_EMPTY;   /* 炸弹本体消失 */
            /* Issue D: 显式清 wall_pos —— Apply_Bomb_Explosion 仅处理 WALL,
             * 若视觉端在推炸弹过程中把 wall_pos 改写成 BOMB(被推到位时), 上面只清 WALL→EMPTY,
             * BOMB 残留, 必须再补一次. 幂等. */
            g_game_map[g_bomb_wall_pos.y][g_bomb_wall_pos.x] = MAP_EMPTY;
            /* B1: 主控本地权威, 锁定地图; 直到本关结束(WAIT_START / DONE / DEADLOCK_RESET) 才解冻.
             * 视觉端不一定在 1 帧内同步爆炸结果, 防 g_game_map 被陈旧帧覆盖. */
            s_map_freeze = 1U;
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
    s_map_freeze = 0U;           /* B12: DEADLOCK 复位 → 解冻地图, 视觉端权威 */
    App_Recognize_Reset();       /* 死局重置后重新跑识别 tour */
    goto_stage(STAGE_WAIT_START);
}

static void stage_done_handler(void)
{
    /* 比赛流程完成，维持静止即可。 */
    s_map_freeze = 0U;           /* B12: 比赛结束 → 解冻地图 */
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

/* ==================================================================
 * 【B17】对外查询: 识别 tour 进度 (转发自 App_Recognize_Get_Debug)
 * 用途: 菜单 / IPS / 上位机显示当前识别到第几个物体, 多数票占比等.
 * ================================================================== */
void Game_Get_Recognize_Debug(AppRecognizeDebug_t *out)
{
    App_Recognize_Get_Debug(out);
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
                /* B4: 恢复瞬间显式刹停 + 清 is_navigating, 防 5ms 窗口车按旧目标继续滑行 */
                chassis_ctrl_stop();
                is_navigating  = 0U;
                /* Issue B: 与 stage_wait_start_handler→RECOGNIZE 路径保持一致,
                 * 进 RECOGNIZE 期间冻结地图, 防 s_items[] 与新快照错位 */
                s_map_freeze   = 1U;
                /* Issue C: 清陈旧映射, 准备让 RECOGNIZE 重新写入 */
                memset(g_box_to_target, 0, sizeof(g_box_to_target));
                current_stage  = STAGE_RECOGNIZE_MAP;
                App_Recognize_Reset();          /* 链路恢复后重新跑一遍识别 tour */
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
     *       上电首帧到达前 s_link_alive=0, g_game_map 维持 BSS 0 = MAP_EMPTY, 业务侧无副作用.
     *
     * B1+B12: s_map_freeze=1 时 (识别 tour / 炸弹爆炸后) 跳过拷贝, 由主控本地权威.
     */
    if (s_link_alive && !s_map_freeze)
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
