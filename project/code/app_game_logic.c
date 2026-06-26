#include "app_game_logic.h"
#include "app_link.h"      /* P0-2: 读取 OpenART1/2 链路时戳; P0-3: 拷贝 seq-lock 地图快照 */
#include "app_vision_fusion.h"
#include "app_recognize.h"

/*
 *  @owner  rt1064-main
 *  @periph none                  状态机编排器，通过 app_link/recognize/solver/ctrl 间接驱动
 *
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

typedef enum
{
    LINK_PAUSE_NONE = 0,
    LINK_PAUSE_MAP,
    LINK_PAUSE_CLASS
} LinkPauseReason_e;

static LinkPauseReason_e s_link_pause_reason = LINK_PAUSE_NONE;

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
 *               (设置后视觉端发的所有更新会被忽略, 直到清除)。
 * O8.2: 所有置位/清零统一经 map_snapshot_freeze()/map_snapshot_release() 两函数,
 *       完整的冻结/解冻生命周期点见这两个函数处的集中注释。
 * --------------------------------------------------------------- */
static uint8               s_map_freeze         = 0U;

/* ----- 【B3b】航点执行 watchdog ------------------------------------
 * 单航点最长允许执行时长下限 = 5s @ 5ms/tick = 1000 tick.
 * 超时后先重发一次 MOVE_TO; 第二次仍超时切 STAGE_DEADLOCK_RESET.
 *
 * O8.3: 固定 5s 对"绕大圈的长航段"易误触发。改为按本段曼哈顿格距线性放宽:
 *   limit = max(WAYPOINT_TIMEOUT_TICKS, 段格距 × WAYPOINT_TICKS_PER_CELL)
 * 刻意保留 5s 作为下限(不缩短既有短航段的超时, 避免引入新的误超时),
 * 只对长航段在其之上加时。每次派发航点时按当前 g_player_pos→目标格重算。
 * --------------------------------------------------------------- */
#define WAYPOINT_TIMEOUT_TICKS         (1000U)
#define WAYPOINT_TICKS_PER_CELL        (300U)    /* 每格放宽 1.5s @5ms */
static uint16              s_wp_timeout_ticks   = 0U;
static uint16              s_wp_timeout_limit   = WAYPOINT_TIMEOUT_TICKS;
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
#define LAUNCH_MAP_STABLE_REQUIRED_FRAMES  (5U)
#define LAUNCH_MAP_STABLE_TIMEOUT_TICKS    (300U)
static uint8               s_launch_drive_issued = 0U;
static uint16              s_launch_drive_ticks  = 0U;
static uint8               s_launch_retry_count  = 0U;
static uint8               s_launch_map_candidate_valid = 0U;
static uint8               s_launch_map_stable_count = 0U;
static uint16              s_launch_map_stable_ticks = 0U;
static uint32              s_launch_map_last_frame_id = 0U;
static uint8               s_launch_map_candidate[MAP_ROWS][MAP_COLS];
static uint8               s_launch_map_observed[MAP_ROWS][MAP_COLS];

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

static void reset_launch_map_stability(void)
{
    s_launch_map_candidate_valid = 0U;
    s_launch_map_stable_count = 0U;
    s_launch_map_stable_ticks = 0U;
    s_launch_map_last_frame_id = g_link_map_frame_id;
}

static uint8 maps_equal(const uint8 a[MAP_ROWS][MAP_COLS],
                        const uint8 b[MAP_ROWS][MAP_COLS])
{
    uint8 r;
    uint8 c;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (a[r][c] != b[r][c]) {
                return 0U;
            }
        }
    }
    return 1U;
}

static void copy_map(uint8 dst[MAP_ROWS][MAP_COLS],
                     const uint8 src[MAP_ROWS][MAP_COLS])
{
    uint8 r;
    uint8 c;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            dst[r][c] = src[r][c];
        }
    }
}

static uint8 launch_map_is_usable(const uint8 map[MAP_ROWS][MAP_COLS])
{
    uint8 r;
    uint8 c;
    uint8 has_wall = 0U;
    uint8 has_box = 0U;
    uint8 has_target = 0U;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (map[r][c] == MAP_WALL) {
                has_wall = 1U;
            } else if (map[r][c] == MAP_BOX) {
                has_box = 1U;
            } else if (map[r][c] == MAP_TARGET) {
                has_target = 1U;
            }
        }
    }

    return (uint8)((has_wall != 0U) && (has_box != 0U) && (has_target != 0U));
}

static uint8 launch_map_stability_tick(void)
{
    uint32 frame_id = g_link_map_frame_id;

    if (frame_id == 0U) {
        return 0U;
    }

    if (s_launch_map_stable_ticks < LAUNCH_MAP_STABLE_TIMEOUT_TICKS) {
        s_launch_map_stable_ticks++;
    } else {
        reset_launch_map_stability();
        return 0U;
    }

    if (frame_id == s_launch_map_last_frame_id) {
        return 0U;
    }
    s_launch_map_last_frame_id = frame_id;

    app_link_get_map_snapshot(s_launch_map_observed);
    if (launch_map_is_usable(s_launch_map_observed) == 0U) {
        reset_launch_map_stability();
        return 0U;
    }

    if (s_launch_map_candidate_valid == 0U) {
        copy_map(s_launch_map_candidate, s_launch_map_observed);
        s_launch_map_candidate_valid = 1U;
        s_launch_map_stable_count = 1U;
        s_launch_map_stable_ticks = 0U;
        return 0U;
    }

    if (maps_equal(s_launch_map_candidate, s_launch_map_observed) != 0U) {
        if (s_launch_map_stable_count < 255U) {
            s_launch_map_stable_count++;
        }
    } else {
        copy_map(s_launch_map_candidate, s_launch_map_observed);
        s_launch_map_stable_count = 1U;
        s_launch_map_stable_ticks = 0U;
    }

    if (s_launch_map_stable_count >= LAUNCH_MAP_STABLE_REQUIRED_FRAMES) {
        copy_map(g_game_map, s_launch_map_candidate);
        return 1U;
    }
    return 0U;
}

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
    s_wp_timeout_limit = WAYPOINT_TIMEOUT_TICKS;
    s_wp_retry_count   = 0U;
    s_wait_phase0_ticks = 0U;       /* Issue A: WAIT_START phase 0 计时跨入口清零 */
    s_launch_drive_issued = 0U;     /* 自动发车横移: 跨入口清零 */
    s_launch_drive_ticks  = 0U;
    s_launch_retry_count  = 0U;
    reset_launch_map_stability();
}

static void goto_stage(GameStage_e next)
{
    current_stage = next;
}

/* O8.2: 地图快照冻结开关的集中入口。
 * 所有对 s_map_freeze 的置位/清零都经由这两个函数, 便于:
 *   - 用函数名一次检索出全部生命周期点 (降低新增 stage 时漏配的风险);
 *   - 把"为何冻结/解冻"的语义集中在此处记录。
 * 注意: freeze 取值依赖运行时条件 (识别是否改图 App_Recognize_Map_Changed、
 * 炸弹爆破后对 PLAN 那一拍的单拍保护), 不能化简为"纯 stage→freeze 静态表"
 * (那样会改变上述条件与时序语义), 故此处只做集中封装, 不改变任何时序。
 *
 * 冻结 (hold, freeze=1) 点: 发车后/链路恢复进 RECOGNIZE 前、识别完成且地图被清障改动、
 *                            炸弹爆破后保护 PLAN_PATH 当拍快照;
 * 解冻 (release, freeze=0) 点: WAIT_START 入口、识别失败、PLAN 完成本拍、
 *                              DEADLOCK_RESET 完成、DONE。 */
static void map_snapshot_freeze(void)
{
    s_map_freeze = 1U;
}

static void map_snapshot_release(void)
{
    s_map_freeze = 0U;
}

/** 进入新识别轮次前废弃旧的箱子到目标映射。 */
static void clear_box_target_mapping(void)
{
    memset(g_box_to_target, 0, sizeof(g_box_to_target));
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

/* O8.3: 按"当前格 → 目标航点格"的曼哈顿距离给出本段 watchdog 超时上限.
 * 维持 5s 下限(不缩短既有短航段超时), 仅对长航段线性放宽。 */
static uint16 waypoint_timeout_limit(Point_t target)
{
    int16 dx = (int16)target.x - (int16)g_player_pos.x;
    int16 dy = (int16)target.y - (int16)g_player_pos.y;
    uint32 limit;

    if (dx < 0) dx = (int16)-dx;
    if (dy < 0) dy = (int16)-dy;
    limit = (uint32)((uint16)(dx + dy)) * (uint32)WAYPOINT_TICKS_PER_CELL;

    if (limit < (uint32)WAYPOINT_TIMEOUT_TICKS) {
        return WAYPOINT_TIMEOUT_TICKS;
    }
    return (limit > 0xFFFFUL) ? 0xFFFFU : (uint16)limit;
}

/* 派发一个航点: 下发目标 + 置导航中 + 复位 watchdog 计数并按段长重算超时上限。 */
static void dispatch_waypoint(const SokoWaypointPath_t *wp, uint16 idx)
{
    HAL_CHASSIS_MOVE_TO(wp->points[idx].x, wp->points[idx].y);
    is_navigating = 1;
    s_wp_timeout_ticks = 0U;
    s_wp_timeout_limit = waypoint_timeout_limit(wp->points[idx]);
}

static uint8 exec_waypoints_common(const SokoWaypointPath_t *wp, uint16 *wp_idx)
{
    if (!is_navigating) {
        if (*wp_idx < wp->count) {
            dispatch_waypoint(wp, *wp_idx);   /* B3b: 派发新航点, watchdog 重置+按段长定上限 */
        }
        return 0;
    }

    /* B3b: 航点 watchdog — 超时(随段长放宽)先重发, 再超时切 DEADLOCK_RESET */
    s_wp_timeout_ticks++;
    if (!chassis_nav_arrived_for_waypoint()) {
        if (s_wp_timeout_ticks > s_wp_timeout_limit) {
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
        dispatch_waypoint(wp, *wp_idx);
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
    static uint8 reach[MAP_ROWS][MAP_COLS];

    if (!blocked_target) return 0;

    blocked_target->x = -1;
    blocked_target->y = -1;

    if (!Algo_Nav_BFS_Flood(g_game_map, g_player_pos, reach, 0)) return 0;

    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_TARGET) {
                Point_t tp = {c, r};
                if (!Algo_Nav_Is_Reachable(reach, tp)) {
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
    map_snapshot_release();

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
            reset_launch_map_stability();
            s_wait_start_phase  = 1U;
            return;
        }
        if (!chassis_nav_arrived_for_waypoint()) return;

        is_navigating = 0;
        s_wait_phase0_ticks = 0U;
        s_launch_drive_issued = 0U;
        s_launch_drive_ticks  = 0U;
        reset_launch_map_stability();
        s_wait_start_phase  = 1U;
        return;
    }

    /* phase 1: 自动横移驶出发车区 (主动发车) */

    /* 已完全离开发车区 → 发车成功, 进入识别 */
    if (chassis_zone_is_fully_outside_launch(s_default_launch_zone)) {
        chassis_ctrl_stop();
        if (launch_map_stability_tick() == 0U) {
            return;
        }
        s_launch_drive_issued = 0U;
        s_launch_drive_ticks  = 0U;
        s_wait_start_phase = 0U;     /* 重置子相位, 供后续 LEVEL_JUDGE 复用 */
        map_snapshot_freeze();       /* B12: 进 RECOGNIZE 前锁定地图, 防 s_items[] 错位 */
        clear_box_target_mapping();
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
            /* 清障推箱后先保留主控动态地图, 直到 PLAN_PATH 本拍完成解算。
             * 否则视觉端的一帧旧地图可能在解算前把移动结果覆盖掉。 */
            if (App_Recognize_Map_Changed()) {
                map_snapshot_freeze();
            } else {
                map_snapshot_release();
            }
            reset_exec_context();
            goto_stage(STAGE_PLAN_PATH);
            return;
        case APP_RECOG_FAIL:
        default:
            map_snapshot_release();        /* B12: 识别失败也解冻 */
            reset_exec_context();
            goto_stage(STAGE_DEADLOCK_RESET);
            return;
    }
}

/* O8.1: 破局炸弹决策链 (由 stage_plan_handler 内联链抽出, 逐分支等价)。
 * 保持原有优先级与"死局短路"语义:
 *   1) 角落/冻结死局: 仅尝试"死箱最近目标"这一种破局, 失败即放弃 (返回 0 → 调用方进死局复位),
 *      刻意不再尝试策略 2/3 —— 与历史控制流一致;
 *   2) 非死局: 先试"玩家走不到的目标", 再试"有炸弹时的通用破局";
 *   任一成功 → 返回 1 (炸弹段已就绪)；全部失败 → 返回 0。
 * 说明: 未采用方案 O8.1 的函数指针表 —— 其统一"逐项回退"语义会抹掉上面的死局短路,
 *       改变行为; 这里以单函数提取达到"消嵌套/集中决策"的同等可读性目标且零行为变更。 */
static uint8 try_build_breakout_bomb(void)
{
    Point_t dead_box = { -1, -1 };
    Point_t blocked_target;

    if (Sokoban_Is_Deadlock(g_game_map, &dead_box)) {
        blocked_target = choose_nearest_target(dead_box);
        return (uint8)((blocked_target.x >= 0) && build_bomb_plan(blocked_target));
    }

    blocked_target.x = -1;
    blocked_target.y = -1;
    if (find_first_unreachable_target(&blocked_target) &&
        build_bomb_plan(blocked_target)) {
        return 1;
    }

    /* 通用炸弹兜底 (多炸弹关键补强):
     * 推箱规划失败, 但既无死局、目标对玩家也都"可走到" —— 典型成因是
     * 炸弹/内墙堵在 *箱子* 的推进通道上 (find_first_unreachable_target 只看
     * 玩家→目标可达, 看不到箱子被堵)。此时以"最大化破局收益"为目标再尝试炸墙开路;
     * Sokoban_Plan_Bomb 内部会确保选中的墙确有炸弹可推, 找不到则失败。 */
    if (map_has_bomb()) {
        Point_t no_target = { -1, -1 };
        if (build_bomb_plan(no_target)) {
            return 1;
        }
    }
    return 0;
}

static void stage_plan_handler(void)
{
    if (g_soko_exec_init) {
        return;
    }

    g_soko_exec_init = 1;
    /* 当前 handler 内同步完成规划; 从下一拍开始可恢复视觉地图刷新。 */
    map_snapshot_release();

    if (get_map_box_count() == 0) {
        goto_stage(STAGE_LEVEL_JUDGE);
        return;
    }

    if (build_push_box_plan()) {
        goto_stage(STAGE_EXECUTE_ACTION);
        return;
    }

    if (try_build_breakout_bomb()) {
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
            g_game_map[g_bomb_pos.y][g_bomb_pos.x]           = MAP_EMPTY;   /* 炸弹本体消失 */
            /* Issue D: 显式清 wall_pos —— Apply_Bomb_Explosion 仅处理 WALL,
             * 若视觉端在推炸弹过程中把 wall_pos 改写成 BOMB(被推到位时), 上面只清 WALL→EMPTY,
             * BOMB 残留, 必须再补一次. 幂等. */
            g_game_map[g_bomb_wall_pos.y][g_bomb_wall_pos.x] = MAP_EMPTY;
            /* B1: 主控本地权威, 锁定地图; 直到本关结束(WAIT_START / DONE / DEADLOCK_RESET) 才解冻.
             * 视觉端不一定在 1 帧内同步爆炸结果, 防 g_game_map 被陈旧帧覆盖. */
            map_snapshot_freeze();
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
    map_snapshot_release();      /* B12: DEADLOCK 复位 → 解冻地图, 视觉端权威 */
    App_Recognize_Reset();       /* 死局重置后重新跑识别 tour */
    goto_stage(STAGE_WAIT_START);
}

static void stage_done_handler(void)
{
    /* 比赛流程完成，维持静止即可。 */
    map_snapshot_release();      /* B12: 比赛结束 → 解冻地图 */
}

/* ==================================================================
 * 【P0-2】视觉链路监控 + 超时回退
 * ----------------------------------------------------------------
 * update_link_state():
 *   每个调度 tick 在 Game_Logic_Task_Run 入口被调用一次
 *   - OpenART1 MAP 链路是全局必需链路
 *   - OpenART2 BOX_CLASS 链路只在需要分类识别的 RECOGNIZE_MAP 阶段必需
 *   - 用 LINK_LOSS_MS / LINK_OK_MS 做迟滞判定
 *   - 状态翻转时:
 *       OK  -> LOSS : 保存 current_stage 到 s_stage_resume, 切到 PAUSE, 立即 chassis_ctrl_stop()
 *       LOSS-> OK   : 重新进入 RECOGNIZE_MAP, 避免用掉线前的陈旧识别结果继续执行
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

static uint32 link_silence_ms(uint32 now_ms, uint32 last_ms)
{
    if (last_ms == 0U)
    {
        return 0xFFFFFFFFUL;
    }
    return (now_ms >= last_ms) ? (now_ms - last_ms) : 0U;
}

static uint8 recognize_stage_needs_class_link(void)
{
    if (current_stage != STAGE_RECOGNIZE_MAP)
    {
        return 0U;
    }
    if ((g_current_level <= 1U) && (map_has_bomb() == 0U))
    {
        return 0U;
    }
    return 1U;
}

static void enter_link_pause(LinkPauseReason_e reason)
{
    s_link_alive = 0U;
    s_link_pause_reason = reason;
    if (current_stage != STAGE_PAUSE_ON_LINK_LOSS)
    {
        s_stage_resume = current_stage;
        current_stage  = STAGE_PAUSE_ON_LINK_LOSS;
        chassis_ctrl_stop();
        is_navigating = 0U;
    }
}

static void recover_from_link_pause(void)
{
    s_link_alive = 1U;
    s_link_pause_reason = LINK_PAUSE_NONE;
    if (current_stage == STAGE_PAUSE_ON_LINK_LOSS)
    {
        chassis_ctrl_stop();
        is_navigating  = 0U;
        map_snapshot_freeze();
        clear_box_target_mapping();
        current_stage  = STAGE_RECOGNIZE_MAP;
        App_Recognize_Reset();
        (void)s_stage_resume;
    }
}

static void update_link_state(void)
{
    uint32 now_ms = app_link_get_ms();
    uint32 map_silence_ms = link_silence_ms(now_ms, g_link_last_map_link_ms);
    uint32 class_silence_ms = link_silence_ms(now_ms, g_link_last_class_link_ms);
    uint8 map_seen = (g_link_last_map_link_ms != 0U) ? 1U : 0U;
    uint8 class_seen = (g_link_last_class_link_ms != 0U) ? 1U : 0U;
    uint8 map_loss = (uint8)((map_seen != 0U) && (map_silence_ms > LINK_LOSS_MS));
    uint8 map_recovered = (uint8)((map_seen != 0U) && (map_silence_ms < LINK_OK_MS));
    uint8 class_loss = (uint8)(((class_seen == 0U) || (class_silence_ms > LINK_LOSS_MS)) &&
                               (recognize_stage_needs_class_link() != 0U));
    uint8 class_recovered = (uint8)((class_seen != 0U) && (class_silence_ms < LINK_OK_MS));

    if (!s_link_ever_alive) {
        /* OpenART1/MAP is the global map authority; class-only traffic does not start the game link. */
        if (map_seen != 0U) {
            s_link_ever_alive  = 1U;
            s_link_alive       = 1U;
            s_link_pause_reason = LINK_PAUSE_NONE;
        }
        return;
    }

    if (s_link_alive) {
        if (map_loss != 0U) {
            enter_link_pause(LINK_PAUSE_MAP);
        }
        else if (class_loss != 0U) {
            enter_link_pause(LINK_PAUSE_CLASS);
        }
    } else {
        if ((s_link_pause_reason == LINK_PAUSE_CLASS) && (map_loss != 0U)) {
            s_link_pause_reason = LINK_PAUSE_MAP;
            return;
        }

        if ((s_link_pause_reason == LINK_PAUSE_MAP) && (map_recovered != 0U)) {
            recover_from_link_pause();
        }
        else if ((s_link_pause_reason == LINK_PAUSE_CLASS) &&
                 (map_loss == 0U) &&
                 (class_recovered != 0U)) {
            recover_from_link_pause();
        }
        else if ((s_link_pause_reason == LINK_PAUSE_NONE) && (map_recovered != 0U)) {
            recover_from_link_pause();
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
