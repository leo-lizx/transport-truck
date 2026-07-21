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
#define APP_GAME_LAUNCH_HOME_X       (1U)
#define APP_GAME_LAUNCH_HOME_Y       (5U)
#define APP_GAME_LAUNCH_EXIT_X       (1U)
#define APP_GAME_LAUNCH_EXIT_Y       (4U)

uint8 g_game_map[MAP_ROWS][MAP_COLS];
Point_t g_player_pos = {(int8)APP_GAME_LAUNCH_HOME_X, (int8)APP_GAME_LAUNCH_HOME_Y};

#define GAME_LOGIC_TASK_PERIOD_MS      (5U)
#define DEADLOCK_RESET_HOLD_MS         (3000U)
#define DEADLOCK_RESET_HOLD_TICKS      (DEADLOCK_RESET_HOLD_MS / GAME_LOGIC_TASK_PERIOD_MS)
/* 赛规: 三个关卡连续比赛。三个关卡均结束后进入 STAGE_DONE 静止收车,
 * 不再无限等待下一张地图；第三关结束并返航后进入 STAGE_DONE。
 * 推完箱子后返回发车点；死局返航并静止 3s 后结束失败关，均计入三关总数。 */
#define APP_GAME_TOTAL_LEVELS          (3U)

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
 *                       0 = 正在返回左侧发车点 (1,5)
 *                       1 = 已到 (1,5), 正在进行发车前视觉校准
 * --------------------------------------------------------------- */
static GameFailureReason_e s_failure_reason     = GAME_FAIL_NONE;
static uint8               s_wait_start_phase   = 0U;

/* ----- 【B1+B12】地图快照冻结标志 ----------------------------------
 * s_map_freeze: 1 = 主循环入口不再 app_link_get_map_snapshot 覆盖 g_game_map
 *               (设置后视觉端发的所有更新会被忽略, 直到清除)。
 * O8.2: 所有置位/清零统一经 map_snapshot_freeze()/map_snapshot_release() 两函数,
 *       完整的冻结/解冻生命周期点见这两个函数处的集中注释。
 * --------------------------------------------------------------- */
static uint8               s_map_freeze         = 0U;

/* ----- 【B3b】航点执行超时保护 -------------------------------------
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

/* ----- 自动发车 (左侧发车点 (1,5) → 上位机触发点 (1,4)) -----------
 * 发车不依赖地图；到达 (1,4) 后停车，等待上位机刷新本关地图。
 * LAUNCH_DRIVE_TIMEOUT_TICKS: 4s @5ms 超时兜底 → 转 DEADLOCK_RESET.
 * --------------------------------------------------------------- */
#define LAUNCH_DRIVE_TIMEOUT_TICKS   (800U)
#define LAUNCH_MAP_STABLE_REQUIRED_FRAMES  (5U)
#define LAUNCH_MAP_STABLE_TIMEOUT_TICKS    (300U)
/* 关间强校准兜底时限: 2s @5ms. 视觉给不出稳定表决时到时放行发车, 永不阻塞。 */
#define LAUNCH_CALIB_TIMEOUT_TICKS   (400U)
static uint8               s_launch_drive_issued = 0U;
static uint16              s_launch_drive_ticks  = 0U;
static uint8               s_launch_retry_count  = 0U;
static uint8               s_launch_calib_done   = 0U;
static uint16              s_launch_calib_ticks  = 0U;
static uint8               s_launch_map_candidate_valid = 0U;
static uint8               s_launch_map_stable_count = 0U;
static uint16              s_launch_map_stable_ticks = 0U;
static uint32              s_launch_map_last_frame_id = 0U;
static uint32              s_launch_depart_frame_id = 0U;
static uint32              s_launch_depart_map_hash = 0U;
static uint8               s_launch_depart_map_valid = 0U;
static uint8               s_launch_map_candidate[MAP_ROWS][MAP_COLS];
static uint8               s_launch_map_observed[MAP_ROWS][MAP_COLS];

/* 断链恢复栅栏。恢复期间不沿用暂停前的冻结地图。 */
static uint32              s_recovery_map_baseline_frame_id = 0U;

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

/* 通关后忽略虚拟墙/炸弹，直接向库位下发一个关键航点。 */
static SokoWaypointPath_t    g_return_waypoints;
static uint16                g_return_wp_idx = 0U;
static uint8                 g_return_path_valid = 0U;
static uint8                 s_home_snap_fresh = 0U;

static uint8                 g_box_to_target[SOKOBAN_MAX_BOXES] = {0};
static ExecMode_e            g_exec_mode = EXEC_NONE;

/* 已结束的关卡数，上电初始化时清零。 */
static uint8                 s_levels_finished = 0U;

/* ==========================================================================
 *  § 1. 执行上下文 / stage 跳转 / 地图查询工具 (全部需主循环单线程调用)
 * ========================================================================== */

/* 正式比赛固定按第一、二、三关顺序执行。已完成关数是唯一进度来源，
 * 地图内容只描述本关布局，不参与关号判断。 */
static uint8 current_level_number(void)
{
    if (s_levels_finished >= APP_GAME_TOTAL_LEVELS) {
        return APP_GAME_TOTAL_LEVELS;
    }
    return (uint8)(s_levels_finished + 1U);
}

static void mark_current_level_finished(void)
{
    if (s_levels_finished < APP_GAME_TOTAL_LEVELS) {
        s_levels_finished++;
    }
}

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

/* 仅用于区分驶出前旧图与上位机刷新后的新图；不参与地图合法性判断。 */
static uint32 map_hash(const uint8 map[MAP_ROWS][MAP_COLS])
{
    uint32 hash = 2166136261UL;
    uint8 r;
    uint8 c;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            hash ^= (uint32)map[r][c];
            hash *= 16777619UL;
        }
    }
    return hash;
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

/* 恢复时地图可能已在掉线期间完成最后一次推箱，此时箱子/目标已消失。
 * 因此恢复图只要含有围墙且多帧一致即可；后续由 LEVEL_JUDGE 确认零箱子。 */
static uint8 recovery_map_is_usable(const uint8 map[MAP_ROWS][MAP_COLS])
{
    uint8 r;
    uint8 c;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (map[r][c] == MAP_WALL) {
                return 1U;
            }
        }
    }
    return 0U;
}

static uint8 map_stability_tick(uint32 baseline_frame_id,
                                uint8 reject_depart_map,
                                uint8 require_new_level_layout)
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

    /* 只接收本次发车指令之后落地的帧。地图可能在车辆行驶到 (1,4) 的途中
     * 已完成刷新，因此帧栅栏必须取发车前值，不能在到达后重置。 */
    if ((frame_id == baseline_frame_id) ||
        (frame_id == s_launch_map_last_frame_id)) {
        return 0U;
    }
    s_launch_map_last_frame_id = frame_id;

    app_link_get_map_snapshot(s_launch_map_observed);
    if (((require_new_level_layout != 0U) &&
         (launch_map_is_usable(s_launch_map_observed) == 0U)) ||
        ((require_new_level_layout == 0U) &&
         (recovery_map_is_usable(s_launch_map_observed) == 0U))) {
        reset_launch_map_stability();
        return 0U;
    }
    if ((reject_depart_map != 0U) &&
        (s_launch_depart_map_valid != 0U) &&
        (map_hash(s_launch_map_observed) == s_launch_depart_map_hash)) {
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
    Sokoban_Stage1_Search_Cancel();
    g_soko_solution.is_solved = 0;
    g_soko_solution.total_boxes = 0;
    g_soko_waypoints.count = 0;
    g_soko_sub_idx = 0;
    g_soko_wp_idx = 0;
    g_soko_exec_init = 0;

    g_bomb_waypoints.count = 0;
    g_bomb_wp_idx = 0;
    g_return_waypoints.count = 0U;
    g_return_wp_idx = 0U;
    g_return_path_valid = 0U;
    s_home_snap_fresh = 0U;
    g_exec_mode = EXEC_NONE;
    is_navigating = 0;
    s_wp_timeout_ticks = 0U;        /* B3b: 航点超时计数清零 */
    s_wp_timeout_limit = WAYPOINT_TIMEOUT_TICKS;
    s_wp_retry_count   = 0U;
    s_wait_phase0_ticks = 0U;       /* Issue A: WAIT_START phase 0 计时跨入口清零 */
    s_wait_start_phase = 0U;
    s_launch_drive_issued = 0U;
    s_launch_drive_ticks  = 0U;
    s_launch_retry_count  = 0U;
    s_launch_calib_done   = 0U;
    s_launch_calib_ticks  = 0U;
    s_launch_depart_frame_id = 0U;
    s_launch_depart_map_hash = 0U;
    s_launch_depart_map_valid = 0U;
    reset_launch_map_stability();
}

static void goto_stage(GameStage_e next)
{
    current_stage = next;
}

static void calibrate_launch_heading(void)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();

    chassis_ctrl_set_pose(pose.x_m,
                          pose.y_m,
                          APP_GAME_LAUNCH_FACE_YAW_DEG);
}

/* O8.2: 地图快照冻结开关的集中入口。
 * 所有对 s_map_freeze 的置位/清零都经由这两个函数, 便于:
 *   - 用函数名一次检索出全部生命周期点 (降低新增 stage 时漏配的风险);
 *   - 把"为何冻结/解冻"的语义集中在此处记录。
 * 注意: freeze 取值依赖运行时条件 (识别是否改图 App_Recognize_Map_Changed、
 * 炸弹爆破后对 PLAN 那一拍的单拍保护), 不能化简为"纯 stage→freeze 静态表"
 * (那样会改变上述条件与时序语义), 故此处只做集中封装, 不改变任何时序。
 *
 * 冻结 (hold, freeze=1) 点: 驶出后新地图稳定/链路恢复进 RECOGNIZE 前、识别完成且地图被清障改动、
 *                            炸弹爆破后保护 PLAN_PATH 当拍快照;
 * 解冻 (release, freeze=0) 点: WAIT_START/LAUNCH_EXIT/WAIT_MAP_REFRESH、识别失败、PLAN 完成本拍、
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

static uint8 chassis_nav_arrived_for_waypoint(uint8 require_snap)
{
    if (chassis_ctrl_is_arrived() == 0U)
    {
        app_vision_fusion_snap_cancel();
        return 0U;
    }

    if (require_snap == 0U) {
        app_vision_fusion_snap_cancel();
        return 1U;
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

/* O8.3: 按"当前格 → 目标航点格"的曼哈顿距离给出本段超时上限.
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

static void move_to_grid_keep_current_yaw(Point_t target)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();

    /* snap 到最近 90° 倍数: 消除识别转向残留的斜角, 保证轴对齐平移精度 */
    chassis_ctrl_move_to_m(chassis_grid_x_to_m((uint8)target.x),
                           chassis_grid_y_to_m((uint8)target.y),
                           chassis_snap_yaw_to_cardinal_deg(pose.yaw_deg));
}

/* 派发一个航点: 按地图坐标移动, 保持当前车头角, 并复位超时计数。 */
static void dispatch_waypoint(const SokoWaypointPath_t *wp, uint16 idx)
{
    move_to_grid_keep_current_yaw(wp->points[idx]);
    is_navigating = 1;
    s_wp_timeout_ticks = 0U;
    s_wp_timeout_limit = waypoint_timeout_limit(wp->points[idx]);
}

static uint8 exec_waypoints_common(const SokoWaypointPath_t *wp, uint16 *wp_idx)
{
    if (!is_navigating) {
        if (*wp_idx < wp->count) {
            dispatch_waypoint(wp, *wp_idx);   /* B3b: 派发新航点, 超时计数重置+按段长定上限 */
        }
        return 0;
    }

    /* B3b: 航点超时保护 — 超时(随段长放宽)先重发, 再超时切 DEADLOCK_RESET */
    s_wp_timeout_ticks++;
    if (!chassis_nav_arrived_for_waypoint(
            (uint8)(wp->kinds[*wp_idx] == (uint8)SOKO_WP_CRITICAL))) {
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
    s_wp_retry_count   = 0U;       /* 到位 → 超时状态全部清零 */
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

        Sokoban_Seq_To_Waypoints(
            &g_soko_solution.sub_solutions[g_soko_sub_idx],
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

    Sokoban_Seq_To_Waypoints(&g_bomb_action_seq,
                             g_player_pos,
                             &g_bomb_waypoints);
    g_bomb_wp_idx = 0;
    g_exec_mode = EXEC_PUSH_BOMB;
    is_navigating = 0;
    return 1;
}

static uint8 activate_push_box_solution(void)
{
    if (!g_soko_solution.is_solved || g_soko_solution.total_boxes == 0U) {
        return 0U;
    }

    g_soko_sub_idx = 0U;
    if (Sokoban_Seq_To_Waypoints(&g_soko_solution.sub_solutions[0],
                                 g_player_pos,
                                 &g_soko_waypoints) == 0U) {
        return 0U;
    }
    g_soko_wp_idx = 0U;
    g_exec_mode = EXEC_PUSH_BOX;
    is_navigating = 0U;
    return 1U;
}

static uint8 build_push_box_plan(void)
{
    uint8 box_count = get_map_box_count();

    if (box_count == 0) {
        return 1;
    }

    /* B2: 第 1 关由 stage_plan_handler 的分时搜索分支处理；
     * 第 2 关 = 必须按数字配对 (Stage2, 用 g_box_to_target[])
     * 第 3 关 = 含炸弹 (Stage2 + 炸弹辅助)
     * 判据只使用固定比赛顺序: 第 2 关无炸弹也必须按映射配对.
     */
    if (current_level_number() < 2U ||
        !Sokoban_Solve_Stage2(g_game_map, g_player_pos,
                              g_box_to_target,
                              box_count,
                              &g_soko_solution)) {
        return 0;
    }

    return activate_push_box_solution();
}

/* ==========================================================================
 *  § 4. Stage 处理器 — WAIT_START / RECOGNIZE / PLAN / EXECUTE / COMPLETE
 *                       / DEADLOCK_RESET / DONE / PAUSE_ON_LINK_LOSS
 * ========================================================================== */

static void prepare_launch_departure(void)
{
    /* 上位机只在车到达 (1,4) 后刷新地图。发车前保存帧栅栏和旧图摘要，
     * 等图阶段据此拒绝仍在连续发送的上一关地图。 */
    s_launch_depart_frame_id = g_link_map_frame_id;
    copy_map(s_launch_map_observed, g_game_map);
    s_launch_depart_map_valid = launch_map_is_usable(s_launch_map_observed);
    s_launch_depart_map_hash = (s_launch_depart_map_valid != 0U)
                             ? map_hash(s_launch_map_observed)
                             : 0U;
    reset_launch_map_stability();
    s_launch_map_last_frame_id = s_launch_depart_frame_id;
}

static void prepare_recovery_map_wait(void)
{
    chassis_ctrl_stop();
    reset_exec_context();
    map_snapshot_release();
    clear_box_target_mapping();
    App_Recognize_Reset();

    s_recovery_map_baseline_frame_id = g_link_map_frame_id;
    reset_launch_map_stability();
    s_launch_map_last_frame_id = s_recovery_map_baseline_frame_id;
    current_stage = STAGE_WAIT_RECOVERY_MAP;
}

void Game_Logic_Init(void)
{
    chassis_ctrl_stop();
    current_stage = STAGE_WAIT_START;
    is_navigating = 0U;
    s_link_alive = 0U;
    s_link_ever_alive = 0U;
    s_stage_resume = STAGE_WAIT_START;
    s_link_pause_reason = LINK_PAUSE_NONE;
    s_failure_reason = GAME_FAIL_NONE;
    s_levels_finished = 0U;
    reset_exec_context();
    map_snapshot_release();
    clear_box_target_mapping();
    App_Recognize_Reset();
    chassis_zone_clear_oob();
    memset(g_game_map, 0, sizeof(g_game_map));
}

static void stage_wait_start_handler(void)
{
    /* 仅保留左侧发车区。phase 0 直线返回 (1,5)，phase 1 完成视觉/航向校准；
     * 地图在发车和到达 (1,4) 后的等待阶段始终解冻。
     */
    map_snapshot_release();

    if (s_wait_start_phase == 0U) {
        if (g_return_path_valid != 0U) {
            if (!exec_waypoints_common(&g_return_waypoints, &g_return_wp_idx)) return;
            g_return_path_valid = 0U;
        } else {
            s_wait_phase0_ticks++;
            if (!is_navigating) {
                HAL_CHASSIS_MOVE_TO(APP_GAME_LAUNCH_HOME_X,
                                    APP_GAME_LAUNCH_HOME_Y);
                is_navigating = 1;
                return;
            }
            /* 航点生成异常时仍直达库位；10s 未到则停车重发。 */
            if (s_wait_phase0_ticks > WAIT_START_PHASE0_TIMEOUT_TICKS) {
                chassis_ctrl_stop();
                is_navigating = 0;
                s_wait_phase0_ticks = 0U;
                return;
            }
            if (!chassis_nav_arrived_for_waypoint(1U)) return;
        }

        chassis_ctrl_stop();
        is_navigating = 0;
        s_wait_phase0_ticks = 0U;
        s_home_snap_fresh = 1U;
        if (s_levels_finished >= APP_GAME_TOTAL_LEVELS) {
            goto_stage(STAGE_DONE);       /* 第三关也先返回 (1,5) 再收车 */
            return;
        }
        s_launch_calib_done = 0U;
        s_launch_calib_ticks = 0U;
        s_wait_start_phase = 1U;
        return;
    }

    /* 关间强视觉校准 (发车前最后一步):
     *   车已静止在 (1,5)，用到站 Snap 状态机对该格做多帧表决:
     *     - 表决通过 → x/y 被拉到视觉表决格中心 (Snap 内部已写 pose);
     *     - 随后 calibrate_launch_heading() 复位航向到 180° (只改 yaw, 保留 Snap 的 x/y);
     *   有 LAUNCH_CALIB_TIMEOUT_TICKS 兜底, 视觉给不出结果也照常发车, 绝不卡死。 */
    if (s_launch_calib_done == 0U) {
        chassis_ctrl_stop();
        s_launch_calib_ticks++;
        if (s_home_snap_fresh != 0U) {
            /* phase 0 的最终关键航点刚完成 Snap，直接复用，避免重复静止表决。 */
            s_home_snap_fresh = 0U;
            calibrate_launch_heading();
            s_launch_calib_done = 1U;
            return;
        }
#if CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE
        if (s_link_alive != 0U) {
            float calib_x_m = chassis_grid_x_to_m(APP_GAME_LAUNCH_HOME_X);
            float calib_y_m = chassis_grid_y_to_m(APP_GAME_LAUNCH_HOME_Y);
            app_vision_snap_state_e st;
            app_vision_fusion_snap_request(calib_x_m, calib_y_m);
            st = app_vision_fusion_snap_state();
            if ((st != APP_VISION_SNAP_DONE)    &&
                (st != APP_VISION_SNAP_TIMEOUT) &&
                (st != APP_VISION_SNAP_REJECT)  &&
                (s_launch_calib_ticks < LAUNCH_CALIB_TIMEOUT_TICKS)) {
                return;              /* 表决进行中, 继续等 (受时限约束) */
            }
            app_vision_fusion_snap_cancel();
        }
#endif
        calibrate_launch_heading();  /* 航向复位 180°, 保留 Snap 校正后的 x/y */
        s_launch_calib_done = 1U;
        return;
    }

    reset_exec_context();
    map_snapshot_release();
    clear_box_target_mapping();
    App_Recognize_Reset();
    prepare_launch_departure();
    goto_stage(STAGE_LAUNCH_EXIT);
}

static void stage_launch_exit_handler(void)
{
    map_snapshot_release();

    if (s_launch_drive_issued == 0U) {
        chassis_ctrl_move_to_m(chassis_grid_x_to_m(APP_GAME_LAUNCH_EXIT_X),
                               chassis_grid_y_to_m(APP_GAME_LAUNCH_EXIT_Y),
                               APP_GAME_LAUNCH_FACE_YAW_DEG);
        s_launch_drive_issued = 1U;
        s_launch_drive_ticks  = 0U;
        return;
    }

    /* 赛规以上位机网格 (1,4) 为刷新触发点，因此以该航点到达为发车成功，
     * 不再用车体外接圆完全离开旧发车区几何作为硬门槛。 */
    if (chassis_nav_arrived_for_waypoint(1U)) {
        chassis_ctrl_stop();
        s_launch_drive_issued = 0U;
        s_launch_drive_ticks = 0U;
        s_launch_retry_count = 0U;
        goto_stage(STAGE_WAIT_MAP_REFRESH);
        return;
    }

    /* 发车超时兜底: 重发一次; 仍超时则转死局复位。 */
    s_launch_drive_ticks++;
    if (s_launch_drive_ticks >= LAUNCH_DRIVE_TIMEOUT_TICKS) {
        chassis_ctrl_stop();
        s_launch_drive_issued = 0U;
        s_launch_drive_ticks  = 0U;
        if (s_launch_retry_count == 0U) {
            s_launch_retry_count = 1U;   /* 允许重发一次 */
        } else {
            s_launch_retry_count = 0U;
            reset_exec_context();
            goto_stage(STAGE_DEADLOCK_RESET);
        }
    }
}

static void stage_wait_map_refresh_handler(void)
{
    /* 到达 (1,4) 后保持停车和地图解冻；只有上位机刷新出的稳定新图才可
     * 进入识别，避免用发车前旧图规划本关。 */
    map_snapshot_release();
    if (map_stability_tick(s_launch_depart_frame_id, 1U, 1U) == 0U) {
        return;
    }

    clear_box_target_mapping();
    App_Recognize_Reset();
    map_snapshot_freeze();
    goto_stage(STAGE_RECOGNIZE_MAP);
}

static void stage_wait_recovery_map_handler(void)
{
    /* 恢复阶段始终保持停车；仅在链路在线时消费新帧。 */
    if (s_link_alive == 0U) {
        return;
    }

    if (map_stability_tick(s_recovery_map_baseline_frame_id, 0U, 0U) == 0U) {
        return;
    }

    clear_box_target_mapping();
    App_Recognize_Reset();
    map_snapshot_freeze();

    /* 掉线期间若已完成最后一箱，直接进入本关完成判定。 */
    current_stage = (get_map_box_count() == 0U)
                  ? STAGE_LEVEL_JUDGE
                  : STAGE_RECOGNIZE_MAP;
}

static void stage_recognize_handler(void)
{
    /* ============================================================
     * 识别 tour 子状态机驱动 (app_recognize.c):
     *   - Stage1 无数字配对要求（有无炸弹均同）→ DONE_NO_NEED 直接放行
     *   - Stage2/3: 遍历每个箱子和目标的观察点, 多数票投出 class_id,
     *               配对成 box→target 映射写入 g_box_to_target[]
     *   - 不可达或视觉持续无识别 → DEADLOCK_RESET 复位重试
     * ============================================================ */
    AppRecognizeStatus_e r = App_Recognize_Tick(g_game_map, g_player_pos,
                                                map_has_bomb(),
                                                current_level_number(),
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
            /* fall through */
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
    if (get_map_box_count() == 0U) {
        Sokoban_Stage1_Search_Cancel();
        goto_stage(STAGE_LEVEL_JUDGE);
        return;
    }

    if (current_level_number() == 1U) {
        SokoSearchStatus_e search_status;
        Point_t home = { (int8)APP_GAME_LAUNCH_HOME_X,
                         (int8)APP_GAME_LAUNCH_HOME_Y };

        if (g_soko_exec_init == 0U) {
            g_soko_exec_init = 1U;
            if (!Sokoban_Stage1_Search_Begin(g_game_map, g_player_pos, home)) {
                map_snapshot_release();
                if (try_build_breakout_bomb()) goto_stage(STAGE_EXECUTE_ACTION);
                else goto_stage(STAGE_DEADLOCK_RESET);
                return;
            }
            /* Begin 已复制规划地图；后续分时搜索不依赖视觉快照持续冻结。 */
            map_snapshot_release();
            return;
        }

        /* 每个 5ms tick 只评估一个箱-目标候选，控制单拍最坏耗时。 */
        search_status = Sokoban_Stage1_Search_Step(1U, &g_soko_solution);
        if (search_status == SOKO_SEARCH_RUNNING) return;
        Sokoban_Stage1_Search_Cancel();
        if (search_status == SOKO_SEARCH_SOLVED && activate_push_box_solution()) {
            goto_stage(STAGE_EXECUTE_ACTION);
            return;
        }
        if (try_build_breakout_bomb()) {
            goto_stage(STAGE_EXECUTE_ACTION);
            return;
        }
        goto_stage(STAGE_DEADLOCK_RESET);
        return;
    }

    if (g_soko_exec_init) {
        return;
    }

    g_soko_exec_init = 1U;
    /* 第二、三关保留固定映射同步规划；规划完成后恢复视觉地图刷新。 */
    map_snapshot_release();
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

    /* 箱子已全部推完 = 本关成功结束。所有关卡均直线返回 (1,5)；
     * 前两关随后再驶到 (1,4) 触发新图，第三关返航后收车。 */
    chassis_ctrl_stop();
    mark_current_level_finished();

    reset_exec_context();
    {
        Point_t home = { (int8)APP_GAME_LAUNCH_HOME_X,
                         (int8)APP_GAME_LAUNCH_HOME_Y };
        if (Sokoban_Build_Return_Waypoints(g_game_map, g_player_pos,
                                           home, &g_return_waypoints)) {
            g_return_path_valid = 1U;
            g_return_wp_idx = 0U;
        }
    }
    map_snapshot_release();
    App_Recognize_Reset();
    goto_stage(STAGE_WAIT_START);
}

static void stage_deadlock_reset_handler(void)
{
    /* 【P0-8】死局重置: 改为 "在发车区内 + 静止 ≥3s" 几何判据,
     * 不再依赖 wall-clock tick, 与 chassis_zone_is_static() (PIT 5ms tick) 同源.
     * (规则: 返回发车区静止 3s = 系统重置)
     */
    if (!is_navigating) {
        HAL_CHASSIS_MOVE_TO(APP_GAME_LAUNCH_HOME_X,
                            APP_GAME_LAUNCH_HOME_Y);
        is_navigating = 1;
        return;
    }

    if (!chassis_nav_arrived_for_waypoint(1U)) return;

    is_navigating = 0;

    /* 必须 "在发车区内" + chassis_zone_is_static() (内部已含 3s 持续判定).
     * chassis_zone_is_static 由 chassis_zone_tick() 周期更新, 在 Run() 入口已调一次. */
    if (!chassis_zone_is_in_launch(LAUNCH_ZONE_LEFT)) {
        return;     /* 还未真正回到发车区, 继续等里程计/视觉拉车进入 */
    }
    if (!chassis_zone_is_static()) {
        return;     /* 还未静止满 3s */
    }

    /* 回发车区静止 3s = 本关失败流程完成；失败关也占用三关中的一关。 */
    mark_current_level_finished();

    map_snapshot_release();      /* B12: DEADLOCK 复位 → 解冻地图, 视觉端权威 */
    if (s_levels_finished < APP_GAME_TOTAL_LEVELS) {
        reset_exec_context();
        App_Recognize_Reset();
        goto_stage(STAGE_WAIT_START);
        return;
    }

    goto_stage(STAGE_DONE);
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
 *       LOSS-> OK   : WAIT_START 重新取图发车；其他业务阶段重新进入 RECOGNIZE_MAP，
 *                     避免用掉线前的陈旧识别结果继续执行
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
 *   2) 当前不在发车/返航、PAUSE_ON_LINK_LOSS、DONE
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
        current_stage == STAGE_DEADLOCK_RESET ||
        current_stage == STAGE_PAUSE_ON_LINK_LOSS ||
        current_stage == STAGE_WAIT_RECOVERY_MAP ||
        current_stage == STAGE_DONE) {
        chassis_zone_clear_oob();
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
    return (uint32)(now_ms - last_ms);
}

static uint8 recognize_stage_needs_class_link(void)
{
    if (current_stage != STAGE_RECOGNIZE_MAP)
    {
        return 0U;
    }
    return (uint8)(current_level_number() >= 2U);
}

static void enter_link_pause(LinkPauseReason_e reason)
{
    /* 返回 (1,5)、驶向 (1,4)、失败返航和最终驻停均不依赖地图内容。
     * 这些阶段只记录链路状态，不切 PAUSE，避免地图尚未刷新反而阻止发车。 */
    if ((current_stage == STAGE_WAIT_START) ||
        (current_stage == STAGE_LAUNCH_EXIT) ||
        (current_stage == STAGE_WAIT_RECOVERY_MAP) ||
        (current_stage == STAGE_DEADLOCK_RESET) ||
        (current_stage == STAGE_DONE)) {
        s_link_alive = 0U;
        s_link_pause_reason = reason;
        return;
    }

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
    if (current_stage == STAGE_WAIT_RECOVERY_MAP)
    {
        /* 恢复等待期再次掉线时，已累计的稳定帧全部作废；
         * 必须从最近一次链路恢复后重新收集。 */
        s_recovery_map_baseline_frame_id = g_link_map_frame_id;
        reset_launch_map_stability();
        s_launch_map_last_frame_id = s_recovery_map_baseline_frame_id;
        return;
    }
    if (current_stage == STAGE_PAUSE_ON_LINK_LOSS)
    {
        chassis_ctrl_stop();
        is_navigating  = 0U;
        if (s_stage_resume == STAGE_WAIT_START)
        {
            /* 返航掉线恢复后重新到 (1,5)，随后仍按“先发车、后等图”执行。 */
            reset_exec_context();
            map_snapshot_release();
            clear_box_target_mapping();
            App_Recognize_Reset();
            current_stage = STAGE_WAIT_START;
            return;
        }
        if (s_stage_resume == STAGE_LAUNCH_EXIT)
        {
            /* 保留发车前帧栅栏；只重发 (1,4) 目标，避免漏掉移动期间已刷新的地图。 */
            s_launch_drive_issued = 0U;
            s_launch_drive_ticks = 0U;
            map_snapshot_release();
            current_stage = STAGE_LAUNCH_EXIT;
            return;
        }
        if (s_stage_resume == STAGE_WAIT_MAP_REFRESH)
        {
            reset_launch_map_stability();
            map_snapshot_release();
            current_stage = STAGE_WAIT_MAP_REFRESH;
            return;
        }
        if ((s_stage_resume == STAGE_DEADLOCK_RESET) ||
            (s_stage_resume == STAGE_DONE))
        {
            current_stage = s_stage_resume;
            return;
        }
        /* 规划/识别/执行期掉线后不得冻结并复用掉线前地图。
         * 从恢复时刻建立新帧栅栏，等待 5 帧新鲜一致地图后再识别/重规划。 */
        prepare_recovery_map_wait();
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
     * 链路恢复由 update_link_state() 按暂停前阶段回 WAIT_MAP_REFRESH 或 RECOGNIZE_MAP.
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
     *       链路 LOSS 期间不刷新；链路恢复后在 WAIT_RECOVERY_MAP 重取稳定新图。
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
        uint8 allow_continuous = (uint8)((s_link_alive != 0U) &&
                                          (current_stage != STAGE_DONE) &&
                                          (current_stage != STAGE_WAIT_RECOVERY_MAP));
        uint8 allow_consistency = (uint8)((s_link_alive != 0U) &&
                                          (current_stage != STAGE_DONE) &&
                                          (current_stage != STAGE_PAUSE_ON_LINK_LOSS) &&
                                          (current_stage != STAGE_WAIT_RECOVERY_MAP));
        app_vision_fusion_task(allow_continuous);
        app_vision_fusion_consistency_tick(allow_consistency);
    }

    sync_player_pos();

    switch (current_stage) {
        case STAGE_WAIT_START:
            stage_wait_start_handler();
            break;

        case STAGE_LAUNCH_EXIT:
            stage_launch_exit_handler();
            break;

        case STAGE_WAIT_MAP_REFRESH:
            stage_wait_map_refresh_handler();
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

        case STAGE_WAIT_RECOVERY_MAP:
            stage_wait_recovery_map_handler();
            break;

        default:
            reset_exec_context();
            goto_stage(STAGE_WAIT_START);
            break;
    }

}
