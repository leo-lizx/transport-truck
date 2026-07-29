#include "app_game_logic.h"
#include "app_link.h"      /* P0-2: 读取 OpenART1/2 链路时戳; P0-3: 拷贝 seq-lock 地图快照 */
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
#define APP_GAME_LAUNCH_EXIT_X       (2U)
#define APP_GAME_LAUNCH_EXIT_Y       (5U)

uint8 g_game_map[MAP_ROWS][MAP_COLS];
Point_t g_player_pos = {(int8)APP_GAME_LAUNCH_HOME_X, (int8)APP_GAME_LAUNCH_HOME_Y};

/* 赛规: 三个关卡连续比赛。三个关卡均结束后进入 STAGE_DONE 静止收车,
 * 不再无限等待下一张地图；第三关结束并返航后进入 STAGE_DONE。
 * 推完箱子后返回发车点；死局自动返航/重置暂时关闭，异常时停车等待。 */
#define APP_GAME_TOTAL_LEVELS          (3U)

/* ==================================================================
 * 【P0-2】视觉链路超时回退参数
 *   LINK_LOSS_MS  : 触发"掉线"判定的静默时长门槛
 *   LINK_OK_MS    : 触发"恢复"判定的静默时长门槛 (< LINK_LOSS_MS, 形成迟滞防抖)
 *   视觉端策略 = 心跳 100ms/次；1s 阈值允许视觉整帧处理和调试输出产生短时抖动。
 * ================================================================== */
#define LINK_LOSS_MS                   (1000U)
#define LINK_OK_MS                     (100U)

typedef enum {
    EXEC_NONE = 0,
    EXEC_PUSH_BOX,
    EXEC_PUSH_BOMB
} ExecMode_e;

typedef enum {
    PLAN_MODE_PUSH = 0,
    PLAN_MODE_BOMB,
    PLAN_MODE_BOMB_VERIFY
} PlanMode_e;

static GameStage_e current_stage = STAGE_WAIT_START;
static uint8 is_navigating = 0;
/* 业务层记录的四正交识别航向，供发车保持和识别 Tour 使用；推箱航点
 * 与硬编码模式一致走 move_to_grid，由底盘按实测角吸附到最近四正交航向。 */
static float s_nav_heading_deg = APP_GAME_LAUNCH_FACE_YAW_DEG;

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

/* ----- 发车相关静态变量 ------------------------------------
 * s_wait_start_phase : WAIT_START 子相位
 *                       0 = 正在返回左侧发车点 (1,5)
 *                       1 = 已到 (1,5), 正在保持 0 度并等待底盘稳定
 * --------------------------------------------------------------- */
static uint8               s_wait_start_phase   = 0U;

/* ----- 【B1+B12】地图快照冻结标志 ----------------------------------
 * s_map_freeze: 1 = 主循环入口不再 app_link_get_map_snapshot 覆盖 g_game_map
 *               (设置后视觉端发的所有更新会被忽略, 直到清除)。
 * O8.2: 所有置位/清零统一经 map_snapshot_freeze()/map_snapshot_release() 两函数,
 *       完整的冻结/解冻生命周期点见这两个函数处的集中注释。
 * --------------------------------------------------------------- */
static uint8               s_map_freeze         = 0U;
static uint32              s_map_snapshot_frame_id = 0U;
static uint8               s_map_has_bomb_cached = 0U;
static uint8               s_map_has_bomb_valid = 0U;

/* ----- 【B8】WAIT_START phase 0 自动复位超时 -----------------------
 * 10s 仍到不了发车区 → 跳过自动复位进 phase 1, 让操作员手动放车.
 * --------------------------------------------------------------- */
#define WAIT_START_PHASE0_TIMEOUT_TICKS  (2000U)
static uint16              s_wait_phase0_ticks  = 0U;
static uint8               s_return_rotate_started = 0U; /* 直线返库后原地转回 0° 的子步骤 */

/* ----- 自动发车 (首次贴边位/关间 (1,5) → 上位机触发点 (2,5)) -------
 * 首次上电从贴边位出发，关间仍从 (1,5) 出发；到达 (2,5) 后停车，
 * 等待上位机刷新本关地图。
 * 与硬编码模式一致，目标发出后持续等待底盘报告到达，不做业务层超时重发。
 * --------------------------------------------------------------- */
#define LAUNCH_STARTUP_SETTLE_TICKS       (200U)  /* 1s @5ms，仅首次上电发车 */
#define MAP_STABLE_REQUIRED_FRAMES          (5U)
#define MAP_STABLE_TIMEOUT_TICKS            (300U)
static uint16              s_launch_startup_settle_ticks = 0U;
static uint8               s_launch_drive_issued = 0U;
static uint8               s_launch_map_candidate_valid = 0U;
static uint8               s_launch_map_stable_count = 0U;
static uint16              s_launch_map_stable_ticks = 0U;
static uint32              s_launch_map_last_frame_id = 0U;
static uint32              s_launch_arrival_frame_id = 0U;
static uint8               s_map_accepted = 0U;
static uint8               s_solve_succeeded = 0U;
static uint8               s_launch_map_candidate[MAP_ROWS][MAP_COLS];
static uint8               s_launch_map_observed[MAP_ROWS][MAP_COLS];
/* IPS only consumes accepted/frozen maps. Keep this copy independent from
 * g_game_map because recognition and bomb handling may modify the latter. */
static uint8               s_display_frozen_map[MAP_ROWS][MAP_COLS];
static uint32              s_display_frozen_map_generation = 0U;

/* 断链恢复栅栏。恢复期间不沿用暂停前的冻结地图。 */
static uint32              s_recovery_map_baseline_frame_id = 0U;

static SokoFullSolution_t    g_soko_solution;
static SokoWaypointPath_t    g_soko_waypoints;
static uint8                 g_soko_sub_idx = 0;
static uint16                g_soko_wp_idx = 0;
static uint8                 g_soko_exec_init = 0;

/* 第一关滚动规划：当前箱执行时，后台只解算其完成后的剩余地图。
 * 不复制约 16KB 的完整动作解；改存航点和每段后的预测地图，约 6.5KB BSS，
 * 供后台尚未结束时无等待地沿用最近一次完整可行解。 */
static uint8                 s_roll_active = 0U;
static uint8                 s_roll_boxes_remaining = 0U;
static uint8                 s_roll_bg_start_pending = 0U;
static uint8                 s_roll_bg_active = 0U;
static uint8                 s_roll_bg_expected_boxes = 0U;
static uint8                 s_roll_after_active_map[MAP_ROWS][MAP_COLS];
static Point_t               s_roll_after_active_player;
static SokoWaypointPath_t    s_roll_queue_waypoints[SOKOBAN_MAX_BOXES];
static uint8                 s_roll_queue_after_map[SOKOBAN_MAX_BOXES][MAP_ROWS][MAP_COLS];
static Point_t               s_roll_queue_after_player[SOKOBAN_MAX_BOXES];
static uint8                 s_roll_queue_count = 0U;
static uint8                 s_roll_queue_next = 0U;

static Point_t               g_bomb_pos;
static Point_t               g_bomb_wall_pos;
static SokoActionSeq_t       g_bomb_action_seq;
static SokoWaypointPath_t    g_bomb_waypoints;
static uint16                g_bomb_wp_idx = 0;

static uint8                 g_box_to_target[SOKOBAN_MAX_BOXES] = {0};
/* 第二/三关类别不完全配平时，只规划本轮最大可行匹配子集。未匹配箱子
 * 在子地图中作为墙保留，避免计划穿过真实仍在场的箱子。 */
static uint8                 s_matched_box_count = 0U;
static uint8                 s_partial_batch_active = 0U;
static uint8                 s_partial_target_mask = 0U;
static uint8                 s_remaining_task_rescan = 0U;
static uint8                 s_completion_verify_active = 0U;
static uint8                 s_bomb_mapping_rescan_active = 0U;
static uint32                s_completion_map_len_err_baseline = 0U;
static uint8                 s_partial_plan_map[MAP_ROWS][MAP_COLS];
static uint8                 s_partial_plan_mapping[SOKOBAN_MAX_BOXES];
static ExecMode_e            g_exec_mode = EXEC_NONE;
static PlanMode_e            s_plan_mode = PLAN_MODE_PUSH;
static uint8                 s_bomb_strategy = 0U;
static uint8                 s_bomb_search_started = 0U;
static uint8                 s_bomb_verify_map[MAP_ROWS][MAP_COLS];
static Point_t               s_bomb_verify_player;

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

static void publish_frozen_map_for_display(void)
{
    copy_map(s_display_frozen_map, g_game_map);
    s_display_frozen_map_generation++;
    if (s_display_frozen_map_generation == 0U) {
        s_display_frozen_map_generation = 1U;
    }
}

static void map_cache_invalidate(void)
{
    s_map_has_bomb_valid = 0U;
}

static uint8 launch_map_is_usable(const uint8 map[MAP_ROWS][MAP_COLS])
{
    uint8 r;
    uint8 c;
    uint8 has_wall = 0U;
    uint8 box_count = 0U;
    uint8 target_count = 0U;
    uint8 launch_cell;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (map[r][c] == MAP_WALL) {
                has_wall = 1U;
            } else if (map[r][c] == MAP_BOX) {
                box_count++;
            } else if (map[r][c] == MAP_TARGET) {
                target_count++;
            }
        }
    }

    launch_cell = map[APP_GAME_LAUNCH_EXIT_Y][APP_GAME_LAUNCH_EXIT_X];
    return (uint8)((has_wall != 0U) &&
                   (box_count != 0U) &&
                   (box_count == target_count) &&
                   (box_count <= SOKOBAN_MAX_BOXES) &&
                   (launch_cell != MAP_WALL) &&
                   (launch_cell != MAP_BOX) &&
                   (launch_cell != MAP_BOMB));
}

/* 恢复或部分批次重读时可能已完成最后一次推箱，此时允许箱子/目标同时为零；
 * 但两者数量仍必须相等，并要求连续多帧一致。 */
static uint8 recovery_map_is_usable(const uint8 map[MAP_ROWS][MAP_COLS])
{
    uint8 r;
    uint8 c;
    uint8 has_wall = 0U;
    uint8 box_count = 0U;
    uint8 target_count = 0U;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (map[r][c] == MAP_WALL) {
                has_wall = 1U;
            } else if (map[r][c] == MAP_BOX) {
                box_count++;
            } else if (map[r][c] == MAP_TARGET) {
                target_count++;
            }
        }
    }
    return (uint8)((has_wall != 0U) &&
                   (box_count == target_count) &&
                   (box_count <= SOKOBAN_MAX_BOXES));
}

/* 完整推箱后的新结果若为空或结构非法，按视觉端“场地已清空”约定直接完成。 */
static uint8 completion_invalid_or_empty_map_received(uint32 baseline_frame_id)
{
    uint32 frame_id_before = g_link_map_frame_id;
    uint32 frame_id_after;
    uint8 r;
    uint8 c;

    if (s_completion_verify_active == 0U) {
        return 0U;
    }
    if (g_link_map_stats.frames_len_err !=
        s_completion_map_len_err_baseline) {
        return 1U;
    }
    if (frame_id_before == 0U ||
        frame_id_before == baseline_frame_id ||
        frame_id_before == s_launch_map_last_frame_id) {
        return 0U;
    }

    app_link_get_map_snapshot(s_launch_map_observed);
    frame_id_after = g_link_map_frame_id;
    if (frame_id_after != frame_id_before) {
        return 0U;
    }

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (s_launch_map_observed[r][c] == MAP_BOX ||
                s_launch_map_observed[r][c] == MAP_TARGET) {
                return (uint8)(recovery_map_is_usable(
                    s_launch_map_observed) == 0U);
            }
        }
    }
    return 1U;
}

/* 到达 (2,5) 后只接纳连续五张完全一致的合法新图。每张图都重新检查
 * 箱子/目标数量，任意非法帧或布局变化都会重新从第一张开始计数。 */
static uint8 launch_map_stability_tick(uint32 baseline_frame_id)
{
    uint32 frame_id_before = g_link_map_frame_id;
    uint32 frame_id_after;

    if (s_launch_map_stable_ticks < MAP_STABLE_TIMEOUT_TICKS) {
        s_launch_map_stable_ticks++;
    } else {
        reset_launch_map_stability();
        return 0U;
    }

    if ((frame_id_before == 0U) ||
        (frame_id_before == baseline_frame_id) ||
        (frame_id_before == s_launch_map_last_frame_id)) {
        return 0U;
    }

    app_link_get_map_snapshot(s_launch_map_observed);
    frame_id_after = g_link_map_frame_id;
    if (frame_id_after != frame_id_before) {
        return 0U;
    }
    s_launch_map_last_frame_id = frame_id_after;

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
        return 0U;
    }

    if (s_launch_map_stable_count < MAP_STABLE_REQUIRED_FRAMES) {
        return 0U;
    }

    copy_map(g_game_map, s_launch_map_candidate);
    s_map_snapshot_frame_id = frame_id_after;
    map_cache_invalidate();
    return 1U;
}

/* 断链恢复与部分批次重读均要求多帧一致，避免用单帧旧箱位恢复执行。 */
static uint8 recovery_map_stability_tick(uint32 baseline_frame_id)
{
    uint32 frame_id = g_link_map_frame_id;

    if (frame_id == 0U) {
        return 0U;
    }

    if (s_launch_map_stable_ticks < MAP_STABLE_TIMEOUT_TICKS) {
        s_launch_map_stable_ticks++;
    } else {
        reset_launch_map_stability();
        return 0U;
    }

    /* 只接收本次链路恢复栅栏之后落地的新帧。 */
    if ((frame_id == baseline_frame_id) ||
        (frame_id == s_launch_map_last_frame_id)) {
        return 0U;
    }
    s_launch_map_last_frame_id = frame_id;

    app_link_get_map_snapshot(s_launch_map_observed);
    if (recovery_map_is_usable(s_launch_map_observed) == 0U) {
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

    if (s_launch_map_stable_count >= MAP_STABLE_REQUIRED_FRAMES) {
        copy_map(g_game_map, s_launch_map_candidate);
        s_map_snapshot_frame_id = frame_id;
        map_cache_invalidate();
        return 1U;
    }
    return 0U;
}

static void reset_exec_context(void)
{
    Sokoban_Stage1_Search_Cancel();
    Sokoban_Bomb_Search_Cancel();
    g_soko_solution.is_solved = 0;
    g_soko_solution.total_boxes = 0;
    g_soko_waypoints.count = 0;
    g_soko_sub_idx = 0;
    g_soko_wp_idx = 0;
    g_soko_exec_init = 0;
    s_roll_active = 0U;
    s_roll_boxes_remaining = 0U;
    s_roll_bg_start_pending = 0U;
    s_roll_bg_active = 0U;
    s_roll_bg_expected_boxes = 0U;
    s_roll_queue_count = 0U;
    s_roll_queue_next = 0U;

    g_bomb_waypoints.count = 0;
    g_bomb_wp_idx = 0;
    g_exec_mode = EXEC_NONE;
    s_plan_mode = PLAN_MODE_PUSH;
    s_bomb_strategy = 0U;
    s_bomb_search_started = 0U;
    Sokoban_Bomb_Search_Clear_Rejections();
    is_navigating = 0;
    s_wait_phase0_ticks = 0U;       /* Issue A: WAIT_START phase 0 计时跨入口清零 */
    s_wait_start_phase = 0U;
    s_launch_drive_issued = 0U;
    s_launch_arrival_frame_id = 0U;
    reset_launch_map_stability();
}

static void goto_stage(GameStage_e next)
{
    if (next == STAGE_DEADLOCK_RESET) {
        /* 自动死局返航/重置暂时关闭；仍保留异常检测，触发时只停车。 */
        chassis_ctrl_stop();
        is_navigating = 0U;
    }
    current_stage = next;
}

/* O8.2: 地图快照冻结开关的集中入口。
 * 所有对 s_map_freeze 的置位/清零都经由这两个函数, 便于:
 *   - 用函数名一次检索出全部生命周期点 (降低新增 stage 时漏配的风险);
 *   - 把"为何冻结/解冻"的语义集中在此处记录。
 * 注意: freeze 取值依赖地图接纳与规划器完成私有拷贝的时序；炸弹爆破后会
 * 直接进入新地图栅栏，不能化简为"纯 stage→freeze 静态表"。
 *
 * 冻结 (hold, freeze=1) 点: 驶出后新地图稳定/恢复或部分批次重读后进 RECOGNIZE 前、
 *                            识别完成进入规划前;
 * 解冻 (release, freeze=0) 点: WAIT_START/LAUNCH_EXIT/WAIT_MAP_REFRESH、识别失败、PLAN 完成本拍、DONE。 */
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
    memset(g_box_to_target, APP_RECOG_TARGET_UNMATCHED,
           sizeof(g_box_to_target));
    s_matched_box_count = 0U;
    s_partial_batch_active = 0U;
    s_partial_target_mask = 0U;
}

/**
 * 将部分类别映射压缩成 Stage2 求解器可直接使用的完整子问题：
 * - 未匹配箱子改成墙，保持其真实阻挡作用；
 * - 未匹配目标恢复为空地，保持可行走属性；
 * - 匹配索引按压缩后行列扫描顺序重新编号。
 */
static uint8 build_partial_plan_problem_from(
    const uint8 source_map[MAP_ROWS][MAP_COLS],
    uint8 output_map[MAP_ROWS][MAP_COLS],
    uint8 output_mapping[SOKOBAN_MAX_BOXES])
{
    uint8 box_count = 0U;
    uint8 target_count = 0U;
    uint8 selected_count = 0U;
    uint8 target_mask = 0U;
    uint8 box_idx = 0U;
    uint8 target_idx = 0U;
    uint8 reduced_box_idx = 0U;
    uint8 r;
    uint8 c;

    if (s_partial_batch_active == 0U ||
        s_matched_box_count == 0U ||
        s_matched_box_count > (uint8)SOKOBAN_MAX_BOXES) {
        return 0U;
    }

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (source_map[r][c] == MAP_BOX) {
                box_count++;
            } else if (source_map[r][c] == MAP_TARGET) {
                target_count++;
            }
        }
    }
    if (box_count == 0U || box_count != target_count ||
        box_count > (uint8)SOKOBAN_MAX_BOXES) {
        return 0U;
    }

    for (box_idx = 0U; box_idx < box_count; ++box_idx) {
        uint8 mapped_target = g_box_to_target[box_idx];
        uint8 target_bit;
        if (mapped_target == APP_RECOG_TARGET_UNMATCHED) {
            continue;
        }
        if (mapped_target >= target_count) {
            return 0U;
        }
        target_bit = (uint8)(1U << mapped_target);
        if ((target_mask & target_bit) != 0U) {
            return 0U;
        }
        target_mask = (uint8)(target_mask | target_bit);
        selected_count++;
    }
    if (selected_count != s_matched_box_count) {
        return 0U;
    }

    copy_map(output_map, source_map);
    box_idx = 0U;
    target_idx = 0U;
    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if (source_map[r][c] == MAP_BOX) {
                uint8 mapped_target = g_box_to_target[box_idx++];
                if (mapped_target == APP_RECOG_TARGET_UNMATCHED) {
                    output_map[r][c] = MAP_WALL;
                } else {
                    uint8 reduced_target_idx = 0U;
                    uint8 ti;
                    for (ti = 0U; ti < mapped_target; ++ti) {
                        if ((target_mask & (uint8)(1U << ti)) != 0U) {
                            reduced_target_idx++;
                        }
                    }
                    output_mapping[reduced_box_idx++] =
                        reduced_target_idx;
                }
            } else if (source_map[r][c] == MAP_TARGET) {
                if ((target_mask & (uint8)(1U << target_idx)) == 0U) {
                    output_map[r][c] = MAP_EMPTY;
                }
                target_idx++;
            }
        }
    }

    s_partial_target_mask = target_mask;
    return (uint8)(reduced_box_idx == s_matched_box_count &&
                   target_idx == target_count);
}

static uint8 build_partial_plan_problem(void)
{
    return build_partial_plan_problem_from(
        g_game_map, s_partial_plan_map, s_partial_plan_mapping);
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

static uint8 get_map_target_count(void)
{
    uint8 count = 0U;
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_TARGET) {
                count++;
            }
        }
    }
    return count;
}

static uint8 map_has_bomb(void)
{
    if (s_map_has_bomb_valid != 0U) return s_map_has_bomb_cached;
    s_map_has_bomb_cached = 0U;
    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_BOMB) {
                s_map_has_bomb_cached = 1U;
                s_map_has_bomb_valid = 1U;
                return 1U;
            }
        }
    }
    s_map_has_bomb_valid = 1U;
    return 0U;
}

static Point_t choose_nearest_target(Point_t ref)
{
    Point_t best = {-1, -1};
    int16 best_d = 32767;
    uint8 target_idx = 0U;

    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] != MAP_TARGET) continue;
            if (s_partial_batch_active != 0U &&
                (target_idx >= (uint8)SOKOBAN_MAX_BOXES ||
                 (s_partial_target_mask & (uint8)(1U << target_idx)) == 0U)) {
                target_idx++;
                continue;
            }
            target_idx++;

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

/* ===========================================================================
 *  § 2. 航点执行公用逻辑 (chassis_nav_arrived / exec_waypoints_common)
 * ========================================================================== */

static uint8 chassis_nav_arrived(void)
{
    return chassis_ctrl_is_arrived();
}

/* 与硬编码模式一致：每个推箱航点使用网格接口下发一次，随后持续等待到达。 */
static void dispatch_waypoint(const SokoWaypointPath_t *wp, uint16 idx)
{
    Point_t target = wp->points[idx];

    chassis_ctrl_move_to_grid((uint8)target.x, (uint8)target.y);
    is_navigating = 1;
}

static uint8 exec_waypoints_common(const SokoWaypointPath_t *wp, uint16 *wp_idx)
{
    if (!is_navigating) {
        if (*wp_idx < wp->count) {
            dispatch_waypoint(wp, *wp_idx);
        }
        return 0;
    }

    if (!chassis_nav_arrived()) {
        return 0;
    }

    is_navigating = 0;
    (*wp_idx)++;
    return (*wp_idx >= wp->count) ? 1U : 0U;
}

static uint8 rolling_seq_is_push(const SokoActionSeq_t *seq, uint16 index)
{
    return (uint8)((seq->push_bitmap[index >> 3] >> (index & 7U)) & 1U);
}

/* 在预测地图上回放一个“单箱到目标”子解。箱子到达该段目标后与目标点
 * 一起消失，输出正好可作为剩余箱规划器的下一张输入地图。 */
static uint8 rolling_apply_subsolution(
    const uint8 input_map[MAP_ROWS][MAP_COLS],
    Point_t start_player,
    const SokoActionSeq_t *seq,
    uint8 output_map[MAP_ROWS][MAP_COLS],
    Point_t *end_player)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    Point_t player = start_player;
    Point_t active_box = {-1, -1};
    uint8 box_under = MAP_EMPTY;
    uint8 has_active_box = 0U;

    if (input_map == NULL || seq == NULL || end_player == NULL ||
        seq->count == 0U || seq->count > (uint16)SOKOBAN_MAX_ACTIONS) {
        return 0U;
    }
    copy_map(output_map, input_map);

    for (uint16 i = 0U; i < seq->count; ++i) {
        uint8 direction = (uint8)seq->actions[i];
        Point_t next;

        if (direction > (uint8)SOKO_ACT_RIGHT) { return 0U; }
        next.x = (int8)(player.x + dc[direction]);
        next.y = (int8)(player.y + dr[direction]);
        if (next.x < 0 || next.x >= (int8)MAP_COLS ||
            next.y < 0 || next.y >= (int8)MAP_ROWS) {
            return 0U;
        }

        if (rolling_seq_is_push(seq, i) != 0U) {
            Point_t box_to;
            uint8 next_under;

            if (output_map[next.y][next.x] != MAP_BOX ||
                (has_active_box != 0U &&
                 (next.x != active_box.x || next.y != active_box.y))) {
                return 0U;
            }
            box_to.x = (int8)(next.x + dc[direction]);
            box_to.y = (int8)(next.y + dr[direction]);
            if (box_to.x < 0 || box_to.x >= (int8)MAP_COLS ||
                box_to.y < 0 || box_to.y >= (int8)MAP_ROWS) {
                return 0U;
            }
            next_under = output_map[box_to.y][box_to.x];
            if (next_under != MAP_EMPTY && next_under != MAP_TARGET) {
                return 0U;
            }

            output_map[next.y][next.x] = box_under;
            output_map[box_to.y][box_to.x] = MAP_BOX;
            box_under = next_under;
            active_box = box_to;
            has_active_box = 1U;
        } else if (output_map[next.y][next.x] != MAP_EMPTY &&
                   output_map[next.y][next.x] != MAP_TARGET) {
            return 0U;
        }
        player = next;
    }

    if (has_active_box == 0U || box_under != MAP_TARGET ||
        output_map[active_box.y][active_box.x] != MAP_BOX) {
        return 0U;
    }
    output_map[active_box.y][active_box.x] = MAP_EMPTY;
    *end_player = player;
    return 1U;
}

/* 把完整可行解压缩为后续箱子的航点/残余地图队列。后台优化只有在整份
 * 新解均成功转换后才会被执行端采用。 */
static uint8 rolling_build_queue(const SokoFullSolution_t *solution,
                                 uint8 first_sub_index,
                                 const uint8 first_map[MAP_ROWS][MAP_COLS],
                                 Point_t first_player)
{
    const uint8 (*source_map)[MAP_COLS] = first_map;
    Point_t player = first_player;
    uint8 queue_count = 0U;

    if (solution == NULL || solution->is_solved == 0U ||
        solution->total_boxes == 0U ||
        solution->total_boxes > (uint8)SOKOBAN_MAX_BOXES ||
        first_sub_index > solution->total_boxes) {
        return 0U;
    }

    for (uint8 sub = first_sub_index; sub < solution->total_boxes; ++sub) {
        Point_t predicted_end;

        if (Sokoban_Seq_To_Waypoints(&solution->sub_solutions[sub],
                                     player,
                                     &s_roll_queue_waypoints[queue_count]) == 0U ||
            !rolling_apply_subsolution(source_map,
                                       player,
                                       &solution->sub_solutions[sub],
                                       s_roll_queue_after_map[queue_count],
                                       &predicted_end) ||
            predicted_end.x != solution->player_end_pos[sub].x ||
            predicted_end.y != solution->player_end_pos[sub].y) {
            s_roll_queue_count = 0U;
            s_roll_queue_next = 0U;
            return 0U;
        }

        s_roll_queue_after_player[queue_count] = predicted_end;
        source_map = s_roll_queue_after_map[queue_count];
        player = predicted_end;
        queue_count++;
    }

    s_roll_queue_count = queue_count;
    s_roll_queue_next = 0U;
    return 1U;
}

static uint8 rolling_activate_initial_solution(void)
{
    Point_t predicted_end;

    if (!g_soko_solution.is_solved || g_soko_solution.total_boxes == 0U ||
        g_soko_solution.total_boxes > (uint8)SOKOBAN_MAX_BOXES ||
        Sokoban_Seq_To_Waypoints(&g_soko_solution.sub_solutions[0],
                                 g_player_pos,
                                 &g_soko_waypoints) == 0U ||
        !rolling_apply_subsolution(g_game_map,
                                   g_player_pos,
                                   &g_soko_solution.sub_solutions[0],
                                   s_roll_after_active_map,
                                   &predicted_end) ||
        predicted_end.x != g_soko_solution.player_end_pos[0].x ||
        predicted_end.y != g_soko_solution.player_end_pos[0].y ||
        !rolling_build_queue(&g_soko_solution,
                             1U,
                             s_roll_after_active_map,
                             predicted_end)) {
        return 0U;
    }

    s_roll_active = 1U;
    s_roll_boxes_remaining = g_soko_solution.total_boxes;
    s_roll_after_active_player = predicted_end;
    s_roll_bg_start_pending =
        (uint8)(s_roll_boxes_remaining > 1U);
    s_roll_bg_active = 0U;
    s_roll_bg_expected_boxes = 0U;
    g_soko_sub_idx = 0U;
    g_soko_wp_idx = 0U;
    g_exec_mode = EXEC_PUSH_BOX;
    is_navigating = 0U;
    s_solve_succeeded = 1U;
    return 1U;
}

static uint8 rolling_promote_queued_solution(void)
{
    uint8 queue_index;

    if (s_roll_queue_next >= s_roll_queue_count) { return 0U; }
    queue_index = s_roll_queue_next++;
    g_soko_waypoints = s_roll_queue_waypoints[queue_index];
    copy_map(s_roll_after_active_map,
             s_roll_queue_after_map[queue_index]);
    s_roll_after_active_player =
        s_roll_queue_after_player[queue_index];
    g_soko_wp_idx = 0U;
    is_navigating = 0U;
    s_roll_bg_start_pending =
        (uint8)(s_roll_boxes_remaining > 1U);
    return 1U;
}

static uint8 exec_push_box_solution(void)
{
    /* exec_waypoints_common() 的完成只表示“当前箱子的子路径结束”，
     * 不能直接向上层报告整关完成，否则比赛模式会在第一箱后提前结算。 */
    if (g_soko_wp_idx < g_soko_waypoints.count &&
        exec_waypoints_common(&g_soko_waypoints, &g_soko_wp_idx) == 0U) {
        return 0U;
    }

    g_soko_sub_idx++;
    if (s_roll_active != 0U) {
        if (s_roll_boxes_remaining == 0U) {
            goto_stage(STAGE_DEADLOCK_RESET);
            return 0U;
        }
        s_roll_boxes_remaining--;
        if (s_roll_boxes_remaining == 0U) { return 1U; }

        /* 后台若尚在搜索，保留旧完整解生成的队列并立即执行，不在箱间停车。 */
        if (s_roll_bg_active != 0U) {
            Sokoban_Stage1_Search_Cancel();
            s_roll_bg_active = 0U;
        }
        s_roll_bg_start_pending = 0U;
        if (!rolling_promote_queued_solution()) {
            goto_stage(STAGE_DEADLOCK_RESET);
        }
        return 0U;
    }

    if (g_soko_sub_idx >= g_soko_solution.total_boxes) {
        return 1U;
    }

    if (Sokoban_Seq_To_Waypoints(
            &g_soko_solution.sub_solutions[g_soko_sub_idx],
            g_player_pos,                              /* B5: 用实时格而非 BFS 预测格 */
            &g_soko_waypoints) == 0U) {
        goto_stage(STAGE_DEADLOCK_RESET);
        return 0U;
    }
    g_soko_wp_idx = 0U;
    is_navigating = 0U;
    return 0U;
}

/* ==========================================================================
 *  § 3. 规划层 — 炸弹计划 / 推箱计划 / 关卡推进判定
 * ========================================================================== */

static uint8 find_first_unreachable_target(Point_t *blocked_target)
{
    static uint8 reach[MAP_ROWS][MAP_COLS];
    uint8 target_idx = 0U;

    if (!blocked_target) return 0;

    blocked_target->x = -1;
    blocked_target->y = -1;

    if (!Algo_Nav_BFS_Flood(g_game_map, g_player_pos, reach, 0)) return 0;

    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; r++) {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; c++) {
            if (g_game_map[r][c] == MAP_TARGET) {
                Point_t tp = {c, r};
                if (s_partial_batch_active != 0U &&
                    (target_idx >= (uint8)SOKOBAN_MAX_BOXES ||
                     (s_partial_target_mask &
                      (uint8)(1U << target_idx)) == 0U)) {
                    target_idx++;
                    continue;
                }
                target_idx++;
                if (!Algo_Nav_Is_Reachable(reach, tp)) {
                    *blocked_target = tp;
                    return 1;
                }
            }
        }
    }

    return 0;
}

static uint8 activate_bomb_plan(void)
{
    if (Sokoban_Seq_To_Waypoints(&g_bomb_action_seq,
                                 g_player_pos,
                                 &g_bomb_waypoints) == 0U) {
        return 0U;
    }
    g_bomb_wp_idx = 0;
    g_exec_mode = EXEC_PUSH_BOMB;
    is_navigating = 0;
    s_solve_succeeded = 1U;
    return 1U;
}

static void start_fresh_bomb_search(void)
{
    Sokoban_Bomb_Search_Clear_Rejections();
    s_plan_mode = PLAN_MODE_BOMB;
    s_bomb_strategy = 0U;
    s_bomb_search_started = 0U;
}

static uint8 predict_bomb_result(void)
{
    Point_t player = g_player_pos;

    if (g_bomb_pos.x < (int8)CHASSIS_GRID_INNER_MIN_X ||
        g_bomb_pos.x > (int8)CHASSIS_GRID_INNER_MAX_X ||
        g_bomb_pos.y < (int8)CHASSIS_GRID_INNER_MIN_Y ||
        g_bomb_pos.y > (int8)CHASSIS_GRID_INNER_MAX_Y ||
        g_bomb_wall_pos.x < (int8)CHASSIS_GRID_INNER_MIN_X ||
        g_bomb_wall_pos.x > (int8)CHASSIS_GRID_INNER_MAX_X ||
        g_bomb_wall_pos.y < (int8)CHASSIS_GRID_INNER_MIN_Y ||
        g_bomb_wall_pos.y > (int8)CHASSIS_GRID_INNER_MAX_Y) {
        return 0U;
    }

    for (uint16 i = 0U; i < g_bomb_action_seq.count; ++i) {
        switch (g_bomb_action_seq.actions[i]) {
            case SOKO_ACT_UP:    --player.y; break;
            case SOKO_ACT_DOWN:  ++player.y; break;
            case SOKO_ACT_LEFT:  --player.x; break;
            case SOKO_ACT_RIGHT: ++player.x; break;
            default: return 0U;
        }
        if (player.x < (int8)CHASSIS_GRID_INNER_MIN_X ||
            player.x > (int8)CHASSIS_GRID_INNER_MAX_X ||
            player.y < (int8)CHASSIS_GRID_INNER_MIN_Y ||
            player.y > (int8)CHASSIS_GRID_INNER_MAX_Y) {
            return 0U;
        }
    }

    copy_map(s_bomb_verify_map, g_game_map);
    s_bomb_verify_map[g_bomb_pos.y][g_bomb_pos.x] = MAP_EMPTY;
    Sokoban_Apply_Bomb_Explosion(s_bomb_verify_map, g_bomb_wall_pos);
    s_bomb_verify_map[g_bomb_wall_pos.y][g_bomb_wall_pos.x] = MAP_EMPTY;
    s_bomb_verify_player = player;
    return 1U;
}

/* 爆炸只改变墙体和炸弹，不应改变箱子/目标位置。位置完全一致时，
 * 行列扫描索引也保持不变，可以安全复用爆炸前已经识别的类别映射。 */
static uint8 bomb_rescan_items_match_prediction(void)
{
    uint8 r;
    uint8 c;

    for (r = 0U; r < (uint8)MAP_ROWS; ++r) {
        for (c = 0U; c < (uint8)MAP_COLS; ++c) {
            if ((g_game_map[r][c] == MAP_BOX) !=
                (s_bomb_verify_map[r][c] == MAP_BOX)) {
                return 0U;
            }
            if ((g_game_map[r][c] == MAP_TARGET) !=
                (s_bomb_verify_map[r][c] == MAP_TARGET)) {
                return 0U;
            }
        }
    }
    return 1U;
}

/* 炸弹联合搜索和 Stage2 共用单箱静态缓冲，因此必须先结束/取消前者，
 * 再在空闲切片中验证爆炸后的固定映射是否可完整执行。 */
static uint8 begin_bomb_candidate_verification(void)
{
    Point_t home = { (int8)APP_GAME_LAUNCH_HOME_X,
                     (int8)APP_GAME_LAUNCH_HOME_Y };
    uint8 begin_ok;

    if (predict_bomb_result() == 0U) return 0U;
    if (s_partial_batch_active != 0U) {
        if (build_partial_plan_problem_from(
                s_bomb_verify_map,
                s_partial_plan_map,
                s_partial_plan_mapping) == 0U) {
            return 0U;
        }
        begin_ok = Sokoban_Stage2_Search_Begin(
            s_partial_plan_map, s_bomb_verify_player,
            s_partial_plan_mapping, s_matched_box_count, home);
    } else {
        begin_ok = Sokoban_Stage2_Search_Begin(
            s_bomb_verify_map, s_bomb_verify_player,
            g_box_to_target, get_map_box_count(), home);
    }
    g_soko_exec_init = begin_ok;
    return begin_ok;
}

static void reject_bomb_candidate_and_resume(void)
{
    if (Sokoban_Bomb_Search_Reject_Pair(
            g_bomb_pos, g_bomb_wall_pos) == 0U) {
        /* 候选必定来自当前搜索上下文；若上下文异常，终止本轮而不是
         * 重复选中同一组合形成静止死循环。 */
        s_bomb_strategy = 3U;
    }
    s_plan_mode = PLAN_MODE_BOMB;
    s_bomb_search_started = 0U;
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
    s_solve_succeeded = 1U;
    return 1U;
}

/* 仅由主循环 tick 间空档调用。Begin 会复制预测地图，此后执行端可以继续
 * 更新自己的滚动队列，不会与求解器共享可变地图。 */
static uint8 rolling_background_idle_step(void)
{
    Point_t home = { (int8)APP_GAME_LAUNCH_HOME_X,
                     (int8)APP_GAME_LAUNCH_HOME_Y };

    if (s_roll_active == 0U || current_stage != STAGE_EXECUTE_ACTION) {
        return 0U;
    }

    if (s_roll_bg_start_pending != 0U) {
        s_roll_bg_start_pending = 0U;
        if (s_roll_boxes_remaining <= 1U) { return 0U; }

        s_roll_bg_expected_boxes =
            (uint8)(s_roll_boxes_remaining - 1U);
        s_roll_bg_active = Sokoban_Stage1_Search_Begin(
            s_roll_after_active_map,
            s_roll_after_active_player,
            home);
        return 1U;
    }

    if (s_roll_bg_active != 0U) {
        SokoSearchStatus_e status =
            Sokoban_Stage1_Search_Step(1U, &g_soko_solution);

        if (status == SOKO_SEARCH_RUNNING) { return 1U; }
        if (status == SOKO_SEARCH_SOLVED &&
            Sokoban_Search_Has_Incumbent() != 0U &&
            g_soko_solution.total_boxes == s_roll_bg_expected_boxes) {
            /* 完整优化结果准备好后再原子式替换兜底队列；执行中的当前
             * g_soko_waypoints 不参与替换。 */
            (void)rolling_build_queue(&g_soko_solution,
                                      0U,
                                      s_roll_after_active_map,
                                      s_roll_after_active_player);
        }
        Sokoban_Stage1_Search_Cancel();
        s_roll_bg_active = 0U;
        return 1U;
    }

    return 0U;
}

/* ==========================================================================
 *  § 4. Stage 处理器 — WAIT_START / RECOGNIZE / PLAN / EXECUTE / COMPLETE
 *                       / DEADLOCK_RESET / DONE / PAUSE_ON_LINK_LOSS
 * ========================================================================== */

static void prepare_launch_departure(void)
{
    /* OpenART 负责连续帧稳定过滤；主控将在到达 (2,5) 并停车后建立
     * 接收栅栏，行驶途中落地的地图不能参与本关解算。 */
    s_launch_arrival_frame_id = 0U;
    s_map_accepted = 0U;
    s_solve_succeeded = 0U;
    reset_launch_map_stability();
}

/* 链路恢复、部分批次完成或完整推箱后的校验，均从当前空地建立新帧栅栏并重读地图。 */
static void prepare_map_rescan_wait_internal(uint8 remaining_task_rescan,
                                             uint8 preserve_mapping)
{
    chassis_ctrl_stop();
    s_map_accepted = 0U;
    s_solve_succeeded = 0U;
    reset_exec_context();
    map_snapshot_release();
    if (preserve_mapping == 0U) {
        clear_box_target_mapping();
    }
    s_remaining_task_rescan = remaining_task_rescan;
    s_completion_verify_active = 0U;
    s_bomb_mapping_rescan_active = preserve_mapping;
    s_completion_map_len_err_baseline =
        g_link_map_stats.frames_len_err;
    App_Recognize_Reset();

    s_recovery_map_baseline_frame_id = g_link_map_frame_id;
    reset_launch_map_stability();
    s_launch_map_last_frame_id = s_recovery_map_baseline_frame_id;
    current_stage = STAGE_WAIT_RECOVERY_MAP;
}

static void prepare_map_rescan_wait(uint8 remaining_task_rescan)
{
    prepare_map_rescan_wait_internal(remaining_task_rescan, 0U);
}

/* 爆炸后的新图只用于校正墙体布局；箱子/目标未变化时保留已识别映射，
 * 避免同一批图案在推炸弹前后重复巡航识别。 */
static void prepare_bomb_map_rescan_wait(void)
{
    prepare_map_rescan_wait_internal(1U, 1U);
}

/* 各关完整计划执行结束后先校验视觉残图；空白或非法结果立即按已推完处理。 */
static void prepare_completion_verify_wait(void)
{
    prepare_map_rescan_wait(1U);
    s_completion_verify_active = 1U;
    s_completion_map_len_err_baseline =
        g_link_map_stats.frames_len_err;
}

void Game_Logic_Init(void)
{
    chassis_ctrl_stop();
    current_stage = STAGE_WAIT_START;
    is_navigating = 0U;
    s_nav_heading_deg = APP_GAME_LAUNCH_FACE_YAW_DEG;
    s_link_alive = 0U;
    s_link_ever_alive = 0U;
    s_stage_resume = STAGE_WAIT_START;
    s_link_pause_reason = LINK_PAUSE_NONE;
    s_levels_finished = 0U;
    s_map_accepted = 0U;
    s_solve_succeeded = 0U;
    s_launch_startup_settle_ticks = 0U;
    s_map_snapshot_frame_id = 0U;
    s_map_has_bomb_cached = 0U;
    s_map_has_bomb_valid = 0U;
    s_remaining_task_rescan = 0U;
    s_completion_verify_active = 0U;
    s_bomb_mapping_rescan_active = 0U;
    s_completion_map_len_err_baseline = 0U;
    reset_exec_context();
    /* 首次上电已由 main 设置为贴边发车位，因此跳过返航子阶段；
     * 关间 reset_exec_context() 会恢复 phase 0，仍先返回 (1,5)。 */
    s_wait_start_phase = 1U;
    map_snapshot_release();
    clear_box_target_mapping();
    App_Recognize_Reset();
    memset(g_game_map, 0, sizeof(g_game_map));
    memset(s_display_frozen_map, 0, sizeof(s_display_frozen_map));
    s_display_frozen_map_generation = 0U;
}

static void stage_wait_start_handler(void)
{
    /* 仅保留左侧发车区。phase 0 依靠里程计返回 (1,5)；phase 1 在首次上电时等待滤波稳定，
     * 关间返航后不重复等待。
     * 地图在发车和到达 (2,5) 后的等待阶段始终解冻。
     */
    map_snapshot_release();

    if (s_wait_start_phase == 0U) {
        s_wait_phase0_ticks++;
        if (!is_navigating) {
            /* 返库走两点直线 (DIRECT_LINE)：保持下发瞬间航向沿固定直线平移，
             * 比逐轴 L 形路径少一段行程；0° 交接航向到库后原地旋转恢复。
             * 关卡完成后全部地图元素（箱/目标/墙均为屏显）随即消失，
             * 返库路径无物理障碍，不需要任何避障检查。 */
            chassis_ctrl_move_to_m_direct(
                chassis_grid_x_to_m(APP_GAME_LAUNCH_HOME_X),
                chassis_grid_y_to_m(APP_GAME_LAUNCH_HOME_Y));
            s_return_rotate_started = 0U;
            is_navigating = 1;
            return;
        }
        /* 10s 未到则停车重发，不进入死局自动恢复。 */
        if (s_wait_phase0_ticks > WAIT_START_PHASE0_TIMEOUT_TICKS) {
            chassis_ctrl_stop();
            is_navigating = 0;
            s_wait_phase0_ticks = 0U;
            return;
        }
        if (!chassis_nav_arrived()) return;

        if (s_return_rotate_started == 0U) {
            /* 直线返库不预旋转；到库后先转回 0° 再交给发车流程，
             * 避免 LAUNCH_EXIT 的逐轴短程移动叠加大角度旋转。 */
            chassis_ctrl_rotate_to_deg(APP_GAME_LAUNCH_FACE_YAW_DEG);
            s_return_rotate_started = 1U;
            s_wait_phase0_ticks = 0U;   /* 旋转子步骤独享一个完整超时窗口 */
            return;
        }

        /* 到库后继续做 0° 航向保持，直到下一次发车目标下发。 */
        s_nav_heading_deg = APP_GAME_LAUNCH_FACE_YAW_DEG;
        chassis_ctrl_hold_yaw(APP_GAME_LAUNCH_FACE_YAW_DEG);
        is_navigating = 0;
        s_wait_phase0_ticks = 0U;
        if (s_levels_finished >= APP_GAME_TOTAL_LEVELS) {
            goto_stage(STAGE_DONE);       /* 第三关也先返回 (1,5) 再收车 */
            return;
        }
        s_wait_start_phase = 1U;
        return;
    }

    if (s_launch_startup_settle_ticks < LAUNCH_STARTUP_SETTLE_TICKS) {
        if (s_launch_startup_settle_ticks == 0U) {
            /* 上电稳定阶段保持 MODE_STOPPED；发车指令下发时再启用闭环。 */
            chassis_ctrl_stop();
        }
        s_launch_startup_settle_ticks++;
        if (s_launch_startup_settle_ticks < LAUNCH_STARTUP_SETTLE_TICKS) {
            return;
        }
    }

    s_nav_heading_deg = APP_GAME_LAUNCH_FACE_YAW_DEG;
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
        s_nav_heading_deg = APP_GAME_LAUNCH_FACE_YAW_DEG;
        chassis_ctrl_move_to_m(chassis_grid_x_to_m(APP_GAME_LAUNCH_EXIT_X),
                               chassis_grid_y_to_m(APP_GAME_LAUNCH_EXIT_Y),
                               APP_GAME_LAUNCH_FACE_YAW_DEG);
        s_launch_drive_issued = 1U;
        return;
    }

    /* 上位机网格 (2,5) 为刷新触发点，因此以该航点到达为发车成功，
     * 不再用车体外接圆完全离开旧发车区几何作为硬门槛。 */
    if (chassis_nav_arrived()) {
        chassis_ctrl_stop();
        s_launch_drive_issued = 0U;
        /* 停车后才建立收图栅栏。reset_launch_map_stability() 会同步记录
         * 此刻已完整落地的最后一帧，后续只接受帧号更晚的新地图。 */
        reset_launch_map_stability();
        s_launch_arrival_frame_id = s_launch_map_last_frame_id;
        goto_stage(STAGE_WAIT_MAP_REFRESH);
    }
}

static void stage_wait_map_refresh_handler(void)
{
    /* 到达 (2,5) 并停车后，在主控侧重新确认五张完整地图完全一致。
     * 这样冻结条件不依赖 OpenART 是否实现了地图投票。 */
    map_snapshot_release();
    if (launch_map_stability_tick(s_launch_arrival_frame_id) == 0U) {
        return;
    }

    s_map_accepted = 1U;
    clear_box_target_mapping();
    App_Recognize_Reset();
    map_snapshot_freeze();
    publish_frozen_map_for_display();
    /* 发车与等待新地图期间保持电机关闭；确认收到合法的新地图后，才按
     * 业务层记录的四正交航向开启保持。后续识别/航点命令会正常覆盖该模式。 */
    chassis_ctrl_hold_yaw(s_nav_heading_deg);
    if (current_level_number() == 1U) {
        /* 第一关不需要分类，直接让规划器复制冻结地图，省去识别空转的一拍。 */
        reset_exec_context();
        goto_stage(STAGE_PLAN_PATH);
    } else {
        goto_stage(STAGE_RECOGNIZE_MAP);
    }
}

static void stage_wait_recovery_map_handler(void)
{
    uint8 box_count;
    uint8 target_count;
    uint8 remaining_task_rescan;
    uint8 bomb_mapping_rescan;

    /* 链路恢复/部分批次重读/完整推箱复核阶段始终停车；仅在线时消费新结果。 */
    if (s_link_alive == 0U) {
        return;
    }

    if (completion_invalid_or_empty_map_received(
            s_recovery_map_baseline_frame_id) != 0U) {
        /* 全空白或非法地图无需等待完整地图的五帧稳定。 */
        s_completion_verify_active = 0U;
        s_remaining_task_rescan = 0U;
        goto_stage(STAGE_LEVEL_JUDGE);
        return;
    }

    if (recovery_map_stability_tick(s_recovery_map_baseline_frame_id) == 0U) {
        return;
    }

    s_map_accepted = 1U;
    bomb_mapping_rescan = s_bomb_mapping_rescan_active;
    if (bomb_mapping_rescan == 0U) {
        clear_box_target_mapping();
    }
    App_Recognize_Reset();
    map_snapshot_freeze();
    publish_frozen_map_for_display();

    box_count = get_map_box_count();
    target_count = get_map_target_count();
    remaining_task_rescan = s_remaining_task_rescan;
    s_remaining_task_rescan = 0U;
    s_completion_verify_active = 0U;
    s_bomb_mapping_rescan_active = 0U;

    if (bomb_mapping_rescan != 0U && box_count != 0U) {
        if (bomb_rescan_items_match_prediction() != 0U) {
            /* 实际墙体地图已经稳定，类别索引未变，直接按原映射重新解算。 */
            reset_exec_context();
            goto_stage(STAGE_PLAN_PATH);
            return;
        }
        /* 视觉结果显示箱子或目标位置异常变化，旧索引不再安全；
         * 仅这种异常情况回退到通用重新识别流程。 */
        clear_box_target_mapping();
    }

    /* 剩余任务复扫若只剩唯一箱子与目标点，其对应关系已唯一，无需再跑类别识别。 */
    if (remaining_task_rescan != 0U &&
        box_count == 1U && target_count == 1U) {
        g_box_to_target[0] = 0U;
        s_matched_box_count = 1U;
        reset_exec_context();
        goto_stage(STAGE_PLAN_PATH);
        return;
    }

    /* 掉线恢复或上一执行批次若已完成最后一箱，直接进入本关完成判定。 */
    current_stage = (box_count == 0U)
                  ? STAGE_LEVEL_JUDGE
                  : STAGE_RECOGNIZE_MAP;
}

static void stage_recognize_handler(void)
{
    /* ============================================================
     * 识别 tour 子状态机驱动 (app_recognize.c):
     *   - Stage1 无数字配对要求（有无炸弹均同）→ DONE_NO_NEED 直接放行
     *   - Stage2/3: BOX/TARGET 混合规划最短观察 Tour；当前物体确认后才切换,
     *               全部实地确认后写入最大可行 box→target 映射。类别不配平时
     *               先执行已匹配子集，再从当前空地重读图继续识别剩余项
     *   - 视觉暂时无结果时循环当前物体观察位；路线不可恢复才停车等待人工处理
     * ============================================================ */
    AppRecognizeStatus_e r = App_Recognize_Tick(g_game_map, g_player_pos,
                                                map_has_bomb(),
                                                current_level_number(),
                                                &s_nav_heading_deg,
                                                g_box_to_target,
                                                &s_matched_box_count);
    switch (r)
    {
        case APP_RECOG_RUNNING:
            return;
        case APP_RECOG_DONE_PARTIAL:
            s_partial_batch_active = 1U;
            if (s_matched_box_count == 0U) {
                /* 本轮没有任何同类可行配对：不空跑求解器，直接等新图重识别。 */
                prepare_map_rescan_wait(1U);
                return;
            }
            map_snapshot_freeze();
            reset_exec_context();
            goto_stage(STAGE_PLAN_PATH);
            return;
        case APP_RECOG_DONE_OK:
        case APP_RECOG_DONE_NO_NEED:
            s_partial_batch_active = 0U;
            s_partial_target_mask = 0U;
            /* 识别使用的是已接纳地图；无论是否清障，都保持冻结到规划器 Begin
             * 完成私有拷贝，避免两阶段之间被迟到地图帧覆盖。 */
            map_snapshot_freeze();
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

/* 破局策略顺序保持不变，但每拍只验证一个炸弹-墙体对，避免同步遍历
 * 全图墙体和全部炸弹阻塞 5ms 比赛主循环。 */
static SokoSearchStatus_e breakout_bomb_search_step(void)
{
    while (s_bomb_strategy < 3U)
    {
        if (s_bomb_search_started == 0U)
        {
            Point_t blocked_target = {-1, -1};
            uint8 strategy_available = 0U;

            if (s_bomb_strategy == 0U)
            {
                Point_t dead_box = {-1, -1};
                if (Sokoban_Is_Deadlock(g_game_map, &dead_box))
                {
                    blocked_target = choose_nearest_target(dead_box);
                    strategy_available = (uint8)(blocked_target.x >= 0);
                }
            }
            else if (s_bomb_strategy == 1U)
            {
                strategy_available = find_first_unreachable_target(&blocked_target);
            }
            else if (map_has_bomb())
            {
                strategy_available = 1U;
            }

            if (strategy_available == 0U ||
                !Sokoban_Bomb_Search_Begin(g_game_map, g_player_pos,
                                           blocked_target))
            {
                ++s_bomb_strategy;
                continue;
            }
            s_bomb_search_started = 1U;
        }

        {
            SokoSearchStatus_e status = Sokoban_Bomb_Search_Step(
                1U, &g_bomb_pos, &g_bomb_wall_pos, &g_bomb_action_seq);
            if (status == SOKO_SEARCH_RUNNING) return status;
            Sokoban_Bomb_Search_Cancel();
            s_bomb_search_started = 0U;
            if (status == SOKO_SEARCH_SOLVED) {
                if (current_level_number() == 3U) {
                    if (begin_bomb_candidate_verification() != 0U) {
                        s_plan_mode = PLAN_MODE_BOMB_VERIFY;
                    } else {
                        reject_bomb_candidate_and_resume();
                    }
                    return SOKO_SEARCH_RUNNING;
                }
                if (activate_bomb_plan() != 0U) {
                    return SOKO_SEARCH_SOLVED;
                }
            }
            ++s_bomb_strategy;
        }
    }
    return SOKO_SEARCH_FAILED;
}

static void stage_plan_handler(void)
{
    SokoSearchStatus_e search_status;
    Point_t home = { (int8)APP_GAME_LAUNCH_HOME_X,
                     (int8)APP_GAME_LAUNCH_HOME_Y };

    if (get_map_box_count() == 0U) {
        Sokoban_Stage1_Search_Cancel();
        Sokoban_Bomb_Search_Cancel();
        goto_stage(STAGE_LEVEL_JUDGE);
        return;
    }

    if (s_plan_mode == PLAN_MODE_BOMB)
    {
        search_status = breakout_bomb_search_step();
        if (search_status == SOKO_SEARCH_RUNNING) return;
        if (search_status == SOKO_SEARCH_SOLVED)
        {
            goto_stage(STAGE_EXECUTE_ACTION);
            return;
        }
        if (current_level_number() == 3U) {
            /* 第三关规划失败统一重读实际地图，不进入永久 DEADLOCK 停车。 */
            prepare_map_rescan_wait(1U);
            return;
        }
        goto_stage(STAGE_DEADLOCK_RESET);
        return;
    }
    if (s_plan_mode == PLAN_MODE_BOMB_VERIFY) return;

    if (g_soko_exec_init == 0U)
    {
        uint8 begin_ok;
        uint8 box_count = get_map_box_count();
        s_solve_succeeded = 0U;
        if (current_level_number() == 1U)
        {
            begin_ok = Sokoban_Stage1_Search_Begin(g_game_map, g_player_pos, home);
        }
        else if (s_partial_batch_active != 0U)
        {
            if (build_partial_plan_problem() == 0U) {
                prepare_map_rescan_wait(1U);
                return;
            }
            begin_ok = Sokoban_Stage2_Search_Begin(
                s_partial_plan_map, g_player_pos,
                s_partial_plan_mapping, s_matched_box_count, home);
        }
        else
        {
            begin_ok = Sokoban_Stage2_Search_Begin(
                g_game_map, g_player_pos, g_box_to_target, box_count, home);
        }
        g_soko_exec_init = 1U;
        /* Begin 已复制地图与固定映射；后续搜索不依赖视觉快照持续冻结。 */
        map_snapshot_release();
        if (begin_ok == 0U)
        {
            g_soko_exec_init = 0U;
            if (s_partial_batch_active != 0U &&
                current_level_number() == 2U) {
                prepare_map_rescan_wait(1U);
                return;
            }
            start_fresh_bomb_search();
        }
        return;
    }
}

/* 推箱搜索只在 wait_for_tick() 空档推进，避免贪心种子生成或队列压缩
 * 落入 Game_Logic_Task_Run 的 800us 实时预算。 */
static void stage_push_search_idle_step(void)
{
    SokoSearchStatus_e search_status;

    if (current_stage != STAGE_PLAN_PATH ||
        s_plan_mode != PLAN_MODE_PUSH ||
        g_soko_exec_init == 0U) {
        return;
    }

    search_status = (current_level_number() == 1U)
                  ? Sokoban_Stage1_Search_Step(1U, &g_soko_solution)
                  : Sokoban_Stage2_Search_Step(1U, &g_soko_solution);

    if (current_level_number() == 1U &&
        Sokoban_Search_Has_Incumbent() != 0U) {
        uint8 activated = rolling_activate_initial_solution();
        Sokoban_Stage1_Search_Cancel();
        g_soko_exec_init = 0U;
        if (activated != 0U) {
            goto_stage(STAGE_EXECUTE_ACTION);
            return;
        }
        start_fresh_bomb_search();
        return;
    }

    if (search_status == SOKO_SEARCH_RUNNING) { return; }
    Sokoban_Stage1_Search_Cancel();
    g_soko_exec_init = 0U;
    if (search_status == SOKO_SEARCH_SOLVED && activate_push_box_solution()) {
        goto_stage(STAGE_EXECUTE_ACTION);
        return;
    }

    if (s_partial_batch_active != 0U &&
        current_level_number() == 2U) {
        prepare_map_rescan_wait(1U);
        return;
    }
    start_fresh_bomb_search();
}

static void stage_bomb_verify_idle_step(void)
{
    SokoSearchStatus_e search_status;

    if (current_stage != STAGE_PLAN_PATH ||
        s_plan_mode != PLAN_MODE_BOMB_VERIFY ||
        g_soko_exec_init == 0U) {
        return;
    }

    search_status = Sokoban_Stage2_Search_Step(1U, &g_soko_solution);
    if (search_status == SOKO_SEARCH_RUNNING) return;

    Sokoban_Stage1_Search_Cancel();
    g_soko_exec_init = 0U;
    if (search_status == SOKO_SEARCH_SOLVED &&
        activate_bomb_plan() != 0U) {
        goto_stage(STAGE_EXECUTE_ACTION);
        return;
    }
    reject_bomb_candidate_and_resume();
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
            map_cache_invalidate();
            /* 物理爆炸后重读真实墙体地图，但保留已经确认的箱子类别映射；
             * 箱子/目标位置异常变化时，恢复阶段才会回退到重新识别。 */
            prepare_bomb_map_rescan_wait();
            return;
        }
    } else {
        goto_stage(STAGE_PLAN_PATH);
        return;
    }

    if (done) {
        if (s_partial_batch_active != 0U) {
            /* 最后一推后车位于原箱子格（空地），直接在此等待新地图。 */
            prepare_map_rescan_wait(1U);
        } else {
            /* 完整计划也可能因识别或执行偏差留下箱子，先用新地图复核再判定通关。 */
            prepare_completion_verify_wait();
        }
    }
}

static void stage_level_judge_handler(void)
{
    /* 各关均在复核新图已无箱子/目标，或完整推箱后的新结果不是合法地图时到达此处。
     * 所有关卡均保持 0° 返回 (1,5)；
     * 前两关随后再驶到 (2,5) 触发新图，第三关返航后收车。 */
    chassis_ctrl_stop();
    mark_current_level_finished();

    reset_exec_context();
    map_snapshot_release();
    App_Recognize_Reset();
    goto_stage(STAGE_WAIT_START);
}

static void stage_deadlock_reset_handler(void)
{
    /* 死局自动返航/重置暂时关闭。goto_stage() 进入本状态时已停车，
     * 此处不增加新动作，等待人工处理或重启。 */
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
 *   - OpenART1 MAP 链路只在收图和恢复阶段是必需链路；地图冻结后的识别/执行
 *     使用主控本地快照
 *   - OpenART2 BOX_CLASS 链路只在车已对准物体的 SAMPLE 子阶段必需；
 *     INIT/NAV/FACE 不能因尚无分类帧而阻止车辆到达观察位
 *   - 用 LINK_LOSS_MS / LINK_OK_MS 做迟滞判定
 *   - 状态翻转时:
 *       必需阶段 OK -> LOSS : 保存 current_stage 到 s_stage_resume, 切到 PAUSE, 立即停车
 *       规划/执行期 MAP LOSS: 只记录链路离线，继续使用已接纳地图和已下发航点
 *       必需阶段 LOSS -> OK: 等待新地图并重新识别/规划
 *       可选阶段 LOSS -> OK: 仅恢复链路在线标志，不重置当前执行上下文
 *
 * stage_pause_on_link_loss_handler():
 *   PAUSE 状态下不做任何业务逻辑, 只是周期性确保电机维持在停车状态
 *   (chassis_ctrl_stop 已是幂等, 但为避免反复清 PID 积分, 这里只在
 *    LOSS 触发瞬间调用一次, handler 内不再重复调.)
 * ================================================================== */

/* ===========================================================================
 *  § 5. 外部查询 API + 链路守护 + 任务主入口
 * ========================================================================== */

uint8 Game_Link_Is_Alive(void)
{
    return s_link_alive;
}

void Game_Get_Runtime_Status(GameRuntimeStatus_t *out)
{
    if (out == NULL) {
        return;
    }

    out->stage = current_stage;
    out->map_accepted = s_map_accepted;
    out->solve_succeeded = s_solve_succeeded;
    out->waypoint_issued = 0U;
    out->waypoint_index = 0U;
    out->waypoint_count = 0U;

    if (g_exec_mode == EXEC_PUSH_BOX) {
        out->waypoint_index = g_soko_wp_idx;
        out->waypoint_count = g_soko_waypoints.count;
        out->waypoint_issued = is_navigating;
    } else if (g_exec_mode == EXEC_PUSH_BOMB) {
        out->waypoint_index = g_bomb_wp_idx;
        out->waypoint_count = g_bomb_waypoints.count;
        out->waypoint_issued = is_navigating;
    }
}

uint8 Game_Get_Frozen_Map(uint8 out[MAP_ROWS][MAP_COLS], uint32 *generation)
{
    if ((out == NULL) || (generation == NULL) ||
        (s_display_frozen_map_generation == 0U)) {
        return 0U;
    }

    copy_map(out, s_display_frozen_map);
    *generation = s_display_frozen_map_generation;
    return 1U;
}

/* ==================================================================
 * 【B17】对外查询: 识别 tour 进度 (转发自 App_Recognize_Get_Debug)
 * 用途: 菜单 / IPS / 上位机显示当前识别到第几个物体和有效采样数等.
 * ================================================================== */
void Game_Get_Recognize_Debug(AppRecognizeDebug_t *out)
{
    App_Recognize_Get_Debug(out);
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
    AppRecognizeDebug_t debug;

    if (current_stage != STAGE_RECOGNIZE_MAP)
    {
        return 0U;
    }

    /* OpenART2 may not produce a class frame until the car reaches an
     * observation point and faces the pattern. Requiring that link during
     * INIT/NAV/FACE creates a circular wait: the car is stopped before it can
     * move into view. */
    App_Recognize_Get_Debug(&debug);
    return (uint8)((current_level_number() >= 2U) &&
                   (debug.sub_state == RECOG_SUB_SAMPLE));
}

static uint8 current_stage_needs_map_link(void)
{
    /* 五帧地图一旦接纳即由主控冻结。识别巡航、规划和执行都只依赖该本地
     * 快照；若继续把 RECOGNIZE 绑定到地图心跳，车离开 (2,5) 后地图端静默
     * 1s 就会停车并错误地重新等待五帧地图。 */
    return (uint8)((current_stage == STAGE_WAIT_MAP_REFRESH) ||
                   (current_stage == STAGE_WAIT_RECOVERY_MAP));
}

static void enter_link_pause(LinkPauseReason_e reason)
{
    /* 返回 (1,5)、驶向 (2,5)、失败返航和最终驻停均不依赖地图内容。
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
    LinkPauseReason_e recovered_reason = s_link_pause_reason;

    s_link_alive = 1U;
    s_link_pause_reason = LINK_PAUSE_NONE;
    if (current_stage == STAGE_WAIT_RECOVERY_MAP)
    {
        /* 恢复等待期再次掉线时，已累计的稳定帧全部作废；
         * 必须从最近一次链路恢复后重新收集。 */
        s_recovery_map_baseline_frame_id = g_link_map_frame_id;
        reset_launch_map_stability();
        s_launch_map_last_frame_id = s_recovery_map_baseline_frame_id;
        s_completion_map_len_err_baseline =
            g_link_map_stats.frames_len_err;
        return;
    }
    if (current_stage == STAGE_PAUSE_ON_LINK_LOSS)
    {
        chassis_ctrl_stop();
        is_navigating  = 0U;
        if ((recovered_reason == LINK_PAUSE_CLASS) &&
            (s_stage_resume == STAGE_RECOGNIZE_MAP))
        {
            /* Class loss is only armed in SAMPLE, where the car is already
             * stopped and facing the object. Resume that sampling window
             * instead of discarding the accepted map and restarting tour. */
            current_stage = STAGE_RECOGNIZE_MAP;
            return;
        }
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
            /* 重新下发 (2,5) 目标；到站后再建立本轮收图栅栏。 */
            s_launch_drive_issued = 0U;
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
        prepare_map_rescan_wait(0U);
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
    uint8 class_loss = (uint8)((class_seen != 0U) &&
                               (class_silence_ms > LINK_LOSS_MS) &&
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
            if (current_stage_needs_map_link() != 0U) {
                enter_link_pause(LINK_PAUSE_MAP);
            } else {
                /* 规划和航点执行只依赖已接纳的本地状态。记录物理掉线但不停车，
                 * 避免短暂视觉静默清空解算结果并反复进入 RECOVERY。 */
                s_link_alive = 0U;
                s_link_pause_reason = LINK_PAUSE_MAP;
            }
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

    /* P0-3: 链路在线时刷新 g_game_map 私有快照 (seq-lock 拷贝).
     *       链路 LOSS 期间不刷新；链路恢复后在 WAIT_RECOVERY_MAP 重取稳定新图。
     *       上电首帧到达前 s_link_alive=0, g_game_map 维持 BSS 0 = MAP_EMPTY, 业务侧无副作用.
     *
     * B1+B12: s_map_freeze=1 时 (识别 tour / 炸弹爆炸后) 跳过拷贝, 由主控本地权威.
     */
    if (s_link_alive && !s_map_freeze &&
        g_link_map_frame_id != s_map_snapshot_frame_id)
    {
        uint32 frame_id_before = g_link_map_frame_id;
        app_link_get_map_snapshot(g_game_map);
        if (g_link_map_frame_id == frame_id_before)
        {
            s_map_snapshot_frame_id = frame_id_before;
            map_cache_invalidate();
        }
    }

    /* 底盘位姿只来自编码器里程计与陀螺仪；视觉只更新地图和分类。 */
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

uint8 Game_Logic_Idle_Plan_Step(void)
{
    if (current_stage == STAGE_PLAN_PATH) {
        if (s_plan_mode == PLAN_MODE_PUSH && g_soko_exec_init != 0U) {
            stage_push_search_idle_step();
        } else if (s_plan_mode == PLAN_MODE_BOMB_VERIFY &&
                   g_soko_exec_init != 0U) {
            stage_bomb_verify_idle_step();
        } else {
            stage_plan_handler();
        }
        return 1U;
    }

    return rolling_background_idle_step();
}
