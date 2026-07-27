/*********************************************************************************************************************
 * 文件名称   : app_recognize.c
 * 模块功能   : 推箱子识别 tour 子状态机 (STAGE_RECOGNIZE_MAP 的真正实现)
 *
 * @owner     rt1064-main
 * @periph    none                纯算法 (地图特征提取 + 箱子-目标匹配)
 *--------------------------------------------------------------------------------------------------------------------
 * 算法概述:
 *   1. 从地图提取所有 BOX 和 TARGET 坐标 (Stage1 跳过本流程)
 *   2. BOX/TARGET 统一按“平移格数 + 四向转角”规划观察 tour，不固定类型顺序
 *   3. 对每个物体:
 *        - 在与物体 4-邻接的可立足格中，由 Tour 选择兼顾路程和转角的观察点
 *        - 保持状态机记录的 90° 基准航向移动到观察点，仅依赖里程计到位
 *        - 按“物体格-观察格”旋转到 0/90/180/-90° 朝向物体
 *        - 转向到位后再静稳一小段时间才开始采样，避免拖影帧计票
 *        - 主控发送当前 BOX/TARGET 类型与 request_id；视觉端只运行对应模型，
 *          主控校验请求、类型和类别约束后立即采信最终结论
 *        - 采样超时后标记当前观察方向失败，将该物体放回未确认集合；
 *          重新对全部未确认 BOX/TARGET 规划混合 Tour，路线合适时再从
 *          该物体的其他方向观察，避免为追单个远观察位反复穿场
 *        - 保持采样后的车头角, 直接继续下一个物体
 *   4. 全部箱子与目标逐一实地确认（不做"最后一个目标"排除法推断——实测不稳定，
 *      识别不到/存疑时改换观察方向并与其他待观察物体统一排程）；
 *      随后按相同 class_id 与静态推送可达性配对生成
 *      g_box_to_target[]，避免仅按直线距离把贴边箱分给无法起推的目标
 *
 * 资源:
 *   - 全部 static (BSS), 不可重入, 不可在 ISR 调用
 *   - 复用 algo_sokoban_solver 内部 BFS (Algo_Nav_BFS) 找观察点路径长度
 *********************************************************************************************************************/

#include "app_recognize.h"
#include "app_recognize_clear.h"
#include "app_link.h"
#include "chassis_ctrl.h"
#include "chassis_config.h"
#include <string.h>

/*===================================================================================================================
 * 调参 (集中, 后续可挪到 chassis_config.h)
 *=================================================================================================================*/

/** OpenART2 最长 1s 完成快速确认/降级投票；主控保留 1200ms 硬上限，
 *  覆盖结果帧偶发丢失、请求重发和视觉端单次推理抖动。 */
#define RECOG_SAMPLE_TIMEOUT_MS        (1200U)

/** SAMPLE 中定期重发同一请求，覆盖 OpenART2 重启或请求帧偶发损坏。 */
#define RECOG_REQUEST_RETRY_MS         (200U)

/** 5ms 周期 */
#define RECOG_TICK_MS                  (5U)

/** 单个分类帧最大年龄；超过该值不得计入当前观察点的连续确认。 */
#define RECOG_CLASS_FRAME_MAX_AGE_MS   (250U)

/** B3a: 单点 NAV 最短时限 5s，长直线按每格 1.5s 线性放宽。 */
#define RECOG_NAV_TIMEOUT_TICKS        (1000U)
#define RECOG_NAV_TICKS_PER_CELL       (300U)

/** B3a: 单点 FACE (旋转到位) 最长时长 — 3s */
#define RECOG_FACE_TIMEOUT_TICKS       (600U)

/** FACE 到位后的静稳门：到位判据允许 |yaw_rate|<10°/s 的残余转动，
 *  立即采样会把拖影/未稳帧计入确认。要求到位状态连续保持 150ms 才开始采样。 */
#define RECOG_FACE_SETTLE_TICKS        (30U)

/** 单个物体的恢复预算：采样/NAV/FACE 失败每标记一次观察方向计 1 次，
 *  第 7 次失败时整轮 FAIL 交回 DEADLOCK_RESET。预算按物体单独保存，
 *  使失败物体被全局 Tour 延后期间不会丢失已用恢复次数。 */
#define RECOG_OBJECT_RECOVER_LIMIT     (6U)

/** 单个观察位的 SAMPLE 最大 tick 数 */
#define RECOG_SAMPLE_MAX_TICKS         (RECOG_SAMPLE_TIMEOUT_MS / RECOG_TICK_MS)
#define RECOG_REQUEST_RETRY_TICKS      (RECOG_REQUEST_RETRY_MS / RECOG_TICK_MS)

/** 从 SOKOBAN_MAX_BOXES 借用的目标列表上限 (理论上 boxes==targets) */
#define RECOG_MAX_TARGETS              (SOKOBAN_MAX_BOXES)

/** class_id 的有效编号上限 (1..N), 对齐 openart2 的 10 类箱子/目标 */
#define RECOG_CLASS_ID_MAX             (10U)

/** 小规模识别 tour 精确搜索上限: 3 箱 + 3 目标。更大地图用 BFS 贪心控制耗时。 */
#define RECOG_EXACT_TOUR_ITEM_LIMIT    (6U)

#define RECOG_TOUR_MASK_COUNT          (1U << RECOG_EXACT_TOUR_ITEM_LIMIT)
#define RECOG_TOUR_MAX_CANDIDATES      (RECOG_EXACT_TOUR_ITEM_LIMIT * 4U)
#define RECOG_ROUTE_COST_INF           (0xFFFFU)

/** 90° 观察转向的等效路程代价（格）。当前 120°/s 角速度下，单次 90°
 *  转向连同加减速和到位稳定明显慢于单格平移；取 4 格用于优先减少大角度
 *  转向，同时仍允许绕行代价较大时选择更近的观察格。 */
#define RECOG_TOUR_QUARTER_TURN_COST   (4U)

/*===================================================================================================================
 * 内部数据结构
 *=================================================================================================================*/

typedef struct
{
    Point_t pos;            /* 物体网格坐标                          */
    Point_t observe;        /* 观察点 (与 pos 4-邻接的可立足格)        */
    uint8   kind;           /* APP_LINK_OBJ_KIND_BOX / TARGET        */
    uint8   class_id;       /* 连续确认输出的类别; 0 = 未识别        */
    uint8   visited;        /* 1 = 已实测确认, 不再选                  */
    uint8   ok;             /* 1 = class_id 已确定 (≠0)              */
    uint8   failed_observe_mask; /* 已失败观察方向 bit0..3              */
    uint8   recover_count;  /* 该物体累计失败观察次数              */
} RecogItem_t;

static AppRecognizeSub_e s_sub_state    = RECOG_SUB_INIT;
static RecogItem_t       s_items[2 * RECOG_MAX_TARGETS];
static uint8             s_item_count   = 0U;
static uint8             s_box_count    = 0U;
static uint8             s_target_count = 0U;
static uint8             s_cur_idx      = 0U;     /* s_items 中当前处理项 */

/* 子阶段状态 */
static uint8  s_nav_started     = 0U;
static uint8  s_face_started    = 0U;
static uint8  s_face_settle_ticks = 0U;   /* FACE 到位后已连续保持到位的 tick 数 */
static uint16 s_subphase_ticks  = 0U;     /* B3a: NAV/FACE 子阶段计时, enter_sub_* 时清零 */
static uint16 s_nav_timeout_limit = RECOG_NAV_TIMEOUT_TICKS;
static AppRecogClearPlan_t s_nav_plan;
static SokoWaypointPath_t  s_nav_waypoints;
static uint16  s_nav_wp_idx     = 0U;
static uint16  s_nav_actions_applied = 0U;
static Point_t s_nav_apply_player;
static uint8   s_map_changed     = 0U;
static uint8   s_nav_replay_map[MAP_ROWS][MAP_COLS];

/* 当前识别请求与最终结果统计 */
static uint16 s_sample_ticks    = 0U;
static uint16 s_sample_total    = 0U;
static uint32 s_last_seen_frame_id = 0U;     /* 已处理过的最大 frame_id, 防重复采信 */
static uint8  s_active_request_id = 0U;
static uint8  s_request_sequence = 0U;       /* 不随单轮 Reset 清零，避免迟到帧重新命中 */
static uint16 s_request_retry_ticks = 0U;

/* 识别 tour 选点 scratch: 小规模 DP、父节点和时间距离约 8KB BSS。 */
static uint8  s_pick_distance[MAP_ROWS][MAP_COLS];
static uint16 s_pick_time_cost[MAP_ROWS][MAP_COLS];
static uint16 s_tour_start_cost[RECOG_TOUR_MAX_CANDIDATES];
static uint16 s_tour_edge_cost[RECOG_TOUR_MAX_CANDIDATES][RECOG_TOUR_MAX_CANDIDATES];
static uint16 s_tour_dp[RECOG_TOUR_MASK_COUNT][RECOG_TOUR_MAX_CANDIDATES];
static uint8  s_tour_first[RECOG_TOUR_MASK_COUNT][RECOG_TOUR_MAX_CANDIDATES];
static uint8  s_tour_parent[RECOG_TOUR_MASK_COUNT][RECOG_TOUR_MAX_CANDIDATES];
static uint8  s_tour_cached_item[RECOG_EXACT_TOUR_ITEM_LIMIT];
static Point_t s_tour_cached_observe[RECOG_EXACT_TOUR_ITEM_LIMIT];
static uint8  s_tour_cached_count;
static uint8  s_tour_cached_next;
static AppRecogClearPlan_t s_pick_clear_plan;
static NavPath_t s_pick_nav_path;
/* 重复类别匹配 scratch：反向推箱 BFS 共用一张距离图和一个 12×16 队列，约 0.6KB BSS。 */
static Point_t s_match_queue[MAP_ROWS * MAP_COLS];
static uint16  s_match_cost[SOKOBAN_MAX_BOXES][SOKOBAN_MAX_BOXES];

/*===================================================================================================================
 * 内部工具
 *=================================================================================================================*/

/**
 * 根据相邻观察格与物体的关系计算"车头朝物体"的四正交目标 yaw (度).
 *  - 车体 +Y 为前进方向, +X 为右
 *  - 地图 +X 向右、+Y 向下
 *  - 观察点必须与物体 4-邻接；不使用里程计浮点位置，避免位置漂移
 *    被 atan2 放大成任意观察角。
 */
static uint8 recog_face_heading_index(Point_t observe, Point_t target,
                                      uint8 *heading_index_out)
{
    int16 dx;
    int16 dy;

    if (heading_index_out == NULL) { return 0U; }

    dx = (int16)target.x - (int16)observe.x;
    dy = (int16)target.y - (int16)observe.y;
    if ((dx != 0) && (dy != 0)) { return 0U; }   /* 非同轴 */

    if ((dx == 0) && (dy > 0))  { *heading_index_out = 0U; return 1U; }
    if ((dx > 0) && (dy == 0))  { *heading_index_out = 1U; return 1U; }
    if ((dx == 0) && (dy < 0))  { *heading_index_out = 2U; return 1U; }
    if ((dx < 0) && (dy == 0))  { *heading_index_out = 3U; return 1U; }

    return 0U;   /* dx==0 且 dy==0：观察格与物体重合 */
}

static uint8 recog_nav_heading_index(float yaw_deg)
{
    float snapped = chassis_snap_yaw_to_cardinal_deg(yaw_deg);

    if (snapped > 45.0f && snapped < 135.0f) { return 1U; }
    if (snapped >= 135.0f || snapped <= -135.0f) { return 2U; }
    if (snapped < -45.0f) { return 3U; }
    return 0U;
}

static uint8 recog_quarter_turns(uint8 from_heading, uint8 to_heading)
{
    uint8 diff = (from_heading > to_heading)
               ? (uint8)(from_heading - to_heading)
               : (uint8)(to_heading - from_heading);
    return (diff > 2U) ? (uint8)(4U - diff) : diff;
}

static uint8 recog_calc_face_yaw_deg(Point_t observe, Point_t target,
                                     float *yaw_deg_out)
{
    static const float yaw_deg[4] = {0.0f, 90.0f, 180.0f, -90.0f};
    uint8 heading_index;

    if (yaw_deg_out == NULL ||
        !recog_face_heading_index(observe, target, &heading_index))
    {
        return 0U;
    }
    *yaw_deg_out = yaw_deg[heading_index];
    return 1U;
}

static void recog_move_to_grid_keep_nav_heading(Point_t target,
                                                float nav_heading_deg)
{
    /* 航向由上层状态机显式维护；不读取当前 IMU 后重新选择最近基准角。 */
    chassis_ctrl_move_to_m(chassis_grid_x_to_m((uint8)target.x),
                           chassis_grid_y_to_m((uint8)target.y),
                           nav_heading_deg);
}

/* 识别巡航的中间点与最终观察点均只使用编码器和陀螺仪到位。 */
static uint8 recog_nav_arrived(void)
{
    return chassis_ctrl_is_arrived();
}

static uint8 class_assignment_still_possible(uint8 kind, uint8 class_id);

/*===================================================================================================================
 * 指定类型识别请求 — 采信视觉端最终结论
 *=================================================================================================================*/

static void sample_state_reset(void)
{
    app_link_box_class_snapshot_t snap;

    s_sample_ticks = 0U;
    s_sample_total = 0U;
    s_request_retry_ticks = 0U;
    /* 丢弃转向完成前已经落地的最后一帧，只处理进入 SAMPLE 后的新帧。 */
    app_link_get_box_class_snapshot(&snap);
    s_last_seen_frame_id = (snap.valid != 0U) ? snap.frame_id : 0U;
}

/**
 * @return  >0 = 视觉端已确认的 class_id;
 *           0 = 还在采样;
 *          -2 = 视觉汇总失败/类型冲突/采样超时，需要更换观察方向
 */
static int16 sample_confirm_step(uint8 expect_kind)
{
    app_link_box_class_snapshot_t snap;
    uint32 now_ms;

    app_link_get_box_class_snapshot(&snap);
    now_ms = app_link_get_ms();
    s_sample_ticks++;
    s_request_retry_ticks++;

    /* 视觉端只在快速确认或降级投票结束后发送结论；主控只负责
     * request_id、类型和类别约束校验，不再对同一缓存结论重复投票。 */
    if ((snap.valid != 0U) && (snap.frame_id != s_last_seen_frame_id))
    {
        s_last_seen_frame_id = snap.frame_id;

        if ((uint32)(now_ms - snap.stamp_ms) > RECOG_CLASS_FRAME_MAX_AGE_MS)
        {
            return 0;
        }

        if (snap.request_id != s_active_request_id)
        {
            /* 旧观察点迟到帧：忽略。 */
        }
        else
        {
            if (s_sample_total < 0xFFFFU)
            {
                s_sample_total++;
            }

            /* class_id=0 只由视觉端在 1s 汇总仍失败时发送；
             * 类型不符同样说明当前观察画面不可靠，立即换观察方向。 */
            if ((snap.class_id == 0U) ||
                (snap.obj_kind != expect_kind) ||
                (snap.class_id > (uint8)RECOG_CLASS_ID_MAX) ||
                !class_assignment_still_possible(expect_kind, snap.class_id))
            {
                return -2;
            }
            return (int16)snap.class_id;
        }
    }

    if (s_request_retry_ticks >= (uint16)RECOG_REQUEST_RETRY_TICKS)
    {
        app_link_send_recog_request(expect_kind, s_active_request_id);
        s_request_retry_ticks = 0U;
    }

    /* 视觉端结果帧丢失或未运行时的硬故障退出上限。 */
    if (s_sample_ticks >= (uint16)RECOG_SAMPLE_MAX_TICKS)
    {
        return -2;
    }

    return 0;
}

/*===================================================================================================================
 * 物体提取 + 访问顺序
 *=================================================================================================================*/

/**
 * 检查识别 tour 是否已经"事实完成":
 * 所有 box / target 都 ok=1.
 */
static uint8 all_resolved(void)
{
    if (s_item_count == 0U) { return 0U; }
    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (!s_items[i].ok) { return 0U; }
    }
    return 1U;
}

/*
 * 混合 Tour 允许先观察任一类型，因此不能再要求“全部箱子已知后目标才有效”。
 * 这里按尚未识别的异类槽位保留容量：当前类别至少仍存在一种完成两侧多重集
 * 配平的可能时才接受，避免已知结果明显超配。
 */
static uint8 class_assignment_still_possible(uint8 kind, uint8 class_id)
{
    uint8 box_matches = 0U;
    uint8 target_matches = 0U;
    uint8 unresolved_boxes = 0U;
    uint8 unresolved_targets = 0U;

    if (class_id == 0U || class_id > (uint8)RECOG_CLASS_ID_MAX) { return 0U; }
    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (s_items[i].kind == APP_LINK_OBJ_KIND_BOX)
        {
            if (!s_items[i].ok) { ++unresolved_boxes; }
            else if (s_items[i].class_id == class_id) { ++box_matches; }
        }
        else if (s_items[i].kind == APP_LINK_OBJ_KIND_TARGET)
        {
            if (!s_items[i].ok) { ++unresolved_targets; }
            else if (s_items[i].class_id == class_id) { ++target_matches; }
        }
    }

    if (kind == APP_LINK_OBJ_KIND_BOX)
    {
        return (uint8)(box_matches < (uint8)(target_matches + unresolved_targets));
    }
    if (kind == APP_LINK_OBJ_KIND_TARGET)
    {
        return (uint8)(target_matches < (uint8)(box_matches + unresolved_boxes));
    }
    return 0U;
}

static void extract_items(const uint8 map[MAP_ROWS][MAP_COLS])
{
    s_item_count   = 0U;
    s_box_count    = 0U;
    s_target_count = 0U;

    for (int8 r = (int8)CHASSIS_GRID_INNER_MIN_Y; r <= (int8)CHASSIS_GRID_INNER_MAX_Y; ++r)
    {
        for (int8 c = (int8)CHASSIS_GRID_INNER_MIN_X; c <= (int8)CHASSIS_GRID_INNER_MAX_X; ++c)
        {
            if ((s_item_count >= (uint8)(2U * RECOG_MAX_TARGETS))) { continue; }
            if (map[r][c] == MAP_BOX)
            {
                s_items[s_item_count].pos.x = c;
                s_items[s_item_count].pos.y = r;
                s_items[s_item_count].kind  = APP_LINK_OBJ_KIND_BOX;
                s_items[s_item_count].class_id = 0U;
                s_items[s_item_count].visited  = 0U;
                s_items[s_item_count].ok       = 0U;
                s_items[s_item_count].failed_observe_mask = 0U;
                s_items[s_item_count].recover_count = 0U;
                ++s_item_count;
                ++s_box_count;
            }
            else if (map[r][c] == MAP_TARGET)
            {
                s_items[s_item_count].pos.x = c;
                s_items[s_item_count].pos.y = r;
                s_items[s_item_count].kind  = APP_LINK_OBJ_KIND_TARGET;
                s_items[s_item_count].class_id = 0U;
                s_items[s_item_count].visited  = 0U;
                s_items[s_item_count].ok       = 0U;
                s_items[s_item_count].failed_observe_mask = 0U;
                s_items[s_item_count].recover_count = 0U;
                ++s_item_count;
                ++s_target_count;
            }
        }
    }
}

static uint8 recog_is_observe_standable(const uint8 map[MAP_ROWS][MAP_COLS],
                                        int8 y,
                                        int8 x)
{
    if (x < (int8)CHASSIS_GRID_INNER_MIN_X ||
        x > (int8)CHASSIS_GRID_INNER_MAX_X ||
        y < (int8)CHASSIS_GRID_INNER_MIN_Y ||
        y > (int8)CHASSIS_GRID_INNER_MAX_Y)
    {
        return 0U;
    }
    return (uint8)(map[y][x] == MAP_EMPTY || map[y][x] == MAP_TARGET);
}

static uint16 observe_route_cost_from_flood(const uint8 map[MAP_ROWS][MAP_COLS],
                                            const uint16 time_cost[MAP_ROWS][MAP_COLS],
                                            Point_t object,
                                            uint8 failed_observe_mask,
                                            uint8 start_heading,
                                            Point_t *best_observe)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    uint16 best = RECOG_ROUTE_COST_INF;

    for (uint8 d = 0U; d < 4U; ++d)
    {
        int8 y = (int8)(object.y + dr[d]);
        int8 x = (int8)(object.x + dc[d]);
        Point_t candidate;
        uint8 heading;
        uint16 cost;

        if ((failed_observe_mask & (uint8)(1U << d)) != 0U ||
            !recog_is_observe_standable(map, y, x)) { continue; }
        if (time_cost[y][x] >= (uint16)ALGO_NAV_TIME_COST_UNREACHABLE) { continue; }

        candidate.x = x;
        candidate.y = y;
        if (!recog_face_heading_index(candidate, object, &heading)) { continue; }
        cost = time_cost[y][x] +
               (uint16)recog_quarter_turns(start_heading, heading) *
               (uint16)RECOG_TOUR_QUARTER_TURN_COST;
        if (cost < best)
        {
            best = cost;
            if (best_observe != NULL) { *best_observe = candidate; }
        }
    }
    return best;
}

/* 观察格所在方向位 (bit0..3 = 上/下/左/右)。 */
static uint8 observe_direction_bit(Point_t object, Point_t observe)
{
    int16 dx = (int16)observe.x - (int16)object.x;
    int16 dy = (int16)observe.y - (int16)object.y;

    if ((dx != 0) && (dy != 0)) { return 0U; }
    if ((dx == 0) && (dy < 0))  { return (uint8)(1U << 0); }
    if ((dx == 0) && (dy > 0))  { return (uint8)(1U << 1); }
    if ((dx < 0) && (dy == 0))  { return (uint8)(1U << 2); }
    if ((dx > 0) && (dy == 0))  { return (uint8)(1U << 3); }
    return 0U;
}

static void mark_current_observe_failed(void)
{
    uint8 bit;
    if (s_cur_idx >= s_item_count) { return; }
    bit = observe_direction_bit(s_items[s_cur_idx].pos,
                                s_items[s_cur_idx].observe);
    s_items[s_cur_idx].failed_observe_mask |= bit;
    if (s_items[s_cur_idx].recover_count < 255U)
    {
        s_items[s_cur_idx].recover_count++;
    }
    /* 原缓存 Tour 可能仍指向刚失败的观察格，必须全部重算。 */
    s_tour_cached_count = 0U;
    s_tour_cached_next = 0U;
}

/* 恢复预算耗尽判定；四个恢复分支在 mark 后统一检查。 */
static uint8 recog_object_recover_exhausted(void)
{
    if (s_cur_idx >= s_item_count) { return 1U; }
    return (uint8)(s_items[s_cur_idx].recover_count >
                   (uint8)RECOG_OBJECT_RECOVER_LIMIT);
}

/**
 * 小规模精确 tour:
 *   - 节点是每个未访问物体的所有可站观察格;
 *   - DP 状态为 (已访问物体集合, 当前观察格), 求平移与四向转角的最小总代价;
 *   - 同时输出首个物体和观察格，执行阶段不得丢弃 Tour 选定的观察方向。
 */
static uint8 pick_exact_direct_tour(const uint8 map[MAP_ROWS][MAP_COLS],
                                    Point_t cur,
                                    float nav_heading_deg,
                                    uint8 *out_idx,
                                    Point_t *out_observe)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    uint8 item_idx[RECOG_EXACT_TOUR_ITEM_LIMIT];
    uint8 cand_item[RECOG_TOUR_MAX_CANDIDATES];
    uint8 cand_heading[RECOG_TOUR_MAX_CANDIDATES];
    Point_t cand_pos[RECOG_TOUR_MAX_CANDIDATES];
    uint8 item_count = 0U;
    uint8 cand_count = 0U;
    uint8 full_mask;
    uint16 best_total = RECOG_ROUTE_COST_INF;
    uint8 best_end_candidate = 0xFFU;
    uint8 start_heading = recog_nav_heading_index(nav_heading_deg);

    if (out_idx == NULL || out_observe == NULL) { return 0U; }

    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (s_items[i].visited) { continue; }
        if (item_count >= (uint8)RECOG_EXACT_TOUR_ITEM_LIMIT) { return 0U; }
        item_idx[item_count++] = i;
    }
    if (item_count == 0U) { return 0U; }

    for (uint8 local = 0U; local < item_count; ++local)
    {
        Point_t object = s_items[item_idx[local]].pos;
        uint8 has_candidate = 0U;
        for (uint8 d = 0U; d < 4U; ++d)
        {
            int8 y = (int8)(object.y + dr[d]);
            int8 x = (int8)(object.x + dc[d]);
            if ((s_items[item_idx[local]].failed_observe_mask &
                 (uint8)(1U << d)) != 0U) { continue; }
            if (!recog_is_observe_standable(map, y, x)) { continue; }
            if (cand_count >= (uint8)RECOG_TOUR_MAX_CANDIDATES) { return 0U; }
            cand_pos[cand_count].x = x;
            cand_pos[cand_count].y = y;
            cand_item[cand_count] = local;
            if (!recog_face_heading_index(cand_pos[cand_count], object,
                                          &cand_heading[cand_count]))
            {
                return 0U;
            }
            cand_count++;
            has_candidate = 1U;
        }
        if (!has_candidate) { return 0U; }
    }
    if (cand_count == 0U) { return 0U; }

    memset(s_tour_dp, 0xFF, sizeof(s_tour_dp));
    memset(s_tour_first, 0xFF, sizeof(s_tour_first));
    memset(s_tour_parent, 0xFF, sizeof(s_tour_parent));
    memset(s_tour_start_cost, 0xFF, sizeof(s_tour_start_cost));
    memset(s_tour_edge_cost, 0xFF, sizeof(s_tour_edge_cost));

    if (!Algo_Nav_Time_Flood(map, cur, SOKO_ACT_NONE,
                             s_pick_time_cost, NULL)) { return 0U; }
    for (uint8 c = 0U; c < cand_count; ++c)
    {
        s_tour_start_cost[c] = s_pick_time_cost[cand_pos[c].y][cand_pos[c].x];
    }

    for (uint8 from = 0U; from < cand_count; ++from)
    {
        if (!Algo_Nav_Time_Flood(map, cand_pos[from], SOKO_ACT_NONE,
                                 s_pick_time_cost, NULL)) { continue; }
        for (uint8 to = 0U; to < cand_count; ++to)
        {
            s_tour_edge_cost[from][to] =
                s_pick_time_cost[cand_pos[to].y][cand_pos[to].x];
        }
    }

    for (uint8 c = 0U; c < cand_count; ++c)
    {
        uint8 item_bit = (uint8)(1U << cand_item[c]);
        if (s_tour_start_cost[c] >= (uint16)ALGO_NAV_TIME_COST_UNREACHABLE) { continue; }
        s_tour_dp[item_bit][c] = (uint16)s_tour_start_cost[c] +
            (uint16)recog_quarter_turns(start_heading, cand_heading[c]) *
            (uint16)RECOG_TOUR_QUARTER_TURN_COST;
        /* 必须保留首个候选观察格，而不只是物体索引；否则执行阶段重新
         * 选择最近观察格，会破坏 DP 已计算出的全局 Tour。 */
        s_tour_first[item_bit][c] = c;
    }

    full_mask = (uint8)((1U << item_count) - 1U);
    for (uint8 mask = 1U; mask <= full_mask; ++mask)
    {
        for (uint8 from = 0U; from < cand_count; ++from)
        {
            uint16 base_cost = s_tour_dp[mask][from];
            if (base_cost >= RECOG_ROUTE_COST_INF) { continue; }

            for (uint8 to = 0U; to < cand_count; ++to)
            {
                uint8 to_bit = (uint8)(1U << cand_item[to]);
                uint8 next_mask;
                uint16 next_cost;

                if ((mask & to_bit) != 0U) { continue; }
                if (s_tour_edge_cost[from][to] >=
                    (uint16)ALGO_NAV_TIME_COST_UNREACHABLE) { continue; }

                next_mask = (uint8)(mask | to_bit);
                next_cost = (uint16)(base_cost +
                    (uint16)s_tour_edge_cost[from][to] +
                    (uint16)recog_quarter_turns(cand_heading[from], cand_heading[to]) *
                    (uint16)RECOG_TOUR_QUARTER_TURN_COST);
                if (next_cost < s_tour_dp[next_mask][to])
                {
                    s_tour_dp[next_mask][to] = next_cost;
                    s_tour_first[next_mask][to] = s_tour_first[mask][from];
                    s_tour_parent[next_mask][to] = from;
                }
            }
        }
    }

    for (uint8 c = 0U; c < cand_count; ++c)
    {
        uint16 terminal_cost = RECOG_ROUTE_COST_INF;
        uint16 total_cost;
        if (s_tour_dp[full_mask][c] >= RECOG_ROUTE_COST_INF) { continue; }

        /* 识别结束后立即进入推箱；把最后观察点到任一箱子观察侧的时间
         * 作为终端下界，避免 Tour 为省最后一步而停在远离全部箱子的目标旁。 */
        for (uint8 to = 0U; to < cand_count; ++to)
        {
            uint8 global_item = item_idx[cand_item[to]];
            if (s_items[global_item].kind != APP_LINK_OBJ_KIND_BOX) { continue; }
            if (s_tour_edge_cost[c][to] < terminal_cost)
            {
                terminal_cost = s_tour_edge_cost[c][to];
            }
        }
        if (terminal_cost >= RECOG_ROUTE_COST_INF) { terminal_cost = 0U; }
        total_cost = (uint16)(s_tour_dp[full_mask][c] + terminal_cost);
        if (total_cost < best_total)
        {
            best_total = total_cost;
            best_end_candidate = c;
        }
    }

    if (best_end_candidate == 0xFFU) { return 0U; }

    /* 一次回溯保存完整 Tour；后续物体直接消费缓存，避免每次采样后重复
     * 运行 1+候选数 次方向状态扩散。地图发生清障时 Reset 会废弃缓存。 */
    {
        uint8 mask = full_mask;
        uint8 node = best_end_candidate;
        uint8 write = item_count;
        while (write > 0U && node != 0xFFU)
        {
            uint8 bit = (uint8)(1U << cand_item[node]);
            uint8 previous = s_tour_parent[mask][node];
            --write;
            s_tour_cached_item[write] = item_idx[cand_item[node]];
            s_tour_cached_observe[write] = cand_pos[node];
            mask = (uint8)(mask & (uint8)~bit);
            node = previous;
        }
        if (write != 0U) { return 0U; }
        s_tour_cached_count = item_count;
        s_tour_cached_next = 1U;
    }

    *out_idx = s_tour_cached_item[0];
    *out_observe = s_tour_cached_observe[0];
    return 1U;
}

static uint8 pick_nearest_direct_item(const uint8 map[MAP_ROWS][MAP_COLS],
                                      Point_t cur,
                                      float nav_heading_deg,
                                      Point_t *out_observe)
{
    Point_t first_observe[2U * RECOG_MAX_TARGETS];
    uint16 first_cost[2U * RECOG_MAX_TARGETS];
    uint8 best = 0xFFU;
    uint8 start_heading = recog_nav_heading_index(nav_heading_deg);
    uint16 best_cost = RECOG_ROUTE_COST_INF;

    if (out_observe == NULL) { return 0xFFU; }

    if (!Algo_Nav_Time_Flood(map, cur, SOKO_ACT_NONE,
                             s_pick_time_cost, NULL)) { return 0xFFU; }
    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        first_cost[i] = RECOG_ROUTE_COST_INF;
        if (!s_items[i].visited)
        {
            first_cost[i] = observe_route_cost_from_flood(
                map, s_pick_time_cost, s_items[i].pos,
                s_items[i].failed_observe_mask, start_heading,
                &first_observe[i]);
        }
    }

    /* 超出精确 DP 上限时使用两步前瞻：比较“本次访问 + 下一物体最小代价”，
     * 避免纯最近邻在大地图上反复横穿。每个候选只额外做一次方向状态扩散。 */
    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        uint16 score;
        uint16 next_best = RECOG_ROUTE_COST_INF;
        uint8 face_heading;
        if (first_cost[i] >= RECOG_ROUTE_COST_INF ||
            !recog_face_heading_index(first_observe[i], s_items[i].pos,
                                      &face_heading)) {
            continue;
        }
        if (Algo_Nav_Time_Flood(map, first_observe[i], SOKO_ACT_NONE,
                                s_pick_time_cost, NULL))
        {
            for (uint8 j = 0U; j < s_item_count; ++j)
            {
                uint16 next_cost;
                if (j == i || s_items[j].visited) continue;
                next_cost = observe_route_cost_from_flood(
                    map, s_pick_time_cost, s_items[j].pos,
                    s_items[j].failed_observe_mask, face_heading, NULL);
                if (next_cost < next_best) next_best = next_cost;
            }
        }
        if (next_best >= RECOG_ROUTE_COST_INF) next_best = 0U;
        score = (uint16)(first_cost[i] + next_best);
        if (score < best_cost)
        {
            best_cost = score;
            best = i;
            *out_observe = first_observe[i];
        }
    }
    return best;
}

static uint8 pick_clearable_item(const uint8 map[MAP_ROWS][MAP_COLS],
                                  Point_t cur)
{
    uint8 best = 0xFFU;
    uint8 best_pushes = 0xFFU;
    uint16 best_actions = RECOG_ROUTE_COST_INF;

    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (s_items[i].visited) { continue; }
        if (!App_Recog_Clear_Plan(map, cur, s_items[i].pos, &s_pick_clear_plan)) { continue; }
        if (s_pick_clear_plan.push_count < best_pushes ||
            (s_pick_clear_plan.push_count == best_pushes &&
             s_pick_clear_plan.actions.count < best_actions))
        {
            best_pushes = s_pick_clear_plan.push_count;
            best_actions = s_pick_clear_plan.actions.count;
            best = i;
        }
    }
    return best;
}

/**
 * 从全部未确认 BOX/TARGET 中选择下一个物体，不设置类型优先级。
 * 6 个以内物体先用转角感知的精确 tour 找全局最优首项和观察格；
 * 更大规模使用一次 BFS flood 的最近观察点。
 * 仅当没有直接可达观察点时才按清障规划成本兜底，避免过早移动其它箱子。
 */
static uint8 pick_next_item(const uint8 map[MAP_ROWS][MAP_COLS],
                            Point_t cur,
                            float nav_heading_deg,
                            Point_t *preferred_observe,
                            uint8 *has_preferred_observe)
{
    uint8 remaining = 0U;
    uint8 best = 0xFFU;

    if (preferred_observe == NULL || has_preferred_observe == NULL) { return 0xFFU; }
    *has_preferred_observe = 0U;

    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (!s_items[i].visited) { remaining++; }
    }
    if (remaining == 0U) { return 0xFFU; }

    while (s_tour_cached_next < s_tour_cached_count)
    {
        uint8 cached_item = s_tour_cached_item[s_tour_cached_next];
        Point_t cached_observe = s_tour_cached_observe[s_tour_cached_next];
        uint8 cached_direction = 0U;
        s_tour_cached_next++;
        if (cached_item < s_item_count)
        {
            cached_direction = observe_direction_bit(s_items[cached_item].pos,
                                                      cached_observe);
        }
        if (cached_item < s_item_count && !s_items[cached_item].visited &&
            (s_items[cached_item].failed_observe_mask & cached_direction) == 0U &&
            recog_is_observe_standable(map, cached_observe.y, cached_observe.x))
        {
            *preferred_observe = cached_observe;
            *has_preferred_observe = 1U;
            return cached_item;
        }
    }
    s_tour_cached_count = 0U;
    s_tour_cached_next = 0U;

    /* 第一轮严格排除已失败的观察方向。只有全部未确认物体都
     * 没有剩余可直达方向时，才清除方向掩码重新循环；这样当前物体
     * 的换位很远时，会先消费附近其他物体，而不是立即追远观察位。 */
    for (uint8 pass = 0U; pass < 2U; ++pass)
    {
        if (pass != 0U)
        {
            for (uint8 i = 0U; i < s_item_count; ++i)
            {
                if (!s_items[i].visited)
                {
                    s_items[i].failed_observe_mask = 0U;
                }
            }
        }

        if (remaining <= (uint8)RECOG_EXACT_TOUR_ITEM_LIMIT &&
            pick_exact_direct_tour(map, cur, nav_heading_deg,
                                   &best, preferred_observe))
        {
            *has_preferred_observe = 1U;
            return best;
        }

        best = pick_nearest_direct_item(map, cur, nav_heading_deg,
                                        preferred_observe);
        if (best != 0xFFU)
        {
            *has_preferred_observe = 1U;
            return best;
        }
    }

    return pick_clearable_item(map, cur);
}

/*===================================================================================================================
 * 配对 — 把 box.class_id 与 target.class_id 配对成 box_to_target_idx 数组
 * 输出: box_to_target_out[i] = "第 i 个箱子 (按 extract 顺序中的相对索引)" 应推到 "第 ? 个目标"
 *      约定与 algo_sokoban_solver 内部 extract_elements 顺序一致
 *=================================================================================================================*/
static uint8 item_class_at(uint8 kind, int8 x, int8 y, uint8 *class_id)
{
    uint8 i;
    for (i = 0U; i < s_item_count; ++i)
    {
        if (s_items[i].kind == kind &&
            s_items[i].pos.x == x && s_items[i].pos.y == y &&
            s_items[i].ok)
        {
            *class_id = s_items[i].class_id;
            return 1U;
        }
    }
    return 0U;
}

static uint8 match_static_cell_open(const uint8 map[MAP_ROWS][MAP_COLS],
                                    int8 y,
                                    int8 x,
                                    uint8 walls_removable)
{
    if (x < (int8)CHASSIS_GRID_INNER_MIN_X ||
        x > (int8)CHASSIS_GRID_INNER_MAX_X ||
        y < (int8)CHASSIS_GRID_INNER_MIN_Y ||
        y > (int8)CHASSIS_GRID_INNER_MAX_Y) {
        return 0U;
    }
    if (walls_removable != 0U) {
        return 1U;
    }
    return (uint8)(map[y][x] != MAP_WALL && map[y][x] != MAP_BOMB);
}

/**
 * 从目标反向枚举箱位。反向一步 cur→prev 等价于正向把箱子从 prev 推到 cur，
 * 因而 prev 后方的站位也必须可用。其他箱子/目标按最终会消失处理；有炸弹时内部墙
 * 也按可能被炸除处理，只保留永远不能越过的内场边界约束。
 */
static void build_static_push_distance(const uint8 map[MAP_ROWS][MAP_COLS],
                                       Point_t target,
                                       uint8 walls_removable)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    uint16 head = 0U;
    uint16 tail = 0U;

    memset(s_pick_distance, 0xFF, sizeof(s_pick_distance));
    if (!match_static_cell_open(map, target.y, target.x, walls_removable)) {
        return;
    }

    s_pick_distance[target.y][target.x] = 0U;
    s_match_queue[tail++] = target;
    while (head < tail) {
        Point_t cur = s_match_queue[head++];
        uint8 next_distance = (uint8)(s_pick_distance[cur.y][cur.x] + 1U);

        for (uint8 d = 0U; d < 4U; ++d) {
            Point_t prev;
            Point_t stand;
            prev.x = (int8)(cur.x - dc[d]);
            prev.y = (int8)(cur.y - dr[d]);
            stand.x = (int8)(prev.x - dc[d]);
            stand.y = (int8)(prev.y - dr[d]);

            if (!match_static_cell_open(map, prev.y, prev.x, walls_removable) ||
                !match_static_cell_open(map, stand.y, stand.x, walls_removable) ||
                s_pick_distance[prev.y][prev.x] != 0xFFU) {
                continue;
            }
            s_pick_distance[prev.y][prev.x] = next_distance;
            s_match_queue[tail++] = prev;
        }
    }
}

static void search_min_cost_match(const uint8 group_boxes[SOKOBAN_MAX_BOXES],
                                  const uint8 group_targets[SOKOBAN_MAX_BOXES],
                                  uint8 group_count,
                                  uint8 depth,
                                  uint8 used_mask,
                                  uint16 cur_cost,
                                  uint16 *best_cost,
                                  uint8 cur_assign[SOKOBAN_MAX_BOXES],
                                  uint8 best_assign[SOKOBAN_MAX_BOXES])
{
    uint8 i;

    if (depth >= group_count) {
        if (cur_cost < *best_cost) {
            *best_cost = cur_cost;
            for (i = 0U; i < group_count; ++i) {
                best_assign[i] = cur_assign[i];
            }
        }
        return;
    }

    if (cur_cost >= *best_cost) {
        return;
    }

    for (i = 0U; i < group_count; ++i) {
        uint8 bit = (uint8)(1U << i);
        uint16 step_cost;
        if ((used_mask & bit) != 0U) { continue; }

        step_cost = s_match_cost[group_boxes[depth]][group_targets[i]];
        if (step_cost == 0xFFFFU) { continue; }
        if ((uint16)(cur_cost + step_cost) < cur_cost) { continue; }

        cur_assign[depth] = group_targets[i];
        search_min_cost_match(group_boxes, group_targets,
                              group_count,
                              (uint8)(depth + 1U),
                              (uint8)(used_mask | bit),
                              (uint16)(cur_cost + step_cost),
                              best_cost,
                              cur_assign,
                              best_assign);
    }
}

static uint8 build_box_to_target_mapping(const uint8 map[MAP_ROWS][MAP_COLS],
                                         uint8 has_bomb,
                                         uint8 box_to_target_out[SOKOBAN_MAX_BOXES])
{
    uint8 box_class[SOKOBAN_MAX_BOXES];
    uint8 tgt_class[SOKOBAN_MAX_BOXES];
    Point_t box_pos[SOKOBAN_MAX_BOXES];
    Point_t tgt_pos[SOKOBAN_MAX_BOXES];
    uint8 box_idx_in_extract = 0U;
    uint8 tgt_idx_in_extract = 0U;
    int8 r, c;

    memset(box_class, 0, sizeof(box_class));
    memset(tgt_class, 0, sizeof(tgt_class));
    memset(box_pos, 0, sizeof(box_pos));
    memset(tgt_pos, 0, sizeof(tgt_pos));

    /*
     * 清障后箱子的坐标和扫描顺序可能改变。必须按“当前地图”的行列顺序
     * 重新生成类别数组, 才与 Sokoban_Solve_Stage2 的 extract_elements 对齐。
     */
    for (r = (int8)CHASSIS_GRID_INNER_MIN_Y;
         r <= (int8)CHASSIS_GRID_INNER_MAX_Y; ++r)
    {
        for (c = (int8)CHASSIS_GRID_INNER_MIN_X;
             c <= (int8)CHASSIS_GRID_INNER_MAX_X; ++c)
        {
            if (map[r][c] == MAP_BOX)
            {
                if (box_idx_in_extract >= (uint8)SOKOBAN_MAX_BOXES ||
                    !item_class_at(APP_LINK_OBJ_KIND_BOX, c, r,
                                   &box_class[box_idx_in_extract]))
                {
                    return 0U;
                }
                box_pos[box_idx_in_extract].x = c;
                box_pos[box_idx_in_extract].y = r;
                ++box_idx_in_extract;
            }
            else if (map[r][c] == MAP_TARGET)
            {
                if (tgt_idx_in_extract >= (uint8)SOKOBAN_MAX_BOXES ||
                    !item_class_at(APP_LINK_OBJ_KIND_TARGET, c, r,
                                   &tgt_class[tgt_idx_in_extract]))
                {
                    return 0U;
                }
                tgt_pos[tgt_idx_in_extract].x = c;
                tgt_pos[tgt_idx_in_extract].y = r;
                ++tgt_idx_in_extract;
            }
        }
    }

    if (box_idx_in_extract != s_box_count)         { return 0U; }
    if (box_idx_in_extract != tgt_idx_in_extract)  { return 0U; }
    if (box_idx_in_extract == 0U)                  { return 0U; }

    /* 先计算每个箱→目标的静态最少推数；0xFFFF 表示连放宽后的推送几何都不可达。 */
    for (uint8 ti = 0U; ti < tgt_idx_in_extract; ++ti) {
        build_static_push_distance(map, tgt_pos[ti], has_bomb);
        for (uint8 bi = 0U; bi < box_idx_in_extract; ++bi) {
            uint8 distance = s_pick_distance[box_pos[bi].y][box_pos[bi].x];
            s_match_cost[bi][ti] = (distance == 0xFFU) ? 0xFFFFU : (uint16)distance;
        }
    }

    /* 同 class_id 内只保留静态可推组合，并按总推数最小做一一匹配。 */
    {
        uint8 box_done[SOKOBAN_MAX_BOXES] = {0};
        uint8 bi;
        for (bi = 0U; bi < box_idx_in_extract; ++bi)
        {
            uint8 cls = box_class[bi];
            uint8 group_boxes[SOKOBAN_MAX_BOXES];
            uint8 group_targets[SOKOBAN_MAX_BOXES];
            uint8 cur_assign[SOKOBAN_MAX_BOXES];
            uint8 best_assign[SOKOBAN_MAX_BOXES];
            uint8 group_box_count = 0U;
            uint8 group_target_count = 0U;
            uint16 best_cost = 0xFFFFU;
            uint8 i;
            uint8 ti;

            if (box_done[bi] != 0U) { continue; }
            if (cls == 0U) { return 0U; }

            for (i = 0U; i < box_idx_in_extract; ++i) {
                if (box_class[i] == cls) {
                    group_boxes[group_box_count++] = i;
                }
            }
            for (ti = 0U; ti < tgt_idx_in_extract; ++ti) {
                if (tgt_class[ti] == cls) {
                    group_targets[group_target_count++] = ti;
                }
            }

            if (group_box_count != group_target_count) { return 0U; }
            if (group_box_count == 0U) { return 0U; }

            memset(cur_assign, 0, sizeof(cur_assign));
            memset(best_assign, 0, sizeof(best_assign));
            search_min_cost_match(group_boxes, group_targets,
                                  group_box_count,
                                  0U, 0U, 0U,
                                  &best_cost,
                                  cur_assign,
                                  best_assign);
            if (best_cost == 0xFFFFU) { return 0U; }

            for (i = 0U; i < group_box_count; ++i) {
                box_to_target_out[group_boxes[i]] = best_assign[i];
                box_done[group_boxes[i]] = 1U;
            }
        }

        for (; bi < (uint8)SOKOBAN_MAX_BOXES; ++bi)
        {
            box_to_target_out[bi] = 0U;
        }
    }
    return 1U;
}

static uint8 prepare_direct_nav_plan(const uint8 map[MAP_ROWS][MAP_COLS],
                                     Point_t player_pos,
                                     Point_t observe)
{
    Point_t previous = player_pos;
    uint16 i;

    if (!Algo_Nav_Time_Path(map, player_pos, observe, SOKO_ACT_NONE,
                            &s_pick_nav_path, NULL, NULL)) { return 0U; }

    memset(&s_nav_plan, 0, sizeof(s_nav_plan));
    for (i = 0U; i < s_pick_nav_path.step_count; ++i)
    {
        Point_t next = s_pick_nav_path.path[i];
        int8 dx = (int8)(next.x - previous.x);
        int8 dy = (int8)(next.y - previous.y);
        SokoAction_e action;

        if      (dx ==  0 && dy == -1) action = SOKO_ACT_UP;
        else if (dx ==  0 && dy ==  1) action = SOKO_ACT_DOWN;
        else if (dx == -1 && dy ==  0) action = SOKO_ACT_LEFT;
        else if (dx ==  1 && dy ==  0) action = SOKO_ACT_RIGHT;
        else return 0U;

        if (s_nav_plan.actions.count >= (uint16)SOKOBAN_MAX_ACTIONS) { return 0U; }
        s_nav_plan.actions.actions[s_nav_plan.actions.count++] = action;
        previous = next;
    }
    s_nav_plan.observe = observe;
    s_nav_plan.player_end = observe;
    s_nav_plan.push_count = 0U;
    return 1U;
}

/* 把已就绪的 s_nav_plan 提交为航点序列并登记观察格；常规规划与往返复位共用。 */
static uint8 commit_nav_plan(const uint8 map[MAP_ROWS][MAP_COLS],
                             Point_t player_pos)
{
    uint16 i;
    Point_t player;

    s_items[s_cur_idx].observe = s_nav_plan.observe;
    s_nav_waypoints.count = 0U;
    memcpy(s_nav_replay_map, map, sizeof(s_nav_replay_map));
    player = player_pos;

    /* 转弯、路径末尾以及每一次推箱后都形成航点。推箱单独成段后，只有底盘
     * 确认到达该格才会更新逻辑地图，避免清障执行到一半时物理/逻辑箱位脱节。 */
    for (i = 0U; i < s_nav_plan.actions.count; ++i)
    {
        uint8 d = (uint8)s_nav_plan.actions.actions[i];
        Point_t next;
        uint8 pushed = 0U;
        static const int8 dr[4] = {-1, 1, 0, 0};
        static const int8 dc[4] = {0, 0, -1, 1};

        if (d > (uint8)SOKO_ACT_RIGHT) return 0U;
        next.x = (int8)(player.x + dc[d]);
        next.y = (int8)(player.y + dr[d]);
        if (next.x < 0 || next.x >= (int8)MAP_COLS ||
            next.y < 0 || next.y >= (int8)MAP_ROWS) return 0U;

        if (s_nav_replay_map[next.y][next.x] == MAP_BOX)
        {
            Point_t box_to;
            box_to.x = (int8)(next.x + dc[d]);
            box_to.y = (int8)(next.y + dr[d]);
            if (box_to.x < 0 || box_to.x >= (int8)MAP_COLS ||
                box_to.y < 0 || box_to.y >= (int8)MAP_ROWS ||
                s_nav_replay_map[box_to.y][box_to.x] != MAP_EMPTY) return 0U;
            s_nav_replay_map[next.y][next.x] = MAP_EMPTY;
            s_nav_replay_map[box_to.y][box_to.x] = MAP_BOX;
            pushed = 1U;
        }
        else if (s_nav_replay_map[next.y][next.x] != MAP_EMPTY &&
                 s_nav_replay_map[next.y][next.x] != MAP_TARGET)
        {
            return 0U;
        }
        player = next;

        if ((pushed != 0U) ||
            (i + 1U == s_nav_plan.actions.count) ||
            (s_nav_plan.actions.actions[i] != s_nav_plan.actions.actions[i + 1U]))
        {
            if (s_nav_waypoints.count >= (uint16)SOKOBAN_MAX_WAYPOINTS) return 0U;
            s_nav_waypoints.points[s_nav_waypoints.count++] = player;
        }
    }

    s_nav_wp_idx = 0U;
    s_nav_actions_applied = 0U;
    s_nav_apply_player = player_pos;
    return 1U;
}

static uint8 prepare_nav_plan(const uint8 map[MAP_ROWS][MAP_COLS],
                              Point_t player_pos,
                              const Point_t *preferred_observe)
{
    /* 精确 Tour 已经选定观察格时必须执行该结果；只有候选路径失效或进入
     * 清障兜底时，才允许清障规划器重新选择观察格。 */
    if (preferred_observe != NULL &&
        prepare_direct_nav_plan(map, player_pos, *preferred_observe))
    {
        /* s_nav_plan 已就绪 */
    }
    else if (!App_Recog_Clear_Plan(map, player_pos, s_items[s_cur_idx].pos,
                                   &s_nav_plan))
    {
        return 0U;
    }

    return commit_nav_plan(map, player_pos);
}

static uint8 apply_nav_plan_to_waypoint(uint8 map[MAP_ROWS][MAP_COLS],
                                        Point_t waypoint)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    Point_t player = s_nav_apply_player;

    while (s_nav_actions_applied < s_nav_plan.actions.count &&
           (player.x != waypoint.x || player.y != waypoint.y))
    {
        uint8 d = (uint8)s_nav_plan.actions.actions[s_nav_actions_applied];
        Point_t next;
        if (d > (uint8)SOKO_ACT_RIGHT) return 0U;
        next.x = (int8)(player.x + dc[d]);
        next.y = (int8)(player.y + dr[d]);
        if (next.x < 0 || next.x >= (int8)MAP_COLS ||
            next.y < 0 || next.y >= (int8)MAP_ROWS) return 0U;

        if (map[next.y][next.x] == MAP_BOX)
        {
            Point_t box_to;
            uint8 item;
            box_to.x = (int8)(next.x + dc[d]);
            box_to.y = (int8)(next.y + dr[d]);
            if (box_to.x < 0 || box_to.x >= (int8)MAP_COLS ||
                box_to.y < 0 || box_to.y >= (int8)MAP_ROWS ||
                map[box_to.y][box_to.x] != MAP_EMPTY) return 0U;

            for (item = 0U; item < s_item_count; ++item)
            {
                if (s_items[item].kind == APP_LINK_OBJ_KIND_BOX &&
                    s_items[item].pos.x == next.x &&
                    s_items[item].pos.y == next.y)
                {
                    s_items[item].pos = box_to;
                    break;
                }
            }
            if (item >= s_item_count) return 0U;
            map[next.y][next.x] = MAP_EMPTY;
            map[box_to.y][box_to.x] = MAP_BOX;
            s_map_changed = 1U;
        }
        else if (map[next.y][next.x] != MAP_EMPTY &&
                 map[next.y][next.x] != MAP_TARGET)
        {
            return 0U;
        }
        player = next;
        s_nav_actions_applied++;
    }

    s_nav_apply_player = player;
    return (uint8)(player.x == waypoint.x && player.y == waypoint.y);
}

static uint16 nav_timeout_limit_for_waypoint(Point_t waypoint)
{
    int16 dx = (int16)waypoint.x - (int16)s_nav_apply_player.x;
    int16 dy = (int16)waypoint.y - (int16)s_nav_apply_player.y;
    uint16 cells;
    uint16 limit;

    if (dx < 0) dx = (int16)-dx;
    if (dy < 0) dy = (int16)-dy;
    cells = (uint16)(dx + dy);
    limit = (uint16)(cells * RECOG_NAV_TICKS_PER_CELL);
    return (limit > RECOG_NAV_TIMEOUT_TICKS) ? limit : RECOG_NAV_TIMEOUT_TICKS;
}

/*===================================================================================================================
 * 子状态 handler
 *=================================================================================================================*/

static void enter_sub_nav(void)
{
    s_sub_state    = RECOG_SUB_NAV;
    s_nav_started  = 0U;
    s_face_started = 0U;
    s_nav_wp_idx   = 0U;
    s_subphase_ticks = 0U;        /* B3a: 进入新子阶段, 超时计数清零 */
    s_nav_timeout_limit = RECOG_NAV_TIMEOUT_TICKS;
}

static void enter_sub_face(void)
{
    s_sub_state    = RECOG_SUB_FACE;
    s_face_started = 0U;
    s_face_settle_ticks = 0U;
    s_subphase_ticks = 0U;        /* B3a */
}

static void enter_sub_sample(void)
{
    s_sub_state = RECOG_SUB_SAMPLE;
    sample_state_reset();
    s_request_sequence++;
    if (s_request_sequence == 0U) { s_request_sequence = 1U; }
    s_active_request_id = s_request_sequence;
    app_link_send_recog_request(s_items[s_cur_idx].kind, s_active_request_id);
}

static void enter_sub_next(void)
{
    s_sub_state = RECOG_SUB_NEXT;
}

/* 当前观察方向失败后不立即追该物体的备选位。将它保留在
 * 未确认集合中，NEXT 会把它与其他 BOX/TARGET 一起重新规划。 */
static uint8 defer_current_observe_failure(void)
{
    mark_current_observe_failed();
    if (recog_object_recover_exhausted()) { return 0U; }
    enter_sub_next();
    return 1U;
}

/*===================================================================================================================
 * 对外: Reset / Tick
 *=================================================================================================================*/

void App_Recognize_Reset(void)
{
    s_sub_state    = RECOG_SUB_INIT;
    s_item_count   = 0U;
    s_box_count    = 0U;
    s_target_count = 0U;
    s_cur_idx      = 0U;
    s_nav_started  = 0U;
    s_face_started = 0U;
    s_face_settle_ticks = 0U;
    s_nav_wp_idx   = 0U;
    s_nav_actions_applied = 0U;
    s_map_changed  = 0U;
    s_tour_cached_count = 0U;
    s_tour_cached_next = 0U;
    s_subphase_ticks = 0U;        /* B3a */
    s_nav_timeout_limit = RECOG_NAV_TIMEOUT_TICKS;
    s_active_request_id = 0U;
    sample_state_reset();
    memset(s_items, 0, sizeof(s_items));
    memset(&s_nav_plan, 0, sizeof(s_nav_plan));
    memset(&s_nav_waypoints, 0, sizeof(s_nav_waypoints));
}

AppRecognizeStatus_e App_Recognize_Tick(uint8 map[MAP_ROWS][MAP_COLS],
                                        Point_t player_pos,
                                        uint8 has_bomb,
                                        uint8 level,
                                        float *nav_heading_deg_io,
                                        uint8 box_to_target_out[SOKOBAN_MAX_BOXES])
{
    if (nav_heading_deg_io == NULL)
    {
        chassis_ctrl_stop();
        s_sub_state = RECOG_SUB_FAIL;
        return APP_RECOG_FAIL;
    }

    /* API 边界兜底：只规范调用方保存的逻辑值，不从 IMU 实测角推断。 */
    *nav_heading_deg_io = chassis_snap_yaw_to_cardinal_deg(*nav_heading_deg_io);

    switch (s_sub_state)
    {
        case RECOG_SUB_INIT:
        {
            Point_t preferred_observe;
            uint8 has_preferred_observe;

            /* 第一关无数字配对要求，跳过整圈分类 Tour，缩短连续计时总时长。 */
            if (level <= 1U)
            {
                return APP_RECOG_DONE_NO_NEED;
            }

            extract_items(map);
            if (s_item_count == 0U)
            {
                /* 地图无 BOX/TARGET → 没东西可识别, 放行 */
                return APP_RECOG_DONE_NO_NEED;
            }
            if (s_box_count != s_target_count)
            {
                /* 数量不一致 (视觉端漏识别 / 地图错乱) → 直接判失败 */
                s_sub_state = RECOG_SUB_FAIL;
                return APP_RECOG_FAIL;
            }
            s_cur_idx = pick_next_item(map, player_pos, *nav_heading_deg_io,
                                       &preferred_observe,
                                       &has_preferred_observe);
            if (s_cur_idx == 0xFFU)
            {
                /* 有待确认物体却不存在观察路线，禁止空映射进入解算。 */
                chassis_ctrl_stop();
                s_sub_state = RECOG_SUB_FAIL;
                return APP_RECOG_FAIL;
            }
            if (!prepare_nav_plan(map, player_pos,
                                  has_preferred_observe ? &preferred_observe : NULL))
            {
                chassis_ctrl_stop();
                s_sub_state = RECOG_SUB_FAIL;
                return APP_RECOG_FAIL;
            }
            enter_sub_nav();
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_NAV:
        {
            /* 按规划得到的转弯航点逐段执行, 确保真实轨迹与 BFS 绕障路径一致。 */
            if (s_nav_wp_idx < s_nav_waypoints.count)
            {
                s_subphase_ticks++;
                if (!s_nav_started)
                {
                    Point_t wp = s_nav_waypoints.points[s_nav_wp_idx];
                    recog_move_to_grid_keep_nav_heading(wp, *nav_heading_deg_io);
                    s_nav_timeout_limit = nav_timeout_limit_for_waypoint(wp);
                    s_nav_started = 1U;
                    s_subphase_ticks = 0U;
                    return APP_RECOG_RUNNING;
                }
                if (s_subphase_ticks > s_nav_timeout_limit)
                {
                    chassis_ctrl_stop();
                    if (s_nav_plan.push_count > 0U)
                    {
                        /* 未确认到达的清障段可能已经接触箱子，继续识别会让逻辑地图
                         * 与真实箱位分叉；整轮失败返航比带错图继续解算更安全。 */
                        s_sub_state = RECOG_SUB_FAIL;
                        return APP_RECOG_FAIL;
                    }
                    if (!defer_current_observe_failure())
                    {
                        s_sub_state = RECOG_SUB_FAIL;
                        return APP_RECOG_FAIL;
                    }
                    return APP_RECOG_RUNNING;
                }
                if (!recog_nav_arrived())
                {
                    return APP_RECOG_RUNNING;
                }
                {
                    Point_t wp = s_nav_waypoints.points[s_nav_wp_idx];
                    if (!apply_nav_plan_to_waypoint(map, wp))
                    {
                        chassis_ctrl_stop();
                        s_sub_state = RECOG_SUB_FAIL;
                        return APP_RECOG_FAIL;
                    }
                }
                s_nav_started = 0U;
                s_subphase_ticks = 0U;
                s_nav_wp_idx++;
                return APP_RECOG_RUNNING;
            }

            if (s_nav_actions_applied != s_nav_plan.actions.count)
            {
                chassis_ctrl_stop();
                s_sub_state = RECOG_SUB_FAIL;
                return APP_RECOG_FAIL;
            }
            enter_sub_face();
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_FACE:
        {
            s_subphase_ticks++;            /* B3a: FACE 超时计时 */
            if (!s_face_started)
            {
                float yaw_target;
                if (!recog_calc_face_yaw_deg(s_items[s_cur_idx].observe,
                                             s_items[s_cur_idx].pos,
                                             &yaw_target))
                {
                    chassis_ctrl_stop();
                    if (!defer_current_observe_failure())
                    {
                        s_sub_state = RECOG_SUB_FAIL;
                        return APP_RECOG_FAIL;
                    }
                    return APP_RECOG_RUNNING;
                }
                *nav_heading_deg_io = yaw_target;
                chassis_ctrl_rotate_to_deg(yaw_target);
                s_face_started = 1U;
                return APP_RECOG_RUNNING;
            }
            /* 车头未确认到位时不得采样，否则相邻物体的旧图案会污染映射。 */
            if (s_subphase_ticks > RECOG_FACE_TIMEOUT_TICKS)
            {
                chassis_ctrl_stop();
                if (!defer_current_observe_failure())
                {
                    s_sub_state = RECOG_SUB_FAIL;
                    return APP_RECOG_FAIL;
                }
                return APP_RECOG_RUNNING;
            }
            if (!chassis_ctrl_is_arrived())
            {
                s_face_settle_ticks = 0U;   /* 到位被打断则重新计稳 */
                return APP_RECOG_RUNNING;
            }
            /* 静稳门：到位判据仍允许小幅残余转动，保持到位 150ms 后才采样，
             * 避免拖影/未稳帧参与连续确认或计票。 */
            if (s_face_settle_ticks < (uint8)RECOG_FACE_SETTLE_TICKS)
            {
                s_face_settle_ticks++;
                return APP_RECOG_RUNNING;
            }
            enter_sub_sample();
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_SAMPLE:
        {
            int16 r = sample_confirm_step(s_items[s_cur_idx].kind);
            if (r > 0)
            {
                s_items[s_cur_idx].class_id = (uint8)r;
                s_items[s_cur_idx].visited  = 1U;
                s_items[s_cur_idx].ok       = 1U;
                enter_sub_next();
            }
            else if (r < 0)
            {
                /* 当前方向采样失败后立即延后该物体；全局 Tour 决定先去
                 * 附近其他物体，还是从该物体的另一方向重试。 */
                chassis_ctrl_stop();
                if (!defer_current_observe_failure())
                {
                    /* 恢复预算耗尽：视觉对该物体持续无法确认，明确失败交回
                     * DEADLOCK_RESET，避免比赛计时内绕单个物体无限巡游。 */
                    s_sub_state = RECOG_SUB_FAIL;
                    return APP_RECOG_FAIL;
                }
            }
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_RETURN_YAW:
        {
            /* 兼容旧调试枚举，但仍遵守“确认后才能切换物体”的约束。 */
            if (s_cur_idx < s_item_count && s_items[s_cur_idx].ok)
            {
                enter_sub_next();
            }
            else
            {
                enter_sub_sample();
            }
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_NEXT:
        {
            Point_t preferred_observe;
            uint8 has_preferred_observe;

            /* 全部实地确认 → 配对。 */
            if (all_resolved())
            {
                if (build_box_to_target_mapping(map, has_bomb, box_to_target_out))
                {
                    s_sub_state = RECOG_SUB_DONE;
                    return APP_RECOG_DONE_OK;
                }
                s_sub_state = RECOG_SUB_FAIL;
                return APP_RECOG_FAIL;
            }

            uint8 next = pick_next_item(map, player_pos, *nav_heading_deg_io,
                                        &preferred_observe,
                                        &has_preferred_observe);
            if (next != 0xFFU)
            {
                s_cur_idx = next;
                if (!prepare_nav_plan(map, player_pos,
                                      has_preferred_observe ? &preferred_observe : NULL))
                {
                    chassis_ctrl_stop();
                    s_sub_state = RECOG_SUB_FAIL;
                    return APP_RECOG_FAIL;
                }
                enter_sub_nav();
                return APP_RECOG_RUNNING;
            }
            /* 仍有未确认物体却没有可用观察路线，禁止跳过。 */
            chassis_ctrl_stop();
            s_sub_state = RECOG_SUB_FAIL;
            return APP_RECOG_FAIL;
        }

        case RECOG_SUB_DONE:
        {
            return APP_RECOG_DONE_OK;
        }

        case RECOG_SUB_FAIL:
        default:
        {
            return APP_RECOG_FAIL;
        }
    }
}

void App_Recognize_Get_Debug(AppRecognizeDebug_t *out)
{
    uint8 box_idx = 0U;
    uint8 target_idx = 0U;

    if (out == NULL) { return; }

    memset(out->box_class_ids, 0, sizeof(out->box_class_ids));
    memset(out->target_class_ids, 0, sizeof(out->target_class_ids));

    out->sub_state         = s_sub_state;
    out->total_targets     = s_item_count;
    out->current_idx       = s_cur_idx;
    if (s_cur_idx < s_item_count)
    {
        out->current_kind     = s_items[s_cur_idx].kind;
        out->current_class_id = s_items[s_cur_idx].class_id;
    }
    else
    {
        out->current_kind     = 0U;
        out->current_class_id = 0U;
    }
    out->sample_count      = s_sample_total;

    /* 实测/已确定 计数 */
    uint8 visited = 0U, rb = 0U, rt = 0U;
    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (s_items[i].visited) { ++visited; }
        if (s_items[i].kind == APP_LINK_OBJ_KIND_BOX)
        {
            if (box_idx < (uint8)SOKOBAN_MAX_BOXES)
            {
                out->box_class_ids[box_idx] = s_items[i].ok ? s_items[i].class_id : 0U;
                ++box_idx;
            }
            if (s_items[i].ok) { ++rb; }
        }
        else if (s_items[i].kind == APP_LINK_OBJ_KIND_TARGET)
        {
            if (target_idx < (uint8)SOKOBAN_MAX_BOXES)
            {
                out->target_class_ids[target_idx] = s_items[i].ok ? s_items[i].class_id : 0U;
                ++target_idx;
            }
            if (s_items[i].ok) { ++rt; }
        }
    }
    out->visited_count   = visited;
    /* 排除法推断已移除（最后一个目标同样实地观察）；字段保留恒 0，
     * 维持 AppRecognizeDebug_t 接口与菜单显示兼容。 */
    out->inferred_count  = 0U;
    out->resolved_box    = rb;
    out->resolved_target = rt;
    out->box_count       = box_idx;
    out->target_count    = target_idx;
    out->target_inferred_mask = 0U;
}

uint8 App_Recognize_Map_Changed(void)
{
    return s_map_changed;
}
