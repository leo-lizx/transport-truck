/*********************************************************************************************************************
 * 文件名称   : app_recognize.c
 * 模块功能   : 推箱子识别 tour 子状态机 (STAGE_RECOGNIZE_MAP 的真正实现)
 *
 * @owner     rt1064-main
 * @periph    none                纯算法 (地图特征提取 + 箱子-目标匹配)
 *--------------------------------------------------------------------------------------------------------------------
 * 算法概述:
 *   1. 从地图提取所有 BOX 和 TARGET 坐标 (Stage1 跳过本流程)
 *   2. 6 个以内物体用精确 tour 选首项；更多物体退化为 BFS 最近观察点
 *   3. 对每个物体:
 *        - 找观察点: 与物体 4-邻接的可立足空地, BFS 求最近
 *        - 以最近的 90° 基准航向移动到观察点，最终航点做视觉 Snap
 *        - 旋转车头朝物体 (chassis_ctrl_rotate_to_deg + chassis_ctrl_is_arrived)
 *        - 等多数票稳定: 在 SAMPLE_WINDOW_MS 内统计 BOX_CLASS 帧, 占比 ≥ MAJORITY_THRESH 即确认
 *        - 保持采样后的车头角, 直接继续下一个物体
 *   4. 所有 box → class_id, target → class_id 收齐后, 按相同 class_id 配对生成 g_box_to_target[]
 *
 * 资源:
 *   - 全部 static (BSS), 不可重入, 不可在 ISR 调用
 *   - 复用 algo_sokoban_solver 内部 BFS (Algo_Nav_BFS) 找观察点路径长度
 *********************************************************************************************************************/

#include "app_recognize.h"
#include "app_recognize_clear.h"
#include "app_link.h"
#include "app_vision_fusion.h"
#include "chassis_ctrl.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

#ifndef PI_F
#define PI_F (3.14159265358979323846f)
#endif

/*===================================================================================================================
 * 调参 (集中, 后续可挪到 chassis_config.h)
 *=================================================================================================================*/

/** 单次采样窗口最长时长 (ms) — 超时仍未稳定则按当前最高票输出 */
#define RECOG_SAMPLE_WINDOW_MS         (1500U)

/** 多数票确认阈值: 同一 class_id 出现次数 / 总采样 ≥ 该比 → 立即确认 (0..100) */
#define RECOG_MAJORITY_THRESH_PCT      (60U)

/** 最少采样次数 — 不到这个数即使比例够也再等等 */
#define RECOG_MIN_SAMPLES              (8U)

/** 视觉端无识别 (class_id=0) 容忍上限: B16 — 改为"连续 N 帧 None"判据.
 *  视觉端 BOX_CLASS 帧约 50~100ms/帧, 连续 30 帧 ≈ 1.5~3s 持续无识别才失败.
 *  允许偶发掉帧不影响整体识别. */
#define RECOG_MAX_CONSEC_NONE          (30U)

/** 5ms 周期 */
#define RECOG_TICK_MS                  (5U)

/** 单个分类帧最大年龄；超过该值不得计入当前观察点的多数票。 */
#define RECOG_CLASS_FRAME_MAX_AGE_MS   (250U)

/** B3a: 单点 NAV 最短时限 5s，长直线按每格 1.5s 线性放宽。 */
#define RECOG_NAV_TIMEOUT_TICKS        (1000U)
#define RECOG_NAV_TICKS_PER_CELL       (300U)

/** B3a: 单点 FACE (旋转到位) 最长时长 — 3s */
#define RECOG_FACE_TIMEOUT_TICKS       (600U)

/** 单次 SAMPLE 阶段最大 tick 数 */
#define RECOG_SAMPLE_MAX_TICKS         (RECOG_SAMPLE_WINDOW_MS / RECOG_TICK_MS)

/** 从 SOKOBAN_MAX_BOXES 借用的目标列表上限 (理论上 boxes==targets) */
#define RECOG_MAX_TARGETS              (SOKOBAN_MAX_BOXES)

/** class_id 的有效编号上限 (1..N), 对齐 openart2 的 10 类箱子/目标 */
#define RECOG_CLASS_ID_MAX             (10U)

/** 小规模识别 tour 精确搜索上限: 3 箱 + 3 目标。更大地图用 BFS 贪心控制耗时。 */
#define RECOG_EXACT_TOUR_ITEM_LIMIT    (6U)

#define RECOG_TOUR_MASK_COUNT          (1U << RECOG_EXACT_TOUR_ITEM_LIMIT)
#define RECOG_TOUR_MAX_CANDIDATES      (RECOG_EXACT_TOUR_ITEM_LIMIT * 4U)
#define RECOG_ROUTE_COST_INF           (0xFFFFU)

/*===================================================================================================================
 * 内部数据结构
 *=================================================================================================================*/

typedef struct
{
    Point_t pos;            /* 物体网格坐标                          */
    Point_t observe;        /* 观察点 (与 pos 4-邻接的可立足格)      */
    uint8   kind;           /* APP_LINK_OBJ_KIND_BOX / TARGET        */
    uint8   class_id;       /* 多数票输出的类别; 0 = 未识别          */
    uint8   visited;        /* 1 = 本轮已尝试访问, 不再选              */
    uint8   ok;             /* 1 = class_id 已确定 (≠0)              */
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
static uint16 s_subphase_ticks  = 0U;     /* B3a: NAV/FACE 子阶段计时, enter_sub_* 时清零 */
static uint16 s_nav_timeout_limit = RECOG_NAV_TIMEOUT_TICKS;
static AppRecogClearPlan_t s_nav_plan;
static SokoWaypointPath_t  s_nav_waypoints;
static uint16  s_nav_wp_idx     = 0U;
static uint16  s_nav_actions_applied = 0U;
static Point_t s_nav_apply_player;
static uint8   s_map_changed     = 0U;
static uint8   s_nav_replay_map[MAP_ROWS][MAP_COLS];

/* 多数票统计 */
static uint16 s_sample_ticks    = 0U;
static uint16 s_sample_total    = 0U;
static uint16 s_sample_consec_none = 0U;     /* B16: 连续 None 计数, 任一有效帧清零 */
static uint16 s_class_hist[RECOG_CLASS_ID_MAX + 1U] = {0};
static uint32 s_last_seen_frame_id = 0U;     /* 已采样过的最大 frame_id, 防重复计票 */

/* 识别 tour 选点 scratch: 小规模 DP 约 5.5KB BSS, 低于单次新增 10KB 约束。 */
static uint8  s_pick_distance[MAP_ROWS][MAP_COLS];
static uint8  s_tour_start_cost[RECOG_TOUR_MAX_CANDIDATES];
static uint8  s_tour_edge_cost[RECOG_TOUR_MAX_CANDIDATES][RECOG_TOUR_MAX_CANDIDATES];
static uint16 s_tour_dp[RECOG_TOUR_MASK_COUNT][RECOG_TOUR_MAX_CANDIDATES];
static uint8  s_tour_first[RECOG_TOUR_MASK_COUNT][RECOG_TOUR_MAX_CANDIDATES];
static AppRecogClearPlan_t s_pick_clear_plan;

/*===================================================================================================================
 * 内部工具
 *=================================================================================================================*/

/**
 * 计算"车头朝物体"的目标 yaw (度). 约定见文件头.
 *  - 车体 +Y 为前进方向, +X 为右
 *  - yaw 是车体相对全局坐标系 +X 的角度 (CCW 正)
 *  - 朝物体 = 车体 +Y 指向物体 → 解得 yaw = atan2(Δx, Δy)
 */
static float recog_calc_face_yaw_deg(Point_t observe, Point_t target)
{
    /* B7: observe 不用格中心, 用底盘 odom 真实位姿 (车不一定停在格正中心,
     * 5cm 偏差 + 14cm 邻接距离 → 格中心算的 yaw 最多偏 20°, 够把物体甩出视场). */
    chassis_pose_t pose = chassis_ctrl_get_pose();
    float ox_m = pose.x_m;
    float oy_m = pose.y_m;
    float tx_m = chassis_grid_x_to_m((uint8)target.x);
    float ty_m = chassis_grid_y_to_m((uint8)target.y);
    float dx   = tx_m - ox_m;
    float dy   = ty_m - oy_m;
    float yaw_rad;

    (void)observe;     /* 仅供调试观测保留, 不再参与计算 */

    /* 退化情形: 重合 (理论上不会发生, 观察点必与物体相邻); 留前向 0° 兜底 */
    if ((dx == 0.0f) && (dy == 0.0f))
    {
        return 0.0f;
    }
    yaw_rad = atan2f(dx, dy);
    return yaw_rad * (180.0f / PI_F);
}

static void recog_move_to_grid_keep_current_yaw(Point_t target)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();

    /* snap 到最近 90° 倍数: 面向物体采样后 yaw 是任意角, 直接保持会让
     * 轴对齐平移控制器两轴耦合 → 走斜线/限速失真, 故先吸附再派发 */
    chassis_ctrl_move_to_m(chassis_grid_x_to_m((uint8)target.x),
                           chassis_grid_y_to_m((uint8)target.y),
                           chassis_snap_yaw_to_cardinal_deg(pose.yaw_deg));
}

/* 识别巡航仅在每个物体的最终观察航点做视觉 Snap。中间转弯点继续使用
 * odom 到位即可，避免每段额外等待最多 800ms；最终点校准后再计算朝物体 yaw。 */
static uint8 recog_nav_arrived_for_waypoint(uint8 require_snap)
{
    if (chassis_ctrl_is_arrived() == 0U)
    {
        app_vision_fusion_snap_cancel();
        return 0U;
    }

#if CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE
    if (require_snap != 0U)
    {
        float target_x_m = 0.0f;
        float target_y_m = 0.0f;
        app_vision_snap_state_e state;

        chassis_ctrl_get_point_nav_target_m(&target_x_m, &target_y_m);
        app_vision_fusion_snap_request(target_x_m, target_y_m);
        state = app_vision_fusion_snap_state();
        return (uint8)((state == APP_VISION_SNAP_DONE) ||
                       (state == APP_VISION_SNAP_TIMEOUT) ||
                       (state == APP_VISION_SNAP_REJECT));
    }
#else
    (void)require_snap;
#endif

    app_vision_fusion_snap_cancel();
    return 1U;
}

/*===================================================================================================================
 * 多数票 — 采样统计与判定
 *=================================================================================================================*/

static void sample_state_reset(void)
{
    app_link_box_class_snapshot_t snap;

    s_sample_ticks       = 0U;
    s_sample_total       = 0U;
    s_sample_consec_none = 0U;
    /* 丢弃转向完成前已经落地的最后一帧，只统计进入 SAMPLE 后的新帧。 */
    app_link_get_box_class_snapshot(&snap);
    s_last_seen_frame_id = (snap.valid != 0U) ? snap.frame_id : 0U;
    memset(s_class_hist, 0, sizeof(s_class_hist));
}

/**
 * @return  >0 = 已确认的 class_id;
 *           0 = 还在采样;
 *          -1 = 视觉持续无识别 (class_id=0 累计过多), 判失败
 *          -2 = 时间窗到了仍无明确多数, 取最高票兜底
 */
static int16 sample_majority_step(uint8 expect_kind)
{
    app_link_box_class_snapshot_t snap;
    uint32 now_ms;
    app_link_get_box_class_snapshot(&snap);
    now_ms = app_link_get_ms();

    s_sample_ticks++;

    /* 仅当新一帧到达时计票 */
    if ((snap.valid != 0U) && (snap.frame_id != s_last_seen_frame_id))
    {
        s_last_seen_frame_id = snap.frame_id;

        /* frame_id 只保证“没重复”，stamp_ms 再保证它确属当前实时观察窗口。 */
        if ((uint32)(now_ms - snap.stamp_ms) > RECOG_CLASS_FRAME_MAX_AGE_MS)
        {
            return 0;
        }

        /* B11: 物体类型不匹配 (主控想看 BOX, 视觉端却给 TARGET): 静默忽略,
         * 不计入 None 也不计入 total. 否则相邻 box/target 会把 None 拉满判失败. */
        if (snap.obj_kind != expect_kind)
        {
            /* skip — 既不计 None 也不计 total */
        }
        else if (snap.class_id == 0U)
        {
            s_sample_consec_none++;            /* B16: 连续 None */
        }
        else if (snap.class_id <= (uint8)RECOG_CLASS_ID_MAX)
        {
            s_class_hist[snap.class_id]++;
            s_sample_total++;
            s_sample_consec_none = 0U;          /* B16: 收到有效帧 → 清零 */
        }
        else
        {
            /* 越界视为 None */
            s_sample_consec_none++;
        }
    }

    /* B16: 视觉端连续无识别 → 失败 (允许偶发掉帧) */
    if (s_sample_consec_none >= (uint16)RECOG_MAX_CONSEC_NONE)
    {
        return -1;
    }

    /* 满足最少采样数后, 看最高票占比 */
    if (s_sample_total >= (uint16)RECOG_MIN_SAMPLES)
    {
        uint8  best_cls = 0U;
        uint16 best_cnt = 0U;
        for (uint8 i = 1U; i <= (uint8)RECOG_CLASS_ID_MAX; ++i)
        {
            if (s_class_hist[i] > best_cnt)
            {
                best_cnt = s_class_hist[i];
                best_cls = i;
            }
        }
        /* 占比达标 → 立即确认 */
        if ((uint32)best_cnt * 100U >= (uint32)s_sample_total * (uint32)RECOG_MAJORITY_THRESH_PCT)
        {
            return (int16)best_cls;
        }
    }

    /* 时间窗到了 → 兜底返回最高票 (即便比例不够, 只要 >0) */
    if (s_sample_ticks >= (uint16)RECOG_SAMPLE_MAX_TICKS)
    {
        uint8  best_cls = 0U;
        uint16 best_cnt = 0U;
        for (uint8 i = 1U; i <= (uint8)RECOG_CLASS_ID_MAX; ++i)
        {
            if (s_class_hist[i] > best_cnt)
            {
                best_cnt = s_class_hist[i];
                best_cls = i;
            }
        }
        if (best_cnt > 0U)
        {
            return (int16)best_cls;
        }
        return -2;     /* 兜底失败 */
    }

    return 0;          /* 继续采样 */
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

static uint16 observe_distance_from_flood(const uint8 map[MAP_ROWS][MAP_COLS],
                                          const uint8 distance_steps[MAP_ROWS][MAP_COLS],
                                          Point_t object)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    uint16 best = RECOG_ROUTE_COST_INF;

    for (uint8 d = 0U; d < 4U; ++d)
    {
        int8 y = (int8)(object.y + dr[d]);
        int8 x = (int8)(object.x + dc[d]);
        if (!recog_is_observe_standable(map, y, x)) { continue; }
        if (distance_steps[y][x] < (uint8)ALGO_NAV_DISTANCE_UNREACHABLE &&
            (uint16)distance_steps[y][x] < best)
        {
            best = (uint16)distance_steps[y][x];
        }
    }
    return best;
}

/**
 * 小规模精确 tour:
 *   - 节点是每个未访问物体的所有可站观察格;
 *   - DP 状态为 (已访问物体集合, 当前观察格), 求覆盖全部物体的最短总步数;
 *   - 只输出当前应访问的首个物体, 真实导航仍由 prepare_nav_plan() 重新规划。
 */
static uint8 pick_exact_direct_tour(const uint8 map[MAP_ROWS][MAP_COLS],
                                    Point_t cur,
                                    uint8 *out_idx)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    uint8 item_idx[RECOG_EXACT_TOUR_ITEM_LIMIT];
    uint8 cand_item[RECOG_TOUR_MAX_CANDIDATES];
    Point_t cand_pos[RECOG_TOUR_MAX_CANDIDATES];
    uint8 item_count = 0U;
    uint8 cand_count = 0U;
    uint8 full_mask;
    uint16 best_total = RECOG_ROUTE_COST_INF;
    uint8 best_first = 0xFFU;

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
            if (!recog_is_observe_standable(map, y, x)) { continue; }
            if (cand_count >= (uint8)RECOG_TOUR_MAX_CANDIDATES) { return 0U; }
            cand_pos[cand_count].x = x;
            cand_pos[cand_count].y = y;
            cand_item[cand_count] = local;
            cand_count++;
            has_candidate = 1U;
        }
        if (!has_candidate) { return 0U; }
    }
    if (cand_count == 0U) { return 0U; }

    memset(s_tour_dp, 0xFF, sizeof(s_tour_dp));
    memset(s_tour_first, 0xFF, sizeof(s_tour_first));
    memset(s_tour_start_cost, ALGO_NAV_DISTANCE_UNREACHABLE, sizeof(s_tour_start_cost));
    memset(s_tour_edge_cost, ALGO_NAV_DISTANCE_UNREACHABLE, sizeof(s_tour_edge_cost));

    if (!Algo_Nav_BFS_Flood(map, cur, NULL, s_pick_distance)) { return 0U; }
    for (uint8 c = 0U; c < cand_count; ++c)
    {
        s_tour_start_cost[c] = s_pick_distance[cand_pos[c].y][cand_pos[c].x];
    }

    for (uint8 from = 0U; from < cand_count; ++from)
    {
        if (!Algo_Nav_BFS_Flood(map, cand_pos[from], NULL, s_pick_distance)) { continue; }
        for (uint8 to = 0U; to < cand_count; ++to)
        {
            s_tour_edge_cost[from][to] = s_pick_distance[cand_pos[to].y][cand_pos[to].x];
        }
    }

    for (uint8 c = 0U; c < cand_count; ++c)
    {
        uint8 item_bit = (uint8)(1U << cand_item[c]);
        if (s_tour_start_cost[c] >= (uint8)ALGO_NAV_DISTANCE_UNREACHABLE) { continue; }
        s_tour_dp[item_bit][c] = (uint16)s_tour_start_cost[c];
        s_tour_first[item_bit][c] = cand_item[c];
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
                if (s_tour_edge_cost[from][to] >= (uint8)ALGO_NAV_DISTANCE_UNREACHABLE) { continue; }

                next_mask = (uint8)(mask | to_bit);
                next_cost = (uint16)(base_cost + (uint16)s_tour_edge_cost[from][to]);
                if (next_cost < s_tour_dp[next_mask][to])
                {
                    s_tour_dp[next_mask][to] = next_cost;
                    s_tour_first[next_mask][to] = s_tour_first[mask][from];
                }
            }
        }
    }

    for (uint8 c = 0U; c < cand_count; ++c)
    {
        if (s_tour_dp[full_mask][c] < best_total)
        {
            best_total = s_tour_dp[full_mask][c];
            best_first = s_tour_first[full_mask][c];
        }
    }

    if (best_first == 0xFFU) { return 0U; }
    *out_idx = item_idx[best_first];
    return 1U;
}

static uint8 pick_nearest_direct_item(const uint8 map[MAP_ROWS][MAP_COLS],
                                      Point_t cur)
{
    uint8 best = 0xFFU;
    uint16 best_cost = RECOG_ROUTE_COST_INF;

    if (!Algo_Nav_BFS_Flood(map, cur, NULL, s_pick_distance)) { return 0xFFU; }
    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        uint16 cost;
        if (s_items[i].visited) { continue; }
        cost = observe_distance_from_flood(map, s_pick_distance, s_items[i].pos);
        if (cost < best_cost)
        {
            best_cost = cost;
            best = i;
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
 * 选下一个未访问的物体。
 * 6 个以内物体先用精确 tour 找全局最短首项；更大规模使用一次 BFS flood 的最近观察点。
 * 仅当没有直接可达观察点时才按清障规划成本兜底，避免过早移动其它箱子。
 */
static uint8 pick_next_item(const uint8 map[MAP_ROWS][MAP_COLS],
                            Point_t cur)
{
    uint8 remaining = 0U;
    uint8 best = 0xFFU;

    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (!s_items[i].visited) { remaining++; }
    }
    if (remaining == 0U) { return 0xFFU; }

    if (remaining <= (uint8)RECOG_EXACT_TOUR_ITEM_LIMIT &&
        pick_exact_direct_tour(map, cur, &best))
    {
        return best;
    }

    best = pick_nearest_direct_item(map, cur);
    if (best != 0xFFU) { return best; }

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

static uint16 point_manhattan_cost(Point_t a, Point_t b)
{
    int16 dx = (int16)a.x - (int16)b.x;
    int16 dy = (int16)a.y - (int16)b.y;

    if (dx < 0) { dx = (int16)-dx; }
    if (dy < 0) { dy = (int16)-dy; }
    return (uint16)(dx + dy);
}

static void search_min_cost_match(const Point_t box_pos[SOKOBAN_MAX_BOXES],
                                  const Point_t tgt_pos[SOKOBAN_MAX_BOXES],
                                  const uint8 group_boxes[SOKOBAN_MAX_BOXES],
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

        step_cost = point_manhattan_cost(box_pos[group_boxes[depth]],
                                         tgt_pos[group_targets[i]]);
        if ((uint16)(cur_cost + step_cost) < cur_cost) { continue; }

        cur_assign[depth] = group_targets[i];
        search_min_cost_match(box_pos, tgt_pos,
                              group_boxes, group_targets,
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

    /* 同 class_id 内按箱→目标曼哈顿总代价最小做一一匹配。 */
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
            search_min_cost_match(box_pos, tgt_pos,
                                  group_boxes, group_targets,
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

static uint8 prepare_nav_plan(const uint8 map[MAP_ROWS][MAP_COLS],
                              Point_t player_pos)
{
    uint16 i;
    Point_t player;

    if (!App_Recog_Clear_Plan(map, player_pos, s_items[s_cur_idx].pos,
                              &s_nav_plan))
    {
        return 0U;
    }

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
    app_vision_fusion_snap_cancel();
    s_sub_state    = RECOG_SUB_FACE;
    s_face_started = 0U;
    s_subphase_ticks = 0U;        /* B3a */
}

static void enter_sub_sample(void)
{
    s_sub_state = RECOG_SUB_SAMPLE;
    sample_state_reset();
}

static void enter_sub_next(void)
{
    s_sub_state = RECOG_SUB_NEXT;
}

/*===================================================================================================================
 * 对外: Reset / Tick
 *=================================================================================================================*/

void App_Recognize_Reset(void)
{
    app_vision_fusion_snap_cancel();
    s_sub_state    = RECOG_SUB_INIT;
    s_item_count   = 0U;
    s_box_count    = 0U;
    s_target_count = 0U;
    s_cur_idx      = 0U;
    s_nav_started  = 0U;
    s_face_started = 0U;
    s_nav_wp_idx   = 0U;
    s_nav_actions_applied = 0U;
    s_map_changed  = 0U;
    s_subphase_ticks = 0U;        /* B3a */
    s_nav_timeout_limit = RECOG_NAV_TIMEOUT_TICKS;
    sample_state_reset();
    memset(s_items, 0, sizeof(s_items));
    memset(&s_nav_plan, 0, sizeof(s_nav_plan));
    memset(&s_nav_waypoints, 0, sizeof(s_nav_waypoints));
}

AppRecognizeStatus_e App_Recognize_Tick(uint8 map[MAP_ROWS][MAP_COLS],
                                        Point_t player_pos,
                                        uint8 has_bomb,
                                        uint8 level,
                                        uint8 box_to_target_out[SOKOBAN_MAX_BOXES])
{
    switch (s_sub_state)
    {
        case RECOG_SUB_INIT:
        {
            /* Stage1 简单贪心模式不需要识别: 关 1 且地图无炸弹直接放行 */
            if ((level <= 1U) && (has_bomb == 0U))
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
            s_cur_idx = pick_next_item(map, player_pos);
            if (s_cur_idx == 0xFFU)
            {
                /* 不应发生 (s_item_count > 0) */
                s_sub_state = RECOG_SUB_DONE;
                return APP_RECOG_RUNNING;
            }
            if (!prepare_nav_plan(map, player_pos))
            {
                s_items[s_cur_idx].visited = 1U;
                s_items[s_cur_idx].ok      = 0U;
                enter_sub_next();
                return APP_RECOG_RUNNING;
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
                    recog_move_to_grid_keep_current_yaw(wp);
                    s_nav_timeout_limit = nav_timeout_limit_for_waypoint(wp);
                    s_nav_started = 1U;
                    s_subphase_ticks = 0U;
                    return APP_RECOG_RUNNING;
                }
                if (s_subphase_ticks > s_nav_timeout_limit)
                {
                    app_vision_fusion_snap_cancel();
                    chassis_ctrl_stop();
                    if (s_nav_plan.push_count > 0U)
                    {
                        /* 未确认到达的清障段可能已经接触箱子，继续识别会让逻辑地图
                         * 与真实箱位分叉；整轮失败返航比带错图继续解算更安全。 */
                        s_sub_state = RECOG_SUB_FAIL;
                        return APP_RECOG_FAIL;
                    }
                    s_items[s_cur_idx].visited = 1U;
                    s_items[s_cur_idx].ok      = 0U;
                    enter_sub_next();
                    return APP_RECOG_RUNNING;
                }
                if (!recog_nav_arrived_for_waypoint(
                        (uint8)((s_nav_wp_idx + 1U) >= s_nav_waypoints.count)))
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
                float yaw_target = recog_calc_face_yaw_deg(s_items[s_cur_idx].observe,
                                                           s_items[s_cur_idx].pos);
                chassis_ctrl_rotate_to_deg(yaw_target);
                s_face_started = 1U;
                return APP_RECOG_RUNNING;
            }
            /* 车头未确认到位时不得采样，否则相邻物体的旧图案会污染映射。 */
            if (s_subphase_ticks > RECOG_FACE_TIMEOUT_TICKS)
            {
                chassis_ctrl_stop();
                s_items[s_cur_idx].visited = 1U;
                s_items[s_cur_idx].ok = 0U;
                enter_sub_next();
                return APP_RECOG_RUNNING;
            }
            if (!chassis_ctrl_is_arrived())
            {
                return APP_RECOG_RUNNING;
            }
            enter_sub_sample();
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_SAMPLE:
        {
            int16 r = sample_majority_step(s_items[s_cur_idx].kind);
            if (r > 0)
            {
                s_items[s_cur_idx].class_id = (uint8)r;
                s_items[s_cur_idx].visited  = 1U;
                s_items[s_cur_idx].ok       = 1U;
                enter_sub_next();
            }
            else if (r == -1 || r == -2)
            {
                /* 当前物体识别失败 — 不立即整体失败, 给后续物体机会;
                 * 最终在 NEXT/DONE 阶段统一判断映射是否完整 */
                s_items[s_cur_idx].visited = 1U;
                s_items[s_cur_idx].ok      = 0U;
                enter_sub_next();
            }
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_RETURN_YAW:
        {
            /* 兼容旧调试枚举: 新策略采样后不回正, 保持当前 yaw 继续导航。 */
            enter_sub_next();
            return APP_RECOG_RUNNING;
        }

        case RECOG_SUB_NEXT:
        {
            /* 已全部识别完 → 配对 */
            if (all_resolved())
            {
                if (build_box_to_target_mapping(map, box_to_target_out))
                {
                    s_sub_state = RECOG_SUB_DONE;
                    return APP_RECOG_DONE_OK;
                }
                s_sub_state = RECOG_SUB_FAIL;
                return APP_RECOG_FAIL;
            }

            uint8 next = pick_next_item(map, player_pos);
            if (next != 0xFFU)
            {
                s_cur_idx = next;
                if (!prepare_nav_plan(map, player_pos))
                {
                    s_items[s_cur_idx].visited = 1U;
                    s_items[s_cur_idx].ok      = 0U;
                    /* 留在 NEXT, 下一 tick 继续挑 */
                    return APP_RECOG_RUNNING;
                }
                enter_sub_nav();
                return APP_RECOG_RUNNING;
            }
            /* 所有物体都尝试过仍未全识别 → 配对失败 */
            if (build_box_to_target_mapping(map, box_to_target_out))
            {
                s_sub_state = RECOG_SUB_DONE;
                return APP_RECOG_DONE_OK;
            }
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
    if (out == NULL) { return; }

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
        if (s_items[i].ok)
        {
            if (s_items[i].kind == APP_LINK_OBJ_KIND_BOX) { ++rb; } else { ++rt; }
        }
    }
    out->visited_count   = visited;
    out->inferred_count  = 0U;
    out->resolved_box    = rb;
    out->resolved_target = rt;
}

uint8 App_Recognize_Map_Changed(void)
{
    return s_map_changed;
}
