/*********************************************************************************************************************
 * 文件名称   : app_recognize.c
 * 模块功能   : 推箱子识别 tour 子状态机 (STAGE_RECOGNIZE_MAP 的真正实现)
 *
 * @owner     rt1064-main
 * @periph    none                纯算法 (地图特征提取 + 箱子-目标匹配)
 *--------------------------------------------------------------------------------------------------------------------
 * 算法概述:
 *   1. 从地图提取所有 BOX 和 TARGET 坐标 (Stage1 跳过本流程)
 *   2. 贪心: 反复挑离当前位置最近 (BFS 步数最短) 的未访问物体
 *   3. 对每个物体:
 *        - 找观察点: 与物体 4-邻接的可立足空地, BFS 求最近
 *        - 移动到观察点 (HAL_CHASSIS_MOVE_TO + chassis_ctrl_is_arrived)
 *        - 旋转车头朝物体 (chassis_ctrl_rotate_to_deg + chassis_ctrl_is_arrived)
 *        - 等多数票稳定: 在 SAMPLE_WINDOW_MS 内统计 BOX_CLASS 帧, 占比 ≥ MAJORITY_THRESH 即确认
 *   4. 所有 box → class_id, target → class_id 收齐后, 按相同 class_id 配对生成 g_box_to_target[]
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
#include <math.h>
#include <string.h>

#include <stdlib.h>     /* abs() */

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

/** B3a: 单点 NAV (跑到观察点) 最长时长 — 5s @ 5ms tick */
#define RECOG_NAV_TIMEOUT_TICKS        (1000U)

/** B3a: 单点 FACE (旋转到位) 最长时长 — 3s */
#define RECOG_FACE_TIMEOUT_TICKS       (600U)

/** 单次 SAMPLE 阶段最大 tick 数 */
#define RECOG_SAMPLE_MAX_TICKS         (RECOG_SAMPLE_WINDOW_MS / RECOG_TICK_MS)

/** 从 SOKOBAN_MAX_BOXES 借用的目标列表上限 (理论上 boxes==targets) */
#define RECOG_MAX_TARGETS              (SOKOBAN_MAX_BOXES)

/** class_id 的有效编号上限 (1..N), 对齐 openart2 的 10 类箱子/目标 */
#define RECOG_CLASS_ID_MAX             (10U)

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
static AppRecogClearPlan_t s_nav_plan;
static SokoWaypointPath_t  s_nav_waypoints;
static Point_t s_nav_plan_start;
static uint16  s_nav_wp_idx     = 0U;
static uint8   s_nav_map_applied = 0U;
static uint8   s_map_changed     = 0U;

/* 多数票统计 */
static uint16 s_sample_ticks    = 0U;
static uint16 s_sample_total    = 0U;
static uint16 s_sample_consec_none = 0U;     /* B16: 连续 None 计数, 任一有效帧清零 */
static uint16 s_class_hist[RECOG_CLASS_ID_MAX + 1U] = {0};
static uint32 s_last_seen_frame_id = 0U;     /* 已采样过的最大 frame_id, 防重复计票 */

/*===================================================================================================================
 * 内部工具
 *=================================================================================================================*/

/**
 * 计算"车头朝物体"的目标 yaw (度). 约定见文件头.
 *  - 车体 +Y 为前进方向, +X 为右
 *  - yaw 是车体相对全局坐标系 +X 的角度 (CCW 正)
 *  - 朝物体 = 车体 +Y 指向物体 → 解得 yaw = atan2(-Δx, Δy)
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
    yaw_rad = atan2f(-dx, dy);
    return yaw_rad * (180.0f / PI_F);
}

/*===================================================================================================================
 * 多数票 — 采样统计与判定
 *=================================================================================================================*/

static void sample_state_reset(void)
{
    s_sample_ticks       = 0U;
    s_sample_total       = 0U;
    s_sample_consec_none = 0U;
    s_last_seen_frame_id = 0U;
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
    app_link_get_box_class_snapshot(&snap);

    s_sample_ticks++;

    /* 仅当新一帧到达时计票 */
    if ((snap.valid != 0U) && (snap.frame_id != s_last_seen_frame_id))
    {
        s_last_seen_frame_id = snap.frame_id;

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

/**
 * 选下一个未访问的物体 — 贪心: 离当前位置曼哈顿距离最近.
 * (BFS 路径长度更准, 但开销大 (每次 O(n*MR*MC)); 内场 14×10 的曼哈顿近似已足够.)
 *
 * @return 选中下标, 没有未访问项时返回 0xFF
 */
static uint8 pick_next_item(Point_t cur)
{
    uint8 best = 0xFFU;
    int16 best_d = 32767;

    for (uint8 i = 0U; i < s_item_count; ++i)
    {
        if (s_items[i].visited) { continue; }
        int16 d = (int16)(abs((int)s_items[i].pos.x - (int)cur.x)
                        + abs((int)s_items[i].pos.y - (int)cur.y));
        if (d < best_d)
        {
            best_d = d;
            best   = i;
        }
    }
    return best;
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

static uint8 build_box_to_target_mapping(const uint8 map[MAP_ROWS][MAP_COLS],
                                         uint8 box_to_target_out[SOKOBAN_MAX_BOXES])
{
    uint8 box_class[SOKOBAN_MAX_BOXES];
    uint8 tgt_class[SOKOBAN_MAX_BOXES];
    uint8 box_idx_in_extract = 0U;
    uint8 tgt_idx_in_extract = 0U;
    int8 r, c;

    memset(box_class, 0, sizeof(box_class));
    memset(tgt_class, 0, sizeof(tgt_class));

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
                ++tgt_idx_in_extract;
            }
        }
    }

    if (box_idx_in_extract != s_box_count)         { return 0U; }
    if (box_idx_in_extract != tgt_idx_in_extract)  { return 0U; }
    if (box_idx_in_extract == 0U)                  { return 0U; }

    /* 为每个 box 找同 class_id 的 target (要求一一匹配, 不可重复占用) */
    {
        uint8 t_used[SOKOBAN_MAX_BOXES] = {0};
        uint8 bi;
        for (bi = 0U; bi < box_idx_in_extract; ++bi)
        {
            uint8 cls = box_class[bi];
            uint8 matched = 0U;
            uint8 ti;
            if (cls == 0U) { return 0U; }

            for (ti = 0U; ti < tgt_idx_in_extract; ++ti)
            {
                if (t_used[ti])           { continue; }
                if (tgt_class[ti] != cls) { continue; }
                box_to_target_out[bi] = ti;
                t_used[ti] = 1U;
                matched = 1U;
                break;
            }
            if (!matched) { return 0U; }
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
    uint16 waypoint_need = 0U;
    uint16 i;

    if (!App_Recog_Clear_Plan(map, player_pos, s_items[s_cur_idx].pos,
                              &s_nav_plan))
    {
        return 0U;
    }

    for (i = 0U; i < s_nav_plan.actions.count; ++i)
    {
        if (i + 1U == s_nav_plan.actions.count ||
            s_nav_plan.actions.actions[i] != s_nav_plan.actions.actions[i + 1U])
        {
            ++waypoint_need;
        }
    }
    if (waypoint_need > (uint16)SOKOBAN_MAX_WAYPOINTS)
    {
        return 0U;
    }

    s_items[s_cur_idx].observe = s_nav_plan.observe;
    s_nav_plan_start = player_pos;
    Sokoban_Actions_To_Waypoints(s_nav_plan.actions.actions,
                                 s_nav_plan.actions.count,
                                 player_pos,
                                 &s_nav_waypoints);
    s_nav_wp_idx = 0U;
    s_nav_map_applied = 0U;
    return 1U;
}

static uint8 apply_nav_plan_to_map(uint8 map[MAP_ROWS][MAP_COLS])
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    Point_t player = s_nav_plan_start;
    uint16 i;

    for (i = 0U; i < s_nav_plan.actions.count; ++i)
    {
        uint8 d = (uint8)s_nav_plan.actions.actions[i];
        Point_t next;
        if (d > (uint8)SOKO_ACT_RIGHT) return 0U;
        next.x = (int8)(player.x + dc[d]);
        next.y = (int8)(player.y + dr[d]);

        if (map[next.y][next.x] == MAP_BOX)
        {
            Point_t box_to;
            uint8 item;
            box_to.x = (int8)(next.x + dc[d]);
            box_to.y = (int8)(next.y + dr[d]);
            if (map[box_to.y][box_to.x] != MAP_EMPTY) return 0U;

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
        }
        else if (map[next.y][next.x] != MAP_EMPTY &&
                 map[next.y][next.x] != MAP_TARGET)
        {
            return 0U;
        }
        player = next;
    }

    s_map_changed = 1U;
    return 1U;
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
    s_nav_map_applied = 0U;
    s_subphase_ticks = 0U;        /* B3a: 进入新子阶段, watchdog 清零 */
}

static void enter_sub_face(void)
{
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
    s_sub_state    = RECOG_SUB_INIT;
    s_item_count   = 0U;
    s_box_count    = 0U;
    s_target_count = 0U;
    s_cur_idx      = 0U;
    s_nav_started  = 0U;
    s_face_started = 0U;
    s_nav_wp_idx   = 0U;
    s_nav_map_applied = 0U;
    s_map_changed  = 0U;
    s_subphase_ticks = 0U;        /* B3a */
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
            s_cur_idx = pick_next_item(player_pos);
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
                    HAL_CHASSIS_MOVE_TO((uint8)wp.x, (uint8)wp.y);
                    s_nav_started = 1U;
                    s_subphase_ticks = 0U;
                    return APP_RECOG_RUNNING;
                }
                if (s_subphase_ticks > RECOG_NAV_TIMEOUT_TICKS)
                {
                    s_items[s_cur_idx].visited = 1U;
                    s_items[s_cur_idx].ok      = 0U;
                    enter_sub_next();
                    return APP_RECOG_RUNNING;
                }
                if (!chassis_ctrl_is_arrived())
                {
                    return APP_RECOG_RUNNING;
                }
                s_nav_started = 0U;
                s_subphase_ticks = 0U;
                s_nav_wp_idx++;
                return APP_RECOG_RUNNING;
            }

            /* 推箱动作全部成功后, 一次性把相同动作应用到主控地图和稳定箱子 ID。 */
            if (s_nav_plan.push_count > 0U && !s_nav_map_applied)
            {
                if (!apply_nav_plan_to_map(map))
                {
                    s_items[s_cur_idx].visited = 1U;
                    s_items[s_cur_idx].ok = 0U;
                    enter_sub_next();
                    return APP_RECOG_RUNNING;
                }
                s_nav_map_applied = 1U;
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
            /* B3a: 3s 仍转不到位 → 直接进 SAMPLE 试试 (车头偏一点视觉也可能识别) */
            if (s_subphase_ticks > RECOG_FACE_TIMEOUT_TICKS)
            {
                enter_sub_sample();
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

            uint8 next = pick_next_item(player_pos);
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
