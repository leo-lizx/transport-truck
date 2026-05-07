/*********************************************************************************************************************
 * app_vision_fusion.c — 视觉位姿融合实现
 *
 * 设计 (见 视觉位姿融合实现说明.md §算法选型):
 *   - 默认走「事件驱动 Snap」: 到位 + 短静止 → 多帧表决 → Snap 到格中心。
 *   - 「连续融合 task」保留为可选 (CHASSIS_VISION_FUSION_ENABLE)，作为亚格坐标升级后的扩展点。
 *   - 「一致性 tick」作为运动中安全网: 大幅打滑/搬车时硬重定位。
 *
 * 状态机 (Snap):
 *   IDLE ──snap_request──► PENDING ──N 帧表决达 VOTE_MIN──► DONE
 *                              │
 *                              ├── 超过 SNAP_TIMEOUT_MS ─► TIMEOUT
 *                              └── 表决格与目标差距 > MAX_GAP_CELLS ─► REJECT
 *
 * 多线程/中断关系:
 *   - 表决 / 监控状态都为主循环侧 (app_game_logic Run()) 串行更新, 不在 ISR 中写。
 *   - 仅通过 app_link_get_car_snapshot() (内部 seq-lock) 读 ISR 写入的车位。
 *********************************************************************************************************************/

#include "app_vision_fusion.h"
#include "app_link.h"
#include "chassis_config.h"
#include "chassis_ctrl.h"
#include <math.h>

/* 须与 app_game_logic.c 中 GAME_LOGIC_TASK_PERIOD_MS 一致 */
#define APP_GAME_LOGIC_TICK_MS  (5U)

/*===================================================================================================================
 * 模块共享: 诊断计数 (主循环侧串行写, 读端可裸读)
 *=================================================================================================================*/
static app_vision_fusion_stats_t s_stats = {0U, 0U, 0U, 0U};

void app_vision_fusion_get_stats(app_vision_fusion_stats_t *out)
{
    if (out == NULL) { return; }
    *out = s_stats;
}

/*===================================================================================================================
 * (1) 运动中连续融合 — 默认编译为空; 仅在 CHASSIS_VISION_FUSION_ENABLE==1 时启用。
 *=================================================================================================================*/
#if CHASSIS_VISION_FUSION_ENABLE

void app_vision_fusion_task(uint8 allow_fuse)
{
    static uint16 s_period_accum_ms = 0U;
    app_link_car_snapshot_t snap;
    uint32 now_ms;
    uint32 age_ms;
    float pred_dt_s;
    float vxg;
    float vyg;
    float pred_x;
    float pred_y;
    chassis_pose_t pose;
    float err_x;
    float err_y;
    float err_mag;
    float alpha;
    float vnorm;
    float nx;
    float ny;
    float sx;
    float sy;
    float step_mag;

    if (allow_fuse == 0U)
    {
        s_period_accum_ms = 0U;
        return;
    }

    s_period_accum_ms = (uint16)(s_period_accum_ms + APP_GAME_LOGIC_TICK_MS);
    if (s_period_accum_ms < (uint16)CHASSIS_VISION_FUSION_PERIOD_MS) { return; }
    s_period_accum_ms = 0U;

    app_link_get_car_snapshot(&snap);
    if (snap.valid == 0U) { return; }

    now_ms = app_link_get_ms();
    age_ms = (now_ms >= snap.stamp_ms) ? (now_ms - snap.stamp_ms) : 0U;
    if (age_ms > (uint32)CHASSIS_VISION_MAX_OBS_AGE_MS) { return; }

    pred_dt_s = ((float)age_ms + (float)CHASSIS_VISION_DELAY_COMP_MS) * 0.001f;
    chassis_ctrl_get_odom_velocity_global_mps(&vxg, &vyg);

    pred_x = chassis_grid_x_to_m(snap.car_x) + vxg * pred_dt_s;
    pred_y = chassis_grid_y_to_m(snap.car_y) + vyg * pred_dt_s;

    pose = chassis_ctrl_get_pose();
    err_x = pred_x - pose.x_m;
    err_y = pred_y - pose.y_m;
    err_mag = sqrtf(err_x * err_x + err_y * err_y);
    if (err_mag < CHASSIS_VISION_IGNORE_ERR_M) { return; }

    alpha = CHASSIS_VISION_SOFT_ALPHA;
    if (err_mag > CHASSIS_VISION_LARGE_ERR_M)
    {
        alpha = alpha * (CHASSIS_VISION_LARGE_ERR_M / err_mag);
    }

    vnorm = sqrtf(vxg * vxg + vyg * vyg);
    if (vnorm < CHASSIS_VISION_STATIC_SPEED_MPS)
    {
        alpha *= CHASSIS_VISION_ALPHA_STATIC_SCALE;
        if (alpha > 1.0f) { alpha = 1.0f; }
    }

    nx = pose.x_m + alpha * err_x;
    ny = pose.y_m + alpha * err_y;

    sx = nx - pose.x_m;
    sy = ny - pose.y_m;
    step_mag = sqrtf(sx * sx + sy * sy);
    if ((step_mag > CHASSIS_VISION_MAX_STEP_M) && (step_mag > 1e-6f))
    {
        float sc = CHASSIS_VISION_MAX_STEP_M / step_mag;
        nx = pose.x_m + sx * sc;
        ny = pose.y_m + sy * sc;
    }

    chassis_ctrl_set_pose(nx, ny, pose.yaw_deg);
}

#else  /* CHASSIS_VISION_FUSION_ENABLE == 0 */

void app_vision_fusion_task(uint8 allow_fuse)
{
    (void)allow_fuse;
}

#endif /* CHASSIS_VISION_FUSION_ENABLE */

/*===================================================================================================================
 * (2) 到站 Snap 状态机
 *-------------------------------------------------------------------------------------------------------------------
 * 行为:
 *   - request 调用频率 = 主循环 5ms, 内部聚合, 直到 DONE/TIMEOUT/REJECT。
 *   - 同 frame_id 不重复计票。
 *   - 短静止判据: 平滑速度 < SETTLE_SPEED 且持续 SETTLE_MS 才允许采样。
 *   - 表决格 (cx, cy) 必须满足 |cx-tgt_grid_x| + |cy-tgt_grid_y| ≤ MAX_GAP_CELLS。
 *   - DONE 时 chassis_ctrl_set_pose(grid_to_m(cx, cy), pose.yaw_deg) (yaw 不动)。
 *=================================================================================================================*/
#if CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE

#define SNAP_VOTE_FRAMES_C    ((uint8)CHASSIS_VISION_SNAP_VOTE_FRAMES)
#define SNAP_VOTE_MIN_C       ((uint8)CHASSIS_VISION_SNAP_VOTE_MIN)
#define SNAP_MAX_GAP_C        ((uint8)CHASSIS_VISION_SNAP_MAX_GAP_CELLS)

typedef struct
{
    uint8  cell_x;
    uint8  cell_y;
    uint8  count;
} snap_vote_bin_t;

static app_vision_snap_state_e s_snap_state         = APP_VISION_SNAP_IDLE;
static float    s_snap_target_x_m       = 0.0f;
static float    s_snap_target_y_m       = 0.0f;
static uint32   s_snap_started_ms       = 0U;       /* 进入 PENDING 时刻 */
static uint16   s_snap_settle_accum_ms  = 0U;       /* 短静止累计 */
static uint32   s_snap_last_seen_fid    = 0U;       /* 已计票的 frame_id (0=未见) */
static snap_vote_bin_t s_snap_bins[SNAP_VOTE_FRAMES_C];
static uint8    s_snap_bin_used         = 0U;
static uint8    s_snap_total_votes      = 0U;

static void snap_reset_to_idle(void)
{
    s_snap_state         = APP_VISION_SNAP_IDLE;
    s_snap_settle_accum_ms = 0U;
    s_snap_last_seen_fid   = 0U;
    s_snap_bin_used        = 0U;
    s_snap_total_votes     = 0U;
}

static void snap_begin_pending(float tgt_x_m, float tgt_y_m, uint32 now_ms)
{
    s_snap_state         = APP_VISION_SNAP_PENDING;
    s_snap_target_x_m    = tgt_x_m;
    s_snap_target_y_m    = tgt_y_m;
    s_snap_started_ms    = now_ms;
    s_snap_settle_accum_ms = 0U;
    s_snap_last_seen_fid   = 0U;
    s_snap_bin_used        = 0U;
    s_snap_total_votes     = 0U;
}

/** 把 (cx, cy) 投到 bin; 返回当前最高票 bin 的 *count* 与索引 (idx_out 可空) */
static uint8 snap_vote_add_and_top(uint8 cx, uint8 cy, uint8 *idx_out)
{
    uint8 i;
    uint8 top_cnt = 0U;
    uint8 top_idx = 0U;

    for (i = 0U; i < s_snap_bin_used; ++i)
    {
        if ((s_snap_bins[i].cell_x == cx) && (s_snap_bins[i].cell_y == cy))
        {
            if (s_snap_bins[i].count < 255U) { s_snap_bins[i].count++; }
            break;
        }
    }
    if (i >= s_snap_bin_used)
    {
        if (s_snap_bin_used < SNAP_VOTE_FRAMES_C)
        {
            s_snap_bins[s_snap_bin_used].cell_x = cx;
            s_snap_bins[s_snap_bin_used].cell_y = cy;
            s_snap_bins[s_snap_bin_used].count  = 1U;
            ++s_snap_bin_used;
        }
        /* 若 bin 已满且新格不同 → 不增 bin, 让既有票决出胜负, 超时则 TIMEOUT */
    }
    if (s_snap_total_votes < 255U) { ++s_snap_total_votes; }

    for (i = 0U; i < s_snap_bin_used; ++i)
    {
        if (s_snap_bins[i].count > top_cnt)
        {
            top_cnt = s_snap_bins[i].count;
            top_idx = i;
        }
    }
    if (idx_out != NULL) { *idx_out = top_idx; }
    return top_cnt;
}

void app_vision_fusion_snap_request(float target_x_m, float target_y_m)
{
    uint32 now_ms = app_link_get_ms();

    /* 终态再来一次 request → 视为开启新一次表决 (上层已经接受过结果) */
    if ((s_snap_state == APP_VISION_SNAP_DONE)    ||
        (s_snap_state == APP_VISION_SNAP_TIMEOUT) ||
        (s_snap_state == APP_VISION_SNAP_REJECT))
    {
        snap_reset_to_idle();
    }

    if (s_snap_state == APP_VISION_SNAP_IDLE)
    {
        snap_begin_pending(target_x_m, target_y_m, now_ms);
    }
    else
    {
        /* PENDING: 若目标点变了 (业务侧切换航点)，重启表决 */
        float dx = target_x_m - s_snap_target_x_m;
        float dy = target_y_m - s_snap_target_y_m;
        if ((fabsf(dx) > 1e-3f) || (fabsf(dy) > 1e-3f))
        {
            snap_begin_pending(target_x_m, target_y_m, now_ms);
        }
    }

    /* ---- 短静止门 ---- */
    {
        float vxg;
        float vyg;
        float vnorm;
        chassis_ctrl_get_odom_velocity_global_mps(&vxg, &vyg);
        vnorm = sqrtf(vxg * vxg + vyg * vyg);
        if (vnorm < CHASSIS_VISION_SNAP_SETTLE_SPEED_MPS)
        {
            if ((uint32)s_snap_settle_accum_ms + (uint32)APP_GAME_LOGIC_TICK_MS <= 0xFFFFU)
            {
                s_snap_settle_accum_ms = (uint16)(s_snap_settle_accum_ms + APP_GAME_LOGIC_TICK_MS);
            }
        }
        else
        {
            s_snap_settle_accum_ms = 0U;
        }
    }

    /* 超时优先判定 (超时也属于终态, 业务侧据此放行) */
    {
        uint32 elapsed = (now_ms >= s_snap_started_ms) ? (now_ms - s_snap_started_ms) : 0U;
        if (elapsed > (uint32)CHASSIS_VISION_SNAP_TIMEOUT_MS)
        {
            s_snap_state = APP_VISION_SNAP_TIMEOUT;
            ++s_stats.snap_timeout;
            return;
        }
    }

    /* 仅在短静止后才采票 */
    if (s_snap_settle_accum_ms < (uint16)CHASSIS_VISION_SNAP_SETTLE_MS) { return; }

    {
        app_link_car_snapshot_t snap;
        uint32 age_ms;
        uint8  top_idx = 0U;
        uint8  top_cnt;

        app_link_get_car_snapshot(&snap);
        if (snap.valid == 0U) { return; }
        if ((snap.frame_id == 0U) || (snap.frame_id == s_snap_last_seen_fid)) { return; }
        age_ms = (now_ms >= snap.stamp_ms) ? (now_ms - snap.stamp_ms) : 0U;
        if (age_ms > (uint32)CHASSIS_VISION_SNAP_MAX_SNAP_AGE_MS) { return; }

        s_snap_last_seen_fid = snap.frame_id;
        top_cnt = snap_vote_add_and_top(snap.car_x, snap.car_y, &top_idx);

        if (top_cnt >= SNAP_VOTE_MIN_C)
        {
            uint8 cx = s_snap_bins[top_idx].cell_x;
            uint8 cy = s_snap_bins[top_idx].cell_y;
            uint8 tgx = chassis_m_to_grid_x(s_snap_target_x_m);
            uint8 tgy = chassis_m_to_grid_y(s_snap_target_y_m);
            int32 mdx = (int32)cx - (int32)tgx; if (mdx < 0) { mdx = -mdx; }
            int32 mdy = (int32)cy - (int32)tgy; if (mdy < 0) { mdy = -mdy; }

            if ((uint32)(mdx + mdy) > (uint32)SNAP_MAX_GAP_C)
            {
                s_snap_state = APP_VISION_SNAP_REJECT;
                ++s_stats.snap_reject;
                return;
            }

            {
                chassis_pose_t pose = chassis_ctrl_get_pose();
                float new_x = chassis_grid_x_to_m(cx);
                float new_y = chassis_grid_y_to_m(cy);
                chassis_ctrl_set_pose(new_x, new_y, pose.yaw_deg);
            }
            s_snap_state = APP_VISION_SNAP_DONE;
            ++s_stats.snap_done;
        }
    }
}

app_vision_snap_state_e app_vision_fusion_snap_state(void)
{
    return s_snap_state;
}

void app_vision_fusion_snap_cancel(void)
{
    snap_reset_to_idle();
}

#else  /* CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE == 0 */

void app_vision_fusion_snap_request(float target_x_m, float target_y_m)
{
    (void)target_x_m;
    (void)target_y_m;
}

app_vision_snap_state_e app_vision_fusion_snap_state(void)
{
    /* 关闭 Snap 时, 视为始终放行 (DONE)。 */
    return APP_VISION_SNAP_DONE;
}

void app_vision_fusion_snap_cancel(void)
{
}

#endif /* CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE */

/*===================================================================================================================
 * (3) 运动中一致性监控
 *-------------------------------------------------------------------------------------------------------------------
 * 思路: 每 PERIOD_MS 比对 (odom 米→格) 与最新视觉格的曼哈顿差;
 *       若 ≥ GAP_CELLS 持续 ≥ HOLD_MS 内不收敛, 视为 odom 大幅打滑或被搬动,
 *       直接 chassis_ctrl_set_pose 到视觉格中心 (yaw 不动) 并进入 COOLDOWN_MS 冷却。
 *=================================================================================================================*/
#if CHASSIS_VISION_CONSISTENCY_ENABLE

static uint16 s_cons_period_accum_ms = 0U;
static uint16 s_cons_gap_hold_ms     = 0U;       /* 当前“gap 持续”累计 */
static uint16 s_cons_cooldown_ms     = 0U;

void app_vision_fusion_consistency_tick(uint8 allow)
{
    if (allow == 0U)
    {
        s_cons_period_accum_ms = 0U;
        s_cons_gap_hold_ms     = 0U;
        /* 不清冷却: 让冷却跨 allow=0 段继续递减, 避免反复触发 */
        if (s_cons_cooldown_ms >= APP_GAME_LOGIC_TICK_MS)
        {
            s_cons_cooldown_ms = (uint16)(s_cons_cooldown_ms - APP_GAME_LOGIC_TICK_MS);
        }
        else
        {
            s_cons_cooldown_ms = 0U;
        }
        return;
    }

    if (s_cons_cooldown_ms >= APP_GAME_LOGIC_TICK_MS)
    {
        s_cons_cooldown_ms = (uint16)(s_cons_cooldown_ms - APP_GAME_LOGIC_TICK_MS);
    }
    else
    {
        s_cons_cooldown_ms = 0U;
    }

    s_cons_period_accum_ms = (uint16)(s_cons_period_accum_ms + APP_GAME_LOGIC_TICK_MS);
    if (s_cons_period_accum_ms < (uint16)CHASSIS_VISION_CONSISTENCY_PERIOD_MS) { return; }
    s_cons_period_accum_ms = 0U;

    if (s_cons_cooldown_ms > 0U) { return; }

    {
        app_link_car_snapshot_t snap;
        uint32 now_ms;
        uint32 age_ms;
        chassis_pose_t pose;
        uint8 ogx;
        uint8 ogy;
        int32 dx;
        int32 dy;
        uint32 gap;

        app_link_get_car_snapshot(&snap);
        if (snap.valid == 0U) { s_cons_gap_hold_ms = 0U; return; }
        now_ms = app_link_get_ms();
        age_ms = (now_ms >= snap.stamp_ms) ? (now_ms - snap.stamp_ms) : 0U;
        if (age_ms > (uint32)CHASSIS_VISION_CONSISTENCY_MAX_AGE_MS)
        {
            s_cons_gap_hold_ms = 0U;
            return;
        }

        pose = chassis_ctrl_get_pose();
        ogx  = chassis_m_to_grid_x(pose.x_m);
        ogy  = chassis_m_to_grid_y(pose.y_m);
        dx   = (int32)snap.car_x - (int32)ogx; if (dx < 0) { dx = -dx; }
        dy   = (int32)snap.car_y - (int32)ogy; if (dy < 0) { dy = -dy; }
        gap  = (uint32)(dx + dy);

        if (gap >= (uint32)CHASSIS_VISION_CONSISTENCY_GAP_CELLS)
        {
            uint32 hold_next = (uint32)s_cons_gap_hold_ms + (uint32)CHASSIS_VISION_CONSISTENCY_PERIOD_MS;
            if (hold_next > 0xFFFFU) { hold_next = 0xFFFFU; }
            s_cons_gap_hold_ms = (uint16)hold_next;

            if (s_cons_gap_hold_ms >= (uint16)CHASSIS_VISION_CONSISTENCY_HOLD_MS)
            {
                float new_x = chassis_grid_x_to_m(snap.car_x);
                float new_y = chassis_grid_y_to_m(snap.car_y);
                chassis_ctrl_set_pose(new_x, new_y, pose.yaw_deg);
                ++s_stats.consistency_fire;
                s_cons_gap_hold_ms = 0U;
                s_cons_cooldown_ms = (uint16)CHASSIS_VISION_CONSISTENCY_COOLDOWN_MS;
            }
        }
        else
        {
            s_cons_gap_hold_ms = 0U;
        }
    }
}

#else  /* CHASSIS_VISION_CONSISTENCY_ENABLE == 0 */

void app_vision_fusion_consistency_tick(uint8 allow)
{
    (void)allow;
}

#endif /* CHASSIS_VISION_CONSISTENCY_ENABLE */
