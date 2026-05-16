/*===========================================================================
 *  chassis_zone.c — 底盘几何区域判定 / 越界 / 静止检测 / 软限位 guard
 *
 *  调用节拍: 主循环 (chassis_zone_tick 5ms) + PIT_IRQn (软限位 guard)
 *  线程安全: file-static 仅主循环单线程访问; 通过 seq-lock 取位姿/地图快照
 *  依赖模块: chassis_ctrl.h (位姿 / cmd 类型), app_link.h (地图快照),
 *           chassis_config.h (物理尺寸 / 阈值), zf_common_headfile.h (基础类型)
 *  对外 API: 见 chassis_zone.h
 *
 *  重构来源 (2026-05-13):
 *    从 chassis_ctrl.c 等价迁出, 行为不变. 仅做命名前缀对齐:
 *    apply_soft_limit_guard()  → chassis_zone_apply_soft_limit_guard()
 *    soft_limit_is_*()         → zone_soft_limit_is_*  (file-static)
 *===========================================================================*/

#include "chassis_zone.h"
#include "chassis_ctrl.h"
#include "app_link.h"
#include "zf_common_headfile.h"
#include <math.h>

/*============================== 软限位常量 ==============================*/
/* 来自 chassis_ctrl.c L62-82: 防止里程计漂移导致虚拟地图"撞墙" */

#define SOFT_LIMIT_LOOKAHEAD_S             (0.22f)   /* 前瞻时间窗, 越大越保守 */
#define SOFT_LIMIT_SIDE_OFFSET_M           (0.08f)   /* 左右试探偏移 */
#define SOFT_LIMIT_BRAKE_SCALE             (0.28f)   /* 触发时线速度缩放 */
#define SOFT_LIMIT_AVOID_WZ_DPS            (35.0f)   /* 避障微调角速度 */
#define SOFT_LIMIT_MIN_MOVE_EPS_MPS        (0.01f)   /* 小于此速度不触发预测 */
#define SOFT_LIMIT_MAP_WALL                (1U)      /* 与地图编码 MAP_WALL 保持一致 */

/*============================== 内部数据 ==============================*/

/* 发车区矩形定义 (X_min, X_max, Y_min, Y_max) — Y 向下为正 */
typedef struct {
    float x_min_m;
    float x_max_m;
    float y_min_m;
    float y_max_m;
} chassis_rect_m_t;

/* 上一帧位姿 (用于差分速度); inited=0 时跳过当帧差分 */
static chassis_pose_t  s_zone_last_pose      = {0.0f, 0.0f, 0.0f};
static uint8           s_zone_last_pose_inited = 0U;

/* 一阶 IIR 平滑后的车体平移速度模长 (m/s) */
static float           s_zone_speed_lpf_mps  = 0.0f;

/* 静止累计计数 (单位: tick), 与 CHASSIS_STATIC_HOLD_MS 比较 */
static uint16          s_zone_static_ticks   = 0U;
#define ZONE_TICK_PERIOD_MS                 (5U)
#define ZONE_STATIC_HOLD_TICKS              ((uint16)(CHASSIS_STATIC_HOLD_MS / ZONE_TICK_PERIOD_MS))

/* 越界滞回标志 (1 = 已置位, 业务不复位则永久保持) */
static uint8           s_zone_oob_latched    = 0U;

/*============================== 内部辅助 ==============================*/

/** 软限位: 网格是否在可通行内场 */
static inline uint8 zone_soft_limit_is_inner_grid(int16 gx, int16 gy)
{
    if (gx < (int16)CHASSIS_GRID_INNER_MIN_X || gx > (int16)CHASSIS_GRID_INNER_MAX_X) return 0;
    if (gy < (int16)CHASSIS_GRID_INNER_MIN_Y || gy > (int16)CHASSIS_GRID_INNER_MAX_Y) return 0;
    return 1;
}

/** 软限位: 判断网格是否为墙/边界障碍 (P0-3: 改读传入的本地地图快照, 不再访 g_game_map) */
static uint8 zone_soft_limit_is_wall_grid(const uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS],
                                          int16 gx, int16 gy)
{
    if (!zone_soft_limit_is_inner_grid(gx, gy)) return 1;
    return (map[gy][gx] == SOFT_LIMIT_MAP_WALL) ? 1 : 0;
}

/** 软限位: 按全局坐标点判断是否会撞墙 */
static uint8 zone_soft_limit_is_wall_point(const uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS],
                                           float x_m, float y_m)
{
    uint8 gx = chassis_m_to_grid_x(x_m);
    uint8 gy = chassis_m_to_grid_y(y_m);
    return zone_soft_limit_is_wall_grid(map, (int16)gx, (int16)gy);
}

/** 取指定发车区矩形 (LAUNCH_ZONE_ANY 由上层拆 LEFT/RIGHT 两次调用) */
static void zone_get_launch_rect(LaunchZone_e zone, chassis_rect_m_t *out)
{
    /* Y 轴向下, "下边界" 即 Y = CHASSIS_MAP_HEIGHT_M */
    out->y_max_m = CHASSIS_MAP_HEIGHT_M - CHASSIS_LAUNCH_ZONE_BOTTOM_OFFSET_M;
    out->y_min_m = out->y_max_m - CHASSIS_LAUNCH_ZONE_H_M;

    if (zone == LAUNCH_ZONE_RIGHT) {
        out->x_max_m = CHASSIS_MAP_WIDTH_M;
        out->x_min_m = CHASSIS_MAP_WIDTH_M - CHASSIS_LAUNCH_ZONE_W_M;
    } else {
        /* 默认左发车区 */
        out->x_min_m = 0.0f;
        out->x_max_m = CHASSIS_LAUNCH_ZONE_W_M;
    }
}

/** 点是否在矩形内 (闭区间) */
static uint8 zone_point_in_rect(float x, float y, const chassis_rect_m_t *r)
{
    return ((x >= r->x_min_m) && (x <= r->x_max_m) &&
            (y >= r->y_min_m) && (y <= r->y_max_m)) ? 1U : 0U;
}

/**
 * 圆 (cx, cy, R) 是否与矩形 r 不相交 (即 "完全离开矩形").
 * 判据: 圆心到矩形最近点的距离 > R.
 */
static uint8 zone_circle_outside_rect(float cx, float cy, float radius_m,
                                      const chassis_rect_m_t *r)
{
    float dx = 0.0f;
    float dy = 0.0f;

    if      (cx < r->x_min_m) dx = r->x_min_m - cx;
    else if (cx > r->x_max_m) dx = cx - r->x_max_m;

    if      (cy < r->y_min_m) dy = r->y_min_m - cy;
    else if (cy > r->y_max_m) dy = cy - r->y_max_m;

    return ((dx * dx + dy * dy) > (radius_m * radius_m)) ? 1U : 0U;
}

/*========================== 软限位 guard (导出) ==========================*/

void chassis_zone_apply_soft_limit_guard(struct chassis_body_speed_cmd_s *cmd)
{
    /* P0-3: 在 PIT_IRQn 上下文里取一致地图 + 一致位姿副本 (LPUART1 ISR 可能抢占) */
    uint8           map_snap[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS];
    chassis_pose_t  pose_snap;
    float yaw_rad;
    float cy, sy;
    float vxg, vyg;
    float v_norm;
    float nx, ny;
    float left_x, left_y;
    float right_x, right_y;
    uint8 front_hit;
    uint8 left_hit;
    uint8 right_hit;

    if (!cmd) return;

    v_norm = sqrtf(cmd->vx_body_mps * cmd->vx_body_mps +
                   cmd->vy_body_mps * cmd->vy_body_mps);
    if (v_norm < SOFT_LIMIT_MIN_MOVE_EPS_MPS) return;

    app_link_get_map_snapshot(map_snap);   /* 192B 栈拷贝, 与 BFS 共享时序 */
    pose_snap = chassis_ctrl_get_pose();   /* P0-3 seq-lock 读 */

    yaw_rad = pose_snap.yaw_deg * CHASSIS_DEG_TO_RAD_F;
    cy = cosf(yaw_rad);
    sy = sinf(yaw_rad);

    /* 车体系速度 -> 全局速度 */
    vxg = cy * cmd->vx_body_mps - sy * cmd->vy_body_mps;
    vyg = sy * cmd->vx_body_mps + cy * cmd->vy_body_mps;

    nx = pose_snap.x_m + vxg * SOFT_LIMIT_LOOKAHEAD_S;
    ny = pose_snap.y_m + vyg * SOFT_LIMIT_LOOKAHEAD_S;
    front_hit = zone_soft_limit_is_wall_point(map_snap, nx, ny);

    if (!front_hit) return;

    /* 触发软限位: 先强制减速, 避免继续顶墙 */
    cmd->vx_body_mps *= SOFT_LIMIT_BRAKE_SCALE;
    cmd->vy_body_mps *= SOFT_LIMIT_BRAKE_SCALE;

    /* 使用法向偏移评估左右绕行可行性 */
    left_x  = nx - sy * SOFT_LIMIT_SIDE_OFFSET_M;
    left_y  = ny + cy * SOFT_LIMIT_SIDE_OFFSET_M;
    right_x = nx + sy * SOFT_LIMIT_SIDE_OFFSET_M;
    right_y = ny - cy * SOFT_LIMIT_SIDE_OFFSET_M;

    left_hit  = zone_soft_limit_is_wall_point(map_snap, left_x, left_y);
    right_hit = zone_soft_limit_is_wall_point(map_snap, right_x, right_y);

    if (left_hit && !right_hit) {
        cmd->wz_dps -= SOFT_LIMIT_AVOID_WZ_DPS;
    } else if (!left_hit && right_hit) {
        cmd->wz_dps += SOFT_LIMIT_AVOID_WZ_DPS;
    } else {
        /* 两侧同样拥挤/开阔: 保留原命令转向方向, 给一个固定偏置 */
        if (cmd->wz_dps >= 0.0f) {
            cmd->wz_dps += SOFT_LIMIT_AVOID_WZ_DPS;
        } else {
            cmd->wz_dps -= SOFT_LIMIT_AVOID_WZ_DPS;
        }
    }
}

/*============================== 区域 API ==============================*/
/* 来自 chassis_ctrl.c L2121-2218, 行为完全等价                          */

void chassis_zone_tick(void)
{
    chassis_pose_t  cur;
    float           dx_m;
    float           dy_m;
    float           inst_speed_mps;
    float           cx;
    float           cy;
    uint8           outside_field;

    cur = chassis_ctrl_get_pose();

    /* (1) 速度估算: 5ms 差分 + 一阶 IIR 平滑 */
    if (s_zone_last_pose_inited) {
        dx_m = cur.x_m - s_zone_last_pose.x_m;
        dy_m = cur.y_m - s_zone_last_pose.y_m;
        /* dt = ZONE_TICK_PERIOD_MS/1000 = 0.005s; 乘 200.0 等价除 0.005, 省一次浮点除 */
        inst_speed_mps = sqrtf(dx_m * dx_m + dy_m * dy_m) * 200.0f;
        s_zone_speed_lpf_mps = (1.0f - CHASSIS_SPEED_LPF_ALPHA) * s_zone_speed_lpf_mps
                             + CHASSIS_SPEED_LPF_ALPHA * inst_speed_mps;
    } else {
        s_zone_last_pose_inited = 1U;
        s_zone_speed_lpf_mps    = 0.0f;
    }
    s_zone_last_pose = cur;

    /* (2) 静止累计: 速度低于阈值才累加; 否则立即清零 */
    if (s_zone_speed_lpf_mps < CHASSIS_STATIC_SPEED_EPS_MPS) {
        if (s_zone_static_ticks < 0xFFFFU) {
            s_zone_static_ticks++;
        }
    } else {
        s_zone_static_ticks = 0U;
    }

    /* (3) OOB 滞回: 单向置位, 必须 chassis_zone_clear_oob() 才能解锁 */
    if (s_zone_oob_latched) {
        return;
    }

    /* 车体外接圆穿出最外圈围墙 > HYSTERESIS 才置位 */
    cx = cur.x_m;
    cy = cur.y_m;
    outside_field = 0U;
    if ((cx + CHASSIS_BODY_RADIUS_M) > (CHASSIS_MAP_WIDTH_M  + CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;
    if ((cx - CHASSIS_BODY_RADIUS_M) < (0.0f                 - CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;
    if ((cy + CHASSIS_BODY_RADIUS_M) > (CHASSIS_MAP_HEIGHT_M + CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;
    if ((cy - CHASSIS_BODY_RADIUS_M) < (0.0f                 - CHASSIS_OOB_HYSTERESIS_M)) outside_field = 1U;

    if (outside_field) {
        s_zone_oob_latched = 1U;
    }
}

uint8 chassis_zone_is_in_launch(LaunchZone_e zone)
{
    chassis_pose_t   pose_snap;
    chassis_rect_m_t rect;

    pose_snap = chassis_ctrl_get_pose();

    if (zone == LAUNCH_ZONE_ANY) {
        zone_get_launch_rect(LAUNCH_ZONE_LEFT, &rect);
        if (zone_point_in_rect(pose_snap.x_m, pose_snap.y_m, &rect)) return 1U;
        zone_get_launch_rect(LAUNCH_ZONE_RIGHT, &rect);
        return zone_point_in_rect(pose_snap.x_m, pose_snap.y_m, &rect);
    }

    zone_get_launch_rect(zone, &rect);
    return zone_point_in_rect(pose_snap.x_m, pose_snap.y_m, &rect);
}

uint8 chassis_zone_is_fully_outside_launch(LaunchZone_e zone)
{
    chassis_pose_t   pose_snap;
    chassis_rect_m_t rect;

    pose_snap = chassis_ctrl_get_pose();

    if (zone == LAUNCH_ZONE_ANY) {
        /* 必须同时离开左 + 右两个发车区 */
        zone_get_launch_rect(LAUNCH_ZONE_LEFT, &rect);
        if (!zone_circle_outside_rect(pose_snap.x_m, pose_snap.y_m,
                                      CHASSIS_BODY_RADIUS_M, &rect)) return 0U;
        zone_get_launch_rect(LAUNCH_ZONE_RIGHT, &rect);
        return zone_circle_outside_rect(pose_snap.x_m, pose_snap.y_m,
                                        CHASSIS_BODY_RADIUS_M, &rect);
    }

    zone_get_launch_rect(zone, &rect);
    return zone_circle_outside_rect(pose_snap.x_m, pose_snap.y_m,
                                    CHASSIS_BODY_RADIUS_M, &rect);
}

uint8 chassis_zone_is_out_of_bounds(void)
{
    return s_zone_oob_latched;
}

void chassis_zone_clear_oob(void)
{
    s_zone_oob_latched = 0U;
}

uint8 chassis_zone_is_static(void)
{
    return (s_zone_static_ticks >= ZONE_STATIC_HOLD_TICKS) ? 1U : 0U;
}
