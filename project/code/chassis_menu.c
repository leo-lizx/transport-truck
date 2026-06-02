#include "chassis_menu.h"

#include "algo_sokoban_solver.h"
#include "app_link.h"
#include "chassis_ctrl.h"
#include "chassis_imu.h"
#include "zf_common_headfile.h"
<<<<<<< HEAD
#include "zf_driver_flash.h"
#include <string.h>
=======
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03

/*===========================================================================
 * [chassis_menu.c] IPS map monitor
 *
 * The old two-level tuning menu has been collapsed into one race-facing
 * screen:
 *   - 16 x 12 map grid.
 *   - Color blocks for map cells, only while fresh MAP frames are arriving.
 *   - Car grid position overlay.
 *   - Current yaw angle below the map.
 *===========================================================================*/

#define CHASSIS_MENU_FLASH_SECTOR      (127U)
#define CHASSIS_MENU_FLASH_PAGE        (FLASH_PAGE_7)
#define CHASSIS_MENU_FLASH_MAGIC       (0x4D4E5455U)
#define CHASSIS_MENU_FLASH_VERSION     (2U)

typedef struct
{
<<<<<<< HEAD
    uint32 magic;
    uint32 version;
    chassis_tune_params_t params;
    uint32 checksum;
} chassis_menu_flash_blob_t;

#define MENU_MAP_STALE_MS              (500U)
#define MENU_MAP_ORIGIN_X              (8U)
#define MENU_MAP_ORIGIN_Y              (32U)
#define MENU_CELL_SIZE                 (12U)
#define MENU_CELL_GAP                  (1U)
#define MENU_MAP_W                     (APP_LINK_MAP_COLS * MENU_CELL_SIZE)
#define MENU_MAP_H                     (APP_LINK_MAP_ROWS * MENU_CELL_SIZE)
=======
    const char *name;  /* 参数显示名 */
    uint8 decimal;     /* 小数位数 */
    float step;        /* 每次调节步进 */
    float min_val;     /* 参数最小允许值 */
    float max_val;     /* 参数最大允许值 */
} menu_param_meta_t;

typedef struct
{
    const char *name;        /* 页面名称（LIMIT/NAV/PID） */
    const uint8 *param_ids;  /* 当前页面包含的参数 ID 列表 */
    uint8 param_count;       /* 当前页面参数数量 */
} menu_page_meta_t;

static menu_level_enum s_menu_level = MENU_LEVEL_ROOT;  /* 当前菜单层级 */
static uint8 s_root_index = 0U;                         /* 一级菜单当前页索引 */
static uint8 s_param_index = 0U;                        /* 二级菜单当前参数索引 */
static uint8 s_need_redraw = 1U;                        /* 界面重绘请求标志 */
static uint8 s_render_ticks = 0U;                       /* 渲染节拍计数（10ms 基准） */
static char s_status_text[24] = "STATUS: READY";       /* 当前状态文本 */
static char s_status_text_prev[24] = "";               /* 上次状态文本（用于变化检测） */
static uint8 s_full_refresh_done = 0U;                  /* 是否至少完成过一次整屏绘制 */
static uint8 s_param_hold_inc_ticks = 0U;               /* K3 长按连发节拍计数（10ms 基准） */
static uint8 s_param_hold_dec_ticks = 0U;               /* K4 长按连发节拍计数（10ms 基准） */
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03

#define MENU_RGB565(r, g, b)           (uint16)((((uint16)(r) & 0xF8U) << 8) | \
                                                (((uint16)(g) & 0xF8U) << 3) | \
                                                (((uint16)(b) & 0xF8U) >> 3))

#define MENU_COLOR_BG                  RGB565_BLACK
#define MENU_COLOR_GRID                RGB565_GRAY
#define MENU_COLOR_TEXT                RGB565_WHITE
#define MENU_COLOR_TITLE               RGB565_CYAN
#define MENU_COLOR_OK                  RGB565_GREEN
#define MENU_COLOR_WAIT                RGB565_YELLOW
#define MENU_COLOR_EMPTY               MENU_COLOR_BG
#define MENU_COLOR_WALL                MENU_RGB565(57U, 65U, 82U)
#define MENU_COLOR_TARGET              MENU_RGB565(231U, 0U, 255U)
#define MENU_COLOR_BOX                 MENU_RGB565(148U, 178U, 0U)
#define MENU_COLOR_BOMB                MENU_RGB565(255U, 24U, 74U)
#define MENU_COLOR_CAR                 RGB565_CYAN

<<<<<<< HEAD
static uint8 s_flash_ready = 0U;
static uint8 s_need_full_refresh = 1U;
static uint8 s_last_map_ready = 0xFFU;
static uint32 s_last_car_frame_id = 0U;
static uint8 s_cached_map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS];  /* 缓存上一帧地图, 用于变化检测 */
static uint8 s_map_cached = 0U;                                    /* s_cached_map 是否有效 */
static uint8 s_last_car_x = 0xFFU;                                 /* 上一帧小车格 X (0xFF=无效) */
static uint8 s_last_car_y = 0xFFU;                                 /* 上一帧小车格 Y */

static uint32 menu_flash_checksum(const uint32 *words, uint16 word_count)
{
    uint16 i;
    uint32 sum = 0U;

    for (i = 0U; i < word_count; ++i)
    {
        sum += words[i];
    }
    return sum;
}

static uint8 menu_load_params_from_flash(void)
{
    uint32 raw_words[(sizeof(chassis_menu_flash_blob_t) + 3U) / 4U];
    chassis_menu_flash_blob_t blob;
    uint32 calc_checksum;
    uint16 payload_words;

    if (0U == s_flash_ready)
    {
        return 0U;
    }

    flash_read_page(CHASSIS_MENU_FLASH_SECTOR, CHASSIS_MENU_FLASH_PAGE,
                    raw_words, (uint16)(sizeof(raw_words) / sizeof(raw_words[0])));
    memcpy(&blob, raw_words, sizeof(blob));

    payload_words = (uint16)((sizeof(chassis_menu_flash_blob_t) - sizeof(uint32)) / 4U);
    calc_checksum = menu_flash_checksum((const uint32 *)&blob, payload_words);

    if ((blob.magic != CHASSIS_MENU_FLASH_MAGIC) ||
        (blob.version != CHASSIS_MENU_FLASH_VERSION) ||
        (blob.checksum != calc_checksum))
    {
        return 0U;
    }

    chassis_ctrl_set_tune_params(&blob.params);
    return 1U;
=======
static const uint8 s_page_pid_lf_params[] = { MENU_PARAM_LF_KP, MENU_PARAM_LF_KI, MENU_PARAM_LF_KD }; /* PID-LF 页参数映射 */
static const uint8 s_page_pid_rf_params[] = { MENU_PARAM_RF_KP, MENU_PARAM_RF_KI, MENU_PARAM_RF_KD }; /* PID-RF 页参数映射 */
static const uint8 s_page_pid_lb_params[] = { MENU_PARAM_LB_KP, MENU_PARAM_LB_KI, MENU_PARAM_LB_KD }; /* PID-LB 页参数映射 */
static const uint8 s_page_pid_rb_params[] = { MENU_PARAM_RB_KP, MENU_PARAM_RB_KI, MENU_PARAM_RB_KD }; /* PID-RB 页参数映射 */
static const uint8 s_page_nav_params[]   = { MENU_PARAM_POS_KP, MENU_PARAM_YAW_KP };                           /* NAV 页参数映射 */
static const uint8 s_page_limit_params[] = { MENU_PARAM_MAX_V, MENU_PARAM_MAX_W, MENU_PARAM_ACC_V, MENU_PARAM_ACC_W }; /* LIMIT 页参数映射 */

static const menu_page_meta_t s_pages[] =
{
    { "LIMIT", s_page_limit_params, (uint8)(sizeof(s_page_limit_params) / sizeof(s_page_limit_params[0])) },
    { "NAV",   s_page_nav_params,   (uint8)(sizeof(s_page_nav_params)   / sizeof(s_page_nav_params[0])) },
    { "PID-LF", s_page_pid_lf_params, (uint8)(sizeof(s_page_pid_lf_params) / sizeof(s_page_pid_lf_params[0])) },
    { "PID-RF", s_page_pid_rf_params, (uint8)(sizeof(s_page_pid_rf_params) / sizeof(s_page_pid_rf_params[0])) },
    { "PID-LB", s_page_pid_lb_params, (uint8)(sizeof(s_page_pid_lb_params) / sizeof(s_page_pid_lb_params[0])) },
    { "PID-RB", s_page_pid_rb_params, (uint8)(sizeof(s_page_pid_rb_params) / sizeof(s_page_pid_rb_params[0])) },
};

#define MENU_PAGE_COUNT ((uint8)(sizeof(s_pages) / sizeof(s_pages[0]))) /* 一级菜单页数 */

/* 根据参数ID读取当前参数值。 */
/* ==========================================================================
 *  § 1. 参数读写 (param_id ↔ g_chassis_tune_params 字段映射)
 * ========================================================================== */

static float menu_get_param_value(menu_param_id_enum id)
{
    chassis_tune_params_t params;

    chassis_ctrl_get_tune_params(&params);

    /* 按参数 ID 路由到当前参数结构中的实际字段。 */
    switch (id)
    {
        case MENU_PARAM_LF_KP:  return params.wheel_pid.kp[CHASSIS_WHEEL_LF];       /* 左前轮 Kp */
        case MENU_PARAM_LF_KI:  return params.wheel_pid.ki[CHASSIS_WHEEL_LF];       /* 左前轮 Ki */
        case MENU_PARAM_LF_KD:  return params.wheel_pid.kd[CHASSIS_WHEEL_LF];       /* 左前轮 Kd */
        case MENU_PARAM_RF_KP:  return params.wheel_pid.kp[CHASSIS_WHEEL_RF];       /* 右前轮 Kp */
        case MENU_PARAM_RF_KI:  return params.wheel_pid.ki[CHASSIS_WHEEL_RF];       /* 右前轮 Ki */
        case MENU_PARAM_RF_KD:  return params.wheel_pid.kd[CHASSIS_WHEEL_RF];       /* 右前轮 Kd */
        case MENU_PARAM_LB_KP:  return params.wheel_pid.kp[CHASSIS_WHEEL_LB];       /* 左后轮 Kp */
        case MENU_PARAM_LB_KI:  return params.wheel_pid.ki[CHASSIS_WHEEL_LB];       /* 左后轮 Ki */
        case MENU_PARAM_LB_KD:  return params.wheel_pid.kd[CHASSIS_WHEEL_LB];       /* 左后轮 Kd */
        case MENU_PARAM_RB_KP:  return params.wheel_pid.kp[CHASSIS_WHEEL_RB];       /* 右后轮 Kp */
        case MENU_PARAM_RB_KI:  return params.wheel_pid.ki[CHASSIS_WHEEL_RB];       /* 右后轮 Ki */
        case MENU_PARAM_RB_KD:  return params.wheel_pid.kd[CHASSIS_WHEEL_RB];       /* 右后轮 Kd */
        case MENU_PARAM_POS_KP: return params.position.kp;                          /* 位置环比例项 */
        case MENU_PARAM_YAW_KP: return params.yaw.kp;                               /* 航向环比例项 */
        case MENU_PARAM_MAX_V:  return params.limit.max_linear_speed_mps;           /* 最大线速度上限 */
        case MENU_PARAM_MAX_W:  return params.limit.max_yaw_speed_dps;              /* 最大角速度上限 */
        case MENU_PARAM_ACC_V:  return params.limit.accel_limit_mps2;               /* 线速度加速度限制 */
        case MENU_PARAM_ACC_W:  return params.limit.yaw_accel_limit_dps2;           /* 角速度加速度限制 */
        default:               return params.wheel_pid.kp[CHASSIS_WHEEL_LF];        /* 异常 ID 回退值 */
    }
}

/* 根据参数ID写入当前参数值。 */
static void menu_set_param_value(chassis_tune_params_t *params, menu_param_id_enum id, float value)
{
    if (params == NULL)
    {
        return;
    }

    /* 按参数 ID 写回局部快照，最后由 chassis_ctrl_set_tune_params() 统一下发。 */
    switch (id)
    {
        case MENU_PARAM_LF_KP: params->wheel_pid.kp[CHASSIS_WHEEL_LF] = value; break; /* 设置左前轮 Kp */
        case MENU_PARAM_LF_KI: params->wheel_pid.ki[CHASSIS_WHEEL_LF] = value; break; /* 设置左前轮 Ki */
        case MENU_PARAM_LF_KD: params->wheel_pid.kd[CHASSIS_WHEEL_LF] = value; break; /* 设置左前轮 Kd */
        case MENU_PARAM_RF_KP: params->wheel_pid.kp[CHASSIS_WHEEL_RF] = value; break; /* 设置右前轮 Kp */
        case MENU_PARAM_RF_KI: params->wheel_pid.ki[CHASSIS_WHEEL_RF] = value; break; /* 设置右前轮 Ki */
        case MENU_PARAM_RF_KD: params->wheel_pid.kd[CHASSIS_WHEEL_RF] = value; break; /* 设置右前轮 Kd */
        case MENU_PARAM_LB_KP: params->wheel_pid.kp[CHASSIS_WHEEL_LB] = value; break; /* 设置左后轮 Kp */
        case MENU_PARAM_LB_KI: params->wheel_pid.ki[CHASSIS_WHEEL_LB] = value; break; /* 设置左后轮 Ki */
        case MENU_PARAM_LB_KD: params->wheel_pid.kd[CHASSIS_WHEEL_LB] = value; break; /* 设置左后轮 Kd */
        case MENU_PARAM_RB_KP: params->wheel_pid.kp[CHASSIS_WHEEL_RB] = value; break; /* 设置右后轮 Kp */
        case MENU_PARAM_RB_KI: params->wheel_pid.ki[CHASSIS_WHEEL_RB] = value; break; /* 设置右后轮 Ki */
        case MENU_PARAM_RB_KD: params->wheel_pid.kd[CHASSIS_WHEEL_RB] = value; break; /* 设置右后轮 Kd */
        case MENU_PARAM_POS_KP: params->position.kp = value; break;                   /* 设置位置环 Kp */
        case MENU_PARAM_YAW_KP: params->yaw.kp = value; break;                        /* 设置航向环 Kp */
        case MENU_PARAM_MAX_V:  params->limit.max_linear_speed_mps = value; break;    /* 设置线速度上限 */
        case MENU_PARAM_MAX_W:  params->limit.max_yaw_speed_dps = value; break;       /* 设置角速度上限 */
        case MENU_PARAM_ACC_V:  params->limit.accel_limit_mps2 = value; break;        /* 设置线加速度上限 */
        case MENU_PARAM_ACC_W:  params->limit.yaw_accel_limit_dps2 = value; break;    /* 设置角加速度上限 */
        default: break;                                                               /* 异常 ID 直接忽略 */
    }
}

/* 将状态文本写入固定缓冲区，用于屏幕反馈。 */
static void menu_set_status(const char *text)
{
    uint8 i = 0U; /* 状态字符串拷贝下标 */
    if (0 == text)
    {
        return; /* 空指针保护：无有效输入则不更新状态文本 */
    }

    for (i = 0U; i < (uint8)(sizeof(s_status_text) - 1U); ++i)
    {
        if ('\0' == text[i])
        {
            break; /* 遇到源字符串结尾，提前停止拷贝 */
        }
        s_status_text[i] = text[i]; /* 逐字符复制到固定缓冲区 */
    }
    s_status_text[i] = '\0'; /* 确保目标缓冲区以 \0 终止 */
}

/* 从 Flash 读取参数，校验通过后覆盖当前参数。 */
static uint8 menu_load_params_from_flash(void)
{
    if (chassis_config_load_from_flash() != 0U)
    {
        menu_set_status("STATUS: LOAD OK");
        return 1U;
    }

    menu_set_status("STATUS: LOAD DEFAULT");
    return 0U;
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
}

static void menu_fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
<<<<<<< HEAD
    ips200_show_rgb565_image(x, y, &color, 1U, 1U, w, h, 0U);
=======
    if (chassis_config_save_to_flash() != 0U)
    {
        menu_set_status("STATUS: SAVE OK");
        return 1U;
    }

    menu_set_status("STATUS: SAVE FAIL");
    return 0U;
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
}

static void menu_clear_text_line(uint16 y)
{
    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, y, "                                        ");
}

static uint8 menu_map_frame_is_fresh(uint32 *age_ms_out)
{
    uint32 now_ms = app_link_get_ms();
    uint32 map_ms = g_link_last_map_ms;
    uint32 age_ms;

<<<<<<< HEAD
    if (0U == map_ms)
    {
        if (age_ms_out != NULL) { *age_ms_out = 0U; }
        return 0U;
    }

    age_ms = now_ms - map_ms;
    if (age_ms_out != NULL) { *age_ms_out = age_ms; }
    return (age_ms <= MENU_MAP_STALE_MS) ? 1U : 0U;
=======
    /* 先叠加增量，再做上下限保护。 */
    value = chassis_clamp_f(value + delta, meta->min_val, meta->max_val); /* 参数值加减并限幅 */
    /* 回写参数并实时下发到底盘控制层。 */
    chassis_ctrl_get_tune_params(&params);        /* 复制一份一致快照用于接口入参 */
    menu_set_param_value(&params, id, value);     /* 写回局部参数副本 */
    chassis_ctrl_set_tune_params(&params);        /* 立即下发到底盘控制模块 */
    s_need_redraw = 1U;                           /* 标记界面需要重绘 */
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03
}

static void menu_invalidate_stale_car(uint8 map_ready, app_link_car_snapshot_t *car)
{
    uint32 now_ms;
    uint32 car_age_ms;

    if (car == NULL) { return; }

    if ((0U == map_ready) || (0U == car->valid))
    {
        car->valid = 0U;
        return;
    }

    now_ms = app_link_get_ms();
    car_age_ms = now_ms - car->stamp_ms;
    if (car_age_ms > MENU_MAP_STALE_MS)
    {
        car->valid = 0U;
    }
}

static uint16 menu_cell_color(uint8 cell)
{
    switch (cell)
    {
        case MAP_WALL:   return MENU_COLOR_WALL;
        case MAP_TARGET: return MENU_COLOR_TARGET;
        case MAP_BOX:    return MENU_COLOR_BOX;
        case MAP_BOMB:   return MENU_COLOR_BOMB;
        case MAP_EMPTY:
        default:         return MENU_COLOR_EMPTY;
    }
}

static void menu_draw_grid_lines(void)
{
    uint8 i;
    uint16 x;
    uint16 y;

    for (i = 0U; i <= APP_LINK_MAP_COLS; ++i)
    {
        x = (uint16)(MENU_MAP_ORIGIN_X + (uint16)i * MENU_CELL_SIZE);
        ips200_draw_line(x, MENU_MAP_ORIGIN_Y, x, (uint16)(MENU_MAP_ORIGIN_Y + MENU_MAP_H), MENU_COLOR_GRID);
    }

    for (i = 0U; i <= APP_LINK_MAP_ROWS; ++i)
    {
        y = (uint16)(MENU_MAP_ORIGIN_Y + (uint16)i * MENU_CELL_SIZE);
        ips200_draw_line(MENU_MAP_ORIGIN_X, y, (uint16)(MENU_MAP_ORIGIN_X + MENU_MAP_W), y, MENU_COLOR_GRID);
    }
}

static void menu_draw_static_layout(uint8 map_ready)
{
    ips200_full(MENU_COLOR_BG);

    ips200_set_color(MENU_COLOR_TITLE, MENU_COLOR_BG);
    ips200_show_string(0U, 0U, "VISUAL MAP MONITOR");

    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(216U, 32U, "LEGEND");
    ips200_show_string(234U, 52U, "WALL");
    ips200_show_string(234U, 72U, "BOX");
    ips200_show_string(234U, 92U, "TARGET");
    ips200_show_string(234U, 112U, "BOMB");
    ips200_show_string(234U, 132U, "CAR");

    menu_fill_rect(216U, 52U, 12U, 12U, MENU_COLOR_WALL);
    menu_fill_rect(216U, 72U, 12U, 12U, MENU_COLOR_BOX);
    menu_fill_rect(216U, 92U, 12U, 12U, MENU_COLOR_TARGET);
    menu_fill_rect(216U, 112U, 12U, 12U, MENU_COLOR_BOMB);
    menu_fill_rect(216U, 132U, 12U, 12U, MENU_COLOR_CAR);

    menu_fill_rect(MENU_MAP_ORIGIN_X, MENU_MAP_ORIGIN_Y, MENU_MAP_W, MENU_MAP_H, MENU_COLOR_BG);
    menu_draw_grid_lines();

    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, 188U, "CAR: --,--  YAW:        deg");
    ips200_show_string(0U, 206U, "POSE X:      Y:      IMU:");

    ips200_set_color(map_ready ? MENU_COLOR_OK : MENU_COLOR_WAIT, MENU_COLOR_BG);
    ips200_show_string(0U, 16U, map_ready ? "MAP: RX    AGE:      ms" : "MAP: LOADING GRID ONLY");
}

static void menu_draw_map_cells(const uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS])
{
    uint8 r;
    uint8 c;
    uint16 x;
    uint16 y;
    uint16 color;

    for (r = 0U; r < APP_LINK_MAP_ROWS; ++r)
    {
        for (c = 0U; c < APP_LINK_MAP_COLS; ++c)
        {
            x = (uint16)(MENU_MAP_ORIGIN_X + (uint16)c * MENU_CELL_SIZE + MENU_CELL_GAP);
            y = (uint16)(MENU_MAP_ORIGIN_Y + (uint16)r * MENU_CELL_SIZE + MENU_CELL_GAP);
            color = menu_cell_color(map[r][c]);
            menu_fill_rect(x, y,
                           (uint16)(MENU_CELL_SIZE - MENU_CELL_GAP),
                           (uint16)(MENU_CELL_SIZE - MENU_CELL_GAP),
                           color);
        }
    }
}

static void menu_draw_car_marker(const app_link_car_snapshot_t *car)
{
    uint16 x;
    uint16 y;

    if ((car == NULL) || (0U == car->valid) ||
        (car->car_x >= APP_LINK_MAP_COLS) || (car->car_y >= APP_LINK_MAP_ROWS))
    {
        return;
    }

    x = (uint16)(MENU_MAP_ORIGIN_X + (uint16)car->car_x * MENU_CELL_SIZE + 2U);
    y = (uint16)(MENU_MAP_ORIGIN_Y + (uint16)car->car_y * MENU_CELL_SIZE + 2U);

    menu_fill_rect(x, y, (uint16)(MENU_CELL_SIZE - 3U), (uint16)(MENU_CELL_SIZE - 3U), MENU_COLOR_CAR);
}

/*
 * 小车移走后，用地图原始颜色恢复该格子（抹掉青色的车标记）。
 * 仅在 s_map_cached 有效时调用，使用缓存的地图数据。
 */
static void menu_restore_cell(uint8 grid_x, uint8 grid_y)
{
    uint16 x, y, color;

    if ((0U == s_map_cached) ||
        (grid_x >= APP_LINK_MAP_COLS) || (grid_y >= APP_LINK_MAP_ROWS))
    {
        return;
    }

    x = (uint16)(MENU_MAP_ORIGIN_X + (uint16)grid_x * MENU_CELL_SIZE + MENU_CELL_GAP);
    y = (uint16)(MENU_MAP_ORIGIN_Y + (uint16)grid_y * MENU_CELL_SIZE + MENU_CELL_GAP);
    color = menu_cell_color(s_cached_map[grid_y][grid_x]);
    menu_fill_rect(x, y,
                   (uint16)(MENU_CELL_SIZE - MENU_CELL_GAP),
                   (uint16)(MENU_CELL_SIZE - MENU_CELL_GAP),
                   color);
}

static void menu_draw_dynamic_text(uint8 map_ready, uint32 map_age_ms, const app_link_car_snapshot_t *car)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();
    float imu_yaw_deg = chassis_imu_get_yaw_deg();

    if (map_ready)
    {
        ips200_set_color(MENU_COLOR_OK, MENU_COLOR_BG);
        ips200_show_string(0U, 16U, "MAP: RX    AGE:      ms");
        ips200_show_uint(112U, 16U, map_age_ms, 5U);
    }
    else
    {
        ips200_set_color(MENU_COLOR_WAIT, MENU_COLOR_BG);
        ips200_show_string(0U, 16U, "MAP: LOADING GRID ONLY     ");
    }

    menu_clear_text_line(188U);
    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, 188U, "CAR:");
    if ((car != NULL) && (0U != car->valid))
    {
        ips200_show_uint(40U, 188U, car->car_x, 2U);
        ips200_show_string(58U, 188U, ",");
        ips200_show_uint(66U, 188U, car->car_y, 2U);
    }
    else
    {
        ips200_show_string(40U, 188U, "--,--");
    }
    ips200_show_string(104U, 188U, "YAW:");
    ips200_show_float(144U, 188U, pose.yaw_deg, 5U, 1U);
    ips200_show_string(200U, 188U, "deg");

    menu_clear_text_line(206U);
    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, 206U, "POSE X:");
    ips200_show_float(64U, 206U, pose.x_m, 3U, 2U);
    ips200_show_string(120U, 206U, "Y:");
    ips200_show_float(144U, 206U, pose.y_m, 3U, 2U);
    ips200_show_string(200U, 206U, "IMU:");
    ips200_show_float(240U, 206U, imu_yaw_deg, 5U, 1U);
}

void chassis_menu_init(void)
{
<<<<<<< HEAD
    s_flash_ready = (0U == flash_init()) ? 1U : 0U;
    (void)menu_load_params_from_flash();
=======
    /* 读取并应用 Flash 参数（失败会保持默认参数）。 */
    menu_load_params_from_flash(); /* 尝试读取并应用掉电保存参数 */
    /* 清空状态缓存，确保首次渲染必定更新状态行。 */
    strcpy(s_status_text_prev, ""); /* 清空旧状态文本缓存 */
    /* 重置菜单状态机到一级页面。 */
    s_menu_level = MENU_LEVEL_ROOT; /* 进入一级菜单层 */
    s_root_index = 0U;              /* 一级菜单默认选中第 0 页 */
    s_param_index = 0U;             /* 二级菜单默认选中第 0 项 */
    s_need_redraw = 1U;             /* 标记需要首次刷新 */
    s_render_ticks = 0U;            /* 渲染节拍计数清零 */
    s_full_refresh_done = 0U;       /* 清除“已全刷”标志 */
    s_param_hold_inc_ticks = 0U;    /* K3 连发计数清零 */
    s_param_hold_dec_ticks = 0U;    /* K4 连发计数清零 */
}
>>>>>>> fb10f7195934aa24ca52652ff9c18c4a13a84a03

    s_need_full_refresh = 1U;
    s_last_map_ready = 0xFFU;
    s_last_car_frame_id = 0U;
}

void chassis_menu_task_10ms(void)
{
    key_scanner();
    key_clear_state(KEY_1);
    key_clear_state(KEY_2);
    key_clear_state(KEY_3);
    key_clear_state(KEY_4);
}

void chassis_menu_render_100ms(void)
{
    uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS];
    app_link_car_snapshot_t car;
    uint32 map_age_ms = 0U;
    uint8 map_ready;
    uint8 full_refresh;

    map_ready = menu_map_frame_is_fresh(&map_age_ms);
    app_link_get_car_snapshot(&car);
    menu_invalidate_stale_car(map_ready, &car);

    /*
     * 全屏刷新仅在以下情况触发:
     *   1. 首次上电 (s_need_full_refresh==1)
     *   2. 地图就绪状态切换 (有图 ↔ 无图)
     * 其余情况只做局部更新, 避免 100ms 周期重绘导致屏幕闪烁。
     */
    full_refresh = (uint8)((0U != s_need_full_refresh) || (map_ready != s_last_map_ready));
    if (0U != full_refresh)
    {
        menu_draw_static_layout(map_ready);
        s_need_full_refresh = 0U;
        s_last_map_ready = map_ready;
        s_map_cached = 0U;          /* 全屏刷新后缓存失效, 下次强制重绘地图 */
    }

    if (map_ready)
    {
        uint8 map_changed = 0U;
        uint8 car_moved   = 0U;

        app_link_get_map_snapshot(map);

        /* 比对地图内容是否变化 */
        if ((0U == s_map_cached) ||
            (0 != memcmp(map, s_cached_map, sizeof(s_cached_map))))
        {
            map_changed = 1U;
        }

        /* 比对小车是否移动 */
        if ((0U != car.valid) &&
            (car.car_x < APP_LINK_MAP_COLS) && (car.car_y < APP_LINK_MAP_ROWS))
        {
            if ((car.car_x != s_last_car_x) || (car.car_y != s_last_car_y))
            {
                car_moved = 1U;
            }
        }

        if (0U != map_changed)
        {
            /* 地图内容变了 → 全量重绘格子 + 车标记 */
            menu_draw_map_cells(map);
            menu_draw_car_marker(&car);
            memcpy(s_cached_map, map, sizeof(s_cached_map));
            s_map_cached = 1U;
        }
        else if (0U != car_moved)
        {
            /* 地图没变, 仅小车移动 → 恢复旧格 + 画新格 */
            menu_restore_cell(s_last_car_x, s_last_car_y);
            menu_draw_car_marker(&car);
        }
        /* else: 地图和小车都没变 → 不碰地图区域, 屏幕保持静止 */

        /* 记录本帧小车位置, 供下一帧移动检测 */
        if (0U != car.valid)
        {
            s_last_car_x = car.car_x;
            s_last_car_y = car.car_y;
        }
        s_last_car_frame_id = car.frame_id;
    }
    else
    {
        /* 地图过期: 仅在状态切换时清空地图区一次 */
        if (0U != full_refresh)
        {
            menu_fill_rect(MENU_MAP_ORIGIN_X, MENU_MAP_ORIGIN_Y,
                           MENU_MAP_W, MENU_MAP_H, MENU_COLOR_BG);
            menu_draw_grid_lines();
            s_last_car_frame_id = 0U;
            s_map_cached = 0U;
        }
    }

    /*
     * 动态文字行每 100ms 必刷新:
     *   - AGE 数值持续更新 → 用户看到数字在跳就知道摄像头还在工作
     *   - 小车坐标 / 航向角 / 位姿也同步刷新
     */
    menu_draw_dynamic_text(map_ready, map_age_ms, &car);
}
