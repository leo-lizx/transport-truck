#include "chassis_menu.h"

#include "algo_sokoban_solver.h"
#include "app_game_logic.h"
#include "app_link.h"
#include "chassis_ctrl.h"
#include "chassis_imu.h"
#include "zf_common_headfile.h"
#include "zf_driver_flash.h"
#include <string.h>

/*===========================================================================
 * [chassis_menu.c] IPS map monitor
 *
 * @owner  rt1064-main
 * @periph SPI3                    IPS200 调试菜单屏 SCK=B0 MOSI=B1 RST=B2 DC=C19 CS=B3 BL=C18
 * @periph GPIO_C15,C14,C13,C12   按键 K1-K4 (key_scanner 轮询)
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
#define CHASSIS_MENU_FLASH_VERSION     (5U)  /* X/Y 独立位置参数改变了 chassis_tune_params_t 布局 */

typedef struct
{
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
#define MENU_SIDE_X                    (216U)
#define MENU_SIDE_W                    (104U)
#define MENU_RECOG_Y                   (152U)

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

static uint8 s_flash_ready = 0U;
static uint8 s_need_full_refresh = 1U;
static uint8 s_last_map_ready = 0xFFU;
static uint8 s_cached_map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS];  /* 缓存上一帧地图, 用于变化检测 */
static uint8 s_map_cached = 0U;                                    /* s_cached_map 是否有效 */

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
}

static void menu_fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    ips200_show_rgb565_image(x, y, &color, 1U, 1U, w, h, 0U);
}

static void menu_clear_text_line(uint16 y)
{
    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, y, "                                        ");
}

static void menu_draw_recognize_object(void)
{
    AppRecognizeDebug_t recog;
    const char *kind = "--";

    Game_Get_Recognize_Debug(&recog);

    menu_fill_rect(MENU_SIDE_X, MENU_RECOG_Y, MENU_SIDE_W, 32U, MENU_COLOR_BG);
    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);

    if ((recog.total_targets == 0U) || (recog.current_idx >= recog.total_targets))
    {
        ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y, "OBJ: --");
        ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y + 16U, "I:--/-- C:--");
        return;
    }

    if (recog.current_kind == APP_LINK_OBJ_KIND_BOX)
    {
        kind = "BOX";
    }
    else if (recog.current_kind == APP_LINK_OBJ_KIND_TARGET)
    {
        kind = "TARGET";
    }

    ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y, "OBJ: ");
    ips200_show_string(MENU_SIDE_X + 40U, MENU_RECOG_Y, kind);
    ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y + 16U, "I:");
    ips200_show_uint(MENU_SIDE_X + 16U, MENU_RECOG_Y + 16U, (uint32)(recog.current_idx + 1U), 2U);
    ips200_show_string(MENU_SIDE_X + 34U, MENU_RECOG_Y + 16U, "/");
    ips200_show_uint(MENU_SIDE_X + 42U, MENU_RECOG_Y + 16U, (uint32)recog.total_targets, 2U);
    ips200_show_string(MENU_SIDE_X + 64U, MENU_RECOG_Y + 16U, "C:");
    if (recog.current_class_id == 0U)
    {
        ips200_show_string(MENU_SIDE_X + 80U, MENU_RECOG_Y + 16U, "--");
    }
    else
    {
        ips200_show_uint(MENU_SIDE_X + 80U, MENU_RECOG_Y + 16U, (uint32)recog.current_class_id, 2U);
    }
}

static uint8 menu_map_frame_is_fresh(uint32 *age_ms_out)
{
    uint32 now_ms = app_link_get_ms();
    uint32 map_ms = g_link_last_map_ms;
    uint32 age_ms;

    if (0U == map_ms)
    {
        if (age_ms_out != NULL) { *age_ms_out = 0U; }
        return 0U;
    }

    age_ms = now_ms - map_ms;
    if (age_ms_out != NULL) { *age_ms_out = age_ms; }
    return (age_ms <= MENU_MAP_STALE_MS) ? 1U : 0U;
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
    ips200_show_string(0U, 0U, "MAP / CLASS MONITOR");

    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(MENU_SIDE_X, 32U, "LEGEND");
    ips200_show_string(234U, 52U, "WALL");
    ips200_show_string(234U, 72U, "BOX");
    ips200_show_string(234U, 92U, "TARGET");
    ips200_show_string(234U, 112U, "BOMB");

    menu_fill_rect(216U, 52U, 12U, 12U, MENU_COLOR_WALL);
    menu_fill_rect(216U, 72U, 12U, 12U, MENU_COLOR_BOX);
    menu_fill_rect(216U, 92U, 12U, 12U, MENU_COLOR_TARGET);
    menu_fill_rect(216U, 112U, 12U, 12U, MENU_COLOR_BOMB);

    ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y, "OBJ: --");
    ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y + 16U, "I:--/-- C:--");

    menu_fill_rect(MENU_MAP_ORIGIN_X, MENU_MAP_ORIGIN_Y, MENU_MAP_W, MENU_MAP_H, MENU_COLOR_BG);
    menu_draw_grid_lines();

    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, 188U, "YAW:        deg");
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

static void menu_draw_dynamic_text(uint8 map_ready, uint32 map_age_ms)
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
    ips200_show_string(0U, 188U, "YAW:");
    ips200_show_float(40U, 188U, pose.yaw_deg, 5U, 1U);
    ips200_show_string(96U, 188U, "deg");

    menu_clear_text_line(206U);
    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, 206U, "POSE X:");
    ips200_show_float(64U, 206U, pose.x_m, 3U, 2U);
    ips200_show_string(120U, 206U, "Y:");
    ips200_show_float(144U, 206U, pose.y_m, 3U, 2U);
    ips200_show_string(200U, 206U, "IMU:");
    ips200_show_float(240U, 206U, imu_yaw_deg, 5U, 1U);

    menu_draw_recognize_object();
}

void chassis_menu_init(void)
{
    s_flash_ready = (0U == flash_init()) ? 1U : 0U;
    (void)menu_load_params_from_flash();

    s_need_full_refresh = 1U;
    s_last_map_ready = 0xFFU;
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
    uint32 map_age_ms = 0U;
    uint8 map_ready;
    uint8 full_refresh;

    map_ready = menu_map_frame_is_fresh(&map_age_ms);

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

        app_link_get_map_snapshot(map);

        /* 比对地图内容是否变化 */
        if ((0U == s_map_cached) ||
            (0 != memcmp(map, s_cached_map, sizeof(s_cached_map))))
        {
            map_changed = 1U;
        }

        if (0U != map_changed)
        {
            /* 地图内容变了 → 全量重绘格子。 */
            menu_draw_map_cells(map);
            memcpy(s_cached_map, map, sizeof(s_cached_map));
            s_map_cached = 1U;
        }
        /* else: 地图没变 → 不碰地图区域, 屏幕保持静止 */
    }
    else
    {
        /* 地图过期: 仅在状态切换时清空地图区一次 */
        if (0U != full_refresh)
        {
            menu_fill_rect(MENU_MAP_ORIGIN_X, MENU_MAP_ORIGIN_Y,
                           MENU_MAP_W, MENU_MAP_H, MENU_COLOR_BG);
            menu_draw_grid_lines();
            s_map_cached = 0U;
        }
    }

    /*
     * 动态文字行每 100ms 必刷新:
     *   - AGE 数值持续更新 → 用户看到数字在跳就知道摄像头还在工作
     *   - 陀螺仪/里程计航向与位姿也同步刷新
     */
    menu_draw_dynamic_text(map_ready, map_age_ms);
}
