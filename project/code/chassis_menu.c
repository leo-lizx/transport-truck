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
 *   - Normal modes show fresh MAP frames; race mode shows accepted frozen maps only.
 *   - Car grid position overlay.
 *   - Normal mode shows pose data; race mode shows all recognized BOX/TARGET class IDs.
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
#define MENU_RECOG_Y                   (152U)
#define MENU_TEXT_COLS                 (40U)
#define MENU_RECOG_BOX_Y               (188U)
#define MENU_RECOG_TARGET_Y            (206U)

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
static uint8 s_dynamic_cached = 0U;
static uint32 s_last_map_age_ms = 0U;
static int32 s_last_pose_yaw_key = 0;
static int32 s_last_pose_x_key = 0;
static int32 s_last_pose_y_key = 0;
static int32 s_last_imu_yaw_key = 0;
static uint8 s_recog_cached = 0U;
static uint32 s_last_frozen_map_generation = 0U;
static uint8 s_last_recog_valid = 0U;
static uint8 s_last_recog_idx = 0U;
static uint8 s_last_recog_total = 0U;
static uint8 s_last_recog_kind = 0U;
static uint8 s_last_recog_class = 0U;
static uint8 s_recog_lists_cached = 0U;
static char s_last_box_line[MENU_TEXT_COLS + 1U];
static char s_last_target_line[MENU_TEXT_COLS + 1U];
static uint8 s_game_status_cached = 0U;
static GameRuntimeStatus_t s_last_game_status;

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

static int32 menu_float_display_key(float value, uint8 point_digits)
{
    float scale = (point_digits == 1U) ? 10.0f : 100.0f;
    int32 key = (int32)(value * scale);

    /* ips200_show_float 会显示 -0.0/-0.00, 这里保留零附近的符号差异。 */
    if ((value < 0.0f) && (key == 0))
    {
        key = (int32)0x80000000UL;
    }
    return key;
}

static void menu_update_recognize_object(const AppRecognizeDebug_t *recog)
{
    const char *kind_text;
    uint8 recog_valid;
    uint8 display_idx;
    uint8 display_total;
    uint8 display_kind;
    uint8 display_class;

    recog_valid = (uint8)((recog->total_targets != 0U) &&
                          (recog->current_idx < recog->total_targets));
    display_idx = (0U != recog_valid) ? (uint8)(recog->current_idx + 1U) : 0U;
    display_total = (0U != recog_valid) ? recog->total_targets : 0U;
    display_kind = (0U != recog_valid) ? recog->current_kind : 0U;
    display_class = (0U != recog_valid) ? recog->current_class_id : 0U;

    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);

    if ((0U == s_recog_cached) ||
        (recog_valid != s_last_recog_valid) ||
        (display_kind != s_last_recog_kind))
    {
        if (display_kind == APP_LINK_OBJ_KIND_BOX)
        {
            kind_text = "BOX   ";
        }
        else if (display_kind == APP_LINK_OBJ_KIND_TARGET)
        {
            kind_text = "TARGET";
        }
        else
        {
            kind_text = "--    ";
        }
        ips200_show_string(MENU_SIDE_X + 40U, MENU_RECOG_Y, kind_text);
    }

    if ((0U == s_recog_cached) ||
        (recog_valid != s_last_recog_valid) ||
        (display_idx != s_last_recog_idx))
    {
        if (0U != recog_valid)
        {
            ips200_show_uint(MENU_SIDE_X + 16U, MENU_RECOG_Y + 16U,
                             (uint32)display_idx, 2U);
        }
        else
        {
            ips200_show_string(MENU_SIDE_X + 16U, MENU_RECOG_Y + 16U, "--");
        }
    }

    if ((0U == s_recog_cached) ||
        (recog_valid != s_last_recog_valid) ||
        (display_total != s_last_recog_total))
    {
        if (0U != recog_valid)
        {
            ips200_show_uint(MENU_SIDE_X + 42U, MENU_RECOG_Y + 16U,
                             (uint32)display_total, 2U);
        }
        else
        {
            ips200_show_string(MENU_SIDE_X + 42U, MENU_RECOG_Y + 16U, "--");
        }
    }

    if ((0U == s_recog_cached) ||
        (recog_valid != s_last_recog_valid) ||
        (display_class != s_last_recog_class))
    {
        if ((0U != recog_valid) && (display_class != 0U))
        {
            ips200_show_uint(MENU_SIDE_X + 80U, MENU_RECOG_Y + 16U,
                             (uint32)display_class, 2U);
        }
        else
        {
            ips200_show_string(MENU_SIDE_X + 80U, MENU_RECOG_Y + 16U, "--");
        }
    }

    s_last_recog_valid = recog_valid;
    s_last_recog_idx = display_idx;
    s_last_recog_total = display_total;
    s_last_recog_kind = display_kind;
    s_last_recog_class = display_class;
    s_recog_cached = 1U;
}

static void menu_build_class_line(char line[MENU_TEXT_COLS + 1U],
                                  const char *prefix,
                                  const uint8 class_ids[SOKOBAN_MAX_BOXES],
                                  uint8 count)
{
    uint8 cursor = 0U;
    uint8 i;

    while ((*prefix != '\0') && (cursor < (uint8)MENU_TEXT_COLS))
    {
        line[cursor++] = *prefix++;
    }

    if (count > (uint8)SOKOBAN_MAX_BOXES) { count = (uint8)SOKOBAN_MAX_BOXES; }
    for (i = 0U; i < count; ++i)
    {
        uint8 class_id = class_ids[i];

        if ((uint8)(cursor + 3U) > (uint8)MENU_TEXT_COLS) { break; }
        line[cursor++] = ' ';
        if (class_id == 0U)
        {
            line[cursor++] = '-';
            line[cursor++] = '-';
        }
        else if (class_id <= 99U)
        {
            line[cursor++] = (char)('0' + (class_id / 10U));
            line[cursor++] = (char)('0' + (class_id % 10U));
        }
        else
        {
            line[cursor++] = '?';
            line[cursor++] = '?';
        }
    }

    while (cursor < (uint8)MENU_TEXT_COLS) { line[cursor++] = ' '; }
    line[MENU_TEXT_COLS] = '\0';
}

static void menu_update_recognize_lists(const AppRecognizeDebug_t *recog)
{
    char box_line[MENU_TEXT_COLS + 1U];
    char target_line[MENU_TEXT_COLS + 1U];

    menu_build_class_line(box_line, "BOX:", recog->box_class_ids, recog->box_count);
    menu_build_class_line(target_line, "TARGET:",
                          recog->target_class_ids, recog->target_count);

    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    if ((s_recog_lists_cached == 0U) ||
        (memcmp(box_line, s_last_box_line, sizeof(box_line)) != 0))
    {
        ips200_show_string(0U, MENU_RECOG_BOX_Y, box_line);
        memcpy(s_last_box_line, box_line, sizeof(box_line));
    }
    if ((s_recog_lists_cached == 0U) ||
        (memcmp(target_line, s_last_target_line, sizeof(target_line)) != 0))
    {
        ips200_show_string(0U, MENU_RECOG_TARGET_Y, target_line);
        memcpy(s_last_target_line, target_line, sizeof(target_line));
    }
    s_recog_lists_cached = 1U;
}

static void menu_update_recognize_display(uint8 show_class_lists)
{
    AppRecognizeDebug_t recog;

    Game_Get_Recognize_Debug(&recog);
    menu_update_recognize_object(&recog);
    if (show_class_lists != 0U)
    {
        menu_update_recognize_lists(&recog);
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

static void menu_draw_static_layout(void)
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

    ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y, "OBJ:");
    ips200_show_string(MENU_SIDE_X, MENU_RECOG_Y + 16U, "I:  /   C:");

    menu_fill_rect(MENU_MAP_ORIGIN_X, MENU_MAP_ORIGIN_Y, MENU_MAP_W, MENU_MAP_H, MENU_COLOR_BG);
    menu_draw_grid_lines();

    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    ips200_show_string(0U, 188U, "YAW:");
    ips200_show_string(112U, 188U, "deg");
    ips200_show_string(0U, 206U, "POSE X:");
    ips200_show_string(120U, 206U, "Y:");
    ips200_show_string(200U, 206U, "IMU:");
    ips200_show_string(0U, 224U, "M:WAIT S:WAIT W:---/--- ST:00 WAIT_START");
}

static void menu_prepare_layout_once(void)
{
    if (s_need_full_refresh == 0U) {
        return;
    }

    menu_draw_static_layout();
    s_need_full_refresh = 0U;
    s_last_map_ready = 0xFFU;
    s_map_cached = 0U;
    s_dynamic_cached = 0U;
    s_recog_cached = 0U;
    s_recog_lists_cached = 0U;
    s_game_status_cached = 0U;
}

static void menu_draw_map_cell(uint8 row, uint8 col, uint8 cell)
{
    uint16 x = (uint16)(MENU_MAP_ORIGIN_X + (uint16)col * MENU_CELL_SIZE + MENU_CELL_GAP);
    uint16 y = (uint16)(MENU_MAP_ORIGIN_Y + (uint16)row * MENU_CELL_SIZE + MENU_CELL_GAP);

    menu_fill_rect(x, y,
                   (uint16)(MENU_CELL_SIZE - MENU_CELL_GAP),
                   (uint16)(MENU_CELL_SIZE - MENU_CELL_GAP),
                   menu_cell_color(cell));
}

static void menu_update_map_cells(const uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS])
{
    uint8 r;
    uint8 c;

    for (r = 0U; r < APP_LINK_MAP_ROWS; ++r)
    {
        for (c = 0U; c < APP_LINK_MAP_COLS; ++c)
        {
            if ((0U == s_map_cached) || (map[r][c] != s_cached_map[r][c]))
            {
                menu_draw_map_cell(r, c, map[r][c]);
                s_cached_map[r][c] = map[r][c];
            }
        }
    }
    s_map_cached = 1U;
}

static void menu_update_map_status(uint8 map_ready, uint32 map_age_ms)
{
    if ((0xFFU == s_last_map_ready) || (map_ready != s_last_map_ready))
    {
        ips200_set_color(map_ready ? MENU_COLOR_OK : MENU_COLOR_WAIT, MENU_COLOR_BG);
        if (0U != map_ready)
        {
            ips200_show_string(0U, 16U, "MAP: RX    AGE:      ms ");
        }
        else
        {
            ips200_show_string(0U, 16U, "MAP: LOADING GRID ONLY  ");
        }
    }

    if ((0U != map_ready) &&
        ((0U == s_dynamic_cached) || (map_age_ms != s_last_map_age_ms) ||
         (map_ready != s_last_map_ready)))
    {
        ips200_set_color(MENU_COLOR_OK, MENU_COLOR_BG);
        ips200_show_uint(120U, 16U, map_age_ms, 5U);
    }

    s_last_map_age_ms = map_age_ms;
    s_last_map_ready = map_ready;
}

static const char *menu_stage_text(GameStage_e stage)
{
    switch (stage)
    {
        case STAGE_WAIT_START:          return "00 WAIT_START";
        case STAGE_LAUNCH_EXIT:         return "01 LAUNCH    ";
        case STAGE_WAIT_MAP_REFRESH:    return "02 WAIT_MAP  ";
        case STAGE_RECOGNIZE_MAP:       return "03 RECOGNIZE ";
        case STAGE_PLAN_PATH:           return "04 PLAN      ";
        case STAGE_EXECUTE_ACTION:      return "05 EXECUTE   ";
        case STAGE_LEVEL_JUDGE:         return "06 LVL_JUDGE ";
        case STAGE_DEADLOCK_RESET:      return "07 DEADLOCK  ";
        case STAGE_DONE:                return "08 DONE      ";
        case STAGE_PAUSE_ON_LINK_LOSS:  return "09 LINK_LOSS ";
        case STAGE_WAIT_RECOVERY_MAP:   return "10 RECOVERY  ";
        default:                        return "?? UNKNOWN   ";
    }
}

static void menu_update_game_status(void)
{
    GameRuntimeStatus_t status;
    const char *solve_text;
    uint16 waypoint_index;
    uint16 waypoint_count;

    Game_Get_Runtime_Status(&status);

    if ((0U == s_game_status_cached) ||
        (status.map_accepted != s_last_game_status.map_accepted))
    {
        ips200_set_color(status.map_accepted ? MENU_COLOR_OK : MENU_COLOR_WAIT,
                         MENU_COLOR_BG);
        ips200_show_string(16U, 224U, status.map_accepted ? "OK  " : "WAIT");
    }

    if (status.stage == STAGE_DEADLOCK_RESET)
    {
        solve_text = "FAIL";
    }
    else if (status.solve_succeeded != 0U)
    {
        solve_text = "OK  ";
    }
    else if (status.stage == STAGE_PLAN_PATH)
    {
        solve_text = "RUN ";
    }
    else
    {
        solve_text = "WAIT";
    }

    if ((0U == s_game_status_cached) ||
        (status.stage != s_last_game_status.stage) ||
        (status.solve_succeeded != s_last_game_status.solve_succeeded))
    {
        ips200_set_color((status.solve_succeeded != 0U) ? MENU_COLOR_OK : MENU_COLOR_WAIT,
                         MENU_COLOR_BG);
        ips200_show_string(72U, 224U, solve_text);
    }

    if ((0U == s_game_status_cached) ||
        (status.waypoint_issued != s_last_game_status.waypoint_issued) ||
        (status.waypoint_index != s_last_game_status.waypoint_index) ||
        (status.waypoint_count != s_last_game_status.waypoint_count))
    {
        ips200_set_color((status.waypoint_issued != 0U) ? MENU_COLOR_OK : MENU_COLOR_WAIT,
                         MENU_COLOR_BG);
        if ((status.waypoint_issued != 0U) && (status.waypoint_count != 0U))
        {
            waypoint_index = (uint16)(status.waypoint_index + 1U);
            waypoint_count = status.waypoint_count;
            if (waypoint_index > 999U) waypoint_index = 999U;
            if (waypoint_count > 999U) waypoint_count = 999U;
            ips200_show_uint(128U, 224U, (uint32)waypoint_index, 3U);
            ips200_show_uint(160U, 224U, (uint32)waypoint_count, 3U);
        }
        else
        {
            ips200_show_string(128U, 224U, "---");
            ips200_show_string(160U, 224U, "---");
        }
    }

    if ((0U == s_game_status_cached) ||
        (status.stage != s_last_game_status.stage))
    {
        ips200_set_color((status.stage == STAGE_DEADLOCK_RESET) ? MENU_COLOR_WAIT : MENU_COLOR_TEXT,
                         MENU_COLOR_BG);
        ips200_show_string(216U, 224U, menu_stage_text(status.stage));
    }

    s_last_game_status = status;
    s_game_status_cached = 1U;
}

static void menu_update_dynamic_text(uint8 map_ready, uint32 map_age_ms)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();
    float imu_yaw_deg = chassis_imu_get_yaw_deg();
    int32 pose_yaw_key = menu_float_display_key(pose.yaw_deg, 1U);
    int32 pose_x_key = menu_float_display_key(pose.x_m, 2U);
    int32 pose_y_key = menu_float_display_key(pose.y_m, 2U);
    int32 imu_yaw_key = menu_float_display_key(imu_yaw_deg, 1U);

    menu_update_map_status(map_ready, map_age_ms);
    ips200_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    if ((0U == s_dynamic_cached) || (pose_yaw_key != s_last_pose_yaw_key))
    {
        ips200_show_float(40U, 188U, pose.yaw_deg, 5U, 1U);
    }
    if ((0U == s_dynamic_cached) || (pose_x_key != s_last_pose_x_key))
    {
        ips200_show_float(64U, 206U, pose.x_m, 3U, 2U);
    }
    if ((0U == s_dynamic_cached) || (pose_y_key != s_last_pose_y_key))
    {
        ips200_show_float(144U, 206U, pose.y_m, 3U, 2U);
    }
    if ((0U == s_dynamic_cached) || (imu_yaw_key != s_last_imu_yaw_key))
    {
        ips200_show_float(240U, 206U, imu_yaw_deg, 5U, 1U);
    }

    s_last_pose_yaw_key = pose_yaw_key;
    s_last_pose_x_key = pose_x_key;
    s_last_pose_y_key = pose_y_key;
    s_last_imu_yaw_key = imu_yaw_key;
    s_dynamic_cached = 1U;

    menu_update_recognize_display(0U);
    menu_update_game_status();
}

void chassis_menu_init(void)
{
    s_flash_ready = (0U == flash_init()) ? 1U : 0U;
    (void)menu_load_params_from_flash();

    s_need_full_refresh = 1U;
    s_last_map_ready = 0xFFU;
    s_map_cached = 0U;
    s_dynamic_cached = 0U;
    s_recog_cached = 0U;
    s_recog_lists_cached = 0U;
    s_game_status_cached = 0U;
    s_last_frozen_map_generation = 0U;
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

    map_ready = menu_map_frame_is_fresh(&map_age_ms);

    /* 静态布局只在首次进入菜单时绘制一次。 */
    menu_prepare_layout_once();

    if (map_ready)
    {
        app_link_get_map_snapshot(map);
        menu_update_map_cells(map);
    }

    /* 地图过期时保留最后一帧；状态栏、数字和识别字段按显示值变化局部更新。 */
    menu_update_dynamic_text(map_ready, map_age_ms);
}

void chassis_menu_render_game_frozen_100ms(void)
{
    uint8 map[APP_LINK_MAP_ROWS][APP_LINK_MAP_COLS];
    uint32 generation = 0U;

    menu_prepare_layout_once();

    /* Race mode keeps the frozen map unchanged, but refreshes recognition
     * diagnostics and stage text when their values change. */
    menu_update_recognize_display(1U);
    menu_update_game_status();

    /* The grid is already black after drawing the layout. Treat it as an
     * empty cached map so the first freeze only transfers non-empty cells. */
    if ((s_map_cached == 0U) && (s_last_frozen_map_generation == 0U)) {
        memset(s_cached_map, MAP_EMPTY, sizeof(s_cached_map));
        s_map_cached = 1U;
    }

    if ((Game_Get_Frozen_Map(map, &generation) == 0U) ||
        (generation == s_last_frozen_map_generation)) {
        return;
    }

    menu_update_map_cells(map);
    ips200_set_color(MENU_COLOR_OK, MENU_COLOR_BG);
    ips200_show_string(0U, 16U, "MAP: FROZEN ID:          ");
    ips200_show_uint(128U, 16U, generation, 5U);
    s_last_frozen_map_generation = generation;
}
