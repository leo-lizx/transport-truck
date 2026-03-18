#include "chassis_menu.h"

#include "chassis_ctrl.h"
#include "chassis_imu.h"
#include "zf_common_headfile.h"
#include "zf_driver_flash.h"

/*===========================================================================
 * [chassis_menu.c] 底盘参数菜单（两级）
 *
 * 作用：
 * 1) 提供 LIMIT/NAV/PID 三组参数的在线调参与显示。
 * 2) 支持一级/二级菜单切换与参数实时下发。
 * 3) 支持参数掉电保存与上电加载（Flash 校验）。
 *
 * 调度约定：
 * - chassis_menu_task_10ms() 按 10ms 节拍调用。
 * - chassis_menu_render_100ms() 与 10ms 调度配合，内部按计数约 100ms 刷新。
 *===========================================================================*/

/*
 * 二级菜单说明：
 * 1) 一级菜单：LIMIT / NAV / PID 页面选择。
 * 2) 二级菜单：进入页面后编辑当前页面参数。
 * 3) 固定按键映射：K1(KEY_1)=C15, K2(KEY_2)=C14, K3(KEY_3)=C13, K4(KEY_4)=C12。
 * 4) 按键行为：
 *    - 一级：K1 上一页，K2 下一页，K3 进入二级。
 *    - 二级：K1 上一项，K2 下一项，K3 增大，K4 减小，K1 长按返回一级。
 */

typedef enum
{
    MENU_LEVEL_ROOT = 0,
    MENU_LEVEL_PARAM,
} menu_level_enum;

typedef enum
{
    MENU_PARAM_WHEEL_KP = 0,
    MENU_PARAM_WHEEL_KI,
    MENU_PARAM_WHEEL_KD,
    MENU_PARAM_POS_KP,
    MENU_PARAM_YAW_KP,
    MENU_PARAM_MAX_V,
    MENU_PARAM_MAX_W,
    MENU_PARAM_ACC_V,
    MENU_PARAM_ACC_W,
    MENU_PARAM_COUNT,
} menu_param_id_enum;

typedef struct
{
    const char *name;
    uint8 decimal;
    float step;
    float min_val;
    float max_val;
} menu_param_meta_t;

typedef struct
{
    const char *name;
    const uint8 *param_ids;
    uint8 param_count;
} menu_page_meta_t;

/* Flash 存储布局：
 * - 使用 127 号扇区的第 7 页存储调参参数（尽量避开程序常用区域）。
 * - 数据头包含 magic/version/checksum，保证掉电后数据可校验。
 */
#define CHASSIS_MENU_FLASH_SECTOR      (127U)
#define CHASSIS_MENU_FLASH_PAGE        (FLASH_PAGE_7)
#define CHASSIS_MENU_FLASH_MAGIC       (0x4D4E5455U)   /* "MNTU" */
#define CHASSIS_MENU_FLASH_VERSION     (1U)

typedef struct
{
    uint32 magic;
    uint32 version;
    chassis_tune_params_t params;
    uint32 checksum;
} chassis_menu_flash_blob_t;

static chassis_tune_params_t s_menu_params;
static menu_level_enum s_menu_level = MENU_LEVEL_ROOT;
static uint8 s_root_index = 0U;
static uint8 s_param_index = 0U;
static uint8 s_need_redraw = 1U;
static uint8 s_render_ticks = 0U;
static char s_status_text[24] = "STATUS: READY";
static char s_status_text_prev[24] = "";
static uint8 s_flash_ready = 0U;
static uint8 s_full_refresh_done = 0U;

static const menu_param_meta_t s_param_meta[MENU_PARAM_COUNT] =
{
    { "Wheel Kp",   1U, 1.00f,  0.00f,  400.0f },
    { "Wheel Ki",   2U, 0.20f,  0.00f,   80.0f },
    { "Wheel Kd",   2U, 0.10f,  0.00f,   40.0f },
    { "Pos Kp",     2U, 0.05f,  0.00f,    5.0f },
    { "Yaw Kp",     2U, 0.05f,  0.00f,   10.0f },
    { "Max V m/s",  2U, 0.01f,  0.05f,    1.5f },
    { "Max W d/s",  1U, 1.00f, 10.00f,  360.0f },
    { "Acc V m/s2", 2U, 0.05f,  0.10f,    5.0f },
    { "Acc W d/s2", 1U, 5.00f, 20.00f, 1000.0f },
};

static const uint8 s_page_pid_params[]   = { MENU_PARAM_WHEEL_KP, MENU_PARAM_WHEEL_KI, MENU_PARAM_WHEEL_KD };
static const uint8 s_page_nav_params[]   = { MENU_PARAM_POS_KP, MENU_PARAM_YAW_KP };
static const uint8 s_page_limit_params[] = { MENU_PARAM_MAX_V, MENU_PARAM_MAX_W, MENU_PARAM_ACC_V, MENU_PARAM_ACC_W };

static const menu_page_meta_t s_pages[] =
{
    { "LIMIT", s_page_limit_params, (uint8)(sizeof(s_page_limit_params) / sizeof(s_page_limit_params[0])) },
    { "NAV",   s_page_nav_params,   (uint8)(sizeof(s_page_nav_params)   / sizeof(s_page_nav_params[0])) },
    { "PID",   s_page_pid_params,   (uint8)(sizeof(s_page_pid_params)   / sizeof(s_page_pid_params[0])) },
};

#define MENU_PAGE_COUNT ((uint8)(sizeof(s_pages) / sizeof(s_pages[0])))

/* 根据参数ID读取当前参数值。 */
static float menu_get_param_value(menu_param_id_enum id)
{
    switch (id)
    {
        case MENU_PARAM_WHEEL_KP: return s_menu_params.wheel_pid_kp;
        case MENU_PARAM_WHEEL_KI: return s_menu_params.wheel_pid_ki;
        case MENU_PARAM_WHEEL_KD: return s_menu_params.wheel_pid_kd;
        case MENU_PARAM_POS_KP:   return s_menu_params.pos_kp;
        case MENU_PARAM_YAW_KP:   return s_menu_params.yaw_kp;
        case MENU_PARAM_MAX_V:    return s_menu_params.max_linear_speed_mps;
        case MENU_PARAM_MAX_W:    return s_menu_params.max_yaw_speed_dps;
        case MENU_PARAM_ACC_V:    return s_menu_params.cmd_accel_limit_mps2;
        case MENU_PARAM_ACC_W:    return s_menu_params.cmd_accel_limit_dps2;
        default:                  return s_menu_params.wheel_pid_kp;
    }
}

/* 根据参数ID写入当前参数值。 */
static void menu_set_param_value(menu_param_id_enum id, float value)
{
    switch (id)
    {
        case MENU_PARAM_WHEEL_KP: s_menu_params.wheel_pid_kp = value; break;
        case MENU_PARAM_WHEEL_KI: s_menu_params.wheel_pid_ki = value; break;
        case MENU_PARAM_WHEEL_KD: s_menu_params.wheel_pid_kd = value; break;
        case MENU_PARAM_POS_KP:   s_menu_params.pos_kp = value; break;
        case MENU_PARAM_YAW_KP:   s_menu_params.yaw_kp = value; break;
        case MENU_PARAM_MAX_V:    s_menu_params.max_linear_speed_mps = value; break;
        case MENU_PARAM_MAX_W:    s_menu_params.max_yaw_speed_dps = value; break;
        case MENU_PARAM_ACC_V:    s_menu_params.cmd_accel_limit_mps2 = value; break;
        case MENU_PARAM_ACC_W:    s_menu_params.cmd_accel_limit_dps2 = value; break;
        default: break;
    }
}

/* 简单加和校验：对除 checksum 之外的全部 32bit 字做累加。 */
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

/* 将状态文本写入固定缓冲区，用于屏幕反馈。 */
static void menu_set_status(const char *text)
{
    uint8 i = 0U;
    if (0 == text)
    {
        return;
    }

    for (i = 0U; i < (uint8)(sizeof(s_status_text) - 1U); ++i)
    {
        if ('\0' == text[i])
        {
            break;
        }
        s_status_text[i] = text[i];
    }
    s_status_text[i] = '\0';
}

/* 从 Flash 读取参数，校验通过后覆盖当前参数。 */
static uint8 menu_load_params_from_flash(void)
{
    uint32 raw_words[(sizeof(chassis_menu_flash_blob_t) + 3U) / 4U];
    chassis_menu_flash_blob_t blob;
    uint32 calc_checksum;
    uint16 payload_words;

    if (0U == s_flash_ready)
    {
        menu_set_status("STATUS: FLASH OFF");
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
        menu_set_status("STATUS: LOAD DEFAULT");
        return 0U;
    }

    chassis_ctrl_set_tune_params(&blob.params);
    chassis_ctrl_get_tune_params(&s_menu_params);
    menu_set_status("STATUS: LOAD OK");
    return 1U;
}

/* 将当前参数写入 Flash。 */
static uint8 menu_save_params_to_flash(void)
{
    chassis_menu_flash_blob_t blob;
    uint16 payload_words;
    uint8 ret;

    if (0U == s_flash_ready)
    {
        menu_set_status("STATUS: FLASH OFF");
        return 0U;
    }

    blob.magic = CHASSIS_MENU_FLASH_MAGIC;
    blob.version = CHASSIS_MENU_FLASH_VERSION;
    blob.params = s_menu_params;
    blob.checksum = 0U;

    payload_words = (uint16)((sizeof(chassis_menu_flash_blob_t) - sizeof(uint32)) / 4U);
    blob.checksum = menu_flash_checksum((const uint32 *)&blob, payload_words);

    ret = flash_write_page(CHASSIS_MENU_FLASH_SECTOR,
                           CHASSIS_MENU_FLASH_PAGE,
                           (const uint32 *)&blob,
                           (uint16)((sizeof(chassis_menu_flash_blob_t) + 3U) / 4U));
    if (0U == ret)
    {
        menu_set_status("STATUS: SAVE OK");
        return 1U;
    }

    menu_set_status("STATUS: SAVE FAIL");
    return 0U;
}

/* 获取当前二级菜单项对应的参数ID。 */
static menu_param_id_enum menu_get_current_param_id(void)
{
    const menu_page_meta_t *page = &s_pages[s_root_index];
    return (menu_param_id_enum)page->param_ids[s_param_index];
}

/* 对当前参数应用增量，限幅后实时下发到底盘控制模块。 */
static void menu_apply_current_param_delta(float delta)
{
    menu_param_id_enum id = menu_get_current_param_id();
    const menu_param_meta_t *meta = &s_param_meta[id];
    float value = menu_get_param_value(id);

    value = chassis_clamp_f(value + delta, meta->min_val, meta->max_val);
    menu_set_param_value(id, value);
    chassis_ctrl_set_tune_params(&s_menu_params);
    s_need_redraw = 1U;
}

/* 一级菜单按键处理：负责页面切换与进入二级。 */
static void menu_handle_root_keys(void)
{
    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        key_clear_state(KEY_1);
        s_root_index = (0U == s_root_index) ? (MENU_PAGE_COUNT - 1U) : (uint8)(s_root_index - 1U);
        s_need_redraw = 1U;
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        key_clear_state(KEY_2);
        s_root_index = (uint8)((s_root_index + 1U) % MENU_PAGE_COUNT);
        s_need_redraw = 1U;
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        key_clear_state(KEY_3);
        s_menu_level = MENU_LEVEL_PARAM;
        s_param_index = 0U;
        s_need_redraw = 1U;
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        key_clear_state(KEY_4);
    }

    if (KEY_LONG_PRESS == key_get_state(KEY_4))
    {
        key_clear_state(KEY_4);
        menu_save_params_to_flash();
        s_need_redraw = 1U;
    }
}

/* 二级菜单按键处理：负责参数项切换、参数增减和返回一级。 */
static void menu_handle_param_keys(void)
{
    const menu_page_meta_t *page = &s_pages[s_root_index];

    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        key_clear_state(KEY_1);
        s_param_index = (0U == s_param_index) ? (uint8)(page->param_count - 1U) : (uint8)(s_param_index - 1U);
        s_need_redraw = 1U;
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        key_clear_state(KEY_2);
        s_param_index = (uint8)((s_param_index + 1U) % page->param_count);
        s_need_redraw = 1U;
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        menu_param_id_enum id = menu_get_current_param_id();
        key_clear_state(KEY_3);
        menu_apply_current_param_delta(s_param_meta[id].step);
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        menu_param_id_enum id = menu_get_current_param_id();
        key_clear_state(KEY_4);
        menu_apply_current_param_delta(-s_param_meta[id].step);
    }

    if (KEY_LONG_PRESS == key_get_state(KEY_1))
    {
        key_clear_state(KEY_1);
        s_menu_level = MENU_LEVEL_ROOT;
        s_need_redraw = 1U;
    }

    if (KEY_LONG_PRESS == key_get_state(KEY_4))
    {
        key_clear_state(KEY_4);
        menu_save_params_to_flash();
        s_need_redraw = 1U;
    }
}

/* 清理长按状态，避免按键长按后卡在某个状态位。 */
static void menu_clear_long_press_flags(void)
{
    if (KEY_LONG_PRESS == key_get_state(KEY_1)) { key_clear_state(KEY_1); }
    if (KEY_LONG_PRESS == key_get_state(KEY_2)) { key_clear_state(KEY_2); }
}

/* 公共头部绘制。 */
static void menu_draw_header(void)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(0, 0, "Chassis Tune 2-Level");

    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    if (MENU_LEVEL_ROOT == s_menu_level)
    {
        ips200_show_string(0, 16, "L1:C15/C14 Sel C13 Enter C12L Save");
    }
    else
    {
        ips200_show_string(0, 16, "L2:C15/C14 Sel C13+/C12- C15LBack");
    }
}

/* 一级菜单绘制：仅显示页面名称。 */
static void menu_draw_root_list(void)
{
    uint8 i;
    uint16 y;

    for (i = 0U; i < MENU_PAGE_COUNT; ++i)
    {
        y = (uint16)(48U + (uint16)i * 18U);
        ips200_set_color((i == s_root_index) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(0, y, (i == s_root_index) ? ">" : " ");
        ips200_show_string(14, y, s_pages[i].name);
    }
}

/* 二级菜单绘制：显示当前页面的参数和值。 */
static void menu_draw_param_list(void)
{
    uint8 i;
    uint16 y;
    const menu_page_meta_t *page = &s_pages[s_root_index];

    ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);
    ips200_show_string(0, 36, "Page:");
    ips200_show_string(48, 36, page->name);

    for (i = 0U; i < page->param_count; ++i)
    {
        menu_param_id_enum id = (menu_param_id_enum)page->param_ids[i];
        const menu_param_meta_t *meta = &s_param_meta[id];
        y = (uint16)(56U + (uint16)i * 16U);

        ips200_set_color((i == s_param_index) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(0, y, (i == s_param_index) ? ">" : " ");
        ips200_show_string(12, y, meta->name);
        ips200_show_float(200, y, menu_get_param_value(id), 6, meta->decimal);
    }
}

/* 底部调试信息绘制：统一显示当前位姿。 */
static void menu_draw_pose(uint8 force_refresh)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();
    float imu_yaw_deg = chassis_imu_get_yaw_deg();
    char status_line[41];
    uint8 i;

    if (0U != force_refresh)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(0, 188, "Pose X / Y / IMU Yaw");
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_float(0,   206, pose.x_m, 4, 2U);
    ips200_show_float(100, 206, pose.y_m, 4, 2U);
    ips200_show_float(200, 206, imu_yaw_deg, 5, 1U);

    if ((0U != force_refresh) || (0 != strcmp(s_status_text_prev, s_status_text)))
    {
        for (i = 0U; i < 40U; ++i)
        {
            status_line[i] = ' ';
        }
        for (i = 0U; (i < 40U) && ('\0' != s_status_text[i]); ++i)
        {
            status_line[i] = s_status_text[i];
        }
        status_line[40] = '\0';

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(0, 224, status_line);
        strcpy(s_status_text_prev, s_status_text);
    }
}

//-------------------------------------------------------------------------
// 函数简介：菜单模块初始化
// 参数说明：无
// 返回参数：无
// 使用示例：系统启动后调用一次
//          chassis_menu_init();
// 备注信息：会尝试从 Flash 加载参数，校验失败则回退默认参数
//-------------------------------------------------------------------------

void chassis_menu_init(void)
{
    chassis_ctrl_get_tune_params(&s_menu_params);
    s_flash_ready = (0U == flash_init()) ? 1U : 0U;
    menu_load_params_from_flash();
    strcpy(s_status_text_prev, "");
    s_menu_level = MENU_LEVEL_ROOT;
    s_root_index = 0U;
    s_param_index = 0U;
    s_need_redraw = 1U;
    s_render_ticks = 0U;
    s_full_refresh_done = 0U;
}

//-------------------------------------------------------------------------
// 函数简介：菜单 10ms 周期任务
// 参数说明：无
// 返回参数：无
// 使用示例：主循环以 10ms 节拍调用
//          chassis_menu_task_10ms();
// 备注信息：负责按键扫描、菜单状态切换与参数修改处理
//-------------------------------------------------------------------------

void chassis_menu_task_10ms(void)
{
    key_scanner();

    if (MENU_LEVEL_ROOT == s_menu_level)
    {
        menu_handle_root_keys();
    }
    else
    {
        menu_handle_param_keys();
    }

    menu_clear_long_press_flags();
}

//-------------------------------------------------------------------------
// 函数简介：菜单渲染任务（约 100ms 刷新）
// 参数说明：无
// 返回参数：无
// 使用示例：与 10ms 调度配合调用
//          chassis_menu_render_100ms();
// 备注信息：无参数变化时会降频刷新，减少屏幕闪烁
//-------------------------------------------------------------------------

void chassis_menu_render_100ms(void)
{
    uint8 do_full_refresh = 0U;

    s_render_ticks++;
    if ((0U == s_need_redraw) && (s_render_ticks < 10U))
    {
        return;
    }

    s_render_ticks = 0U;
    if ((0U == s_full_refresh_done) || (0U != s_need_redraw))
    {
        do_full_refresh = 1U;
        s_need_redraw = 0U;
        s_full_refresh_done = 1U;

        ips200_full(RGB565_BLACK);
        menu_draw_header();

        if (MENU_LEVEL_ROOT == s_menu_level)
        {
            menu_draw_root_list();
        }
        else
        {
            menu_draw_param_list();
        }
    }

    /* 常态仅更新位姿区，避免整屏反复刷导致闪烁。 */
    menu_draw_pose(do_full_refresh);
}
