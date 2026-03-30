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
    MENU_LEVEL_ROOT = 0, /* 一级菜单：页面选择层 */
    MENU_LEVEL_PARAM,    /* 二级菜单：参数编辑层 */
} menu_level_enum;

typedef enum
{
    MENU_PARAM_LF_KP = 0, /* 左前轮 PID 比例项 */
    MENU_PARAM_LF_KI,     /* 左前轮 PID 积分项 */
    MENU_PARAM_LF_KD,     /* 左前轮 PID 微分项 */
    MENU_PARAM_RF_KP,     /* 右前轮 PID 比例项 */
    MENU_PARAM_RF_KI,     /* 右前轮 PID 积分项 */
    MENU_PARAM_RF_KD,     /* 右前轮 PID 微分项 */
    MENU_PARAM_LB_KP,     /* 左后轮 PID 比例项 */
    MENU_PARAM_LB_KI,     /* 左后轮 PID 积分项 */
    MENU_PARAM_LB_KD,     /* 左后轮 PID 微分项 */
    MENU_PARAM_RB_KP,     /* 右后轮 PID 比例项 */
    MENU_PARAM_RB_KI,     /* 右后轮 PID 积分项 */
    MENU_PARAM_RB_KD,     /* 右后轮 PID 微分项 */
    MENU_PARAM_POS_KP,    /* 位置环比例项 */
    MENU_PARAM_YAW_KP,    /* 航向环比例项 */
    MENU_PARAM_MAX_V,     /* 线速度上限 */
    MENU_PARAM_MAX_W,     /* 角速度上限 */
    MENU_PARAM_ACC_V,     /* 线加速度上限 */
    MENU_PARAM_ACC_W,     /* 角加速度上限 */
    MENU_PARAM_COUNT,     /* 参数总数 */
} menu_param_id_enum;

typedef struct
{
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

/* Flash 存储布局：
 * - 使用 127 号扇区的第 7 页存储调参参数（尽量避开程序常用区域）。
 * - 数据头包含 magic/version/checksum，保证掉电后数据可校验。
 */
#define CHASSIS_MENU_FLASH_SECTOR      (127U)       /* 参数存储扇区编号 */
#define CHASSIS_MENU_FLASH_PAGE        (FLASH_PAGE_7) /* 参数存储页编号 */
#define CHASSIS_MENU_FLASH_MAGIC       (0x4D4E5455U)   /* "MNTU" */
#define CHASSIS_MENU_FLASH_VERSION     (2U)         /* 当前参数布局版本 */

typedef struct
{
    uint32 magic;                 /* 数据块魔数，用于识别有效配置 */
    uint32 version;               /* 数据结构版本号 */
    chassis_tune_params_t params; /* 参数有效载荷 */
    uint32 checksum;              /* 数据完整性校验和 */
} chassis_menu_flash_blob_t;

static menu_level_enum s_menu_level = MENU_LEVEL_ROOT;  /* 当前菜单层级 */
static uint8 s_root_index = 0U;                         /* 一级菜单当前页索引 */
static uint8 s_param_index = 0U;                        /* 二级菜单当前参数索引 */
static uint8 s_need_redraw = 1U;                        /* 界面重绘请求标志 */
static uint8 s_render_ticks = 0U;                       /* 渲染节拍计数（10ms 基准） */
static char s_status_text[24] = "STATUS: READY";       /* 当前状态文本 */
static char s_status_text_prev[24] = "";               /* 上次状态文本（用于变化检测） */
static uint8 s_flash_ready = 0U;                        /* Flash 可用标志 */
static uint8 s_full_refresh_done = 0U;                  /* 是否至少完成过一次整屏绘制 */

static const menu_param_meta_t s_param_meta[MENU_PARAM_COUNT] =
{
    { "LF Kp",      1U, 1.00f,  0.00f,  400.0f },
    { "LF Ki",      2U, 0.20f,  0.00f,   80.0f },
    { "LF Kd",      2U, 0.10f,  0.00f,   40.0f },
    { "RF Kp",      1U, 1.00f,  0.00f,  400.0f },
    { "RF Ki",      2U, 0.20f,  0.00f,   80.0f },
    { "RF Kd",      2U, 0.10f,  0.00f,   40.0f },
    { "LB Kp",      1U, 1.00f,  0.00f,  400.0f },
    { "LB Ki",      2U, 0.20f,  0.00f,   80.0f },
    { "LB Kd",      2U, 0.10f,  0.00f,   40.0f },
    { "RB Kp",      1U, 1.00f,  0.00f,  400.0f },
    { "RB Ki",      2U, 0.20f,  0.00f,   80.0f },
    { "RB Kd",      2U, 0.10f,  0.00f,   40.0f },
    { "Pos Kp",     2U, 0.05f,  0.00f,    5.0f },
    { "Yaw Kp",     2U, 0.05f,  0.00f,   10.0f },
    { "Max V m/s",  2U, 0.01f,  0.05f,    1.5f },
    { "Max W d/s",  1U, 1.00f, 10.00f,  360.0f },
    { "Acc V m/s2", 2U, 0.05f,  0.10f,    5.0f },
    { "Acc W d/s2", 1U, 5.00f, 20.00f, 1000.0f },
};

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
static float menu_get_param_value(menu_param_id_enum id)
{
    /* 按参数 ID 路由到当前参数结构中的实际字段。 */
    switch (id)
    {
        case MENU_PARAM_LF_KP:  return g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_LF]; /* 左前轮 Kp */
        case MENU_PARAM_LF_KI:  return g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_LF]; /* 左前轮 Ki */
        case MENU_PARAM_LF_KD:  return g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_LF]; /* 左前轮 Kd */
        case MENU_PARAM_RF_KP:  return g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_RF]; /* 右前轮 Kp */
        case MENU_PARAM_RF_KI:  return g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_RF]; /* 右前轮 Ki */
        case MENU_PARAM_RF_KD:  return g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_RF]; /* 右前轮 Kd */
        case MENU_PARAM_LB_KP:  return g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_LB]; /* 左后轮 Kp */
        case MENU_PARAM_LB_KI:  return g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_LB]; /* 左后轮 Ki */
        case MENU_PARAM_LB_KD:  return g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_LB]; /* 左后轮 Kd */
        case MENU_PARAM_RB_KP:  return g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_RB]; /* 右后轮 Kp */
        case MENU_PARAM_RB_KI:  return g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_RB]; /* 右后轮 Ki */
        case MENU_PARAM_RB_KD:  return g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_RB]; /* 右后轮 Kd */
        case MENU_PARAM_POS_KP: return g_chassis_tune_params.pos_kp;                             /* 位置环比例项 */
        case MENU_PARAM_YAW_KP: return g_chassis_tune_params.yaw_kp;                             /* 航向环比例项 */
        case MENU_PARAM_MAX_V:  return g_chassis_tune_params.max_linear_speed_mps;               /* 最大线速度上限 */
        case MENU_PARAM_MAX_W:  return g_chassis_tune_params.max_yaw_speed_dps;                  /* 最大角速度上限 */
        case MENU_PARAM_ACC_V:  return g_chassis_tune_params.cmd_accel_limit_mps2;               /* 线速度加速度限制 */
        case MENU_PARAM_ACC_W:  return g_chassis_tune_params.cmd_accel_limit_dps2;               /* 角速度加速度限制 */
        default:               return g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_LF];      /* 异常 ID 回退值 */
    }
}

/* 根据参数ID写入当前参数值。 */
static void menu_set_param_value(menu_param_id_enum id, float value)
{
    /* 按参数 ID 写回对应字段，所有修改都落在全局参数单源中。 */
    switch (id)
    {
        case MENU_PARAM_LF_KP: g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_LF] = value; break; /* 设置左前轮 Kp */
        case MENU_PARAM_LF_KI: g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_LF] = value; break; /* 设置左前轮 Ki */
        case MENU_PARAM_LF_KD: g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_LF] = value; break; /* 设置左前轮 Kd */
        case MENU_PARAM_RF_KP: g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_RF] = value; break; /* 设置右前轮 Kp */
        case MENU_PARAM_RF_KI: g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_RF] = value; break; /* 设置右前轮 Ki */
        case MENU_PARAM_RF_KD: g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_RF] = value; break; /* 设置右前轮 Kd */
        case MENU_PARAM_LB_KP: g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_LB] = value; break; /* 设置左后轮 Kp */
        case MENU_PARAM_LB_KI: g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_LB] = value; break; /* 设置左后轮 Ki */
        case MENU_PARAM_LB_KD: g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_LB] = value; break; /* 设置左后轮 Kd */
        case MENU_PARAM_RB_KP: g_chassis_tune_params.wheel_pid_kp[CHASSIS_WHEEL_RB] = value; break; /* 设置右后轮 Kp */
        case MENU_PARAM_RB_KI: g_chassis_tune_params.wheel_pid_ki[CHASSIS_WHEEL_RB] = value; break; /* 设置右后轮 Ki */
        case MENU_PARAM_RB_KD: g_chassis_tune_params.wheel_pid_kd[CHASSIS_WHEEL_RB] = value; break; /* 设置右后轮 Kd */
        case MENU_PARAM_POS_KP: g_chassis_tune_params.pos_kp = value; break;                           /* 设置位置环 Kp */
        case MENU_PARAM_YAW_KP: g_chassis_tune_params.yaw_kp = value; break;                           /* 设置航向环 Kp */
        case MENU_PARAM_MAX_V:  g_chassis_tune_params.max_linear_speed_mps = value; break;             /* 设置线速度上限 */
        case MENU_PARAM_MAX_W:  g_chassis_tune_params.max_yaw_speed_dps = value; break;                /* 设置角速度上限 */
        case MENU_PARAM_ACC_V:  g_chassis_tune_params.cmd_accel_limit_mps2 = value; break;             /* 设置线加速度上限 */
        case MENU_PARAM_ACC_W:  g_chassis_tune_params.cmd_accel_limit_dps2 = value; break;             /* 设置角加速度上限 */
        default: break;                                                                          /* 异常 ID 直接忽略 */
    }
}

/* 简单加和校验：对除 checksum 之外的全部 32bit 字做累加。 */
static uint32 menu_flash_checksum(const uint32 *words, uint16 word_count)
{
    uint16 i;        /* 循环下标：遍历每个 32bit 数据字 */
    uint32 sum = 0U; /* 校验累加值 */

    for (i = 0U; i < word_count; ++i)
    {
        sum += words[i];
    }
    return sum;
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
    uint32 raw_words[(sizeof(chassis_menu_flash_blob_t) + 3U) / 4U]; /* Flash 原始字缓冲 */
    chassis_menu_flash_blob_t blob;                                  /* 反序列化后的结构体 */
    uint32 calc_checksum;                                            /* 重新计算得到的校验和 */
    uint16 payload_words;                                            /* 除 checksum 之外的数据字数量 */

    if (0U == s_flash_ready)
    {
        menu_set_status("STATUS: FLASH OFF"); /* 提示 Flash 功能不可用 */
        return 0U;                             /* 返回失败 */
    }

    /* 从指定扇区页读取参数镜像。 */
    flash_read_page(CHASSIS_MENU_FLASH_SECTOR, CHASSIS_MENU_FLASH_PAGE,
                    raw_words, (uint16)(sizeof(raw_words) / sizeof(raw_words[0])));
    /* 将原始字流还原为参数结构体。 */
    memcpy(&blob, raw_words, sizeof(blob));

    /* 只对 header+params 区域做校验，不包含 checksum 字段本身。 */
    payload_words = (uint16)((sizeof(chassis_menu_flash_blob_t) - sizeof(uint32)) / 4U);
    calc_checksum = menu_flash_checksum((const uint32 *)&blob, payload_words);

    if ((blob.magic != CHASSIS_MENU_FLASH_MAGIC) ||
        (blob.version != CHASSIS_MENU_FLASH_VERSION) ||
        (blob.checksum != calc_checksum))
    {
        menu_set_status("STATUS: LOAD DEFAULT"); /* 校验不通过：保持默认参数 */
        return 0U;                                /* 返回失败 */
    }

    chassis_ctrl_set_tune_params(&blob.params);  /* 将 Flash 参数下发到控制层并触发限幅 */
    menu_set_status("STATUS: LOAD OK");         /* 更新状态提示 */
    return 1U;                                    /* 返回成功 */
}

/* 将当前参数写入 Flash。 */
static uint8 menu_save_params_to_flash(void)
{
    chassis_menu_flash_blob_t blob; /* 待写入 Flash 的完整镜像 */
    uint16 payload_words;           /* 参与校验的 32bit 字数量 */
    uint8 ret;                      /* Flash 写入返回码 */

    if (0U == s_flash_ready)
    {
        menu_set_status("STATUS: FLASH OFF"); /* 提示不可写 Flash */
        return 0U;                             /* 返回失败 */
    }

    /* 组织写入镜像，先写头和参数，再回填校验和。 */
    blob.magic = CHASSIS_MENU_FLASH_MAGIC;     /* 写入魔数，识别有效数据块 */
    blob.version = CHASSIS_MENU_FLASH_VERSION; /* 写入版本号，便于未来扩展 */
    blob.params = g_chassis_tune_params;       /* 写入当前参数快照 */
    blob.checksum = 0U;                        /* 校验前先清零 */

    payload_words = (uint16)((sizeof(chassis_menu_flash_blob_t) - sizeof(uint32)) / 4U); /* 参与校验的数据字数 */
    blob.checksum = menu_flash_checksum((const uint32 *)&blob, payload_words);            /* 计算并回填 checksum */

    /* 按页写入，返回 0 表示成功。 */
    ret = flash_write_page(CHASSIS_MENU_FLASH_SECTOR,
                           CHASSIS_MENU_FLASH_PAGE,
                           (const uint32 *)&blob,
                           (uint16)((sizeof(chassis_menu_flash_blob_t) + 3U) / 4U));
    if (0U == ret)
    {
        menu_set_status("STATUS: SAVE OK"); /* 写入成功提示 */
        return 1U;                           /* 返回成功 */
    }

    menu_set_status("STATUS: SAVE FAIL"); /* 写入失败提示 */
    return 0U;                             /* 返回失败 */
}

/* 获取当前二级菜单项对应的参数ID。 */
static menu_param_id_enum menu_get_current_param_id(void)
{
    const menu_page_meta_t *page = &s_pages[s_root_index]; /* 当前一级页面元信息 */
    return (menu_param_id_enum)page->param_ids[s_param_index]; /* 用当前二级索引取得参数 ID */
}

/* 对当前参数应用增量，限幅后实时下发到底盘控制模块。 */
static void menu_apply_current_param_delta(float delta)
{
    menu_param_id_enum id = menu_get_current_param_id(); /* 当前选中的参数 ID */
    const menu_param_meta_t *meta = &s_param_meta[id];   /* 参数元数据（步进/限幅） */
    float value = menu_get_param_value(id);              /* 参数当前值 */
    chassis_tune_params_t params;                        /* 下发给控制层的参数副本 */

    /* 先叠加增量，再做上下限保护。 */
    value = chassis_clamp_f(value + delta, meta->min_val, meta->max_val); /* 参数值加减并限幅 */
    /* 回写参数并实时下发到底盘控制层。 */
    menu_set_param_value(id, value);              /* 写回全局参数单源 */
    params = g_chassis_tune_params;               /* 复制一份非 volatile 副本用于接口入参 */
    chassis_ctrl_set_tune_params(&params);        /* 立即下发到底盘控制模块 */
    s_need_redraw = 1U;                           /* 标记界面需要重绘 */
}

/* 一级菜单按键处理：负责页面切换与进入二级。 */
static void menu_handle_root_keys(void)
{
    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        /* K1 短按：一级菜单上一页。 */
        key_clear_state(KEY_1); /* 消费该次按键事件 */
        s_root_index = (0U == s_root_index) ? (MENU_PAGE_COUNT - 1U) : (uint8)(s_root_index - 1U); /* 环形切换到上一页 */
        s_need_redraw = 1U; /* 通知渲染层更新显示 */
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        /* K2 短按：一级菜单下一页。 */
        key_clear_state(KEY_2); /* 消费该次按键事件 */
        s_root_index = (uint8)((s_root_index + 1U) % MENU_PAGE_COUNT); /* 环形切换到下一页 */
        s_need_redraw = 1U; /* 通知渲染层更新显示 */
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        /* K3 短按：进入二级参数编辑。 */
        key_clear_state(KEY_3);         /* 消费该次按键事件 */
        s_menu_level = MENU_LEVEL_PARAM; /* 进入二级参数编辑层 */
        s_param_index = 0U;             /* 二级默认选中第一个参数 */
        s_need_redraw = 1U;             /* 通知渲染层更新显示 */
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        /* K4 短按：一级页面不使用，清状态防止残留。 */
        key_clear_state(KEY_4); /* 清理无效短按事件，防止状态堆积 */
    }

    if (KEY_LONG_PRESS == key_get_state(KEY_4))
    {
        /* K4 长按：保存当前参数到 Flash。 */
        key_clear_state(KEY_4);       /* 消费该次长按事件 */
        menu_save_params_to_flash();  /* 执行参数保存 */
        s_need_redraw = 1U;           /* 刷新状态行显示保存结果 */
    }
}

/* 二级菜单按键处理：负责参数项切换、参数增减和返回一级。 */
static void menu_handle_param_keys(void)
{
    const menu_page_meta_t *page = &s_pages[s_root_index]; /* 当前页面参数集合 */

    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        /* K1 短按：切换到上一个参数项。 */
        key_clear_state(KEY_1); /* 消费该次按键事件 */
        s_param_index = (0U == s_param_index) ? (uint8)(page->param_count - 1U) : (uint8)(s_param_index - 1U); /* 环形切换上一项 */
        s_need_redraw = 1U; /* 通知渲染层更新光标位置 */
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        /* K2 短按：切换到下一个参数项。 */
        key_clear_state(KEY_2); /* 消费该次按键事件 */
        s_param_index = (uint8)((s_param_index + 1U) % page->param_count); /* 环形切换下一项 */
        s_need_redraw = 1U; /* 通知渲染层更新光标位置 */
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        menu_param_id_enum id = menu_get_current_param_id(); /* 当前参数 ID */
        /* K3 短按：按 step 增加参数值。 */
        key_clear_state(KEY_3);                           /* 消费该次按键事件 */
        menu_apply_current_param_delta(s_param_meta[id].step); /* 参数增加一个步进 */
    }

    if (KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        menu_param_id_enum id = menu_get_current_param_id(); /* 当前参数 ID */
        /* K4 短按：按 step 减小参数值。 */
        key_clear_state(KEY_4);                            /* 消费该次按键事件 */
        menu_apply_current_param_delta(-s_param_meta[id].step); /* 参数减少一个步进 */
    }

    if (KEY_LONG_PRESS == key_get_state(KEY_1))
    {
        /* K1 长按：退出二级，返回一级菜单。 */
        key_clear_state(KEY_1);        /* 消费该次长按事件 */
        s_menu_level = MENU_LEVEL_ROOT; /* 切回一级菜单层 */
        s_need_redraw = 1U;            /* 通知渲染层全刷 */
    }

    if (KEY_LONG_PRESS == key_get_state(KEY_4))
    {
        /* K4 长按：保存当前参数到 Flash。 */
        key_clear_state(KEY_4);      /* 消费该次长按事件 */
        menu_save_params_to_flash(); /* 执行参数保存 */
        s_need_redraw = 1U;          /* 刷新状态行显示保存结果 */
    }
}

/* 清理长按状态，避免按键长按后卡在某个状态位。 */
static void menu_clear_long_press_flags(void)
{
    if (KEY_LONG_PRESS == key_get_state(KEY_1)) { key_clear_state(KEY_1); } /* 清理 KEY_1 长按残留状态 */
    if (KEY_LONG_PRESS == key_get_state(KEY_2)) { key_clear_state(KEY_2); } /* 清理 KEY_2 长按残留状态 */
}

/* 公共头部绘制。 */
static void menu_draw_header(void)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);       /* 设置标题颜色：绿字黑底 */
    ips200_show_string(0, 0, "Chassis Tune 2-Level");  /* 绘制标题文本 */

    ips200_set_color(RGB565_CYAN, RGB565_BLACK); /* 设置提示条颜色：青字黑底 */
    if (MENU_LEVEL_ROOT == s_menu_level)
    {
        ips200_show_string(0, 16, "L1:C15/C14 Sel C13 Enter C12L Save"); /* 一级菜单提示 */
    }
    else
    {
        ips200_show_string(0, 16, "L2:C15/C14 Sel C13+/C12- C15LBack"); /* 二级菜单提示 */
    }
}

/* 一级菜单绘制：仅显示页面名称。 */
static void menu_draw_root_list(void)
{
    uint8 i;   /* 页面遍历下标 */
    uint16 y;  /* 每行显示的 y 坐标 */

    for (i = 0U; i < MENU_PAGE_COUNT; ++i)
    {
        y = (uint16)(48U + (uint16)i * 18U); /* 计算该页面行坐标 */
        ips200_set_color((i == s_root_index) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK); /* 选中高亮 */
        ips200_show_string(0, y, (i == s_root_index) ? ">" : " "); /* 绘制选中箭头 */
        ips200_show_string(14, y, s_pages[i].name); /* 绘制页面名称 */
    }
}

/* 二级菜单绘制：显示当前页面的参数和值。 */
static void menu_draw_param_list(void)
{
    uint8 i;                               /* 参数项遍历下标 */
    uint16 y;                              /* 参数项显示行坐标 */
    const menu_page_meta_t *page = &s_pages[s_root_index]; /* 当前页面元数据 */

    ips200_set_color(RGB565_MAGENTA, RGB565_BLACK); /* 二级页面头颜色 */
    ips200_show_string(0, 36, "Page:");            /* 绘制前缀 */
    ips200_show_string(48, 36, page->name);         /* 绘制当前页面名 */

    for (i = 0U; i < page->param_count; ++i)
    {
        menu_param_id_enum id = (menu_param_id_enum)page->param_ids[i]; /* 当前参数 ID */
        const menu_param_meta_t *meta = &s_param_meta[id];               /* 当前参数显示配置 */
        y = (uint16)(56U + (uint16)i * 16U); /* 计算当前参数项显示行 */

        ips200_set_color((i == s_param_index) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK); /* 选中项高亮 */
        ips200_show_string(0, y, (i == s_param_index) ? ">" : " "); /* 绘制选中箭头 */
        ips200_show_string(12, y, meta->name); /* 绘制参数名称 */
        ips200_show_float(200, y, menu_get_param_value(id), 6, meta->decimal); /* 绘制参数值 */
    }
}

/* 底部调试信息绘制：统一显示当前位姿。 */
static void menu_draw_pose(uint8 force_refresh)
{
    chassis_pose_t pose = chassis_ctrl_get_pose(); /* 当前里程计位姿副本 */
    float imu_yaw_deg = chassis_imu_get_yaw_deg(); /* 当前 IMU 航向角 */
    char status_line[41];                          /* 固定宽度状态行缓冲 */
    uint8 i;                                       /* 字符串处理下标 */

    if (0U != force_refresh)
    {
        /* 全刷时补画标题，增量刷新时不重复绘制。 */
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(0, 152, "Map 12x16 | Inner 10x14");
        ips200_show_string(0, 170, "SX:0.000 SY:0.000 ST:00,00");
        ips200_show_string(0, 188, "Pose X / Y / IMU Yaw");
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);      /* 数据区使用白字 */
    ips200_show_float(24,  170, CHASSIS_GRID_STEP_X_M, 5, 3U); /* 绘制 X 方向步长 */
    ips200_show_float(96,  170, CHASSIS_GRID_STEP_Y_M, 5, 3U); /* 绘制 Y 方向步长 */
    ips200_show_uint(168,  170, CHASSIS_START_GRID_X,  2U);    /* 对齐到 "ST:00,00" 中第一个 "00" */
    ips200_show_uint(192,  170, CHASSIS_START_GRID_Y,  2U);    /* 对齐到 "ST:00,00" 中第二个 "00" */
    ips200_show_float(0,   206, pose.x_m, 4, 2U);      /* 绘制 X 坐标 */
    ips200_show_float(100, 206, pose.y_m, 4, 2U);      /* 绘制 Y 坐标 */
    ips200_show_float(200, 206, imu_yaw_deg, 5, 1U);   /* 绘制 IMU 航向角 */

    if ((0U != force_refresh) || (0 != strcmp(s_status_text_prev, s_status_text)))
    {
        /* 先用空格铺满一行，确保短字符串也能覆盖旧内容。 */
        for (i = 0U; i < 40U; ++i)
        {
            status_line[i] = ' '; /* 默认填充空格用于清除旧字符 */
        }
        /* 将状态文本拷贝到行缓冲起始位置。 */
        for (i = 0U; (i < 40U) && ('\0' != s_status_text[i]); ++i)
        {
            status_line[i] = s_status_text[i]; /* 拷贝状态字符串字符 */
        }
        status_line[40] = '\0'; /* 行缓冲末尾补终止符 */

        ips200_set_color(RGB565_CYAN, RGB565_BLACK); /* 状态行使用青字 */
        ips200_show_string(0, 224, status_line);     /* 绘制状态行 */
        strcpy(s_status_text_prev, s_status_text);   /* 记录本次状态以便下次比较 */
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
    /* 初始化 Flash 驱动：0 表示成功。 */
    s_flash_ready = (0U == flash_init()) ? 1U : 0U; /* 初始化 Flash 并记录可用标志 */
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
    /* 每周期先采样按键状态。 */
    key_scanner(); /* 扫描并更新按键状态机 */

    if (MENU_LEVEL_ROOT == s_menu_level)
    {
        /* 一级菜单处理：翻页/进入二级/保存。 */
        menu_handle_root_keys(); /* 处理一级菜单按键逻辑 */
    }
    else
    {
        /* 二级菜单处理：选项切换与参数增减。 */
        menu_handle_param_keys(); /* 处理二级菜单按键逻辑 */
    }

    /* 统一清理长按状态，避免状态位滞留。 */
    menu_clear_long_press_flags(); /* 清理未消费的长按标志 */
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
    uint8 do_full_refresh = 0U; /* 本周期是否需要整屏刷新 */

    /* 按 10ms 调度计数，累计到约 100ms 执行一次。 */
    s_render_ticks++; /* 累积 10ms 调用次数 */
    if ((0U == s_need_redraw) && (s_render_ticks < 10U))
    {
        /* 无界面变化且未到周期时机，直接返回减少刷新负载。 */
        return; /* 未到周期且无重绘请求：跳过本次渲染 */
    }

    /* 进入刷新窗口后清周期计数。 */
    s_render_ticks = 0U; /* 达到渲染窗口后重置计数 */
    if ((0U == s_full_refresh_done) || (0U != s_need_redraw))
    {
        /* 首次渲染或有菜单变化时做整屏重绘。 */
        do_full_refresh = 1U;   /* 置位“本次全刷”标志 */
        s_need_redraw = 0U;     /* 清除重绘请求 */
        s_full_refresh_done = 1U; /* 标记全刷已执行 */

        ips200_full(RGB565_BLACK); /* 清屏到黑底 */
        menu_draw_header();        /* 重绘通用头部 */

        if (MENU_LEVEL_ROOT == s_menu_level)
        {
            menu_draw_root_list(); /* 绘制一级页面列表 */
        }
        else
        {
            menu_draw_param_list(); /* 绘制二级参数列表 */
        }
    }

    /* 常态仅更新位姿区，避免整屏反复刷导致闪烁。 */
    menu_draw_pose(do_full_refresh); /* 更新位姿/状态区域 */
}
