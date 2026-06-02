/*===========================================================================
 * [chassis_config.c] 底盘运行时调参配置与 Flash 持久化
 *
 * 职责:
 *   1. 提供底盘调参默认值与活动全局配置。
 *   2. 对人工调参量做统一限幅。
 *   3. 使用 sector 127 的 FLASH_PAGE_6/7 双槽保存配置。
 *
 * 调度约束:
 *   - 菜单和 main loop 可调用加载/保存接口。
 *   - 中断只允许读取 g_chassis_tune_params，禁止调用 Flash 写入接口。
 *   - chassis_config_apply() 关中断复制结构体，避免 ISR 读到半更新配置。
 *===========================================================================*/

#include "chassis_config.h"
#include "zf_common_headfile.h"
#include "zf_driver_flash.h"
#include <stddef.h>
#include <string.h>

#define CHASSIS_CONFIG_FLASH_SECTOR       (127U)
#define CHASSIS_CONFIG_FLASH_SLOT0_PAGE   (FLASH_PAGE_6)
#define CHASSIS_CONFIG_FLASH_SLOT1_PAGE   (FLASH_PAGE_7)
#define CHASSIS_CONFIG_FLASH_MAGIC        (0x43484346UL) /* "CHCF" */
#define CHASSIS_CONFIG_FLASH_VERSION      (3U)

#define CHASSIS_TUNE_DEFAULT_INITIALIZER                                      \
{                                                                             \
    {                                                                         \
        { CHASSIS_WHEEL_PID_LF_KP, CHASSIS_WHEEL_PID_RF_KP,                   \
          CHASSIS_WHEEL_PID_LB_KP, CHASSIS_WHEEL_PID_RB_KP },                 \
        { CHASSIS_WHEEL_PID_LF_KI, CHASSIS_WHEEL_PID_RF_KI,                   \
          CHASSIS_WHEEL_PID_LB_KI, CHASSIS_WHEEL_PID_RB_KI },                 \
        { CHASSIS_WHEEL_PID_LF_KD, CHASSIS_WHEEL_PID_RF_KD,                   \
          CHASSIS_WHEEL_PID_LB_KD, CHASSIS_WHEEL_PID_RB_KD }                  \
    },                                                                        \
    {                                                                         \
        CHASSIS_POS_KP, CHASSIS_POS_KD, CHASSIS_POS_KI,                       \
        CHASSIS_POS_I_LIMIT, CHASSIS_POS_I_BAND_M,                            \
        CHASSIS_POS_CTE_KP, CHASSIS_POS_CTE_KD,                               \
        CHASSIS_POS_AXIS_HOLD_MAX_SPEED_MPS,                                  \
        CHASSIS_POS_BRAKE_DIST_M, CHASSIS_POS_BRAKE_FLOOR_MPS,                \
        CHASSIS_TARGET_REACHED_EPSILON_M                                      \
    },                                                                        \
    {                                                                         \
        CHASSIS_YAW_KP, CHASSIS_YAW_DEADZONE_DEG, CHASSIS_YAW_KI,             \
        CHASSIS_YAW_I_LIMIT, CHASSIS_YAW_RATE_KP, CHASSIS_YAW_RATE_KI,        \
        CHASSIS_YAW_RATE_I_LIMIT, CHASSIS_YAW_RATE_I_LEAK                     \
    },                                                                        \
    {                                                                         \
        CHASSIS_MAX_LINEAR_SPEED_MPS, CHASSIS_MAX_YAW_SPEED_DPS,              \
        CHASSIS_CMD_ACCEL_LIMIT_MPS2, CHASSIS_CMD_ACCEL_LIMIT_DPS2            \
    },                                                                        \
    {                                                                         \
        CHASSIS_WHEEL_BREAKAWAY_TARGET_EPS_MPS,                               \
        CHASSIS_WHEEL_BREAKAWAY_PWM_FLOOR,                                    \
        800.0f                                                                \
    },                                                                        \
    { CHASSIS_ODOM_SCALE_X, CHASSIS_ODOM_SCALE_Y }                            \
}

typedef struct
{
    uint32 magic;
    uint32 version;
    uint32 size;
    uint32 seq;
    chassis_tune_params_t payload;
    uint32 crc32;
} chassis_config_flash_blob_t;

#define CHASSIS_CONFIG_FLASH_WORDS \
    ((uint16)((sizeof(chassis_config_flash_blob_t) + sizeof(uint32) - 1U) / sizeof(uint32)))

static const chassis_tune_params_t s_chassis_tune_defaults = CHASSIS_TUNE_DEFAULT_INITIALIZER;
volatile chassis_tune_params_t g_chassis_tune_params = CHASSIS_TUNE_DEFAULT_INITIALIZER;

static uint8 s_flash_ready = 0U;

static uint8 chassis_config_flash_ready(void)
{
    if (s_flash_ready == 0U)
    {
        s_flash_ready = (flash_init() == 0U) ? 1U : 0U;
    }
    return s_flash_ready;
}

static flash_page_enum chassis_config_slot_page(uint8 slot)
{
    return (slot == 0U) ? CHASSIS_CONFIG_FLASH_SLOT0_PAGE
                        : CHASSIS_CONFIG_FLASH_SLOT1_PAGE;
}

static uint32 chassis_config_crc32(const uint8 *data, uint32 len)
{
    uint32 crc = 0xFFFFFFFFUL;
    uint32 i;

    for (i = 0U; i < len; ++i)
    {
        uint8 bit;
        crc ^= (uint32)data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static uint8 chassis_config_blob_valid(const chassis_config_flash_blob_t *blob)
{
    uint32 calc_crc;

    if (blob == NULL)
    {
        return 0U;
    }

    if ((blob->magic != CHASSIS_CONFIG_FLASH_MAGIC) ||
        (blob->version != CHASSIS_CONFIG_FLASH_VERSION) ||
        (blob->size != (uint32)sizeof(chassis_tune_params_t)))
    {
        return 0U;
    }

    calc_crc = chassis_config_crc32((const uint8 *)blob,
                                    (uint32)offsetof(chassis_config_flash_blob_t, crc32));
    return (calc_crc == blob->crc32) ? 1U : 0U;
}

static uint8 chassis_config_read_slot(uint8 slot, chassis_config_flash_blob_t *blob)
{
    uint32 words[CHASSIS_CONFIG_FLASH_WORDS];

    if ((blob == NULL) || (chassis_config_flash_ready() == 0U))
    {
        return 0U;
    }

    flash_read_page(CHASSIS_CONFIG_FLASH_SECTOR,
                    chassis_config_slot_page(slot),
                    words,
                    CHASSIS_CONFIG_FLASH_WORDS);
    memcpy(blob, words, sizeof(*blob));

    return chassis_config_blob_valid(blob);
}

const chassis_tune_params_t *chassis_config_defaults(void)
{
    return &s_chassis_tune_defaults;
}

void chassis_config_sanitize_tune(chassis_tune_params_t *params)
{
    uint8 i;

    if (params == NULL)
    {
        return;
    }

    for (i = 0U; i < CHASSIS_CTRL_TUNE_WHEEL_COUNT; ++i)
    {
        params->wheel_pid.kp[i] = chassis_clamp_f(params->wheel_pid.kp[i], 0.0f, 400.0f);
        params->wheel_pid.ki[i] = chassis_clamp_f(params->wheel_pid.ki[i], 0.0f, 80.0f);
        params->wheel_pid.kd[i] = chassis_clamp_f(params->wheel_pid.kd[i], 0.0f, 40.0f);
    }

    params->position.kp = chassis_clamp_f(params->position.kp, 0.0f, 5.0f);
    params->position.kd = chassis_clamp_f(params->position.kd, 0.0f, 5.0f);
    params->position.ki = chassis_clamp_f(params->position.ki, 0.0f, 1.0f);
    params->position.i_limit_mps = chassis_clamp_f(params->position.i_limit_mps, 0.0f, 0.5f);
    params->position.i_band_m = chassis_clamp_f(params->position.i_band_m, 0.0f, 1.0f);
    params->position.cte_kp = chassis_clamp_f(params->position.cte_kp, 0.0f, 20.0f);
    params->position.cte_kd = chassis_clamp_f(params->position.cte_kd, 0.0f, 5.0f);
    params->position.axis_hold_max_speed_mps =
        chassis_clamp_f(params->position.axis_hold_max_speed_mps, 0.02f, 0.5f);
    params->position.brake_dist_m = chassis_clamp_f(params->position.brake_dist_m, 0.05f, 1.0f);
    params->position.brake_floor_mps = chassis_clamp_f(params->position.brake_floor_mps, 0.0f, 0.5f);
    params->position.target_reached_epsilon_m =
        chassis_clamp_f(params->position.target_reached_epsilon_m, 0.01f, 0.2f);

    params->yaw.kp = chassis_clamp_f(params->yaw.kp, 0.0f, 10.0f);
    params->yaw.deadzone_deg = chassis_clamp_f(params->yaw.deadzone_deg, 0.0f, 5.0f);
    params->yaw.ki = chassis_clamp_f(params->yaw.ki, 0.0f, 5.0f);
    params->yaw.i_limit = chassis_clamp_f(params->yaw.i_limit, 0.0f, 200.0f);
    params->yaw.rate_kp = chassis_clamp_f(params->yaw.rate_kp, 0.0f, 2.0f);
    params->yaw.rate_ki = chassis_clamp_f(params->yaw.rate_ki, 0.0f, 5.0f);
    params->yaw.rate_i_limit = chassis_clamp_f(params->yaw.rate_i_limit, 0.0f, 500.0f);
    params->yaw.rate_i_leak = chassis_clamp_f(params->yaw.rate_i_leak, 0.0f, 0.05f);

    params->limit.max_linear_speed_mps =
        chassis_clamp_f(params->limit.max_linear_speed_mps, 0.05f,
                        CHASSIS_TUNE_MAX_LINEAR_SPEED_LIMIT_MPS);
    params->limit.max_yaw_speed_dps =
        chassis_clamp_f(params->limit.max_yaw_speed_dps, 10.0f,
                        CHASSIS_TUNE_MAX_YAW_SPEED_LIMIT_DPS);
    params->limit.accel_limit_mps2 = chassis_clamp_f(params->limit.accel_limit_mps2, 0.10f, 5.0f);
    params->limit.yaw_accel_limit_dps2 =
        chassis_clamp_f(params->limit.yaw_accel_limit_dps2, 20.0f, 1000.0f);
    params->wheel_ff.breakaway_target_eps_mps =
        chassis_clamp_f(params->wheel_ff.breakaway_target_eps_mps, 0.0f, 0.1f);
    params->wheel_ff.breakaway_pwm_floor =
        chassis_clamp_f(params->wheel_ff.breakaway_pwm_floor, 0.0f, 3000.0f);
    params->wheel_ff.debug_start_pwm_min =
        chassis_clamp_f(params->wheel_ff.debug_start_pwm_min, 0.0f, 3000.0f);
    params->odom.scale_x = chassis_clamp_f(params->odom.scale_x, 0.1f, 2.0f);
    params->odom.scale_y = chassis_clamp_f(params->odom.scale_y, 0.1f, 2.0f);
}

void chassis_config_get_snapshot(chassis_tune_params_t *out_params)
{
    if (out_params == NULL)
    {
        return;
    }

    __disable_irq();
    *out_params = g_chassis_tune_params;
    __enable_irq();
}

uint8 chassis_config_set_wheel_pid(uint8 wheel_index, float kp, float ki, float kd)
{
    chassis_tune_params_t params;

    if (wheel_index >= CHASSIS_CTRL_TUNE_WHEEL_COUNT)
    {
        return 0U;
    }

    chassis_config_get_snapshot(&params);
    params.wheel_pid.kp[wheel_index] = kp;
    params.wheel_pid.ki[wheel_index] = ki;
    params.wheel_pid.kd[wheel_index] = kd;
    chassis_config_apply(&params);
    return 1U;
}

uint8 chassis_config_set_position_tune(const chassis_position_tune_t *position)
{
    chassis_tune_params_t params;

    if (position == NULL)
    {
        return 0U;
    }

    chassis_config_get_snapshot(&params);
    params.position = *position;
    chassis_config_apply(&params);
    return 1U;
}

uint8 chassis_config_set_yaw_tune(const chassis_yaw_tune_t *yaw)
{
    chassis_tune_params_t params;

    if (yaw == NULL)
    {
        return 0U;
    }

    chassis_config_get_snapshot(&params);
    params.yaw = *yaw;
    chassis_config_apply(&params);
    return 1U;
}

uint8 chassis_config_set_motion_limit_tune(const chassis_motion_limit_tune_t *limit)
{
    chassis_tune_params_t params;

    if (limit == NULL)
    {
        return 0U;
    }

    chassis_config_get_snapshot(&params);
    params.limit = *limit;
    chassis_config_apply(&params);
    return 1U;
}

uint8 chassis_config_set_wheel_feedforward_tune(const chassis_wheel_feedforward_tune_t *wheel_ff)
{
    chassis_tune_params_t params;

    if (wheel_ff == NULL)
    {
        return 0U;
    }

    chassis_config_get_snapshot(&params);
    params.wheel_ff = *wheel_ff;
    chassis_config_apply(&params);
    return 1U;
}

uint8 chassis_config_set_odom_tune(const chassis_odom_tune_t *odom)
{
    chassis_tune_params_t params;

    if (odom == NULL)
    {
        return 0U;
    }

    chassis_config_get_snapshot(&params);
    params.odom = *odom;
    chassis_config_apply(&params);
    return 1U;
}

void chassis_config_apply(const chassis_tune_params_t *params)
{
    chassis_tune_params_t safe;

    if (params == NULL)
    {
        return;
    }

    safe = *params;
    chassis_config_sanitize_tune(&safe);

    __disable_irq();
    g_chassis_tune_params = safe;
    __enable_irq();
}

uint8 chassis_config_load_from_flash(void)
{
    chassis_config_flash_blob_t slot0;
    chassis_config_flash_blob_t slot1;
    uint8 valid0;
    uint8 valid1;
    const chassis_config_flash_blob_t *best = NULL;

    valid0 = chassis_config_read_slot(0U, &slot0);
    valid1 = chassis_config_read_slot(1U, &slot1);

    if ((valid0 != 0U) && (valid1 != 0U))
    {
        best = (((int32)(slot1.seq - slot0.seq)) > 0) ? &slot1 : &slot0;
    }
    else if (valid0 != 0U)
    {
        best = &slot0;
    }
    else if (valid1 != 0U)
    {
        best = &slot1;
    }

    if (best == NULL)
    {
        chassis_config_reset_to_defaults();
        return 0U;
    }

    chassis_config_apply(&best->payload);
    return 1U;
}

uint8 chassis_config_save_to_flash(void)
{
    chassis_config_flash_blob_t slot0;
    chassis_config_flash_blob_t slot1;
    chassis_config_flash_blob_t blob;
    uint32 words[CHASSIS_CONFIG_FLASH_WORDS];
    uint8 valid0;
    uint8 valid1;
    uint8 target_slot = 0U;
    uint32 next_seq = 1U;

    if (chassis_config_flash_ready() == 0U)
    {
        return 0U;
    }

    valid0 = chassis_config_read_slot(0U, &slot0);
    valid1 = chassis_config_read_slot(1U, &slot1);

    if ((valid0 != 0U) && (valid1 != 0U))
    {
        if (((int32)(slot1.seq - slot0.seq)) > 0)
        {
            next_seq = slot1.seq + 1U;
            target_slot = 0U;
        }
        else
        {
            next_seq = slot0.seq + 1U;
            target_slot = 1U;
        }
    }
    else if (valid0 != 0U)
    {
        next_seq = slot0.seq + 1U;
        target_slot = 1U;
    }
    else if (valid1 != 0U)
    {
        next_seq = slot1.seq + 1U;
        target_slot = 0U;
    }

    memset(&blob, 0, sizeof(blob));
    blob.magic = CHASSIS_CONFIG_FLASH_MAGIC;
    blob.version = CHASSIS_CONFIG_FLASH_VERSION;
    blob.size = (uint32)sizeof(chassis_tune_params_t);
    blob.seq = next_seq;
    chassis_config_get_snapshot(&blob.payload);
    chassis_config_sanitize_tune(&blob.payload);
    blob.crc32 = chassis_config_crc32((const uint8 *)&blob,
                                      (uint32)offsetof(chassis_config_flash_blob_t, crc32));

    memset(words, 0xFF, sizeof(words));
    memcpy(words, &blob, sizeof(blob));

    return (flash_write_page(CHASSIS_CONFIG_FLASH_SECTOR,
                             chassis_config_slot_page(target_slot),
                             words,
                             CHASSIS_CONFIG_FLASH_WORDS) == 0U) ? 1U : 0U;
}

void chassis_config_reset_to_defaults(void)
{
    chassis_config_apply(&s_chassis_tune_defaults);
}
