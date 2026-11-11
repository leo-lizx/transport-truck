/*
 * 升降步进驱动：RMN28 脉冲/方向输入 + FSK30 丝杆模组。
 *   D12 → P+（P- 接 GND）脉冲；D13 → D+（D- 接 GND）方向；D4 ← 限位开关。
 * GPT1 工作在重启模式：每次比较匹配进入中断并翻转脉冲脚，
 * 一个微步 = 低半周期 + 高半周期，半周期由 stepper_profile 逐步给出。
 * 低半周期结束时检查剩余步数和限位开关，再发下一步，
 * 所以方向脚在第一个上升沿前至少保持一个半周期，满足驱动器方向建立时间。
 * GPT2 已被 system_delay 使用，PIT 由底盘使用，这里只用 GPT1。
 */
#include "drv_stepper.h"

#include "config/configStepper.h"
#include "config/pinMap.h"
#include "stepper_profile.h"
#include "zf_common_clock.h"
#include "zf_driver_gpio.h"
#include "fsl_gpt.h"

#define STEPPER_GPT             GPT1
#define STEPPER_GPT_CLOCK_HZ    ((float)BOARD_XTAL_FREQ)   /* 24 MHz 晶振，分频 1 */

static stepper_profile_t s_profile;
static volatile int32_t s_pos_steps;
static volatile int8_t s_dir;              /* +1 远离开关，-1 朝向开关 */
static volatile uint8_t s_busy;
static volatile uint8_t s_pulse_high;
static volatile uint8_t s_home_hit;
static uint8_t s_home_count;
static uint32_t s_half_counts;

static float stepper_clamp_hz(float hz)
{
    if (hz < STEPPER_DRIVER_MIN_HZ) {
        return STEPPER_DRIVER_MIN_HZ;
    }
    if (hz > STEPPER_DRIVER_MAX_HZ) {
        return STEPPER_DRIVER_MAX_HZ;
    }
    return hz;
}

static uint32_t stepper_half_counts(float step_hz)
{
    uint32_t counts = (uint32_t)(STEPPER_GPT_CLOCK_HZ / (2.0f * step_hz));
    return (counts < 2U) ? 2U : counts;
}

static uint8_t stepper_home_switch_pressed(void)
{
    return (uint8_t)(gpio_get_level(STEPPER_HOME_SWITCH_PIN) == GPIO_LOW);
}

static void stepper_finish(void)
{
    GPT_StopTimer(STEPPER_GPT);
    gpio_low(STEPPER_PULSE_PIN);
    s_pulse_high = 0U;
    s_busy = 0U;
}

void stepper_init(void)
{
    gpt_config_t cfg;

    gpio_init(STEPPER_PULSE_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(STEPPER_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(STEPPER_HOME_SWITCH_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    GPT_GetDefaultConfig(&cfg);
    cfg.clockSource = kGPT_ClockSource_Osc;
    cfg.divider = 1U;
    cfg.enableFreeRun = false;             /* 重启模式：匹配后计数清零 */
    /* 与 system_delay_init 相同：先 Init/Deinit 一次以打开时钟。 */
    GPT_Init(STEPPER_GPT, &cfg);
    GPT_Deinit(STEPPER_GPT);
    GPT_Init(STEPPER_GPT, &cfg);
    GPT_EnableInterrupts(STEPPER_GPT, kGPT_OutputCompare1InterruptEnable);
    EnableIRQ(GPT1_IRQn);

    s_pos_steps = 0;
    s_dir = 1;
    s_busy = 0U;
    s_pulse_high = 0U;
    s_home_hit = 0U;
    s_home_count = 0U;
}

uint8_t stepper_move(int32_t delta_steps, float speed_sps, float accel_sps2)
{
    uint32_t steps;
    float vmin;
    float vmax;
    uint8_t away_level = (uint8_t)STEPPER_DIR_LEVEL_AWAY_FROM_HOME;

    if (s_busy != 0U) {
        return 0U;
    }
    if (delta_steps == 0) {
        return 1U;
    }

    steps = (delta_steps > 0) ? (uint32_t)delta_steps : (uint32_t)(-delta_steps);
    vmin = stepper_clamp_hz(STEPPER_START_SPEED_MMPS * STEPPER_STEPS_PER_MM);
    vmax = stepper_clamp_hz(speed_sps);

    s_dir = (delta_steps > 0) ? (int8_t)1 : (int8_t)-1;
    gpio_set_level(STEPPER_DIR_PIN, (s_dir > 0) ? away_level : (uint8_t)(away_level ^ 1U));
    gpio_low(STEPPER_PULSE_PIN);

    stepper_profile_start(&s_profile, steps, vmin, vmax, accel_sps2);
    s_home_hit = 0U;
    s_home_count = 0U;
    s_pulse_high = 0U;
    s_half_counts = stepper_half_counts(vmin);
    s_busy = 1U;

    GPT_StopTimer(STEPPER_GPT);
    GPT_SetOutputCompareValue(STEPPER_GPT, kGPT_OutputCompare_Channel1, s_half_counts);
    GPT_ClearStatusFlags(STEPPER_GPT, kGPT_OutputCompare1Flag);
    GPT_StartTimer(STEPPER_GPT);
    return 1U;
}

void stepper_stop(void)
{
    __disable_irq();
    stepper_finish();
    __enable_irq();
}

uint8_t stepper_is_busy(void)
{
    return s_busy;
}

int32_t stepper_get_position(void)
{
    return s_pos_steps;
}

void stepper_set_position(int32_t steps)
{
    __disable_irq();
    s_pos_steps = steps;
    __enable_irq();
}

uint8_t stepper_home_hit(void)
{
    return s_home_hit;
}

void stepper_isr(void)
{
    GPT_ClearStatusFlags(STEPPER_GPT, kGPT_OutputCompare1Flag);

    if (s_busy == 0U) {
        GPT_StopTimer(STEPPER_GPT);
        return;
    }

    if (s_pulse_high != 0U) {
        /* 高半周期结束：这一步完成。 */
        gpio_low(STEPPER_PULSE_PIN);
        s_pulse_high = 0U;
        s_pos_steps += s_dir;
        GPT_SetOutputCompareValue(STEPPER_GPT, kGPT_OutputCompare_Channel1, s_half_counts);
        return;
    }

    /* 低半周期结束：决定是否发下一步。 */
    if (s_profile.remaining == 0U) {
        stepper_finish();
        return;
    }
    if (s_dir < 0) {
        if (stepper_home_switch_pressed() != 0U) {
            if (++s_home_count >= STEPPER_HOME_DEBOUNCE_STEPS) {
                s_home_hit = 1U;
                stepper_finish();
                return;
            }
        } else {
            s_home_count = 0U;
        }
    }

    s_half_counts = stepper_half_counts(stepper_clamp_hz(stepper_profile_next(&s_profile)));
    gpio_high(STEPPER_PULSE_PIN);
    s_pulse_high = 1U;
    GPT_SetOutputCompareValue(STEPPER_GPT, kGPT_OutputCompare_Channel1, s_half_counts);
}
