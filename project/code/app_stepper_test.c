#include "app_stepper_test.h"

#include "config/configStepper.h"
#include "config/pinMap.h"
#include "drv_stepper.h"
#include "zf_driver_gpio.h"

#define KEY_COUNT               (4U)
#define KEY_UP                  (0U)   /* K1：远离开关 */
#define KEY_DOWN                (1U)   /* K2：朝向开关 */
#define KEY_REHOME              (2U)   /* K3：重新回原点 */
#define KEY_STOP                (3U)   /* K4：急停 */
#define KEY_STABLE_SAMPLES      (STEPPER_TEST_KEY_DEBOUNCE_MS / STEPPER_TEST_TASK_PERIOD_MS)

static const gpio_pin_enum s_key_pins[KEY_COUNT] = {
    KEY_K1_PIN, KEY_K2_PIN, KEY_K3_PIN, KEY_K4_PIN
};

static app_stepper_state_t s_state;
static uint32_t s_start_ms;
static uint8_t s_have_start;
static uint8_t s_key_raw[KEY_COUNT];
static uint8_t s_key_count[KEY_COUNT];
static uint8_t s_key_stable[KEY_COUNT];

static int32_t mm_to_steps(float mm)
{
    float steps = mm * STEPPER_STEPS_PER_MM;
    return (int32_t)((steps >= 0.0f) ? (steps + 0.5f) : (steps - 0.5f));
}

static float mmps_to_sps(float mmps)
{
    return mmps * STEPPER_STEPS_PER_MM;
}

/* 返回本拍新按下的按键位图；上电时按住的键须先松开才会产生按下事件。 */
static uint8_t keys_update(void)
{
    uint8_t pressed_events = 0U;
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; ++i) {
        uint8_t raw = (uint8_t)(gpio_get_level(s_key_pins[i]) == GPIO_LOW);

        if (raw != s_key_raw[i]) {
            s_key_raw[i] = raw;
            s_key_count[i] = 0U;
        }
        if (s_key_count[i] < KEY_STABLE_SAMPLES) {
            ++s_key_count[i];
            if ((s_key_count[i] >= KEY_STABLE_SAMPLES) && (raw != s_key_stable[i])) {
                s_key_stable[i] = raw;
                if (raw != 0U) {
                    pressed_events |= (uint8_t)(1U << i);
                }
            }
        }
    }
    return pressed_events;
}

static void start_homing(void)
{
    (void)stepper_move(-mm_to_steps(STEPPER_HOME_SEARCH_MM),
                       mmps_to_sps(STEPPER_HOME_FAST_MMPS),
                       mmps_to_sps(STEPPER_ACCEL_MMPS2));
    s_state = APP_STEPPER_HOME_FAST;
}

static void jog(int32_t delta_steps)
{
    int32_t min_steps = mm_to_steps(STEPPER_SOFT_MIN_MM);
    int32_t max_steps = mm_to_steps(STEPPER_SOFT_MAX_MM);
    int32_t target = stepper_get_position() + delta_steps;

    if (target < min_steps) {
        target = min_steps;
    }
    if (target > max_steps) {
        target = max_steps;
    }
    (void)stepper_move(target - stepper_get_position(),
                       mmps_to_sps(STEPPER_JOG_SPEED_MMPS),
                       mmps_to_sps(STEPPER_ACCEL_MMPS2));
}

void app_stepper_test_init(void)
{
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; ++i) {
        gpio_init(s_key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        s_key_raw[i] = 1U;
        s_key_count[i] = 0U;
        s_key_stable[i] = 1U;   /* 视为按住，要求先松开 */
    }
    stepper_init();
    s_state = APP_STEPPER_WAIT_START;
    s_start_ms = 0U;
    s_have_start = 0U;
}

app_stepper_state_t app_stepper_test_get_state(void)
{
    return s_state;
}

void app_stepper_test_task(uint32_t now_ms)
{
    uint8_t events = keys_update();
    float slow_sps = mmps_to_sps(STEPPER_HOME_SLOW_MMPS);
    float accel_sps2 = mmps_to_sps(STEPPER_ACCEL_MMPS2);

    if ((events & (1U << KEY_STOP)) != 0U) {
        stepper_stop();
        s_state = APP_STEPPER_STOPPED;
        return;
    }

    switch (s_state) {
    case APP_STEPPER_WAIT_START:
        if (s_have_start == 0U) {
            s_start_ms = now_ms;
            s_have_start = 1U;
        } else if ((uint32_t)(now_ms - s_start_ms) >= STEPPER_TEST_START_DELAY_MS) {
            start_homing();
        }
        break;

    case APP_STEPPER_HOME_FAST:
        if (stepper_is_busy() == 0U) {
            if (stepper_home_hit() != 0U) {
                (void)stepper_move(mm_to_steps(STEPPER_HOME_BACKOFF_MM), slow_sps, accel_sps2);
                s_state = APP_STEPPER_HOME_BACKOFF;
            } else {
                s_state = APP_STEPPER_FAULT_NO_SWITCH;
            }
        }
        break;

    case APP_STEPPER_HOME_BACKOFF:
        if (stepper_is_busy() == 0U) {
            (void)stepper_move(-mm_to_steps(STEPPER_HOME_SLOW_SEARCH_MM), slow_sps, accel_sps2);
            s_state = APP_STEPPER_HOME_SLOW;
        }
        break;

    case APP_STEPPER_HOME_SLOW:
        if (stepper_is_busy() == 0U) {
            if (stepper_home_hit() != 0U) {
                stepper_set_position(0);
                (void)stepper_move(mm_to_steps(STEPPER_READY_MM), slow_sps, accel_sps2);
                s_state = APP_STEPPER_TO_READY;
            } else {
                s_state = APP_STEPPER_FAULT_NO_SWITCH;
            }
        }
        break;

    case APP_STEPPER_TO_READY:
        if (stepper_is_busy() == 0U) {
            s_state = APP_STEPPER_READY;
        }
        break;

    case APP_STEPPER_READY:
        if ((events & (1U << KEY_REHOME)) != 0U) {
            if (stepper_is_busy() == 0U) {
                start_homing();
            }
        } else if (stepper_is_busy() == 0U) {
            if ((events & (1U << KEY_UP)) != 0U) {
                jog(mm_to_steps(STEPPER_JOG_MM));
            } else if ((events & (1U << KEY_DOWN)) != 0U) {
                jog(-mm_to_steps(STEPPER_JOG_MM));
            }
        }
        break;

    case APP_STEPPER_FAULT_NO_SWITCH:
    case APP_STEPPER_STOPPED:
    default:
        if ((events & (1U << KEY_REHOME)) != 0U) {
            start_homing();
        }
        break;
    }
}
