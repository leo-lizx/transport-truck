#include "app_suction_test.h"

#include "config/configSuction.h"
#include "config/pinMap.h"
#include "drv_suction.h"
#include "zf_driver_gpio.h"

typedef enum
{
    SUCTION_TEST_WAIT_RELEASE = 0,
    SUCTION_TEST_IDLE,
    SUCTION_TEST_SUCKING,
    SUCTION_TEST_RELEASING
} suction_test_state_t;

static suction_test_state_t s_state;
static uint32_t s_candidate_since_ms;
static uint32_t s_last_sample_ms;
static uint32_t s_release_since_ms;
static uint8_t s_have_sample;
static uint8_t s_candidate_pressed;
static uint8_t s_stable_pressed;
static uint8_t s_have_stable_key;

void app_suction_test_init(void)
{
    gpio_init(SUCTION_TEST_BUTTON_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    suction_init();
    s_state = SUCTION_TEST_WAIT_RELEASE;
    s_candidate_since_ms = 0U;
    s_last_sample_ms = 0U;
    s_release_since_ms = 0U;
    s_have_sample = 0U;
    s_candidate_pressed = 0U;
    s_stable_pressed = 0U;
    s_have_stable_key = 0U;
}

void app_suction_test_task(uint32_t now_ms)
{
    uint8_t pressed = (uint8_t)(gpio_get_level(SUCTION_TEST_BUTTON_PIN) == GPIO_LOW);

    /* 漏采或重复时间戳不算连续稳定采样，重新开始本次去抖。 */
    if ((s_have_sample == 0U) || (pressed != s_candidate_pressed) ||
        ((uint32_t)(now_ms - s_last_sample_ms) != SUCTION_TEST_SAMPLE_MS))
    {
        s_candidate_pressed = pressed;
        s_candidate_since_ms = now_ms;
        s_have_sample = 1U;
    }
    s_last_sample_ms = now_ms;
    if ((uint32_t)(now_ms - s_candidate_since_ms) >= SUCTION_TEST_DEBOUNCE_MS)
    {
        s_stable_pressed = s_candidate_pressed;
        s_have_stable_key = 1U;
    }

    if (s_state == SUCTION_TEST_WAIT_RELEASE)
    {
        if ((s_have_stable_key != 0U) && (s_stable_pressed == 0U))
        {
            s_state = SUCTION_TEST_IDLE;
        }
        return;
    }

    if (s_stable_pressed != 0U)
    {
        if (s_state != SUCTION_TEST_SUCKING)
        {
            suction_set(1U, 0U);
            s_state = SUCTION_TEST_SUCKING;
        }
        return;
    }

    if (s_state == SUCTION_TEST_SUCKING)
    {
        suction_set(0U, 1U);
        s_release_since_ms = now_ms;
        s_state = SUCTION_TEST_RELEASING;
    }
    else if ((s_state == SUCTION_TEST_RELEASING) &&
             ((uint32_t)(now_ms - s_release_since_ms) >= SUCTION_TEST_RELEASE_MS))
    {
        suction_set(0U, 0U);
        s_state = SUCTION_TEST_IDLE;
    }
}
