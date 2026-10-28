#include "app_servo_test.h"

#include "config/configServo.h"
#include "drv_servo.h"

static float s_angle;
static int8_t s_direction;
static uint32_t s_last_step_ms;
static uint8_t s_have_last_ms;

void app_servo_test_init(void)
{
    servo_init(SERVO_TEST_START_DEG);
    s_angle = SERVO_TEST_START_DEG;
    s_direction = 1;
    s_last_step_ms = 0U;
    s_have_last_ms = 0U;
}

void app_servo_test_task(uint32_t now_ms)
{
    if (s_have_last_ms == 0U)
    {
        s_last_step_ms = now_ms;
        s_have_last_ms = 1U;
        return;
    }

    /* 合并节拍时只推进一步，避免一次跳多个角度。 */
    if ((uint32_t)(now_ms - s_last_step_ms) < SERVO_TEST_STEP_MS)
    {
        return;
    }
    s_last_step_ms = now_ms;

    s_angle += (float)s_direction * SERVO_TEST_STEP_DEG;
    if (s_angle >= SERVO_TEST_MAX_DEG)
    {
        s_angle = SERVO_TEST_MAX_DEG;
        s_direction = -1;
    }
    else if (s_angle <= SERVO_TEST_MIN_DEG)
    {
        s_angle = SERVO_TEST_MIN_DEG;
        s_direction = 1;
    }
    servo_set_angle(s_angle);
}
