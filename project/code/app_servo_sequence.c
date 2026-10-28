#include "app_servo_sequence.h"

#include "config/configServo.h"
#include "drv_servo.h"

typedef enum
{
    SERVO_SEQ_WAIT_0 = 0,    /* 已上电停在 90°，等待 1.5s 去 0° */
    SERVO_SEQ_WAIT_180,      /* 已发 0°，等待 1.5s 去 180° */
    SERVO_SEQ_WAIT_HOME,     /* 已发 180°，等待 1.5s 回 90° */
    SERVO_SEQ_DONE           /* 已回 90°，周期完成 */
} servo_seq_state_t;

static servo_seq_state_t s_seq_state;
static uint32_t s_seq_step_ms;
static uint8_t s_seq_have_last_ms;

void app_servo_sequence_init(void)
{
    servo_init(SERVO_SEQ_HOME_DEG);   /* 上电先到 90° */
    s_seq_state = SERVO_SEQ_WAIT_0;
    s_seq_step_ms = 0U;
    s_seq_have_last_ms = 0U;
}

void app_servo_sequence_task(uint32_t now_ms)
{
    if (s_seq_state == SERVO_SEQ_DONE)
    {
        return;
    }
    if (s_seq_have_last_ms == 0U)
    {
        s_seq_step_ms = now_ms;
        s_seq_have_last_ms = 1U;
        return;
    }

    /* 合并节拍时只推进一步；每步先停够 SERVO_SEQ_PAUSE_MS。 */
    if ((uint32_t)(now_ms - s_seq_step_ms) < SERVO_SEQ_PAUSE_MS)
    {
        return;
    }
    s_seq_step_ms = now_ms;

    switch (s_seq_state)
    {
    case SERVO_SEQ_WAIT_0:
        servo_set_angle(SERVO_SEQ_LOW_DEG);
        s_seq_state = SERVO_SEQ_WAIT_180;
        break;
    case SERVO_SEQ_WAIT_180:
        servo_set_angle(SERVO_SEQ_HIGH_DEG);
        s_seq_state = SERVO_SEQ_WAIT_HOME;
        break;
    case SERVO_SEQ_WAIT_HOME:
        servo_set_angle(SERVO_SEQ_HOME_DEG);
        s_seq_state = SERVO_SEQ_DONE;
        break;
    default:
        s_seq_state = SERVO_SEQ_DONE;
        break;
    }
}
