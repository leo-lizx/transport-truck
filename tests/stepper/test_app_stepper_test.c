#include <stdint.h>
#include <stdio.h>

#include "app_stepper_test.h"
#include "config/configStepper.h"
#include "drv_stepper.h"
#include "zf_driver_gpio.h"

/* ---------------- 桩：按键 GPIO ---------------- */
static uint8_t s_level[4];   /* 下标 0..3 = C12..C15，1 = 松开 */

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8_t value, uint32_t pinconf)
{
    (void)pin; (void)dir; (void)value; (void)pinconf;
}

uint8_t gpio_get_level(gpio_pin_enum pin)
{
    return s_level[(int)pin - (int)C12];
}

/* K1=C15 K2=C14 K3=C13 K4=C12 */
static void key_set(int k, int pressed)
{
    static const gpio_pin_enum pins[4] = { C15, C14, C13, C12 };
    s_level[(int)pins[k - 1] - (int)C12] = pressed ? 0U : 1U;
}

/* ---------------- 桩：步进驱动 ---------------- */
static unsigned s_init_calls;
static unsigned s_move_calls;
static unsigned s_stop_calls;
static int32_t s_last_delta;
static float s_last_speed;
static uint8_t s_busy;
static uint8_t s_hit;
static int32_t s_pos;
static int32_t s_pending_delta;

void stepper_init(void) { ++s_init_calls; }

uint8_t stepper_move(int32_t delta_steps, float speed_sps, float accel_sps2)
{
    (void)accel_sps2;
    if (s_busy) {
        return 0U;
    }
    ++s_move_calls;
    s_last_delta = delta_steps;
    s_last_speed = speed_sps;
    s_hit = 0U;
    if (delta_steps != 0) {
        s_busy = 1U;
        s_pending_delta = delta_steps;
    }
    return 1U;
}

void stepper_stop(void) { ++s_stop_calls; s_busy = 0U; }
uint8_t stepper_is_busy(void) { return s_busy; }
int32_t stepper_get_position(void) { return s_pos; }
void stepper_set_position(int32_t steps) { s_pos = steps; }
uint8_t stepper_home_hit(void) { return s_hit; }
void stepper_isr(void) {}

/* 让当前运动结束：hit=1 表示在 travel 步处碰到开关。 */
static void finish_move(int hit, int32_t travel)
{
    s_busy = 0U;
    s_hit = (uint8_t)hit;
    s_pos += hit ? travel : s_pending_delta;
}

/* ---------------- 断言工具 ---------------- */
static unsigned s_assertions;
static unsigned s_failures;
static unsigned s_cases;
static const char *s_case_name;
static uint32_t s_now;

static void expect_int(long actual, long expected, int line)
{
    ++s_assertions;
    if (actual != expected) {
        ++s_failures;
        printf("FAIL %s:%d got=%ld expected=%ld\n", s_case_name, line, actual, expected);
    }
}

#define EXPECT_EQ(a, e) expect_int((long)(a), (long)(e), __LINE__)

static int32_t mm(float v)
{
    float s = v * STEPPER_STEPS_PER_MM;
    return (int32_t)((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
}

static void tick(void)
{
    app_stepper_test_task(s_now);
    s_now += STEPPER_TEST_TASK_PERIOD_MS;
}

static void ticks(unsigned n)
{
    while (n--) {
        tick();
    }
}

/* 按住到去抖完成，再松开到去抖完成。 */
static void press(int k)
{
    key_set(k, 1);
    ticks(STEPPER_TEST_KEY_DEBOUNCE_MS / STEPPER_TEST_TASK_PERIOD_MS + 1U);
    key_set(k, 0);
    ticks(STEPPER_TEST_KEY_DEBOUNCE_MS / STEPPER_TEST_TASK_PERIOD_MS + 1U);
}

static void reset_all(uint32_t start_ms)
{
    int k;
    for (k = 1; k <= 4; ++k) {
        key_set(k, 0);
    }
    s_init_calls = s_move_calls = s_stop_calls = 0U;
    s_last_delta = 0;
    s_last_speed = 0.0f;
    s_busy = s_hit = 0U;
    s_pos = 12345;              /* 上电位置未知 */
    s_pending_delta = 0;
    s_now = start_ms;
    app_stepper_test_init();
}

/* 走完回原点流程直到 READY，零点设在开关触发处。 */
static void do_homing(void)
{
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);
    finish_move(1, -mm(20.0f));               /* 快速段碰到开关 */
    tick();
    finish_move(0, 0);                        /* 退出 2 mm */
    tick();
    finish_move(1, -mm(1.9f));                /* 慢速再碰 */
    tick();
    finish_move(0, 0);                        /* 去就绪位 */
    tick();
}

static void test_waits_before_homing(void)
{
    reset_all(0U);
    EXPECT_EQ(s_init_calls, 1U);
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS);
    EXPECT_EQ(s_move_calls, 0U);
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_WAIT_START);
    ticks(2U);
    EXPECT_EQ(s_move_calls, 1U);
    EXPECT_EQ(s_last_delta, -mm(STEPPER_HOME_SEARCH_MM));
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_FAST);
}

static void test_homing_sequence_sets_zero(void)
{
    reset_all(0U);
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);

    finish_move(1, -mm(20.0f));
    tick();
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_BACKOFF);
    EXPECT_EQ(s_last_delta, mm(STEPPER_HOME_BACKOFF_MM));

    finish_move(0, 0);
    tick();
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_SLOW);
    EXPECT_EQ(s_last_delta, -mm(STEPPER_HOME_SLOW_SEARCH_MM));

    finish_move(1, -mm(1.9f));
    tick();
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_TO_READY);
    EXPECT_EQ(s_last_delta, mm(STEPPER_READY_MM));
    EXPECT_EQ(s_pos, 0);                      /* 零点 = 慢速触发点 */

    finish_move(0, 0);
    tick();
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_READY);
    EXPECT_EQ(s_pos, mm(STEPPER_READY_MM));
}

static void test_fast_search_without_switch_faults(void)
{
    reset_all(0U);
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);
    finish_move(0, 0);                        /* 走满仍未碰到 */
    tick();
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_FAULT_NO_SWITCH);
    {
        unsigned moves = s_move_calls;
        ticks(100U);
        EXPECT_EQ(s_move_calls, moves);       /* 故障后不再自动运动 */
    }
}

static void test_slow_search_without_switch_faults(void)
{
    reset_all(0U);
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);
    finish_move(1, -mm(20.0f));
    tick();
    finish_move(0, 0);
    tick();
    finish_move(0, 0);                        /* 慢速段未碰到 */
    tick();
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_FAULT_NO_SWITCH);
}

static void test_jog_up_and_down(void)
{
    reset_all(0U);
    do_homing();

    press(1);
    EXPECT_EQ(s_last_delta, mm(STEPPER_JOG_MM));
    EXPECT_EQ((long)s_last_speed, (long)(STEPPER_JOG_SPEED_MMPS * STEPPER_STEPS_PER_MM));
    finish_move(0, 0);
    tick();

    press(2);
    EXPECT_EQ(s_last_delta, -mm(STEPPER_JOG_MM));
    finish_move(0, 0);
    tick();
    EXPECT_EQ(s_pos, mm(STEPPER_READY_MM));
}

static void test_jog_clamped_to_soft_limits(void)
{
    unsigned i;
    reset_all(0U);
    do_homing();

    /* 连按上升，位置不得超过软件上限。 */
    for (i = 0U; i < 10U; ++i) {
        press(1);
        if (s_busy) {
            finish_move(0, 0);
            tick();
        }
    }
    EXPECT_EQ(s_pos, mm(STEPPER_SOFT_MAX_MM));

    /* 连按下降，位置不得低于软件下限。 */
    for (i = 0U; i < 10U; ++i) {
        press(2);
        if (s_busy) {
            finish_move(0, 0);
            tick();
        }
    }
    EXPECT_EQ(s_pos, mm(STEPPER_SOFT_MIN_MM));
}

static void test_jog_ignored_while_busy(void)
{
    reset_all(0U);
    do_homing();
    press(1);
    {
        unsigned moves = s_move_calls;
        press(1);                             /* 上一段还没走完 */
        EXPECT_EQ(s_move_calls, moves);
    }
}

static void test_stop_key_stops_any_time(void)
{
    reset_all(0U);
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_FAST);
    press(4);
    EXPECT_EQ(s_stop_calls, 1U);
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_STOPPED);
    {
        unsigned moves = s_move_calls;
        press(1);                             /* 停机后点动无效 */
        EXPECT_EQ(s_move_calls, moves);
    }
}

static void test_rehome_from_stopped_and_ready(void)
{
    reset_all(0U);
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);
    press(4);
    press(3);
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_FAST);
    EXPECT_EQ(s_last_delta, -mm(STEPPER_HOME_SEARCH_MM));

    reset_all(0U);
    do_homing();
    press(3);
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_FAST);
}

static void test_key_held_at_boot_needs_release(void)
{
    reset_all(0U);
    key_set(4, 1);                            /* 上电时 K4 一直按着 */
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);
    EXPECT_EQ(s_stop_calls, 0U);
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_FAST);
    key_set(4, 0);
    ticks(10U);
    EXPECT_EQ(s_stop_calls, 0U);
}

static void test_short_glitch_not_a_press(void)
{
    reset_all(0U);
    do_homing();
    key_set(1, 1);
    tick();                                   /* 只按 5 ms */
    key_set(1, 0);
    ticks(10U);
    EXPECT_EQ(s_busy, 0U);
}

static void test_uint32_wrap_start_delay(void)
{
    reset_all(UINT32_MAX - 200U);
    ticks(STEPPER_TEST_START_DELAY_MS / STEPPER_TEST_TASK_PERIOD_MS + 2U);
    EXPECT_EQ(app_stepper_test_get_state(), APP_STEPPER_HOME_FAST);
}

static void run_case(const char *name, void (*test)(void))
{
    unsigned failures_before = s_failures;
    s_case_name = name;
    ++s_cases;
    test();
    printf("%s %s\n", (s_failures == failures_before) ? "PASS" : "FAIL", name);
}

#define RUN_CASE(test) run_case(#test, test)

int main(void)
{
    RUN_CASE(test_waits_before_homing);
    RUN_CASE(test_homing_sequence_sets_zero);
    RUN_CASE(test_fast_search_without_switch_faults);
    RUN_CASE(test_slow_search_without_switch_faults);
    RUN_CASE(test_jog_up_and_down);
    RUN_CASE(test_jog_clamped_to_soft_limits);
    RUN_CASE(test_jog_ignored_while_busy);
    RUN_CASE(test_stop_key_stops_any_time);
    RUN_CASE(test_rehome_from_stopped_and_ready);
    RUN_CASE(test_key_held_at_boot_needs_release);
    RUN_CASE(test_short_glitch_not_a_press);
    RUN_CASE(test_uint32_wrap_start_delay);
    printf("RESULT %u cases, %u assertions, %u failures\n",
           s_cases, s_assertions, s_failures);
    return (s_failures == 0U) ? 0 : 1;
}
