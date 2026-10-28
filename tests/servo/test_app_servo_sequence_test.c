#include <stdint.h>
#include <stdio.h>

#include "app_servo_sequence.h"
#include "config/configServo.h"
#include "drv_servo.h"

static unsigned s_init_calls;
static unsigned s_set_calls;
static float s_last_angle;
static unsigned s_assertions;
static unsigned s_failures;
static unsigned s_cases;
static const char *s_case_name;

void servo_init(float home_deg)
{
    ++s_init_calls;
    s_last_angle = home_deg;
}

void servo_set_angle(float deg)
{
    ++s_set_calls;
    s_last_angle = deg;
}

static void expect_angle(float expected, int line)
{
    float diff = s_last_angle - expected;
    if (diff < 0.0f)
    {
        diff = -diff;
    }
    ++s_assertions;
    if (diff > 0.001f)
    {
        ++s_failures;
        printf("FAIL %s:%d angle=%.3f expected=%.3f\n",
               s_case_name, line, (double)s_last_angle, (double)expected);
    }
}

static void expect_count(unsigned actual, unsigned expected, int line)
{
    ++s_assertions;
    if (actual != expected)
    {
        ++s_failures;
        printf("FAIL %s:%d count=%u expected=%u\n",
               s_case_name, line, actual, expected);
    }
}

#define EXPECT_ANGLE(a)     expect_angle((a), __LINE__)
#define EXPECT_COUNT(a, e)  expect_count((a), (e), __LINE__)

static void reset_stats(void)
{
    s_init_calls = 0U;
    s_set_calls = 0U;
    s_last_angle = 0.0f;
}

static void test_init_stops_home_and_waits(void)
{
    reset_stats();
    app_servo_sequence_init();
    EXPECT_COUNT(s_init_calls, 1U);
    EXPECT_COUNT(s_set_calls, 0U);
    EXPECT_ANGLE(SERVO_SEQ_HOME_DEG);
}

static void test_first_step_after_pause(void)
{
    reset_stats();
    app_servo_sequence_init();
    app_servo_sequence_task(0U);
    EXPECT_COUNT(s_set_calls, 0U);

    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS - 1U);
    EXPECT_COUNT(s_set_calls, 0U);

    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_SEQ_LOW_DEG);
}

static void test_full_cycle_ends_at_home(void)
{
    reset_stats();
    app_servo_sequence_init();
    app_servo_sequence_task(0U);

    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS);
    EXPECT_ANGLE(SERVO_SEQ_LOW_DEG);

    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS * 2U);
    EXPECT_ANGLE(SERVO_SEQ_HIGH_DEG);

    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS * 3U);
    EXPECT_ANGLE(SERVO_SEQ_HOME_DEG);
    EXPECT_COUNT(s_set_calls, 3U);

    /* 完成后不再发新角度。 */
    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS * 6U);
    EXPECT_COUNT(s_set_calls, 3U);
    EXPECT_ANGLE(SERVO_SEQ_HOME_DEG);
}

static void test_merged_tick_advances_one_step(void)
{
    reset_stats();
    app_servo_sequence_init();
    app_servo_sequence_task(0U);
    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS * 5U);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_SEQ_LOW_DEG);
}

static void test_uint32_wrap(void)
{
    uint32_t base = UINT32_MAX - 100U;

    reset_stats();
    app_servo_sequence_init();
    app_servo_sequence_task(base);
    app_servo_sequence_task(base + SERVO_SEQ_PAUSE_MS);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_SEQ_LOW_DEG);
}

static void test_reinit_restarts_cycle(void)
{
    reset_stats();
    app_servo_sequence_init();
    app_servo_sequence_task(0U);
    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS);
    EXPECT_ANGLE(SERVO_SEQ_LOW_DEG);

    reset_stats();
    app_servo_sequence_init();
    EXPECT_COUNT(s_init_calls, 1U);
    EXPECT_COUNT(s_set_calls, 0U);
    EXPECT_ANGLE(SERVO_SEQ_HOME_DEG);

    app_servo_sequence_task(0U);
    app_servo_sequence_task(SERVO_SEQ_PAUSE_MS);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_SEQ_LOW_DEG);
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
    RUN_CASE(test_init_stops_home_and_waits);
    RUN_CASE(test_first_step_after_pause);
    RUN_CASE(test_full_cycle_ends_at_home);
    RUN_CASE(test_merged_tick_advances_one_step);
    RUN_CASE(test_uint32_wrap);
    RUN_CASE(test_reinit_restarts_cycle);
    printf("RESULT %u cases, %u assertions, %u failures\n",
           s_cases, s_assertions, s_failures);
    return (s_failures == 0U) ? 0 : 1;
}
