#include <stdint.h>
#include <stdio.h>

#include "app_servo_test.h"
#include "config/configServo.h"
#include "drv_servo.h"

static unsigned s_init_calls;
static unsigned s_set_calls;
static float s_last_angle;
static unsigned s_assertions;
static unsigned s_failures;
static unsigned s_cases;
static const char *s_case_name;

void servo_init(void)
{
    ++s_init_calls;
    s_last_angle = SERVO_TEST_START_DEG;
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

static void arm(uint32_t t_ms)
{
    app_servo_test_task(t_ms);
}

static void step_at(uint32_t t_ms)
{
    app_servo_test_task(t_ms);
}

static void test_init_only_calls_driver_init(void)
{
    reset_stats();
    app_servo_test_init();
    EXPECT_COUNT(s_init_calls, 1U);
    EXPECT_COUNT(s_set_calls, 0U);
    EXPECT_ANGLE(SERVO_TEST_START_DEG);
}

static void test_first_task_arms_timer(void)
{
    reset_stats();
    app_servo_test_init();
    arm(0U);
    EXPECT_COUNT(s_set_calls, 0U);
}

static void test_step_interval_boundary(void)
{
    reset_stats();
    app_servo_test_init();
    arm(0U);
    step_at(SERVO_TEST_STEP_MS - 1U);
    EXPECT_COUNT(s_set_calls, 0U);
    step_at(SERVO_TEST_STEP_MS);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_TEST_START_DEG + SERVO_TEST_STEP_DEG);
}

static void test_upper_limit_reverses(void)
{
    uint32_t t = 0U;
    unsigned i;
    unsigned steps_to_max;

    reset_stats();
    app_servo_test_init();
    arm(t);
    steps_to_max = (unsigned)((SERVO_TEST_MAX_DEG - SERVO_TEST_START_DEG) /
                              SERVO_TEST_STEP_DEG);
    for (i = 0U; i < steps_to_max; ++i)
    {
        t += SERVO_TEST_STEP_MS;
        step_at(t);
    }
    EXPECT_ANGLE(SERVO_TEST_MAX_DEG);

    t += SERVO_TEST_STEP_MS;
    step_at(t);
    EXPECT_ANGLE(SERVO_TEST_MAX_DEG - SERVO_TEST_STEP_DEG);
}

static void test_lower_limit_reverses(void)
{
    uint32_t t = 0U;
    unsigned i;
    unsigned steps_to_min;

    reset_stats();
    app_servo_test_init();
    arm(t);
    steps_to_min = (unsigned)((SERVO_TEST_MAX_DEG - SERVO_TEST_MIN_DEG) /
                              SERVO_TEST_STEP_DEG) +
                   (unsigned)((SERVO_TEST_MAX_DEG - SERVO_TEST_START_DEG) /
                              SERVO_TEST_STEP_DEG);
    for (i = 0U; i < steps_to_min; ++i)
    {
        t += SERVO_TEST_STEP_MS;
        step_at(t);
    }
    EXPECT_ANGLE(SERVO_TEST_MIN_DEG);

    t += SERVO_TEST_STEP_MS;
    step_at(t);
    EXPECT_ANGLE(SERVO_TEST_MIN_DEG + SERVO_TEST_STEP_DEG);
}

static void test_merged_tick_steps_once(void)
{
    reset_stats();
    app_servo_test_init();
    arm(0U);
    step_at(SERVO_TEST_STEP_MS * 10U);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_TEST_START_DEG + SERVO_TEST_STEP_DEG);
}

static void test_reinit_restarts_from_start(void)
{
    uint32_t t = 0U;
    unsigned i;

    reset_stats();
    app_servo_test_init();
    arm(t);
    for (i = 0U; i < 5U; ++i)
    {
        t += SERVO_TEST_STEP_MS;
        step_at(t);
    }
    EXPECT_ANGLE(SERVO_TEST_START_DEG + 5.0f * SERVO_TEST_STEP_DEG);

    reset_stats();
    app_servo_test_init();
    arm(0U);
    step_at(SERVO_TEST_STEP_MS);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_TEST_START_DEG + SERVO_TEST_STEP_DEG);
}

static void test_uint32_wrap(void)
{
    uint32_t base = UINT32_MAX - 20U;

    reset_stats();
    app_servo_test_init();
    arm(base);
    step_at(base + SERVO_TEST_STEP_MS);
    EXPECT_COUNT(s_set_calls, 1U);
    EXPECT_ANGLE(SERVO_TEST_START_DEG + SERVO_TEST_STEP_DEG);
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
    RUN_CASE(test_init_only_calls_driver_init);
    RUN_CASE(test_first_task_arms_timer);
    RUN_CASE(test_step_interval_boundary);
    RUN_CASE(test_upper_limit_reverses);
    RUN_CASE(test_lower_limit_reverses);
    RUN_CASE(test_merged_tick_steps_once);
    RUN_CASE(test_reinit_restarts_from_start);
    RUN_CASE(test_uint32_wrap);
    printf("RESULT %u cases, %u assertions, %u failures\n",
           s_cases, s_assertions, s_failures);
    return (s_failures == 0U) ? 0 : 1;
}
