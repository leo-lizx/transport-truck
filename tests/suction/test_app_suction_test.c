#include <stdint.h>
#include <stdio.h>

#include "app_suction_test.h"
#include "drv_suction.h"
#include "zf_driver_gpio.h"

static uint8_t s_key_level;
static uint8_t s_pump_on;
static uint8_t s_valve_on;
static unsigned s_gpio_init_calls;
static unsigned s_gpio_read_calls;
static unsigned s_suction_init_calls;
static unsigned s_set_calls;
static unsigned s_bad_gpio_calls;
static unsigned s_bad_commands;
static unsigned s_assertions;
static unsigned s_failures;
static unsigned s_cases;
static const char *s_case_name;

static void expect_equal(unsigned long actual, unsigned long expected,
                         const char *expression, int line)
{
    ++s_assertions;
    if (actual != expected)
    {
        ++s_failures;
        printf("FAIL %s:%d %s: got %lu, expected %lu\n",
               s_case_name, line, expression, actual, expected);
    }
}

#define EXPECT_EQ(actual, expected) \
    expect_equal((unsigned long)(actual), (unsigned long)(expected), \
                 #actual, __LINE__)
#define EXPECT_OUTPUT(pump, valve) \
    do { EXPECT_EQ(s_pump_on, pump); EXPECT_EQ(s_valve_on, valve); } while (0)

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8_t value,
               uint32_t pinconf)
{
    ++s_gpio_init_calls;
    if ((pin != C15) || (dir != GPI) || (value != GPIO_HIGH) ||
        (pinconf != GPI_PULL_UP))
    {
        ++s_bad_gpio_calls;
    }
}

uint8_t gpio_get_level(gpio_pin_enum pin)
{
    ++s_gpio_read_calls;
    if (pin != C15)
    {
        ++s_bad_gpio_calls;
    }
    return s_key_level;
}

void suction_init(void)
{
    ++s_suction_init_calls;
    s_pump_on = 0U;
    s_valve_on = 0U;
}

void suction_set(uint8_t pump_on, uint8_t valve_on)
{
    ++s_set_calls;
    if ((pump_on > 1U) || (valve_on > 1U) ||
        ((pump_on != 0U) && (valve_on != 0U)))
    {
        ++s_bad_commands;
    }
    s_pump_on = pump_on;
    s_valve_on = valve_on;
}

static void boot(uint8_t key_level)
{
    s_key_level = key_level;
    s_pump_on = 1U;
    s_valve_on = 1U;
    s_gpio_init_calls = 0U;
    s_gpio_read_calls = 0U;
    s_suction_init_calls = 0U;
    s_set_calls = 0U;
    s_bad_gpio_calls = 0U;
    s_bad_commands = 0U;
    app_suction_test_init();
}

static void sample(uint32_t now_ms, uint8_t key_level)
{
    s_key_level = key_level;
    app_suction_test_task(now_ms);
}

static void samples(uint32_t first_ms, unsigned count, uint8_t key_level)
{
    unsigned i;
    for (i = 0U; i < count; ++i)
    {
        sample(first_ms + (uint32_t)i * 5U, key_level);
    }
}

static void arm(uint32_t base_ms)
{
    boot(GPIO_HIGH);
    samples(base_ms, 5U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
}

static void start_sucking(uint32_t base_ms)
{
    arm(base_ms);
    samples(base_ms + 25U, 5U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void start_releasing(uint32_t base_ms)
{
    start_sucking(base_ms);
    samples(base_ms + 50U, 5U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
}

static void test_init_only_configures_c15(void)
{
    boot(GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    EXPECT_EQ(s_gpio_init_calls, 1U);
    EXPECT_EQ(s_gpio_read_calls, 0U);
    EXPECT_EQ(s_suction_init_calls, 1U);
    EXPECT_EQ(s_set_calls, 0U);
}

static void test_boot_held_requires_stable_release(void)
{
    boot(GPIO_LOW);
    samples(0U, 21U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    samples(105U, 4U, GPIO_HIGH);
    samples(125U, 5U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    samples(150U, 5U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
    samples(175U, 4U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    sample(195U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void test_initial_release_needs_full_20ms(void)
{
    boot(GPIO_HIGH);
    samples(0U, 4U, GPIO_HIGH);
    samples(20U, 5U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    samples(45U, 5U, GPIO_HIGH);
    samples(70U, 4U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    sample(90U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void test_press_and_release_20ms_boundaries(void)
{
    arm(0U);
    samples(25U, 4U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    sample(45U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
    samples(50U, 4U, GPIO_HIGH);
    EXPECT_OUTPUT(1U, 0U);
    sample(70U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
}

static void test_press_bounce_does_not_start_pump(void)
{
    arm(0U);
    samples(25U, 2U, GPIO_LOW);
    sample(35U, GPIO_HIGH);
    samples(40U, 2U, GPIO_LOW);
    sample(50U, GPIO_HIGH);
    samples(55U, 4U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    sample(75U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
    EXPECT_EQ(s_set_calls, 1U);
}

static void test_release_bounce_does_not_open_valve(void)
{
    start_sucking(0U);
    samples(50U, 2U, GPIO_HIGH);
    sample(60U, GPIO_LOW);
    samples(65U, 2U, GPIO_HIGH);
    sample(75U, GPIO_LOW);
    samples(80U, 4U, GPIO_HIGH);
    EXPECT_OUTPUT(1U, 0U);
    sample(100U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
    EXPECT_EQ(s_set_calls, 2U);
}

static void test_release_2s_boundary_is_not_restarted(void)
{
    start_releasing(0U);
    samples(75U, 399U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
    EXPECT_EQ(s_set_calls, 2U);
    sample(2070U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
    samples(2075U, 50U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
    EXPECT_EQ(s_set_calls, 3U);
}

static void test_press_during_release_resumes_suction(void)
{
    start_releasing(0U);
    samples(75U, 5U, GPIO_HIGH);
    samples(100U, 4U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 1U);
    sample(120U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
    samples(125U, 5U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
    samples(150U, 399U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
    sample(2145U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
    EXPECT_EQ(s_set_calls, 5U);
}

static void test_merged_tick_does_not_confirm_press(void)
{
    arm(0U);
    sample(25U, GPIO_LOW);
    sample(45U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    samples(50U, 3U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    sample(65U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void test_merged_tick_does_not_confirm_release(void)
{
    start_sucking(0U);
    sample(50U, GPIO_HIGH);
    sample(70U, GPIO_HIGH);
    EXPECT_OUTPUT(1U, 0U);
    samples(75U, 3U, GPIO_HIGH);
    EXPECT_OUTPUT(1U, 0U);
    sample(90U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
}

static void test_duplicate_timestamp_is_not_an_extra_sample(void)
{
    arm(0U);
    samples(25U, 3U, GPIO_LOW);
    sample(35U, GPIO_LOW);
    samples(40U, 3U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    sample(55U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void test_sampling_gaps_do_not_unlock_boot(void)
{
    boot(GPIO_HIGH);
    sample(0U, GPIO_HIGH);
    sample(20U, GPIO_HIGH);
    sample(40U, GPIO_HIGH);
    samples(45U, 5U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    samples(70U, 5U, GPIO_HIGH);
    samples(95U, 5U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void test_release_timer_uses_elapsed_time(void)
{
    start_releasing(0U);
    sample(2069U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
    sample(2070U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
}

static void test_idle_release_does_not_pulse_valve(void)
{
    boot(GPIO_HIGH);
    samples(0U, 101U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
    EXPECT_EQ(s_set_calls, 0U);
}

static void test_held_press_keeps_only_pump_on(void)
{
    start_sucking(0U);
    samples(50U, 200U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
    EXPECT_EQ(s_set_calls, 1U);
}

static void test_debounce_across_uint32_wrap(void)
{
    uint32_t base_ms = UINT32_MAX - 40U;
    arm(base_ms);
    samples(base_ms + 25U, 4U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    sample(base_ms + 45U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void test_release_across_uint32_wrap(void)
{
    uint32_t base_ms = UINT32_MAX - 100U;
    uint32_t release_ms = base_ms + 70U;
    start_releasing(base_ms);
    sample(release_ms + 1999U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 1U);
    sample(release_ms + 2000U, GPIO_HIGH);
    EXPECT_OUTPUT(0U, 0U);
}

static void test_reinit_while_sucking_turns_off_and_relocks(void)
{
    start_sucking(0U);
    s_key_level = GPIO_LOW;
    app_suction_test_init();
    EXPECT_OUTPUT(0U, 0U);
    EXPECT_EQ(s_suction_init_calls, 2U);
    EXPECT_EQ(s_gpio_init_calls, 2U);
    samples(0U, 21U, GPIO_LOW);
    EXPECT_OUTPUT(0U, 0U);
    samples(105U, 5U, GPIO_HIGH);
    samples(130U, 5U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void test_reinit_while_releasing_cancels_old_timer(void)
{
    start_releasing(0U);
    s_key_level = GPIO_HIGH;
    app_suction_test_init();
    EXPECT_OUTPUT(0U, 0U);
    samples(0U, 5U, GPIO_HIGH);
    samples(25U, 5U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
    sample(2070U, GPIO_LOW);
    EXPECT_OUTPUT(1U, 0U);
}

static void run_case(const char *name, void (*test)(void))
{
    unsigned failures_before = s_failures;
    s_case_name = name;
    ++s_cases;
    test();
    EXPECT_EQ(s_bad_gpio_calls, 0U);
    EXPECT_EQ(s_bad_commands, 0U);
    printf("%s %s\n", (s_failures == failures_before) ? "PASS" : "FAIL", name);
}

#define RUN_CASE(test) run_case(#test, test)

int main(void)
{
    RUN_CASE(test_init_only_configures_c15);
    RUN_CASE(test_boot_held_requires_stable_release);
    RUN_CASE(test_initial_release_needs_full_20ms);
    RUN_CASE(test_press_and_release_20ms_boundaries);
    RUN_CASE(test_press_bounce_does_not_start_pump);
    RUN_CASE(test_release_bounce_does_not_open_valve);
    RUN_CASE(test_release_2s_boundary_is_not_restarted);
    RUN_CASE(test_press_during_release_resumes_suction);
    RUN_CASE(test_merged_tick_does_not_confirm_press);
    RUN_CASE(test_merged_tick_does_not_confirm_release);
    RUN_CASE(test_duplicate_timestamp_is_not_an_extra_sample);
    RUN_CASE(test_sampling_gaps_do_not_unlock_boot);
    RUN_CASE(test_release_timer_uses_elapsed_time);
    RUN_CASE(test_idle_release_does_not_pulse_valve);
    RUN_CASE(test_held_press_keeps_only_pump_on);
    RUN_CASE(test_debounce_across_uint32_wrap);
    RUN_CASE(test_release_across_uint32_wrap);
    RUN_CASE(test_reinit_while_sucking_turns_off_and_relocks);
    RUN_CASE(test_reinit_while_releasing_cancels_old_timer);
    printf("RESULT %u cases, %u assertions, %u failures\n",
           s_cases, s_assertions, s_failures);
    return (s_failures == 0U) ? 0 : 1;
}
