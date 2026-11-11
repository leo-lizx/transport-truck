#include <stdint.h>
#include <stdio.h>

#include "stepper_profile.h"

static unsigned s_assertions;
static unsigned s_failures;
static unsigned s_cases;
static const char *s_case_name;

static void expect_true(int cond, const char *what, int line)
{
    ++s_assertions;
    if (!cond) {
        ++s_failures;
        printf("FAIL %s:%d %s\n", s_case_name, line, what);
    }
}

#define EXPECT(c) expect_true((c), #c, __LINE__)

typedef struct
{
    uint32_t steps;
    float vfirst;
    float vlast;
    float vpeak;
    float time_s;
    int out_of_range;
    int accel_violation;
} run_result_t;

/* 跑完整个规划，统计速度范围、单步加速度是否超限、总用时。 */
static run_result_t run_profile(uint32_t steps, float vmin, float vmax, float accel)
{
    stepper_profile_t p;
    run_result_t r = {0U, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
    float prev = vmin;

    stepper_profile_start(&p, steps, vmin, vmax, accel);
    while (p.remaining > 0U) {
        float v = stepper_profile_next(&p);
        float vhi = (vmax < vmin) ? vmin : vmax;
        if (r.steps == 0U) {
            r.vfirst = v;
        }
        if ((v < vmin - 1e-3f) || (v > vhi + 1e-3f)) {
            r.out_of_range = 1;
        }
        /* 允许 v² 每步最多变化 2a（加 1% 浮点余量）。 */
        if (accel > 0.0f) {
            float dv2 = v * v - prev * prev;
            if (dv2 < 0.0f) {
                dv2 = -dv2;
            }
            if (dv2 > 2.0f * accel * 1.01f + 1e-3f) {
                r.accel_violation = 1;
            }
        }
        if (v > r.vpeak) {
            r.vpeak = v;
        }
        r.time_s += 1.0f / v;
        r.vlast = v;
        prev = v;
        ++r.steps;
    }
    EXPECT(stepper_profile_next(&p) == 0.0f);
    return r;
}

/* 12 mm 导程、1/8 细分：133.33 步/mm。 */
#define SPM (1600.0f / 12.0f)

static void test_long_move_trapezoid(void)
{
    /* 40 mm，起停 2 mm/s，巡航 10 mm/s，加速度 100 mm/s²。 */
    run_result_t r = run_profile((uint32_t)(40.0f * SPM), 2.0f * SPM, 10.0f * SPM, 100.0f * SPM);
    float ideal = 40.0f / 10.0f;

    EXPECT(r.steps == (uint32_t)(40.0f * SPM));
    EXPECT(r.out_of_range == 0);
    EXPECT(r.accel_violation == 0);
    EXPECT(r.vfirst == 2.0f * SPM);
    EXPECT(r.vpeak > 10.0f * SPM - 1e-3f);
    EXPECT(r.vlast < 2.0f * SPM + 0.5f * SPM);   /* 末速接近起停速度 */
    /* 加减速各约 0.1 s，总时长应略长于 4 s 且不超过 4.3 s。 */
    EXPECT(r.time_s > ideal);
    EXPECT(r.time_s < ideal + 0.3f);
}

static void test_short_move_triangle(void)
{
    /* 加到 10 mm/s 再减回需约 0.96 mm；0.5 mm 只能走三角形曲线（峰值约 7.3 mm/s）。 */
    run_result_t r = run_profile((uint32_t)(0.5f * SPM), 2.0f * SPM, 10.0f * SPM, 100.0f * SPM);

    EXPECT(r.steps == (uint32_t)(0.5f * SPM));
    EXPECT(r.out_of_range == 0);
    EXPECT(r.accel_violation == 0);
    EXPECT(r.vpeak < 10.0f * SPM);
    EXPECT(r.vlast < 2.0f * SPM + 0.5f * SPM);
}

static void test_single_step(void)
{
    run_result_t r = run_profile(1U, 266.0f, 1333.0f, 13333.0f);

    EXPECT(r.steps == 1U);
    EXPECT(r.vfirst == 266.0f);
}

static void test_zero_steps(void)
{
    stepper_profile_t p;

    stepper_profile_start(&p, 0U, 266.0f, 1333.0f, 13333.0f);
    EXPECT(p.remaining == 0U);
    EXPECT(stepper_profile_next(&p) == 0.0f);
}

static void test_no_accel_runs_constant(void)
{
    run_result_t r = run_profile(500U, 266.0f, 1333.0f, 0.0f);

    EXPECT(r.steps == 500U);
    EXPECT(r.vfirst == 1333.0f);
    EXPECT(r.vlast == 1333.0f);
}

static void test_vmax_below_vmin_clamps_to_vmin(void)
{
    run_result_t r = run_profile(300U, 266.0f, 100.0f, 13333.0f);

    EXPECT(r.out_of_range == 0);
    EXPECT(r.vpeak <= 266.0f + 1e-3f);
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
    RUN_CASE(test_long_move_trapezoid);
    RUN_CASE(test_short_move_triangle);
    RUN_CASE(test_single_step);
    RUN_CASE(test_zero_steps);
    RUN_CASE(test_no_accel_runs_constant);
    RUN_CASE(test_vmax_below_vmin_clamps_to_vmin);
    printf("RESULT %u cases, %u assertions, %u failures\n",
           s_cases, s_assertions, s_failures);
    return (s_failures == 0U) ? 0 : 1;
}
