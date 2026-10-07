/**
 * @file test_pid.c
 * @brief Unit tests for tc_pid.
 *
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stddef.h>

#include "reference/reference_data.h"
#include "test_helpers.h"
#include "tinyctrl/tc_pid.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static tc_pid_config make_config(tc_real kp, tc_real ki, tc_real kd, tc_real tf, tc_real ts)
{
    tc_pid_config cfg;

    cfg.kp = kp;
    cfg.ki = ki;
    cfg.kd = kd;
    cfg.tf = tf;
    cfg.ts = ts;
    cfg.out_min = -TC_REAL_MAX;
    cfg.out_max = TC_REAL_MAX;

    return cfg;
}

static tc_pid make_pid(tc_pid_config cfg)
{
    tc_pid pid;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_init(&pid, &cfg));

    return pid;
}

static void check_against_reference(const double params[5], const double *expected)
{
    tc_pid pid = make_pid(make_config((tc_real)params[0], (tc_real)params[1], (tc_real)params[2], (tc_real)params[3],
                                      (tc_real)params[4]));
    tc_real output[REF_N];
    int k;

    /* MATLAB's filter() starts from a zero state. */
    tc_pid_reset(&pid, TC_R(0.0), TC_R(0.0), TC_R(0.0));
    for (k = 0; k < REF_N; ++k)
    {
        output[k] = tc_pid_update(&pid, TC_R(0.0), (tc_real)ref_input[k]);
    }

    ASSERT_SEQUENCE_CLOSE(expected, output, REF_N);
}

static void test_matches_matlab_pid_with_filtered_derivative(void)
{
    check_against_reference(ref_pid1_params, ref_pid1_output);
}

static void test_matches_matlab_pi_controller(void)
{
    check_against_reference(ref_pid2_params, ref_pid2_output);
}

static void test_init_rejects_invalid_arguments(void)
{
    const tc_pid_config valid = make_config(TC_R(2.0), TC_R(5.0), TC_R(0.1), TC_R(0.01), TC_R(0.001));
    tc_pid_config cfg;
    tc_pid pid;

    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(NULL, &valid));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, NULL));

    cfg = valid;
    cfg.ts = TC_R(0.0);
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.ts = TC_R(-0.001);
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.tf = TC_R(0.0); /* a derivative term needs a filter */
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.tf = TC_R(-0.01);
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.kd = TC_R(0.0);
    cfg.tf = TC_R(-0.01); /* rejected even when it is unused */
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.tf = TC_R(0.0004); /* below Ts / 2: the derivative filter would ring */
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.tf = TC_R(0.0005); /* exactly Ts / 2 is allowed */
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.out_min = TC_R(1.0);
    cfg.out_max = TC_R(-1.0);
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.kp = (tc_real)NAN;
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    cfg = valid;
    cfg.ts = (tc_real)INFINITY;
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_init(&pid, &cfg));

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_init(&pid, &valid));
}

static void test_pi_without_filter_is_accepted(void)
{
    tc_pid pid;
    const tc_pid_config cfg = make_config(TC_R(1.0), TC_R(2.0), TC_R(0.0), TC_R(0.0), TC_R(0.01));

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_init(&pid, &cfg));
}

static void test_proportional_only(void)
{
    tc_pid pid = make_pid(make_config(TC_R(3.0), TC_R(0.0), TC_R(0.0), TC_R(0.0), TC_R(0.01)));

    ASSERT_REAL_CLOSE(9.0, tc_pid_update(&pid, TC_R(5.0), TC_R(2.0)));
    ASSERT_REAL_CLOSE(-3.0, tc_pid_update(&pid, TC_R(1.0), TC_R(2.0)));
}

static void test_setpoint_step_causes_no_derivative_kick(void)
{
    tc_pid pid = make_pid(make_config(TC_R(0.0), TC_R(0.0), TC_R(1.0), TC_R(0.01), TC_R(0.001)));

    ASSERT_REAL_CLOSE(0.0, tc_pid_update(&pid, TC_R(0.0), TC_R(1.0)));
    /* The derivative acts on the measurement only: a setpoint step leaves the output unchanged. */
    ASSERT_REAL_CLOSE(0.0, tc_pid_update(&pid, TC_R(10.0), TC_R(1.0)));
    /* A measurement step does move the output, against the direction of the change. */
    TEST_ASSERT_TRUE(tc_pid_update(&pid, TC_R(10.0), TC_R(2.0)) < TC_R(0.0));
}

static void test_first_update_takes_history_from_its_inputs(void)
{
    tc_pid pid = make_pid(make_config(TC_R(0.0), TC_R(0.0), TC_R(1.0), TC_R(0.01), TC_R(0.001)));

    /* Without a reset the first sample primes the history, so a large measurement does not cause a kick. */
    ASSERT_REAL_CLOSE(0.0, tc_pid_update(&pid, TC_R(0.0), TC_R(5.0)));

    /* After resetting to measurement 0, the same sample is a step and produces a derivative response. */
    tc_pid_reset(&pid, TC_R(0.0), TC_R(0.0), TC_R(0.0));
    TEST_ASSERT_TRUE(tc_pid_update(&pid, TC_R(0.0), TC_R(5.0)) < TC_R(0.0));
}

static void test_output_is_limited(void)
{
    tc_pid_config cfg = make_config(TC_R(10.0), TC_R(0.0), TC_R(0.0), TC_R(0.0), TC_R(0.01));
    tc_pid pid;

    cfg.out_min = TC_R(-1.0);
    cfg.out_max = TC_R(1.0);
    pid = make_pid(cfg);

    ASSERT_REAL_CLOSE(1.0, tc_pid_update(&pid, TC_R(1.0), TC_R(0.0)));
    ASSERT_REAL_CLOSE(-1.0, tc_pid_update(&pid, TC_R(-1.0), TC_R(0.0)));
    ASSERT_REAL_CLOSE(0.5, tc_pid_update(&pid, TC_R(0.05), TC_R(0.0)));
}

static void test_anti_windup_recovers_quickly_after_saturation(void)
{
    tc_pid_config cfg = make_config(TC_R(1.0), TC_R(10.0), TC_R(0.0), TC_R(0.0), TC_R(0.1));
    tc_pid pid;
    tc_real output = TC_R(0.0);
    int k;

    cfg.out_min = TC_R(-1.0);
    cfg.out_max = TC_R(1.0);
    pid = make_pid(cfg);

    /* A large error saturates the output for a long time. Without anti-windup the integrator would grow to
     * about 250 and need hundreds of samples to unwind. */
    for (k = 0; k < 50; ++k)
    {
        ASSERT_REAL_CLOSE(1.0, tc_pid_update(&pid, TC_R(5.0), TC_R(0.0)));
    }

    /* When the error reverses, the output must leave the upper limit within a few samples. */
    for (k = 0; k < 3; ++k)
    {
        output = tc_pid_update(&pid, TC_R(-0.5), TC_R(0.0));
    }
    TEST_ASSERT_TRUE(output < TC_R(0.0));
}

static void test_reset_is_bumpless(void)
{
    tc_pid pid = make_pid(make_config(TC_R(2.0), TC_R(1.0), TC_R(0.0), TC_R(0.0), TC_R(0.01)));

    tc_pid_reset(&pid, TC_R(10.0), TC_R(8.0), TC_R(4.0));

    /* Same setpoint and measurement: the output continues from 4 plus one integration step (Ki Ts e = 0.02). */
    ASSERT_REAL_CLOSE(4.02, tc_pid_update(&pid, TC_R(10.0), TC_R(8.0)));
}

static void test_reset_clamps_output_to_limits(void)
{
    tc_pid_config cfg = make_config(TC_R(1.0), TC_R(0.0), TC_R(0.0), TC_R(0.0), TC_R(0.01));
    tc_pid pid;

    cfg.out_min = TC_R(0.0);
    cfg.out_max = TC_R(3.0);
    pid = make_pid(cfg);

    tc_pid_reset(&pid, TC_R(1.0), TC_R(1.0), TC_R(10.0));
    ASSERT_REAL_CLOSE(3.0, tc_pid_update(&pid, TC_R(1.0), TC_R(1.0)));

    /* The integrator itself holds the clamped value, not 10: widening the limits must not release the excess. */
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_set_limits(&pid, TC_R(-100.0), TC_R(100.0)));
    ASSERT_REAL_CLOSE(3.0, tc_pid_update(&pid, TC_R(1.0), TC_R(1.0)));
}

static void test_set_limits(void)
{
    tc_pid pid = make_pid(make_config(TC_R(10.0), TC_R(0.0), TC_R(0.0), TC_R(0.0), TC_R(0.01)));

    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_set_limits(NULL, TC_R(0.0), TC_R(1.0)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_pid_set_limits(&pid, TC_R(2.0), TC_R(1.0)));
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_set_limits(&pid, TC_R(-2.0), TC_R(2.0)));

    ASSERT_REAL_CLOSE(2.0, tc_pid_update(&pid, TC_R(1.0), TC_R(0.0)));
    ASSERT_REAL_CLOSE(-2.0, tc_pid_update(&pid, TC_R(-1.0), TC_R(0.0)));
}

static void test_first_update_integrates_a_full_step(void)
{
    /* Without a reset the first sample is its own history, so the trapezoid covers e[0] for the whole step. */
    tc_pid pid = make_pid(make_config(TC_R(0.0), TC_R(10.0), TC_R(0.0), TC_R(0.0), TC_R(0.1)));

    ASSERT_REAL_CLOSE(2.0, tc_pid_update(&pid, TC_R(2.0), TC_R(0.0)));
    ASSERT_REAL_CLOSE(4.0, tc_pid_update(&pid, TC_R(2.0), TC_R(0.0)));
}

static tc_pid make_saturated_pid(tc_real reset_output)
{
    tc_pid_config cfg = make_config(TC_R(1.0), TC_R(1.0), TC_R(0.0), TC_R(0.0), TC_R(1.0));
    tc_pid pid;

    cfg.out_min = TC_R(-100.0);
    cfg.out_max = TC_R(100.0);
    pid = make_pid(cfg);

    /* Leave the integrator beyond the new limits, as after a limit change at run time. */
    tc_pid_reset(&pid, TC_R(0.0), TC_R(0.0), reset_output);
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_set_limits(&pid, TC_R(-5.0), TC_R(5.0)));

    return pid;
}

static void test_integrator_unwinds_while_saturated_high(void)
{
    tc_pid pid = make_saturated_pid(TC_R(8.0));

    /* Integrator 8 -> 7.5 -> 6.5 -> 5.5; the output is 6.5, 5.5 and 4.5 before limiting. */
    ASSERT_REAL_CLOSE(5.0, tc_pid_update(&pid, TC_R(0.0), TC_R(1.0)));
    ASSERT_REAL_CLOSE(5.0, tc_pid_update(&pid, TC_R(0.0), TC_R(1.0)));
    ASSERT_REAL_CLOSE(4.5, tc_pid_update(&pid, TC_R(0.0), TC_R(1.0)));
}

static void test_integrator_unwinds_while_saturated_low(void)
{
    tc_pid pid = make_saturated_pid(TC_R(-8.0));

    ASSERT_REAL_CLOSE(-5.0, tc_pid_update(&pid, TC_R(0.0), TC_R(-1.0)));
    ASSERT_REAL_CLOSE(-5.0, tc_pid_update(&pid, TC_R(0.0), TC_R(-1.0)));
    ASSERT_REAL_CLOSE(-4.5, tc_pid_update(&pid, TC_R(0.0), TC_R(-1.0)));
}

static void test_anti_windup_at_the_lower_limit(void)
{
    tc_pid_config cfg = make_config(TC_R(1.0), TC_R(10.0), TC_R(0.0), TC_R(0.0), TC_R(0.1));
    tc_pid pid;
    tc_real output = TC_R(0.0);
    int k;

    cfg.out_min = TC_R(-1.0);
    cfg.out_max = TC_R(1.0);
    pid = make_pid(cfg);

    for (k = 0; k < 50; ++k)
    {
        ASSERT_REAL_CLOSE(-1.0, tc_pid_update(&pid, TC_R(-5.0), TC_R(0.0)));
    }
    for (k = 0; k < 3; ++k)
    {
        output = tc_pid_update(&pid, TC_R(0.5), TC_R(0.0));
    }
    TEST_ASSERT_TRUE(output > TC_R(0.0));
}

static void test_reset_clears_derivative_state(void)
{
    tc_pid pid = make_pid(make_config(TC_R(0.0), TC_R(0.0), TC_R(1.0), TC_R(0.01), TC_R(0.001)));

    (void)tc_pid_update(&pid, TC_R(0.0), TC_R(0.0));
    TEST_ASSERT_TRUE(tc_pid_update(&pid, TC_R(0.0), TC_R(5.0)) < TC_R(0.0));

    tc_pid_reset(&pid, TC_R(0.0), TC_R(5.0), TC_R(0.0));
    ASSERT_REAL_CLOSE(0.0, tc_pid_update(&pid, TC_R(0.0), TC_R(5.0)));
}

static void test_filter_at_half_sample_time_is_a_backward_difference(void)
{
    /* Tf = Ts / 2 puts the filter pole at 0: the derivative is Kd * (y[k-1] - y[k]) / Ts and lasts one sample. */
    tc_pid pid = make_pid(make_config(TC_R(0.0), TC_R(0.0), TC_R(1.0), TC_R(0.25), TC_R(0.5)));

    tc_pid_reset(&pid, TC_R(0.0), TC_R(0.0), TC_R(0.0));
    ASSERT_REAL_CLOSE(-2.0, tc_pid_update(&pid, TC_R(0.0), TC_R(1.0)));
    ASSERT_REAL_CLOSE(0.0, tc_pid_update(&pid, TC_R(0.0), TC_R(1.0)));
}

static void test_non_finite_inputs_hold_the_output(void)
{
    tc_pid pid = make_pid(make_config(TC_R(1.0), TC_R(1.0), TC_R(0.1), TC_R(0.5), TC_R(1.0)));
    tc_pid twin = make_pid(make_config(TC_R(1.0), TC_R(1.0), TC_R(0.1), TC_R(0.5), TC_R(1.0)));
    tc_real held;

    (void)tc_pid_update(&twin, TC_R(1.0), TC_R(0.0));
    (void)tc_pid_update(&pid, TC_R(1.0), TC_R(0.0));
    held = tc_pid_update(&twin, TC_R(1.0), TC_R(0.5));
    ASSERT_REAL_CLOSE(held, tc_pid_update(&pid, TC_R(1.0), TC_R(0.5)));

    ASSERT_REAL_CLOSE(held, tc_pid_update(&pid, (tc_real)NAN, TC_R(0.5)));
    ASSERT_REAL_CLOSE(held, tc_pid_update(&pid, TC_R(1.0), (tc_real)NAN));
    ASSERT_REAL_CLOSE(held, tc_pid_update(&pid, TC_R(1.0), (tc_real)INFINITY));
    ASSERT_REAL_CLOSE(held, tc_pid_update(&pid, (tc_real)-INFINITY, TC_R(0.5)));

    /* The bad samples left no trace: both controllers continue identically. */
    ASSERT_REAL_CLOSE(tc_pid_update(&twin, TC_R(1.0), TC_R(0.25)), tc_pid_update(&pid, TC_R(1.0), TC_R(0.25)));
}

static void test_non_finite_first_sample_returns_the_initial_output(void)
{
    tc_pid_config cfg = make_config(TC_R(0.0), TC_R(0.0), TC_R(1.0), TC_R(0.01), TC_R(0.001));
    tc_pid pid;

    cfg.out_min = TC_R(1.0);
    cfg.out_max = TC_R(5.0);
    pid = make_pid(cfg);

    /* Before any valid sample the output is 0 limited to the output range. */
    ASSERT_REAL_CLOSE(1.0, tc_pid_update(&pid, TC_R(0.0), (tc_real)NAN));

    /* The bad sample did not prime the history, so the first valid sample still causes no derivative kick. */
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_set_limits(&pid, -TC_REAL_MAX, TC_REAL_MAX));
    ASSERT_REAL_CLOSE(0.0, tc_pid_update(&pid, TC_R(0.0), TC_R(5.0)));
}

static void test_set_limits_also_limits_the_held_output(void)
{
    tc_pid pid = make_pid(make_config(TC_R(10.0), TC_R(0.0), TC_R(0.0), TC_R(0.0), TC_R(0.01)));

    ASSERT_REAL_CLOSE(50.0, tc_pid_update(&pid, TC_R(5.0), TC_R(0.0)));
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_pid_set_limits(&pid, TC_R(-2.0), TC_R(2.0)));
    ASSERT_REAL_CLOSE(2.0, tc_pid_update(&pid, (tc_real)NAN, TC_R(0.0)));
}

static void test_reset_ignores_invalid_arguments(void)
{
    tc_pid pid = make_pid(make_config(TC_R(2.0), TC_R(1.0), TC_R(0.0), TC_R(0.0), TC_R(0.01)));

    tc_pid_reset(NULL, TC_R(0.0), TC_R(0.0), TC_R(0.0)); /* must not crash */
    tc_pid_reset(&pid, TC_R(10.0), TC_R(8.0), TC_R(4.0));
    tc_pid_reset(&pid, (tc_real)NAN, TC_R(8.0), TC_R(1.0));
    tc_pid_reset(&pid, TC_R(10.0), TC_R(8.0), (tc_real)INFINITY);

    /* The state is still the one from the valid reset, see test_reset_is_bumpless. */
    ASSERT_REAL_CLOSE(4.02, tc_pid_update(&pid, TC_R(10.0), TC_R(8.0)));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_matches_matlab_pid_with_filtered_derivative);
    RUN_TEST(test_matches_matlab_pi_controller);
    RUN_TEST(test_init_rejects_invalid_arguments);
    RUN_TEST(test_pi_without_filter_is_accepted);
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_setpoint_step_causes_no_derivative_kick);
    RUN_TEST(test_first_update_takes_history_from_its_inputs);
    RUN_TEST(test_output_is_limited);
    RUN_TEST(test_anti_windup_recovers_quickly_after_saturation);
    RUN_TEST(test_reset_is_bumpless);
    RUN_TEST(test_reset_clamps_output_to_limits);
    RUN_TEST(test_set_limits);
    RUN_TEST(test_first_update_integrates_a_full_step);
    RUN_TEST(test_integrator_unwinds_while_saturated_high);
    RUN_TEST(test_integrator_unwinds_while_saturated_low);
    RUN_TEST(test_anti_windup_at_the_lower_limit);
    RUN_TEST(test_reset_clears_derivative_state);
    RUN_TEST(test_filter_at_half_sample_time_is_a_backward_difference);
    RUN_TEST(test_non_finite_inputs_hold_the_output);
    RUN_TEST(test_non_finite_first_sample_returns_the_initial_output);
    RUN_TEST(test_set_limits_also_limits_the_held_output);
    RUN_TEST(test_reset_ignores_invalid_arguments);
    return UNITY_END();
}
