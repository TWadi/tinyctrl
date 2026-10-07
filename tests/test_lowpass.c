/**
 * @file test_lowpass.c
 * @brief Unit tests for tc_lowpass.
 *
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stddef.h>

#include "reference/reference_data.h"
#include "test_helpers.h"
#include "tinyctrl/tc_lowpass.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_matches_matlab_filter(void)
{
    tc_lowpass f;
    tc_real output[REF_N];
    int k;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init(&f, (tc_real)ref_lowpass_params[0], (tc_real)ref_lowpass_params[1]));
    tc_lowpass_reset(&f, TC_R(0.0));
    for (k = 0; k < REF_N; ++k)
    {
        output[k] = tc_lowpass_update(&f, (tc_real)ref_input[k]);
    }

    ASSERT_SEQUENCE_CLOSE(ref_lowpass_output, output, REF_N);
}

static void test_step_reaches_63_percent_after_one_time_constant(void)
{
    tc_lowpass f;
    tc_real y = TC_R(0.0);
    int k;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init(&f, TC_R(0.1), TC_R(0.001)));
    tc_lowpass_reset(&f, TC_R(0.0));
    for (k = 0; k < 100; ++k)
    {
        y = tc_lowpass_update(&f, TC_R(1.0));
    }

    ASSERT_REAL_WITHIN(1e-4, 1.0 - exp(-1.0), y);
}

static void test_first_update_returns_its_input(void)
{
    tc_lowpass f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init(&f, TC_R(0.1), TC_R(0.001)));
    ASSERT_REAL_CLOSE(7.0, tc_lowpass_update(&f, TC_R(7.0)));
}

static void test_cutoff_init_matches_time_constant_init(void)
{
    tc_lowpass from_tau;
    tc_lowpass from_cutoff;
    const double tau = 1.0 / (2.0 * 3.14159265358979323846 * 5.0);

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init(&from_tau, (tc_real)tau, TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init_cutoff(&from_cutoff, TC_R(5.0), TC_R(0.001)));

    tc_lowpass_reset(&from_tau, TC_R(0.0));
    tc_lowpass_reset(&from_cutoff, TC_R(0.0));
    ASSERT_REAL_CLOSE(tc_lowpass_update(&from_tau, TC_R(1.0)), tc_lowpass_update(&from_cutoff, TC_R(1.0)));
}

static void check_alpha(double ratio)
{
    /* The first step from rest is alpha = 1 - exp(-Ts / tau), which must be accurate on both sides of the switch
     * from the Taylor series to exp() at Ts / tau = 0.01. */
    const tc_real ts = (tc_real)ratio;
    const double exact = -expm1(-(double)ts);
    tc_lowpass f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init(&f, TC_R(1.0), ts));
    tc_lowpass_reset(&f, TC_R(0.0));
    ASSERT_REAL_WITHIN(1e-6 * exact, exact, tc_lowpass_update(&f, TC_R(1.0)));
}

static void test_alpha_is_accurate_for_small_sample_times(void)
{
    check_alpha(1e-6);
    check_alpha(1e-4);
    check_alpha(1e-3);
    check_alpha(5e-3);
    check_alpha(9.9e-3);
    check_alpha(1.01e-2);
    check_alpha(0.5);
}

static void test_init_alpha(void)
{
    tc_lowpass f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init_alpha(&f, TC_R(0.25)));
    tc_lowpass_reset(&f, TC_R(0.0));
    ASSERT_REAL_CLOSE(0.25, tc_lowpass_update(&f, TC_R(1.0)));
    ASSERT_REAL_CLOSE(0.4375, tc_lowpass_update(&f, TC_R(1.0)));

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init_alpha(&f, TC_R(1.0)));
    tc_lowpass_reset(&f, TC_R(0.0));
    ASSERT_REAL_CLOSE(5.0, tc_lowpass_update(&f, TC_R(5.0)));

    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init_alpha(NULL, TC_R(0.5)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init_alpha(&f, TC_R(0.0)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init_alpha(&f, TC_R(1.5)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init_alpha(&f, (tc_real)NAN));
}

static void test_non_finite_input_holds_the_output(void)
{
    tc_lowpass f;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init_alpha(&f, TC_R(0.5)));

    /* Before the first finite sample the output is 0, and a bad sample does not prime the filter. */
    ASSERT_REAL_CLOSE(0.0, tc_lowpass_update(&f, (tc_real)NAN));
    ASSERT_REAL_CLOSE(4.0, tc_lowpass_update(&f, TC_R(4.0)));

    ASSERT_REAL_CLOSE(4.0, tc_lowpass_update(&f, (tc_real)INFINITY));
    ASSERT_REAL_CLOSE(4.0, tc_lowpass_update(&f, (tc_real)NAN));
    ASSERT_REAL_CLOSE(3.0, tc_lowpass_update(&f, TC_R(2.0)));
}

static void test_reset_ignores_invalid_arguments(void)
{
    tc_lowpass f;

    tc_lowpass_reset(NULL, TC_R(1.0)); /* must not crash */
    TEST_ASSERT_EQUAL_INT(TC_OK, tc_lowpass_init_alpha(&f, TC_R(0.5)));
    tc_lowpass_reset(&f, TC_R(2.0));
    tc_lowpass_reset(&f, (tc_real)NAN);
    ASSERT_REAL_CLOSE(3.0, tc_lowpass_update(&f, TC_R(4.0)));
}

static void test_init_rejects_invalid_arguments(void)
{
    tc_lowpass f;

    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init(NULL, TC_R(0.1), TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init(&f, TC_R(0.0), TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init(&f, TC_R(0.1), TC_R(0.0)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init(&f, (tc_real)NAN, TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init(&f, (tc_real)INFINITY, TC_R(0.001))); /* alpha would be 0 */
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init(&f, TC_R(0.1), (tc_real)INFINITY));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init_cutoff(&f, TC_R(0.0), TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_lowpass_init_cutoff(&f, TC_R(-5.0), TC_R(0.001)));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_matches_matlab_filter);
    RUN_TEST(test_step_reaches_63_percent_after_one_time_constant);
    RUN_TEST(test_first_update_returns_its_input);
    RUN_TEST(test_cutoff_init_matches_time_constant_init);
    RUN_TEST(test_alpha_is_accurate_for_small_sample_times);
    RUN_TEST(test_init_alpha);
    RUN_TEST(test_non_finite_input_holds_the_output);
    RUN_TEST(test_reset_ignores_invalid_arguments);
    RUN_TEST(test_init_rejects_invalid_arguments);
    return UNITY_END();
}
