/**
 * @file test_leadlag.c
 * @brief Unit tests for tc_leadlag.
 *
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stddef.h>

#include "reference/reference_data.h"
#include "test_helpers.h"
#include "tinyctrl/tc_leadlag.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static void check_against_reference(const double params[4], const double *expected)
{
    tc_leadlag c;
    tc_real output[REF_N];
    int k;

    TEST_ASSERT_EQUAL_INT(
        TC_OK, tc_leadlag_init(&c, (tc_real)params[0], (tc_real)params[1], (tc_real)params[2], (tc_real)params[3]));
    tc_leadlag_reset(&c, TC_R(0.0)); /* MATLAB's lsim() starts from rest */
    for (k = 0; k < REF_N; ++k)
    {
        output[k] = tc_leadlag_update(&c, (tc_real)ref_input[k]);
    }

    ASSERT_SEQUENCE_CLOSE(expected, output, REF_N);
}

static void test_matches_matlab_lead(void)
{
    check_against_reference(ref_leadlag1_params, ref_leadlag1_output);
}

static void test_matches_matlab_lag(void)
{
    check_against_reference(ref_leadlag2_params, ref_leadlag2_output);
}

static void test_dc_gain(void)
{
    tc_leadlag c;
    int k;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_leadlag_init(&c, TC_R(3.0), TC_R(10.0), TC_R(100.0), TC_R(0.001)));
    tc_leadlag_reset(&c, TC_R(1.0));
    for (k = 0; k < 10; ++k)
    {
        ASSERT_REAL_CLOSE(3.0, tc_leadlag_update(&c, TC_R(1.0)));
    }
}

static void test_first_update_starts_in_steady_state(void)
{
    tc_leadlag c;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_leadlag_init(&c, TC_R(3.0), TC_R(10.0), TC_R(100.0), TC_R(0.001)));
    ASSERT_REAL_CLOSE(6.0, tc_leadlag_update(&c, TC_R(2.0)));
    ASSERT_REAL_CLOSE(6.0, tc_leadlag_update(&c, TC_R(2.0)));
}

static void test_non_finite_input_holds_the_output(void)
{
    tc_leadlag c;
    tc_real held;

    TEST_ASSERT_EQUAL_INT(TC_OK, tc_leadlag_init(&c, TC_R(1.0), TC_R(10.0), TC_R(100.0), TC_R(0.001)));
    tc_leadlag_reset(&c, TC_R(0.0));
    held = tc_leadlag_update(&c, TC_R(1.0));
    ASSERT_REAL_CLOSE(held, tc_leadlag_update(&c, (tc_real)NAN));
    tc_leadlag_reset(NULL, TC_R(0.0)); /* must not crash */
}

static void test_init_rejects_invalid_arguments(void)
{
    tc_leadlag c;

    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_leadlag_init(NULL, TC_R(1.0), TC_R(10.0), TC_R(100.0), TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_leadlag_init(&c, TC_R(1.0), TC_R(0.0), TC_R(100.0), TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_leadlag_init(&c, TC_R(1.0), TC_R(10.0), TC_R(-1.0), TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_leadlag_init(&c, TC_R(1.0), TC_R(10.0), TC_R(100.0), TC_R(0.0)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_leadlag_init(&c, (tc_real)NAN, TC_R(10.0), TC_R(100.0), TC_R(0.001)));
    TEST_ASSERT_EQUAL_INT(TC_ERR_ARG, tc_leadlag_init(&c, TC_R(1.0), (tc_real)INFINITY, TC_R(100.0), TC_R(0.001)));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_matches_matlab_lead);
    RUN_TEST(test_matches_matlab_lag);
    RUN_TEST(test_dc_gain);
    RUN_TEST(test_first_update_starts_in_steady_state);
    RUN_TEST(test_non_finite_input_holds_the_output);
    RUN_TEST(test_init_rejects_invalid_arguments);
    return UNITY_END();
}
